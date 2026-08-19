"""Utilidades compartidas por los nodos de control del robot Yaren.

Este modulo no contiene ningun nodo. Reune lo que de otro modo se
repetiria en los tres programas de control: la lectura y validacion del
catalogo de posturas, la construccion de mensajes de trayectoria y la
lectura de los limites articulares a partir de la descripcion del robot.

Centralizar la validacion aqui tiene una consecuencia practica: un error
en el catalogo (una postura con un numero de valores incorrecto, una
secuencia que nombra una postura inexistente) se detecta al cargar el
fichero y se comunica con un mensaje concreto, en lugar de manifestarse
mas tarde como una consigna silenciosamente equivocada enviada al robot.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple

import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ErrorCatalogo(Exception):
    """El catalogo de posturas es inconsistente y no puede usarse."""


@dataclass(frozen=True)
class PasoItinerario:
    """Una postura de una secuencia junto con el tiempo para alcanzarla."""

    pose: str
    duracion: float


class CatalogoPoses:
    """Lectura y validacion del fichero de posturas y secuencias.

    Args:
        ruta: camino al fichero YAML del catalogo.

    Raises:
        ErrorCatalogo: si el fichero no existe, no es legible o su
            contenido es inconsistente.
    """

    def __init__(self, ruta: str) -> None:
        self.ruta = ruta

        try:
            with open(ruta, 'r', encoding='utf-8') as fichero:
                datos = yaml.safe_load(fichero)
        except OSError as error:
            raise ErrorCatalogo(
                f'no se pudo abrir el catalogo de posturas "{ruta}": {error}'
            ) from error
        except yaml.YAMLError as error:
            raise ErrorCatalogo(
                f'el catalogo "{ruta}" no es un YAML valido: {error}'
            ) from error

        if not isinstance(datos, dict):
            raise ErrorCatalogo(f'el catalogo "{ruta}" esta vacio')

        self.articulaciones: List[str] = list(datos.get('articulaciones', []))
        if not self.articulaciones:
            raise ErrorCatalogo(
                'el catalogo no declara la lista "articulaciones"; sin ella no '
                'es posible saber a que eje corresponde cada valor de una postura'
            )

        self.duracion_por_defecto: float = float(
            datos.get('duracion_por_defecto', 3.0)
        )
        self.pose_inicial: str = str(datos.get('pose_inicial', 'home'))

        self.poses: Dict[str, List[float]] = {}
        for nombre, valores in (datos.get('poses') or {}).items():
            if not isinstance(valores, (list, tuple)):
                raise ErrorCatalogo(
                    f'la postura "{nombre}" no es una lista de valores'
                )
            if len(valores) != len(self.articulaciones):
                raise ErrorCatalogo(
                    f'la postura "{nombre}" tiene {len(valores)} valores pero '
                    f'el catalogo declara {len(self.articulaciones)} articulaciones'
                )
            self.poses[str(nombre)] = [float(v) for v in valores]

        if not self.poses:
            raise ErrorCatalogo('el catalogo no declara ninguna postura')

        if self.pose_inicial not in self.poses:
            raise ErrorCatalogo(
                f'"pose_inicial" apunta a la postura "{self.pose_inicial}", '
                'que no esta declarada'
            )

        self.secuencias: Dict[str, List[PasoItinerario]] = {}
        for nombre, pasos in (datos.get('secuencias') or {}).items():
            self.secuencias[str(nombre)] = self._leer_secuencia(str(nombre), pasos)

    def _leer_secuencia(self, nombre: str, pasos) -> List[PasoItinerario]:
        """Convierte una secuencia del YAML y comprueba sus referencias."""
        if not isinstance(pasos, (list, tuple)) or not pasos:
            raise ErrorCatalogo(f'la secuencia "{nombre}" esta vacia')

        resultado: List[PasoItinerario] = []
        for indice, paso in enumerate(pasos):
            if not isinstance(paso, dict) or 'pose' not in paso:
                raise ErrorCatalogo(
                    f'el paso {indice + 1} de la secuencia "{nombre}" no indica '
                    'ninguna postura'
                )
            postura = str(paso['pose'])
            if postura not in self.poses:
                raise ErrorCatalogo(
                    f'el paso {indice + 1} de la secuencia "{nombre}" referencia '
                    f'la postura "{postura}", que no esta declarada'
                )
            duracion = float(paso.get('duracion', self.duracion_por_defecto))
            if duracion <= 0.0:
                raise ErrorCatalogo(
                    f'el paso {indice + 1} de la secuencia "{nombre}" tiene una '
                    f'duracion no positiva ({duracion})'
                )
            resultado.append(PasoItinerario(pose=postura, duracion=duracion))
        return resultado

    # -- consulta --------------------------------------------------------

    def existe(self, nombre: str) -> bool:
        return nombre in self.poses

    def pose(self, nombre: str) -> List[float]:
        """Valores articulares de una postura, en el orden del catalogo."""
        if nombre not in self.poses:
            raise KeyError(nombre)
        return list(self.poses[nombre])

    def secuencia(self, nombre: str) -> List[PasoItinerario]:
        if nombre not in self.secuencias:
            raise KeyError(nombre)
        return list(self.secuencias[nombre])

    def nombres_poses(self) -> List[str]:
        return sorted(self.poses)

    def nombres_secuencias(self) -> List[str]:
        return sorted(self.secuencias)


# ----------------------------------------------------------------------
# Construccion de mensajes
# ----------------------------------------------------------------------

def _a_duracion(segundos: float) -> Duration:
    """Convierte segundos en el par (sec, nanosec) del mensaje de tiempo."""
    enteros = int(segundos)
    return Duration(
        sec=enteros,
        nanosec=int(round((segundos - enteros) * 1e9)),
    )


def trayectoria_a_postura(
    articulaciones: Sequence[str],
    valores: Sequence[float],
    duracion: float,
) -> JointTrajectory:
    """Trayectoria de un unico punto: la postura destino.

    El controlador interpola por si mismo entre la posicion actual y ese
    punto, asi que no hace falta generar puntos intermedios. Se envian
    tambien velocidades nulas en el destino para que el movimiento
    termine en reposo y no con el robot todavia en marcha.
    """
    mensaje = JointTrajectory()
    mensaje.joint_names = list(articulaciones)

    punto = JointTrajectoryPoint()
    punto.positions = [float(v) for v in valores]
    punto.velocities = [0.0] * len(valores)
    punto.time_from_start = _a_duracion(duracion)

    mensaje.points = [punto]
    return mensaje


def meta_seguir_trayectoria(
    trayectoria: JointTrajectory,
) -> FollowJointTrajectory.Goal:
    """Envuelve una trayectoria en la meta que espera el controlador."""
    meta = FollowJointTrajectory.Goal()
    meta.trajectory = trayectoria
    return meta


# ----------------------------------------------------------------------
# Lectura de los limites articulares
# ----------------------------------------------------------------------

def limites_desde_urdf(urdf: str) -> Dict[str, Tuple[float, float]]:
    """Extrae los topes de cada articulacion revoluta de un URDF.

    Los limites se leen de la descripcion publicada por el robot y no se
    copian en la configuracion de los nodos: asi existe una unica fuente
    de verdad y ajustar un tope en la descripcion basta para que todos
    los programas de control lo respeten.

    Args:
        urdf: contenido del URDF ya expandido.

    Returns:
        Diccionario nombre de articulacion -> (inferior, superior). Las
        articulaciones sin limites declarados no aparecen.
    """
    limites: Dict[str, Tuple[float, float]] = {}
    try:
        raiz = ET.fromstring(urdf)
    except ET.ParseError:
        return limites

    for articulacion in raiz.iter('joint'):
        nombre = articulacion.get('name')
        tipo = articulacion.get('type')
        if not nombre or tipo not in ('revolute', 'prismatic'):
            continue
        limite = articulacion.find('limit')
        if limite is None:
            continue
        inferior = limite.get('lower')
        superior = limite.get('upper')
        if inferior is None or superior is None:
            continue
        limites[nombre] = (float(inferior), float(superior))

    return limites


def saturar(valor: float, limites: Tuple[float, float] | None) -> float:
    """Recorta un valor a su intervalo permitido."""
    if limites is None:
        return valor
    inferior, superior = limites
    return max(inferior, min(superior, valor))


def reordenar(
    valores: Sequence[float],
    orden_origen: Sequence[str],
    orden_destino: Sequence[str],
) -> List[float]:
    """Reordena un vector de valores de un orden articular a otro.

    Las articulaciones de 'orden_destino' que no figuren en el origen
    reciben cero.
    """
    indice = {nombre: i for i, nombre in enumerate(orden_origen)}
    return [
        float(valores[indice[nombre]]) if nombre in indice else 0.0
        for nombre in orden_destino
    ]
