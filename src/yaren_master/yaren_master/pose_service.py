"""Servicios para ordenar posturas del catalogo.

El nodo recorre el catalogo de posturas y crea un servicio por cada una,
mas uno de retorno a la postura inicial:

    /yaren/home
    /yaren/pose/reposo
    /yaren/pose/extendido
    /yaren/pose/saludo_a
    ...

Generar los servicios a partir del catalogo, en lugar de exponer un unico
servicio que reciba el nombre de la postura como argumento, tiene dos
consecuencias practicas. La primera es que las posturas disponibles se
descubren con 'ros2 service list', sin necesidad de conocer de antemano
que nombres son validos. La segunda es que anadir una postura al fichero
de configuracion basta para que aparezca su servicio, sin tocar codigo.

Todos usan std_srvs/srv/Trigger, que no lleva datos de entrada. La
duracion del movimiento es el parametro 'duracion' del nodo: es una
configuracion del comportamiento, no un dato de cada peticion.

Cada llamada espera a que el controlador complete la trayectoria antes de
responder, de modo que quien invoca el servicio recibe el resultado real
del movimiento y no un simple acuse de recibo.

Al no necesitar interfaz grafica, es la via mas fiable para comprobar que
la cadena de control funciona cuando la maquina va justa de recursos:
basta una terminal.

Uso:
    ros2 run yaren_master pose_service
    ros2 service list | grep yaren
    ros2 service call /yaren/pose/saludo_a std_srvs/srv/Trigger
    ros2 service call /yaren/home std_srvs/srv/Trigger
"""

import os
import time
from functools import partial

import rclpy
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from yaren_master.pose_library import (
    CatalogoPoses,
    ErrorCatalogo,
    meta_seguir_trayectoria,
    trayectoria_a_postura,
)

ACCION_POR_DEFECTO = '/yaren_jtc/follow_joint_trajectory'


class ServidorPosturas(Node):
    """Expone una postura del catalogo por servicio."""

    def __init__(self) -> None:
        super().__init__('servidor_posturas')

        catalogo_por_defecto = os.path.join(
            get_package_share_directory('yaren_master'), 'config', 'poses.yaml'
        )

        self.declare_parameter('catalogo_poses', catalogo_por_defecto)
        self.declare_parameter('servidor_accion', ACCION_POR_DEFECTO)
        self.declare_parameter('duracion', 0.0)
        self.declare_parameter('espera_aceptacion', 5.0)
        self.declare_parameter('margen_resultado', 15.0)

        self.catalogo = CatalogoPoses(self.get_parameter('catalogo_poses').value)

        # Un valor no positivo significa "usar la duracion por defecto del
        # catalogo", que es donde estan declarados los tiempos del resto
        # del paquete.
        self.duracion = float(self.get_parameter('duracion').value)
        if self.duracion <= 0.0:
            self.duracion = self.catalogo.duracion_por_defecto

        self.espera_aceptacion = float(
            self.get_parameter('espera_aceptacion').value
        )
        self.margen_resultado = float(self.get_parameter('margen_resultado').value)

        # Un unico grupo reentrante para el cliente de accion y para los
        # servicios. Es lo que permite que, mientras una peticion espera a
        # que termine la trayectoria, el ejecutor siga atendiendo las
        # respuestas del controlador en otro hilo. Con el grupo mutuamente
        # excluyente por defecto, la espera bloquearia al propio hilo que
        # tiene que procesar la respuesta y la peticion no terminaria nunca.
        self.grupo = ReentrantCallbackGroup()

        self.cliente = ActionClient(
            self,
            FollowJointTrajectory,
            self.get_parameter('servidor_accion').value,
            callback_group=self.grupo,
        )

        # Un servicio por postura. partial fija el nombre en el momento de
        # crear cada servicio; una lambda que capturase la variable del
        # bucle acabaria refiriendose a la ultima postura en todas las
        # llamadas.
        self.servicios = []
        for nombre in self.catalogo.nombres_poses():
            self.servicios.append(self.create_service(
                Trigger,
                f'/yaren/pose/{nombre}',
                partial(self.atender_postura, nombre),
                callback_group=self.grupo,
            ))

        # Atajo a la postura inicial, con nombre fijo para que quien use el
        # robot no tenga que saber como se llama en el catalogo.
        self.srv_home = self.create_service(
            Trigger,
            '/yaren/home',
            partial(self.atender_postura, self.catalogo.pose_inicial),
            callback_group=self.grupo,
        )

        self.get_logger().info(
            f'{len(self.servicios)} posturas expuestas en /yaren/pose/*  '
            f'(duracion {self.duracion:.1f} s)'
        )
        self.get_logger().info(
            'Disponibles: ' + ', '.join(self.catalogo.nombres_poses())
        )

    # ------------------------------------------------------------------

    def _esperar(self, futuro, plazo: float):
        """Espera a que un futuro se resuelva, sin reentrar en el ejecutor.

        No se usa spin_until_future_complete: ese metodo hace girar el
        ejecutor desde este mismo hilo, y como aqui ya estamos dentro de
        una llamada gestionada por el ejecutor, la anidacion es lo que
        provoca el bloqueo. Con un ejecutor multihilo basta con vigilar el
        futuro, porque quien lo completa es otro hilo.
        """
        limite = time.monotonic() + plazo
        while rclpy.ok() and time.monotonic() < limite:
            if futuro.done():
                return futuro.result()
            time.sleep(0.02)
        return None

    def _ejecutar_postura(self, nombre: str):
        """Envia una postura y devuelve (exito, mensaje)."""
        if not self.cliente.server_is_ready():
            if not self.cliente.wait_for_server(timeout_sec=self.espera_aceptacion):
                return False, (
                    'el controlador de trayectoria no esta disponible; '
                    'comprobar que la simulacion esta en marcha y que el '
                    'controlador figura como activo'
                )

        trayectoria = trayectoria_a_postura(
            self.catalogo.articulaciones,
            self.catalogo.pose(nombre),
            self.duracion,
        )

        self.get_logger().info(f'Postura "{nombre}" en {self.duracion:.1f} s')

        envio = self.cliente.send_goal_async(meta_seguir_trayectoria(trayectoria))
        gestor = self._esperar(envio, self.espera_aceptacion)

        if gestor is None:
            return False, 'el controlador no respondio al envio de la trayectoria'
        if not gestor.accepted:
            return False, 'el controlador rechazo la trayectoria'

        # El plazo del resultado es la duracion pedida mas un margen: si el
        # simulador corre por debajo del tiempo real, el movimiento tarda
        # mas en completarse que lo que marca el reloj de pared.
        respuesta = self._esperar(
            gestor.get_result_async(), self.duracion + self.margen_resultado
        )

        if respuesta is None:
            return False, 'se agoto la espera del resultado de la trayectoria'

        codigo = respuesta.result.error_code
        if codigo != FollowJointTrajectory.Result.SUCCESSFUL:
            return False, (
                f'el controlador aborto la trayectoria (codigo {codigo}): '
                f'{respuesta.result.error_string}'
            )

        return True, f'postura "{nombre}" alcanzada'

    def atender_postura(self, nombre: str, peticion, respuesta):
        """Callback comun a todos los servicios; 'nombre' lo fija partial."""
        del peticion  # Trigger no lleva datos de entrada
        respuesta.success, respuesta.message = self._ejecutar_postura(nombre)
        if not respuesta.success:
            self.get_logger().warn(respuesta.message)
        return respuesta


def main(args=None) -> None:
    rclpy.init(args=args)

    nodo = None
    ejecutor = None
    try:
        nodo = ServidorPosturas()
        ejecutor = MultiThreadedExecutor()
        ejecutor.add_node(nodo)
        ejecutor.spin()
    except ErrorCatalogo as error:
        print(f'[servidor_posturas] catalogo de posturas invalido: {error}')
    except KeyboardInterrupt:
        pass
    finally:
        if ejecutor is not None:
            ejecutor.shutdown()
        if nodo is not None:
            nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
