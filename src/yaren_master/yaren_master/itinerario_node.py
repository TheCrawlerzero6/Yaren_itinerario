"""Recorrido automatico de una secuencia de posturas.

Toma una secuencia del catalogo y la ejecuta paso a paso contra el
controlador de trayectoria articular, esperando a que cada postura se
complete antes de enviar la siguiente.

La espera es deliberada y no un simple retardo. El controlador expone su
seguimiento como una accion, de modo que informa de si la trayectoria
fue aceptada, si termino correctamente o si se aborto. Encadenar los
pasos con temporizadores fijos parece mas sencillo, pero desincroniza el
programa del robot en cuanto el simulador se ejecuta por debajo del
tiempo real, algo habitual en una maquina virtual: los envios se
acumularian y cada postura interrumpiria a la anterior.

Uso:
    ros2 run yaren_master itinerario_node
    ros2 run yaren_master itinerario_node --ros-args -p secuencia:=saludo
"""

import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node

from yaren_master.pose_library import (
    CatalogoPoses,
    ErrorCatalogo,
    meta_seguir_trayectoria,
    trayectoria_a_postura,
)

ACCION_POR_DEFECTO = '/yaren_jtc/follow_joint_trajectory'


class NodoItinerario(Node):
    """Cliente de accion que recorre una secuencia de posturas."""

    def __init__(self) -> None:
        super().__init__('itinerario')

        catalogo_por_defecto = os.path.join(
            get_package_share_directory('yaren_master'), 'config', 'poses.yaml'
        )

        self.declare_parameter('catalogo_poses', catalogo_por_defecto)
        self.declare_parameter('secuencia', 'demostracion')
        self.declare_parameter('repetir', False)
        self.declare_parameter('escala_tiempo', 1.0)
        self.declare_parameter('servidor_accion', ACCION_POR_DEFECTO)

        ruta = self.get_parameter('catalogo_poses').value
        self.nombre_secuencia = self.get_parameter('secuencia').value
        self.repetir = bool(self.get_parameter('repetir').value)

        # Un factor mayor que uno alarga todas las duraciones por igual.
        # Sirve para ralentizar el itinerario completo sin reescribir el
        # catalogo, por ejemplo al grabar la demostracion.
        self.escala_tiempo = float(self.get_parameter('escala_tiempo').value)
        if self.escala_tiempo <= 0.0:
            self.get_logger().warn(
                f'escala_tiempo={self.escala_tiempo} no es valida; se usa 1.0'
            )
            self.escala_tiempo = 1.0

        self.catalogo = CatalogoPoses(ruta)

        if self.nombre_secuencia not in self.catalogo.secuencias:
            disponibles = ', '.join(self.catalogo.nombres_secuencias())
            raise ErrorCatalogo(
                f'la secuencia "{self.nombre_secuencia}" no esta declarada en '
                f'{ruta}. Secuencias disponibles: {disponibles}'
            )

        self.cliente = ActionClient(
            self,
            FollowJointTrajectory,
            self.get_parameter('servidor_accion').value,
        )

    # ------------------------------------------------------------------

    def esperar_controlador(self) -> None:
        """Bloquea hasta que el controlador ofrezca su servidor de accion.

        No se fija un plazo maximo: cuando este nodo se arranca junto con
        la simulacion, el controlador puede tardar en activarse tanto
        como tarde el mundo en cargarse, y ese tiempo depende por
        completo de la maquina.
        """
        self.get_logger().info('Esperando al controlador de trayectoria...')
        while not self.cliente.wait_for_server(timeout_sec=2.0):
            if not rclpy.ok():
                raise KeyboardInterrupt
            self.get_logger().info('  ...el controlador todavia no responde')
        self.get_logger().info('Controlador disponible.')

    def ir_a_postura(self, nombre: str, duracion: float) -> bool:
        """Envia una postura y espera a que el controlador la complete.

        Returns:
            True si la trayectoria termino correctamente.
        """
        trayectoria = trayectoria_a_postura(
            self.catalogo.articulaciones,
            self.catalogo.pose(nombre),
            duracion,
        )
        meta = meta_seguir_trayectoria(trayectoria)

        self.get_logger().info(f'-> "{nombre}" en {duracion:.1f} s')

        envio = self.cliente.send_goal_async(meta)
        rclpy.spin_until_future_complete(self, envio)
        gestor = envio.result()

        if gestor is None or not gestor.accepted:
            self.get_logger().error(
                f'   el controlador rechazo la postura "{nombre}"'
            )
            return False

        resultado_futuro = gestor.get_result_async()
        rclpy.spin_until_future_complete(self, resultado_futuro)
        respuesta = resultado_futuro.result()

        if respuesta is None:
            self.get_logger().error(
                f'   no se recibio resultado para la postura "{nombre}"'
            )
            return False

        codigo = respuesta.result.error_code
        if codigo != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(
                f'   la postura "{nombre}" fallo (codigo {codigo}): '
                f'{respuesta.result.error_string}'
            )
            return False

        return True

    def ejecutar(self) -> None:
        """Recorre la secuencia, una o infinitas veces segun 'repetir'."""
        self.esperar_controlador()

        pasos = self.catalogo.secuencia(self.nombre_secuencia)
        vuelta = 0

        while rclpy.ok():
            vuelta += 1
            self.get_logger().info(
                f'=== Secuencia "{self.nombre_secuencia}", '
                f'vuelta {vuelta}, {len(pasos)} posturas ==='
            )

            for paso in pasos:
                if not rclpy.ok():
                    return
                if not self.ir_a_postura(
                    paso.pose, paso.duracion * self.escala_tiempo
                ):
                    self.get_logger().error('Itinerario interrumpido.')
                    return

            self.get_logger().info('=== Secuencia completada ===')

            if not self.repetir:
                return


def main(args=None) -> None:
    rclpy.init(args=args)

    nodo = None
    try:
        nodo = NodoItinerario()
        nodo.ejecutar()
    except ErrorCatalogo as error:
        # El catalogo es la entrada de datos del nodo: si es incorrecto no
        # tiene sentido arrancar, y el mensaje debe decir exactamente que
        # corregir en el fichero.
        print(f'[itinerario] catalogo de posturas invalido: {error}')
    except KeyboardInterrupt:
        pass
    finally:
        if nodo is not None:
            nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
