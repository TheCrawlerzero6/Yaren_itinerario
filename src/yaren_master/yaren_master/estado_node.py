"""Vigilancia del estado articular del robot.

Escucha /joint_states y publica en /yaren/estado un resumen legible de la
situacion de las 12 articulaciones. Ademas avisa por el registro de dos
condiciones que no son errores del sistema pero que explican la mayoria
de los comportamientos extranos durante las pruebas:

  * Una articulacion que trabaja pegada a su tope. El controlador la
    recorta en silencio, asi que el robot no alcanza la postura pedida y
    no aparece ningun error en ninguna parte. Este aviso es lo que
    permite relacionar una postura del catalogo con un limite demasiado
    estrecho en la descripcion del robot.

  * Ausencia de mensajes de estado. Si el difusor de estado no llego a
    activarse, el robot se ve inmovil en el simulador y todo lo demas
    parece correcto; el aviso senala directamente donde mirar.

Uso:
    ros2 run yaren_master estado_node
    ros2 topic echo /yaren/estado
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from yaren_master.pose_library import limites_desde_urdf


class VigilanteEstado(Node):
    """Resume el estado articular y avisa de condiciones de riesgo."""

    def __init__(self) -> None:
        super().__init__('vigilante_estado')

        self.declare_parameter('frecuencia', 2.0)
        self.declare_parameter('margen_limite', 0.05)
        self.declare_parameter('plazo_silencio', 3.0)

        self.margen_limite = float(self.get_parameter('margen_limite').value)
        self.plazo_silencio = float(self.get_parameter('plazo_silencio').value)

        self.limites: dict[str, tuple[float, float]] = {}
        self.ultimo_estado: JointState | None = None
        self.instante_ultimo_estado = None

        # Conjunto de articulaciones que ya estan en su tope. Se guarda
        # para avisar solo en la transicion y no repetir el mismo mensaje
        # en cada ciclo mientras la articulacion siga ahi.
        self.en_tope: set[str] = set()
        self.silencio_avisado = False

        self.publicador = self.create_publisher(String, '/yaren/estado', 10)

        self.create_subscription(
            JointState, '/joint_states', self._recibir_estado, 10
        )

        self.create_subscription(
            String,
            '/robot_description',
            self._recibir_descripcion,
            QoSProfile(
                depth=1,
                history=HistoryPolicy.KEEP_LAST,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        periodo = 1.0 / max(0.1, float(self.get_parameter('frecuencia').value))
        self.create_timer(periodo, self._publicar_resumen)

    # ------------------------------------------------------------------

    def _recibir_estado(self, mensaje: JointState) -> None:
        self.ultimo_estado = mensaje
        self.instante_ultimo_estado = self.get_clock().now()
        if self.silencio_avisado:
            self.get_logger().info('Se reanudo la recepcion de /joint_states.')
            self.silencio_avisado = False

    def _recibir_descripcion(self, mensaje: String) -> None:
        self.limites = limites_desde_urdf(mensaje.data)
        self.get_logger().info(
            f'Limites leidos de la descripcion: {len(self.limites)} articulaciones'
        )

    # ------------------------------------------------------------------

    def _vigilar_silencio(self) -> bool:
        """Devuelve True si hay estado reciente; avisa una vez si no."""
        if self.instante_ultimo_estado is None:
            if not self.silencio_avisado:
                self.get_logger().warn(
                    'Todavia no se ha recibido ningun /joint_states. Comprobar '
                    'que el difusor de estado figura como activo con '
                    '"ros2 control list_controllers".'
                )
                self.silencio_avisado = True
            return False

        transcurrido = (
            self.get_clock().now() - self.instante_ultimo_estado
        ).nanoseconds / 1e9

        if transcurrido > self.plazo_silencio:
            if not self.silencio_avisado:
                self.get_logger().warn(
                    f'Sin /joint_states desde hace {transcurrido:.1f} s.'
                )
                self.silencio_avisado = True
            return False

        return True

    def _vigilar_topes(self, nombre: str, posicion: float) -> str:
        """Marca una articulacion que trabaja en su tope y avisa al entrar."""
        limites = self.limites.get(nombre)
        if limites is None:
            return ' '

        inferior, superior = limites
        pegada = (
            posicion <= inferior + self.margen_limite
            or posicion >= superior - self.margen_limite
        )

        if pegada and nombre not in self.en_tope:
            self.en_tope.add(nombre)
            self.get_logger().warn(
                f'{nombre} esta en su tope ({posicion:+.3f} rad, '
                f'limites {inferior:+.3f} .. {superior:+.3f})'
            )
        elif not pegada and nombre in self.en_tope:
            self.en_tope.discard(nombre)

        return '!' if pegada else ' '

    def _publicar_resumen(self) -> None:
        if not self._vigilar_silencio() or self.ultimo_estado is None:
            return

        estado = self.ultimo_estado
        velocidades = list(estado.velocity) or [0.0] * len(estado.name)

        lineas = ['articulacion   posicion[rad]  velocidad[rad/s]  tope']
        for indice, nombre in enumerate(estado.name):
            posicion = estado.position[indice]
            velocidad = velocidades[indice] if indice < len(velocidades) else 0.0
            marca = self._vigilar_topes(nombre, posicion)
            lineas.append(
                f'{nombre:<13} {posicion:>+13.4f}  {velocidad:>+15.4f}   {marca}'
            )

        en_movimiento = sum(1 for v in velocidades if abs(v) > 1e-3)
        lineas.append(
            f'articulaciones en movimiento: {en_movimiento}/{len(estado.name)}   '
            f'en tope: {len(self.en_tope)}'
        )

        self.publicador.publish(String(data='\n'.join(lineas)))


def main(args=None) -> None:
    rclpy.init(args=args)
    nodo = VigilanteEstado()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
