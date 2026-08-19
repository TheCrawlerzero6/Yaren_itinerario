"""Teleoperacion articulacion por articulacion desde el teclado.

Permite mover el robot en directo seleccionando un eje y aumentando o
disminuyendo su consigna. Cada pulsacion produce una trayectoria de un
solo punto y muy corta, que se publica en el topico de comando del
controlador de trayectoria.

Esa es la razon de que la teleoperacion no necesite un controlador
propio: enviar consignas sueltas a traves del mismo controlador de
trayectoria evita tener dos controladores disputandose las interfaces de
comando de las mismas articulaciones, que es una situacion que el gestor
no permite y que obligaria a conmutar entre ellos en cada cambio de modo.

Los limites de cada eje no se codifican aqui. Se leen de la descripcion
del robot publicada en /robot_description, de modo que la teleoperacion
respeta automaticamente cualquier ajuste que se haga en el modelo.

Requiere una terminal interactiva de tipo Unix, ya que lee el teclado en
modo carater a carater.

Controles:
    [  ]        articulacion anterior / siguiente
    w  s        aumentar / disminuir la consigna (tambien flechas)
    +  -        aumentar / reducir el paso
    h           volver todas las articulaciones a cero
    espacio     mostrar el estado actual
    q           salir

Uso:
    ros2 run yaren_master teleop_teclado
"""

import select
import sys
import termios
import threading
import tty

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory

from yaren_master.pose_library import (
    limites_desde_urdf,
    saturar,
    trayectoria_a_postura,
)

AYUDA = """
  [  ]        articulacion anterior / siguiente
  w  s        aumentar / disminuir la consigna (tambien flechas)
  +  -        aumentar / reducir el paso
  h           volver todas las articulaciones a cero
  espacio     mostrar el estado actual
  q           salir
"""


class TeleoperacionTeclado(Node):
    """Mantiene el vector de consignas y lo publica al controlador."""

    def __init__(self) -> None:
        super().__init__('teleop_teclado')

        self.declare_parameter('topico_comando', '/yaren_jtc/joint_trajectory')
        self.declare_parameter('paso', 0.05)
        self.declare_parameter('duracion_paso', 0.2)

        self.paso = float(self.get_parameter('paso').value)
        self.duracion_paso = float(self.get_parameter('duracion_paso').value)

        self.publicador = self.create_publisher(
            JointTrajectory, self.get_parameter('topico_comando').value, 10
        )

        self.articulaciones: list[str] = []
        self.consignas: list[float] = []
        self.limites: dict[str, tuple[float, float]] = {}
        self.seleccionada = 0
        self._estado_inicializado = False
        self._cerrojo = threading.Lock()

        self.create_subscription(
            JointState, '/joint_states', self._recibir_estado, 10
        )

        # La descripcion del robot se publica una unica vez, al arrancar.
        # Para recibirla aunque este nodo se suscriba mas tarde hay que
        # pedir durabilidad transitoria: es lo que hace que el publicador
        # conserve el ultimo mensaje y se lo entregue a quien llegue
        # despues.
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

    # ------------------------------------------------------------------

    def _recibir_estado(self, mensaje: JointState) -> None:
        """Toma el estado real del robot como punto de partida.

        Solo se atiende el primer mensaje: a partir de ahi el vector de
        consignas lo gobierna el teclado. Si se siguiera copiando el
        estado medido, el pequeno error de seguimiento del controlador se
        realimentaria y la consigna derivaria sola.
        """
        with self._cerrojo:
            if self._estado_inicializado or not mensaje.name:
                return
            self.articulaciones = list(mensaje.name)
            self.consignas = list(mensaje.position)
            self._estado_inicializado = True

    def _recibir_descripcion(self, mensaje: String) -> None:
        with self._cerrojo:
            self.limites = limites_desde_urdf(mensaje.data)

    def listo(self) -> bool:
        with self._cerrojo:
            return self._estado_inicializado

    # ------------------------------------------------------------------

    def _publicar(self) -> None:
        """Envia el vector de consignas actual como trayectoria corta."""
        self.publicador.publish(
            trayectoria_a_postura(
                self.articulaciones, self.consignas, self.duracion_paso
            )
        )

    def mover(self, incremento: float) -> None:
        with self._cerrojo:
            nombre = self.articulaciones[self.seleccionada]
            objetivo = self.consignas[self.seleccionada] + incremento
            recortado = saturar(objetivo, self.limites.get(nombre))

            if recortado != objetivo:
                self.get_logger().warn(f'{nombre} en su tope ({recortado:+.3f} rad)')

            self.consignas[self.seleccionada] = recortado
            self._publicar()

    def seleccionar(self, desplazamiento: int) -> None:
        with self._cerrojo:
            total = len(self.articulaciones)
            self.seleccionada = (self.seleccionada + desplazamiento) % total

    def a_cero(self) -> None:
        with self._cerrojo:
            self.consignas = [0.0] * len(self.articulaciones)
            self._publicar()

    def ajustar_paso(self, factor: float) -> None:
        self.paso = max(0.005, min(0.5, self.paso * factor))

    def linea_estado(self) -> str:
        with self._cerrojo:
            nombre = self.articulaciones[self.seleccionada]
            valor = self.consignas[self.seleccionada]
            inferior, superior = self.limites.get(nombre, (float('nan'),) * 2)
            return (
                f'[{self.seleccionada + 1}/{len(self.articulaciones)}] '
                f'{nombre:>9} = {valor:+.3f} rad  '
                f'(limites {inferior:+.2f} .. {superior:+.2f})  '
                f'paso {self.paso:.3f}'
            )


def _leer_tecla(plazo: float = 0.1) -> str:
    """Lee una pulsacion sin bloquear indefinidamente.

    Las flechas no producen un caracter sino una secuencia de escape de
    tres bytes; se leen los tres de golpe y se devuelven juntos para que
    quien decide pueda distinguirlas de la tecla Escape suelta.
    """
    if not select.select([sys.stdin], [], [], plazo)[0]:
        return ''

    caracter = sys.stdin.read(1)
    if caracter == '\x1b' and select.select([sys.stdin], [], [], 0.01)[0]:
        return caracter + sys.stdin.read(2)
    return caracter


def bucle_teclado(nodo: TeleoperacionTeclado) -> None:
    """Lee el teclado y traduce cada pulsacion en una orden."""
    print(AYUDA)
    print()
    print('Esperando el estado del robot...')

    while rclpy.ok() and not nodo.listo():
        _leer_tecla(0.2)

    if not rclpy.ok():
        return

    print(nodo.linea_estado())

    while rclpy.ok():
        tecla = _leer_tecla()
        if not tecla:
            continue

        if tecla in ('q', '\x03'):          # q o Ctrl-C
            print('\nSaliendo.')
            return
        elif tecla == ']':
            nodo.seleccionar(+1)
        elif tecla == '[':
            nodo.seleccionar(-1)
        elif tecla in ('w', '\x1b[A'):      # w o flecha arriba
            nodo.mover(+nodo.paso)
        elif tecla in ('s', '\x1b[B'):      # s o flecha abajo
            nodo.mover(-nodo.paso)
        elif tecla in ('+', '='):
            nodo.ajustar_paso(2.0)
        elif tecla in ('-', '_'):
            nodo.ajustar_paso(0.5)
        elif tecla == 'h':
            nodo.a_cero()
        elif tecla == ' ':
            pass                            # solo refresca la linea de estado
        else:
            continue

        print(nodo.linea_estado())


def main(args=None) -> None:
    rclpy.init(args=args)

    nodo = TeleoperacionTeclado()

    # El bucle de teclado bloquea el hilo principal, asi que las
    # suscripciones se atienden en un hilo aparte.
    ejecutor = SingleThreadedExecutor()
    ejecutor.add_node(nodo)
    hilo = threading.Thread(target=ejecutor.spin, daemon=True)
    hilo.start()

    descriptor = sys.stdin.fileno()
    ajustes_previos = termios.tcgetattr(descriptor)

    try:
        # Modo cbreak: cada tecla llega de inmediato, sin esperar a que se
        # pulse Intro. Los ajustes originales se restauran siempre, para
        # no dejar la terminal inutilizable si el nodo termina por error.
        tty.setcbreak(descriptor)
        bucle_teclado(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(descriptor, termios.TCSADRAIN, ajustes_previos)
        ejecutor.shutdown()
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
