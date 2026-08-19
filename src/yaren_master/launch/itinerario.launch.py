"""Simulacion completa mas la ejecucion automatica de un itinerario.

Encadena el arranque de la simulacion con el nodo que recorre una
secuencia de posturas del catalogo. Es el modo pensado para grabar la
demostracion: una sola orden deja el robot moviendose solo.

El nodo se lanza con un retardo corto, pero no depende de el: antes de
enviar nada espera activamente a que el servidor de accion del
controlador de trayectoria este disponible. El retardo solo evita que
imprima avisos de espera mientras el mundo todavia se esta cargando.

Uso:
    ros2 launch yaren_master itinerario.launch.py
    ros2 launch yaren_master itinerario.launch.py secuencia:=saludo
    ros2 launch yaren_master itinerario.launch.py secuencia:=verificacion repetir:=true
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    master_share = get_package_share_directory('yaren_master')
    master = FindPackageShare('yaren_master')

    catalogo_poses = PathJoinSubstitution([master, 'config', 'poses.yaml'])

    arg_secuencia = DeclareLaunchArgument(
        'secuencia',
        default_value='demostracion',
        description='Nombre de la secuencia del catalogo de posturas a ejecutar.',
    )
    arg_repetir = DeclareLaunchArgument(
        'repetir',
        default_value='false',
        description='Repetir la secuencia de forma indefinida.',
    )
    arg_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Abrir la ventana del simulador.',
    )
    arg_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Abrir RViz ademas del simulador.',
    )

    simulacion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(master_share, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'rviz': LaunchConfiguration('rviz'),
        }.items(),
    )

    itinerario = Node(
        package='yaren_master',
        executable='itinerario_node',
        name='itinerario',
        output='screen',
        parameters=[{
            'catalogo_poses': catalogo_poses,
            'secuencia': LaunchConfiguration('secuencia'),
            # El valor de un argumento de arranque llega siempre como
            # texto. Sin la conversion explicita el nodo recibiria la
            # cadena "false" donde declara un booleano y rechazaria el
            # parametro por discrepancia de tipo.
            'repetir': ParameterValue(
                LaunchConfiguration('repetir'), value_type=bool
            ),
            'use_sim_time': True,
        }],
    )

    return LaunchDescription([
        arg_secuencia,
        arg_repetir,
        arg_gui,
        arg_rviz,
        simulacion,
        TimerAction(period=12.0, actions=[itinerario]),
    ])
