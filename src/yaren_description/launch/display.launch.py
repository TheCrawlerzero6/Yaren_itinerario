"""Visualizacion del modelo del robot sin simulacion dinamica.

Levanta la cadena minima necesaria para ver el robot y su arbol TF:

    joint_state_publisher_gui  ->  /joint_states
    robot_state_publisher      ->  /tf, /robot_description
    rviz2                      ->  representacion grafica

No interviene ningun motor de fisica ni ningun controlador: las
articulaciones se mueven directamente desde los deslizadores de la
interfaz. Es el modo adecuado para revisar la geometria, comprobar los
limites articulares y verificar que el arbol de transformadas se
construye completo antes de pasar a la simulacion.

Vive en el paquete de descripcion y no en el de control porque lo que
levanta es exclusivamente el modelo del robot: no arranca fisica, ni
gestor de controladores, ni mundo. El paquete de control aporta los otros
dos ficheros de arranque, los que si necesitan simulador.

Uso:
    ros2 launch yaren_description display.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    modelo = FindPackageShare('yaren_description')

    urdf_xacro = PathJoinSubstitution([modelo, 'urdf', 'yaren.urdf.xacro'])
    rviz_config = PathJoinSubstitution([modelo, 'rviz', 'yaren.rviz'])

    arg_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=rviz_config,
        description='Fichero de configuracion de RViz a cargar.',
    )

    # El URDF se genera expandiendo la plantilla en tiempo de arranque.
    #
    # ParameterValue con value_type=str no es opcional: el resultado de
    # Command es texto sin tipo, y sin la anotacion explicita el sistema
    # de parametros intenta deducir el tipo del contenido del URDF, lo
    # que provoca un error de conversion al arrancar.
    descripcion_robot = ParameterValue(
        Command(['xacro ', urdf_xacro, ' sim:=false']),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': descripcion_robot}],
    )

    # Publica /joint_states a partir de un deslizador por articulacion.
    # Toma la lista de articulaciones y sus limites del propio URDF, de
    # modo que los deslizadores quedan acotados a los topes declarados.
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )

    return LaunchDescription([
        arg_rviz_config,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
    ])
