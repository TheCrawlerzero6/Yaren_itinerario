"""Simulacion dinamica del robot Yaren con su sistema de control.

Este fichero encadena siete pasos cuyo orden importa. Cada uno depende
de que el anterior haya dejado algo disponible:

  1. Ruta de recursos del simulador.  El simulador debe poder resolver
     las URIs 'package://' de las mallas; si no encuentra la ruta el
     modelo se carga pero se dibuja vacio.
  2. Arranque del mundo.
  3. robot_state_publisher.  Publica el URDF expandido en el topico
     /robot_description, que es de donde lo toma el paso 5.
  4. Puente del reloj.  El gestor de controladores toma el tiempo del
     simulador; sin este puente su reloj nunca avanza y ningun
     controlador llega a activarse.
  5. Insercion del robot en el mundo.  Al cargarse el modelo, el plugin
     de control arranca el gestor de controladores.
  6. Activacion de los controladores, encadenada: primero el difusor de
     estado y, cuando termina, el controlador de trayectoria. Lanzarlos
     a la vez produce fallos intermitentes de activacion, porque ambos
     compiten por el gestor mientras este todavia se esta inicializando.
  7. RViz, opcional.

Uso:
    ros2 launch yaren_master simulation.launch.py
    ros2 launch yaren_master simulation.launch.py rviz:=true
    ros2 launch yaren_master simulation.launch.py gui:=false      # sin ventana
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    modelo_share = get_package_share_directory('yaren_description')
    master_share = get_package_share_directory('yaren_master')

    modelo = FindPackageShare('yaren_description')
    master = FindPackageShare('yaren_master')

    urdf_xacro = PathJoinSubstitution([modelo, 'urdf', 'yaren.urdf.xacro'])
    rviz_config = PathJoinSubstitution([modelo, 'rviz', 'yaren.rviz'])
    controladores = PathJoinSubstitution([master, 'config', 'yaren_controllers.yaml'])

    # ------------------------------------------------------------------
    # Argumentos
    # ------------------------------------------------------------------
    arg_mundo = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(master_share, 'worlds', 'laboratorio.sdf'),
        description='Fichero SDF del mundo a cargar.',
    )
    arg_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Abrir la ventana del simulador. Con false se ejecuta '
                    'solo la fisica, util cuando la maquina no tiene '
                    'aceleracion grafica.',
    )
    arg_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Abrir RViz ademas del simulador.',
    )

    # ------------------------------------------------------------------
    # 1. Ruta de recursos del simulador
    #
    # Las URIs del URDF tienen la forma package://yaren_description/meshes/...
    # El simulador las resuelve buscando un directorio 'yaren_description'
    # dentro de las rutas de esta variable, asi que hay que anadir el
    # directorio que contiene al paquete, es decir el padre de su
    # directorio share.
    # ------------------------------------------------------------------
    ruta_recursos = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.dirname(modelo_share),
    )

    # ------------------------------------------------------------------
    # 2. Mundo
    #
    #   -r  arranca la simulacion en marcha en lugar de en pausa
    #   -s  modo servidor, sin interfaz grafica
    #   -v 3  nivel de detalle de los mensajes, suficiente para ver los
    #         errores de carga de mallas sin inundar la terminal
    # ------------------------------------------------------------------
    gz_sim_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
    )

    simulador_con_ventana = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        condition=IfCondition(LaunchConfiguration('gui')),
        launch_arguments={
            'gz_args': ['-r -v 3 ', LaunchConfiguration('world')],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    simulador_sin_ventana = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        condition=UnlessCondition(LaunchConfiguration('gui')),
        launch_arguments={
            'gz_args': ['-s -r -v 3 ', LaunchConfiguration('world')],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # ------------------------------------------------------------------
    # 3. Publicacion de la descripcion del robot
    #
    # Se expande con sim:=true para que el bloque de control declare la
    # interfaz de hardware simulada, y se le indica donde esta el fichero
    # de controladores que debera cargar el plugin del simulador.
    # ------------------------------------------------------------------
    descripcion_robot = ParameterValue(
        Command([
            'xacro ', urdf_xacro,
            ' sim:=true',
            ' controllers_file:=', controladores,
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': descripcion_robot,
            'use_sim_time': True,
        }],
    )

    # ------------------------------------------------------------------
    # 4. Puente del reloj de simulacion
    # ------------------------------------------------------------------
    puente_reloj = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='puente_reloj',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': True}],
    )

    # ------------------------------------------------------------------
    # 5. Insercion del robot
    #
    # Se inserta a partir del topico y no de un fichero, de modo que el
    # modelo que entra en el mundo es exactamente el mismo que publica
    # robot_state_publisher y no puede haber discrepancia entre lo que se
    # simula y lo que se visualiza.
    # ------------------------------------------------------------------
    insertar_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='insertar_yaren',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'yaren',
            '-allow_renaming', 'true',
        ],
        parameters=[{'use_sim_time': True}],
    )

    # ------------------------------------------------------------------
    # 6. Activacion de los controladores
    #
    # El margen de espera es amplio a proposito: en una maquina virtual
    # el gestor de controladores puede tardar bastante en responder
    # despues de que el modelo entre en el mundo.
    # ------------------------------------------------------------------
    difusor_estado = Node(
        package='controller_manager',
        executable='spawner',
        name='activar_joint_state_broadcaster',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120',
        ],
    )

    controlador_trayectoria = Node(
        package='controller_manager',
        executable='spawner',
        name='activar_yaren_jtc',
        output='screen',
        arguments=[
            'yaren_jtc',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120',
        ],
    )

    tras_insertar_robot = RegisterEventHandler(
        OnProcessExit(
            target_action=insertar_robot,
            on_exit=[difusor_estado],
        )
    )

    tras_difusor_estado = RegisterEventHandler(
        OnProcessExit(
            target_action=difusor_estado,
            on_exit=[controlador_trayectoria],
        )
    )

    # ------------------------------------------------------------------
    # 7. Visualizacion opcional
    # ------------------------------------------------------------------
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        arg_mundo,
        arg_gui,
        arg_rviz,
        ruta_recursos,
        simulador_con_ventana,
        simulador_sin_ventana,
        robot_state_publisher,
        puente_reloj,
        insertar_robot,
        tras_insertar_robot,
        tras_difusor_estado,
        rviz,
    ])
