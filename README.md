# Yaren — simulación y control

Espacio de trabajo ROS 2 para la descripción, simulación dinámica y
control del robot Yaren.

- **ROS 2** Jazzy Jalisco
- **Simulador** Gazebo Harmonic (`gz-sim` 8)
- **Sistema de control** `ros2_control` con `gz_ros2_control`

## Paquetes

| Paquete | Tipo | Contenido | Arranque |
|---|---|---|---|
| `yaren_description` | `ament_cmake` | Descripción del robot: URDF en xacro, mallas, propiedades inerciales y de colisión, interfaces de hardware | `display.launch.py` |
| `yaren_master` | `ament_python` | Mundo de simulación, configuración de controladores y programas de control | `simulation.launch.py`, `itinerario.launch.py` |

## Estructura cinemática

Yaren es un **torso humanoide de base fija**: 13 eslabones y 12
articulaciones revolutas, más un anclaje fijo al marco inercial. La
cadena no es serial — de `link_2`, la pieza central del torso, parten
tres ramas.

```
world
  │ base_joint   (articulación fija)
base_link
  │ joint_1
link_1
  │ joint_2
link_2
  │
  ├─ joint_3  → link_3  →  joint_4  → link_4
  │
  ├─ joint_5  → link_5  →  joint_6  → link_6
  │             joint_7  → link_7  →  joint_8  → link_8
  │
  └─ joint_9  → link_9  →  joint_10 → link_10
                joint_11 → link_11 →  joint_12 → link_12
```

### Función de cada articulación

El URDF declara cada eje en el marco local del eslabón, que tras acumular
las rotaciones de la cadena no coincide con lo que se ve. La columna del
eje en el mundo se obtuvo propagando las rotaciones desde `world` con
todas las articulaciones en cero.

| Articulación | Eje en el mundo | Función |
|---|---|---|
| `joint_1` | `+Z` | Giro de cintura |
| `joint_2` | ≈ `−Y` | Inclinación del torso |
| `joint_3` | ≈ `+Z` | Giro de la cabeza |
| `joint_4` | — | Inclinación de la cabeza |
| `joint_5` / `joint_9` | `−X` / `+X` | Hombro, primer eje (en espejo) |
| `joint_6` / `joint_10` | ≈ `±Y` | Hombro, segundo eje |
| `joint_7` / `joint_11` | — | Hombro, tercer eje |
| `joint_8` / `joint_12` | — | Codo |

Las doce declaran el mismo límite en el modelo, `±10 rad` (±573°): en la
práctica ninguna está acotada. Los topes reales habría que medirlos sobre
el robot físico. Las posturas del catálogo se mantienen dentro de ±2.6 rad
por criterio propio, no porque el modelo lo imponga.

Los tres primeros ejes de cada brazo están separados solo 0.037 m y
0.031 m, mientras que el cuarto queda a 0.091 m del anterior: es la
distribución de un hombro de 3 GDL con los ejes casi concurrentes,
seguido del brazo y del codo.

### Posiciones características

Con todas las articulaciones en cero:

| Elemento | Posición |
|---|---|
| Hombro de la rama `joint_5` | `(−0.106, 0.000, 0.255)` m |
| Hombro de la rama `joint_9` | `(0.106, 0.000, 0.255)` m |
| Cuello | `(0.000, −0.012, 0.271)` m |
| Alcance por brazo | ≈ 0.24 m desde el hombro |

## Requisitos

```bash
sudo apt install \
  ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-joint-trajectory-controller ros-jazzy-joint-state-broadcaster \
  ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui
```

## Compilación

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Ejecución

### Visualización del modelo, sin física

```bash
ros2 launch yaren_description display.launch.py
```

Abre RViz con deslizadores por articulación. Es el modo para revisar la
geometría y el árbol TF.

### Simulación dinámica

```bash
ros2 launch yaren_master simulation.launch.py
ros2 launch yaren_master simulation.launch.py rviz:=true
ros2 launch yaren_master simulation.launch.py gui:=false   # solo física
```

### Itinerario automático

```bash
ros2 launch yaren_master itinerario.launch.py
ros2 launch yaren_master itinerario.launch.py secuencia:=saludo
ros2 launch yaren_master itinerario.launch.py secuencia:=verificacion repetir:=true
```

Secuencias disponibles en `yaren_master/config/poses.yaml`:
`demostracion`, `saludo`, `verificacion`.

### Control por servicios

El nodo crea un servicio `std_srvs/Trigger` por cada postura del catálogo,
más `/yaren/home`. Con la simulación en marcha, en otra terminal:

```bash
ros2 run yaren_master pose_service

ros2 service list | grep yaren          # posturas disponibles
ros2 service call /yaren/pose/saludo_a std_srvs/srv/Trigger
ros2 service call /yaren/home std_srvs/srv/Trigger
```

La duración del movimiento es el parámetro `duracion` del nodo:

```bash
ros2 run yaren_master pose_service --ros-args -p duracion:=5.0
```

### Teleoperación por teclado

```bash
ros2 run yaren_master teleop_teclado
```

| Tecla | Acción |
|---|---|
| `[` `]` | articulación anterior / siguiente |
| `w` `s` o flechas | aumentar / disminuir consigna |
| `+` `-` | ajustar el paso |
| `h` | todas las articulaciones a cero |
| `q` | salir |

### Vigilancia del estado

```bash
ros2 run yaren_master estado_node
ros2 topic echo /yaren/estado
```

## Verificación

Ejecutar en orden; cada paso depende del anterior.

```bash
# 1. La plantilla expande y el árbol cinemático es coherente
xacro src/yaren_description/urdf/yaren.urdf.xacro sim:=true > /tmp/yaren.urdf
check_urdf /tmp/yaren.urdf

# 2. Diagrama del árbol TF (genera frames.pdf)
ros2 launch yaren_description display.launch.py   # en una terminal
ros2 run tf2_tools view_frames                    # en otra

# 3. Con la simulación en marcha: controladores activos
ros2 control list_controllers      # ambos deben figurar como 'active'
ros2 topic echo /joint_states      # 12 articulaciones publicando

# 4. Comando manual directo al controlador
ros2 topic pub --once /yaren_jtc/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6,
                  joint_7, joint_8, joint_9, joint_10, joint_11, joint_12],
    points: [{positions: [0.5, 0, 0, 0, -0.8, 0, 0, 0, 0.8, 0, 0, 0],
              time_from_start: {sec: 3}}]}"
```

### Lo que confirma el paso 2

Dos cosas no pueden deducirse leyendo el modelo y solo se resuelven
viéndolo. Conviene comprobarlas antes de dar por buena la simulación:

| Incógnita | Cómo comprobarla | Corrección si falla |
|---|---|---|
| Hacia dónde mira el robot (`+Y` o `−Y`) | Orientación de la cabeza en RViz | Invertir el signo de `y` de la mesa y de las tres piezas en `laboratorio.sdf` |
| Si el codo es `joint_8` o `joint_7` | Mover cada uno con el deslizador y ver dónde flexiona el brazo | Ajustar los comentarios del URDF y la tabla de arriba |

### Diagnóstico rápido

| Síntoma | Causa habitual |
|---|---|
| El robot aparece invisible en el simulador | `GZ_SIM_RESOURCE_PATH` no incluye el directorio que contiene `yaren_description` |
| `ros2 control list_controllers` no devuelve nada | El puente de `/clock` no está activo: el gestor de controladores no avanza |
| El modelo cae o se deforma al arrancar | Algún eslabón sin masa o con inercia incoherente |
| Una postura no se alcanza del todo | El controlador no llegó a completar la trayectoria; revisar el aviso del nodo que la ordenó |
| Las manos no llegan a las piezas de la mesa | La mesa está fuera del alcance de 0.24 m, o el robot mira al lado contrario |

## Ejecución en máquina virtual

Gazebo Harmonic renderiza con Ogre2, que degrada mucho sin aceleración
gráfica:

```bash
export LIBGL_ALWAYS_SOFTWARE=1
```

Si aun así el rendimiento es bajo, ejecutar la simulación sin ventana
(`gui:=false`) y observar el robot desde RViz, que es mucho más ligero:

```bash
ros2 launch yaren_master simulation.launch.py gui:=false rviz:=true
```
