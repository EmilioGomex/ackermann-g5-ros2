# Robot Ackermann · Grupo 5

Hicimos el robot en Fusion 360 y lo probamos con ROS 2 Jazzy y Gazebo
Harmonic en una VM con Linux Mint. El robot da una vuelta de 0,75 m de radio
con Pure Pursuit. Los puntos verdes en Gazebo marcan el recorrido.

## Instalación

Si ya tienes Jazzy instalado, revisa las dependencias. Para una instalación
desde cero en Ubuntu 24.04, sigue la [guía de ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  python3-colcon-common-extensions
```

## Compilar y correr

Desde la carpeta del repositorio, donde están este README y `src/`:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ackermann_g5
source install/setup.bash
ros2 launch ackermann_g5 simulation.launch.py
```

Para grabar, inicia la simulación con 30 segundos de espera. La cuenta
empieza cuando aparece `Odometry ready`.

```bash
ros2 launch ackermann_g5 simulation.launch.py start_delay:=30.0
```

El robot se detiene después de una vuelta. La línea `Lap complete` muestra
el RMSE radial, el error máximo y la distancia al punto de partida, todos
calculados con la odometría de ruedas. Para ver la odometría en otra terminal,
carga Jazzy y ejecuta `ros2 topic echo /g5/odom --once`.

## LiDAR y SLAM

El modo de mapeo usa un LiDAR 2D virtual y SLAM Toolbox. Instala estos paquetes
si vas a probarlo:

```bash
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-nav2-map-server ros-jazzy-rviz2
ros2 launch ackermann_g5 simulation.launch.py slam:=true
```

En otra terminal con Jazzy cargado, abre RViz con
`rviz2 --ros-args -p use_sim_time:=true`. Selecciona `map` como marco fijo y
agrega `/map` y `/g5/scan`. Para el escaneo usa fiabilidad *Best Effort*.
Para guardar el mapa:

```bash
ros2 run nav2_map_server map_saver_cli -f g5_map --ros-args -p use_sim_time:=true
```

## Archivos

- `src/ackermann_g5/scripts/follow_circle.py` contiene el controlador.
- `src/ackermann_g5/urdf/` contiene el modelo y los parámetros del robot.
- `src/ackermann_g5/worlds/arena.sdf` contiene la pista.
