# Robot Ackermann · Grupo 5

Hicimos el robot en Fusion 360 y lo probamos con ROS 2 Jazzy y Gazebo
Harmonic en una VM con Linux Mint. La trayectoria actual dibuja AP con
Pure Pursuit. Los puntos verdes marcan las letras y los grises las maniobras
entre trazos.

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

El robot se detiene al terminar la P. La línea `AP complete` muestra el RMSE
de seguimiento, el error máximo y la distancia al punto final, calculados
con la odometría de ruedas. Para ver la odometría en otra terminal,
carga Jazzy y ejecuta `ros2 topic echo /g5/odom --once`.

## Archivos

- `src/ackermann_g5/scripts/follow_ap.py` contiene el controlador.
- `src/ackermann_g5/urdf/` contiene el modelo y los parámetros del robot.
- `src/ackermann_g5/config/ap_path.csv` contiene los puntos de la trayectoria.
- `src/ackermann_g5/worlds/arena.sdf` contiene la pista.
