# Robot Ackermann · Grupo 5

El robot Ackermann del Grupo 5 sigue el contorno de un paraguas en Gazebo
Harmonic con ROS 2 Jazzy. El recorrido es cerrado, con dos tramos largos y rectos debajo de la cúpula.
Los puntos verdes marcan la referencia. Lo controlamos con Pure Pursuit.

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
empieza cuando aparece `Gazebo pose ready`.

```bash
ros2 launch ackermann_g5 simulation.launch.py start_delay:=30.0
```

El robot se detiene al volver al punto de partida. La línea `Umbrella complete` muestra el RMSE
de seguimiento, el error máximo y la distancia al punto final, calculados
con la pose del robot en Gazebo. El nodo se detiene si se desvía más
de 0,30 m. La odometría de ruedas sigue disponible en `/g5/odom`
para compararla con `/g5/ground_truth`.

## Archivos

- `src/ackermann_g5/scripts/follow_path.py` contiene el controlador.
- `src/ackermann_g5/urdf/` contiene el modelo y los parámetros del robot.
- `src/ackermann_g5/config/umbrella_path.csv` contiene los puntos de la trayectoria.
- `src/ackermann_g5/worlds/arena.sdf` contiene la pista.
