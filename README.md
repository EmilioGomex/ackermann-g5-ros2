# Proyecto 1 · Robot Ackermann G5

Simulación del modelo de Fusion 360 con ROS 2 Jazzy y Gazebo Harmonic.
El controlador sigue una vuelta antihoraria de radio 0,75 m con *Pure Pursuit*
y odometría de ruedas. El anillo verde marca la referencia en Gazebo.

## Instalar y compilar

Usar Ubuntu 24.04, nativo o en WSL 2 con WSLg para abrir Gazebo y RViz.
En Windows, si WSL no está instalado, ejecutar en PowerShell como
administrador `wsl --install -d Ubuntu-24.04`, reiniciar si Windows lo solicita
y abrir Ubuntu. Configurar el repositorio APT de ROS y completar la
[instalación de ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).

En Ubuntu:

```bash
sudo apt update
sudo apt install ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-xacro \
  ros-jazzy-slam-toolbox ros-jazzy-nav2-map-server python3-colcon-common-extensions
source /opt/ros/jazzy/setup.bash
```

Abrir una terminal en la carpeta que contiene este README y `src/`.
Si el proyecto está en la ubicación original de Windows, esa carpeta es
`/mnt/d/Robots/07_Proyectos/Proyecto1_G5` dentro de WSL. Estos comandos usan
el sistema de archivos de Ubuntu para compilar:

```bash
mkdir -p ~/g5_ws/src
cp -r src/ackermann_g5 ~/g5_ws/src/
cd ~/g5_ws
colcon build --packages-select ackermann_g5
source install/setup.bash
ros2 launch ackermann_g5 simulation.launch.py
```

El robot espera odometría y transcurren cinco segundos simulados antes de
arrancar. Al completar una vuelta, se detiene y el nodo imprime RMSE radial,
error radial máximo y distancia al punto inicial derivados de la odometría.
Para grabar con más tiempo de preparación, usar `start_delay:=15.0`.
Para ejecutar solo el servidor de Gazebo, usar `gui:=false`.
Para arrancar sin el controlador automático, usar `controller:=false`.

## Comprobar y registrar la simulación

Desde otra terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/g5_ws/install/setup.bash
ros2 topic echo /g5/odom --once
ros2 run tf2_ros tf2_echo odom base_link
```

Para registrar datos antes del arranque, lanzar con `start_delay:=15.0` y
ejecutar en la segunda terminal:

```bash
ros2 bag record -o g5_circle /clock /g5/odom /g5/cmd_vel /g5/joint_states
```

Detener la grabación con Ctrl+C después de `Lap complete`. Para el video,
encuadrar el anillo verde desde arriba y grabar desde que empieza a moverse
hasta que se detiene. El recorrido nominal mide 4,71 m y dura alrededor de
30 s más la desaceleración y la espera inicial. Las métricas del log son
estimaciones por odometría. Para medir el error físico de la simulación,
usar pose real del modelo en Gazebo y llevarla al mismo marco que la referencia.

## LiDAR 2D y SLAM

Detener el lanzamiento anterior antes de iniciar esta variante:

```bash
ros2 launch ackermann_g5 simulation.launch.py slam:=true start_delay:=10.0
```

En otra terminal con el entorno cargado:

```bash
ros2 topic echo /g5/scan --once --field header --qos-reliability best_effort
ros2 run tf2_ros tf2_echo map lidar_link
rviz2 --ros-args -p use_sim_time:=true
```

En RViz, elegir `map` como *Fixed Frame*, agregar *Map* con el tópico
`/map` y *LaserScan* con `/g5/scan` y fiabilidad *Best Effort*. Con el
mapa visible, exportarlo desde otra terminal:

```bash
ros2 run nav2_map_server map_saver_cli -f g5_map --ros-args -p use_sim_time:=true
```

El Xacro agrega un LiDAR virtual a 90 mm de altura respecto del origen CAD
del chasis, por encima de la bandera. Para el requisito de cuatro
integrantes, incorporar también el sensor al ensamblaje de Fusion y actualizar
el plano correspondiente.

## Modelo y control

| Parámetro | Valor |
|---|---:|
| Distancia entre ejes | 80 mm |
| Vía trasera entre centros de rueda | 115 mm |
| Vía entre pivotes de dirección | 100 mm |
| Vía delantera entre centros de rueda en recta | 125 mm |
| Radio de rueda | 25 mm |
| Giro máximo de cada articulación delantera | ±35° |
| Velocidad nominal del controlador | 0,16 m/s |
| Radio de referencia | 0,75 m |

El origen de `base_link` se desplazó longitudinalmente hasta el punto medio
del eje trasero para concentrar la referencia cinemática en ese eje.
Se mantienen las mallas originales y se usan colisiones cilíndricas para las
ruedas. La tracción se aplica al eje trasero.

En el modelo bicicleta, ẋ = v cos θ, ẏ = v sin θ y θ̇ = v tan(δ)/L.
Para un punto objetivo (xₜ, yₜ) expresado en el marco del vehículo, el
controlador usa κ = 2yₜ/(xₜ² + yₜ²) y envía ω = vκ al plugin de Gazebo.
El plugin coordina las dos articulaciones de dirección y asigna velocidades
distintas a las ruedas traseras. El nodo espera datos de odometría y envía
velocidad cero si deja de recibirlos durante más de 0,5 s de tiempo simulado.

## Archivos principales

- `src/ackermann_g5/urdf/` y `meshes/` contienen el modelo del G5.
- `src/ackermann_g5/scripts/follow_circle.py` contiene el controlador ROS 2.
- `src/ackermann_g5/launch/simulation.launch.py` inicia Gazebo, el puente, el controlador y opcionalmente SLAM.
- `src/ackermann_g5/worlds/arena.sdf` contiene la arena y la referencia visual.

Implementación basada en la [documentación de Gazebo Harmonic](https://gazebosim.org/docs/harmonic/)
y el [sistema AckermannSteering de Gazebo Sim 8](https://github.com/gazebosim/gz-sim/tree/gz-sim8/src/systems/ackermann_steering).
