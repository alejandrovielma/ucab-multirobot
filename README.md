# Robotica Phineas: robots recolectores Phineas y Ferb (ROS 2 Jazzy)

Monorepo ROS 2 Jazzy de los dos robots recolectores idénticos del equipo UCAB Robótica.
Phineas recoge cubos rojos, naranjas y amarillos; Ferb recoge verdes, azules y morados.
Ambos ejecutan **el mismo código**: solo cambia el archivo de parámetros (`config/<robot>.yaml`) y el
espacio de nombres (`/phineas` o `/ferb`).

La idea central es que los nodos de decisión (visión, comportamiento, seguridad) sean idénticos en
Webots y en el robot real; lo único que se intercambia es la capa de hardware: en simulación la
reemplaza un plugin de `webots_ros2`, en el robot real la reemplazan los nodos que hablan con el HAT,
el sensor de línea y la cámara. Detalle completo en [docs/architecture.md](docs/architecture.md).

## Estado

| Parte | Estado |
|---|---|
| Interfaces, lógica pura (cinemática, seguridad, colores, FSM, arbitraje de zona) | Implementado y con tests |
| Nodos ROS 2 y launch del robot real | Implementado; arrancan sin hardware (modo *dry run*) |
| Paquete de Webots (mundo, plugin, URDF, launch) | Implementado; **no probado en Webots** (no había Webots en el entorno de desarrollo) |
| Transporte y entrega del cubo (estado `CARRY`) | Pendiente del equipo |
| Cámara real en Pi 5, localizador cenital real, prueba del HAT | Pendiente del equipo |

La lista de tareas pendientes con criterios de aceptación está en [docs/team-todo.md](docs/team-todo.md).

## Mapa de paquetes

| Paquete | Tipo | Contenido |
|---|---|---|
| [`multirobot_interfaces`](src/multirobot_interfaces) | ament_cmake | `RobotPose`, `RobotState`, `CubeDetection(Array)`, servicios `RequestZone` y `ReleaseZone` |
| [`mobile_base`](src/mobile_base) | ament_python | `core/` lógica pura sin ROS; `vision.py`; nodos `behavior`, `safety_gate`, `cube_detector`, `hat_motor_driver`, `line_sensor`, `camera`; `config/phineas.yaml`, `config/ferb.yaml` |
| [`coordinator`](src/coordinator) | ament_python | `zone_arbiter`: préstamo exclusivo (*lease*) del cuadrado de entrega y conteo oficial de cubos |
| [`overhead_localization`](src/overhead_localization) | ament_python | Esqueleto del localizador ArUco cenital y homografía píxel → mesa (con tests) |
| [`multirobot_sim`](src/multirobot_sim) | ament_python | Mundo `pista_ucab.wbt`, protos, plugin `RecolectorDriver`, URDF por robot, controlador de teclado |
| [`bringup`](src/bringup) | ament_python | `sim.launch.py`, `robot.launch.py`, `robot_logic.launch.py`, `perry.launch.py` |

El módulo `mobile_base/core/` no importa `rclpy`, OpenCV ni `lgpio`: es lógica pura con el tiempo
pasado como argumento, por eso se prueba con `pytest` sin ROS corriendo.

## Ejecutar los tests (Windows + Docker)

Requisito: Docker Desktop encendido. No hace falta ROS 2 en Windows.

```powershell
powershell -File scripts/test_in_docker.ps1          # construye la imagen y corre todo
powershell -File scripts/test_in_docker.ps1 -NoBuild # reutiliza la imagen ya construida
```

El script ([scripts/test_in_docker.sh](scripts/test_in_docker.sh)) copia `src/` a un workspace dentro
del contenedor (para no escribir `build/` ni `install/` en el disco de Windows) y ejecuta:

1. `colcon build --symlink-install` de los 6 paquetes.
2. `colcon test` y `colcon test-result --verbose` (falla si algún test falla).
3. Importación de cada módulo Python (los de `multirobot_sim` y `sim.launch.py` solo se compilan,
   porque `webots_ros2_driver` no está instalado en la imagen).
4. `ros2 launch bringup robot.launch.py` para Phineas y Ferb durante unos segundos, sin hardware:
   falla si algún nodo se cae al arrancar.

Desde Linux o WSL: `docker run --rm -v "$PWD:/ws:ro" -w /ws ros:jazzy-ros-base bash ./scripts/test_in_docker.sh`
(instala OpenCV con apt si falta).

## Ejecutar la simulación (Webots en Windows + ROS 2 en WSL2)

Configuración prevista; **no verificada todavía en una máquina del equipo**.

1. Instalar [Webots R2025a](https://cyberbotics.com) en Windows.
2. Instalar WSL2 con Ubuntu 24.04 y ROS 2 Jazzy dentro de WSL
   ([guía oficial de ROS 2 para Webots en Windows](https://docs.ros.org/en/jazzy/Tutorials/Advanced/Simulators/Webots/Installation-Windows.html)).
3. En WSL:

   ```bash
   sudo apt install ros-jazzy-webots-ros2 python3-opencv
   export WEBOTS_HOME=/mnt/c/Program\ Files/Webots
   cd ~/ws && git clone <este repo> src/robotica-phineas   # o enlazar la carpeta del repo
   source /opt/ros/jazzy/setup.bash
   colcon build --symlink-install
   source install/setup.bash
   ros2 launch bringup sim.launch.py
   ```

`sim.launch.py` abre el mundo, conecta los dos robots (`controller "<extern>"`) con el plugin
`multirobot_sim.recolector_driver.RecolectorDriver`, arranca la lógica de cada robot y el
coordinador. Al cerrar Webots se cierra todo el launch. Con `logic:=false` solo arrancan Webots y
los plugins, útil para mandar `cmd_vel` a mano.

Problema conocido: en WSL2 a veces los controladores no logran conectarse con Webots en Windows; ver
[cyberbotics/webots_ros2#1031](https://github.com/cyberbotics/webots_ros2/issues/1031).

Para mover un robot a mano sin ROS, cambiar su campo `controller` a `"recolector_teleop"` en el
mundo (las teclas están en la cabecera de
[recolector_teleop.py](src/multirobot_sim/controllers/recolector_teleop/recolector_teleop.py)).

![Vista cenital del mundo](docs/img/vista_cenital.png)

## Ejecutar en el robot real

En la Pi 5 de cada robot (Ubuntu 24.04 + ROS 2 Jazzy + `python3-lgpio`):

```bash
ros2 launch bringup robot.launch.py robot:=phineas   # o robot:=ferb
```

En la Pi de Perry (coordinador y cámara cenital):

```bash
ros2 launch bringup perry.launch.py
```

Las tres Pi deben compartir el mismo `ROS_DOMAIN_ID` en la misma red y tener los relojes sincronizados
(chrony/NTP): el `safety_gate` descarta poses con más de 0,5 s de antigüedad usando la marca de tiempo
del localizador.

Por seguridad el robot **no se mueve** si no recibe su pose cenital o el latido de `/zone/occupant`.
Para una prueba en banco sin cámara cenital ni coordinador:

```bash
ros2 launch bringup robot.launch.py robot:=phineas require_pose:=false require_heartbeat:=false
```
