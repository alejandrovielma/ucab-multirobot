# Tareas pendientes del equipo

Cada tarea es acotada y tiene un criterio de aceptación verificable. Antes de dar una tarea por
terminada, `powershell -File scripts/test_in_docker.ps1` debe seguir pasando.

## 1. Comportamiento de transporte y entrega (`CARRY`) — implementado

Estado: implementado en la rama `feat/carry-delivery` (ver [odd/tasks/carry-delivery.md](../odd/tasks/carry-delivery.md)).
Se eligió la reversa acotada bajo la llave del cuadro. Quedan pendientes: medir el cuadro real y
ajustar `zone_*` en los YAML, y validar la entrada al cuadro con la cámara cenital real.
Criterios originales:

Dónde: `BehaviorFSM._carry` en [src/mobile_base/mobile_base/core/behavior.py](../src/mobile_base/mobile_base/core/behavior.py)
y el nodo `behavior_node.py` (clientes de servicio).

Criterios de aceptación:
- Localiza el cuadrado de entrega (ArUco id 10 a su lado) con la pose cenital y/o la cámara.
- Llama a `/zone/request` y no entra al cuadrado hasta recibir `granted=true`; mientras se le niega,
  espera a distancia segura.
- Entra al cuadrado por el lado norte y empuja el cubo dentro.
- Retrocede **solo** cuando el cubo ya está dentro del cuadrado.
- Llama a `/zone/release` con `delivered=true` y el color del cubo, publica `delivered` en `state` con
  el `delivered_total` devuelto y vuelve a `EXPLORE`.
- `vx` nunca es negativo mientras `carrying` es verdadero (ya hay un test que lo comprueba; debe seguir pasando).
- Con cubo, el robot gira solo en arco (avanzando) y nunca en el sitio ni de lado: en Webots, un giro en el
  sitio a velocidad 0.36 expulsó el cubo de la pala fija a los ~45°. `safety_gate` ya lo impone
  (`carry_min_forward`, `carry_max_turn`), pero la trayectoria de `CARRY` debe planearse con giros amplios.
- Tests unitarios nuevos para cada transición, con el tiempo inyectado como en los tests existentes.
- **Conflicto a resolver:** desde el commit `493c167`, `safety_gate` elimina todo `vx` negativo (solo hay
  un sensor de línea delantero y retroceder es ciego al borde). Para soltar el cubo hay que retroceder,
  así que la entrega necesita una excepción acotada en `safety_gate`: permitir reversa lenta y corta
  (por ejemplo 0,03 m/s durante ≤ 1 s) **solo** mientras el robot tiene la llave del cuadro
  (`/zone/occupant` = su nombre) y `carrying` acaba de pasar a falso, con un test que lo cubra.
  Alternativa sin reversa: salir del cuadro girando en arco hacia adelante, si el cuadro y la
  pala lo permiten sin arrastrar el cubo.

## 2. Validar los signos de las ruedas en Webots (prueba de desplazamiento lateral)

Dónde: `<wheel_signs>` en [src/multirobot_sim/resource/phineas.urdf](../src/multirobot_sim/resource/phineas.urdf) y `ferb.urdf`.

Criterios de aceptación:
- Con `ros2 launch bringup sim.launch.py logic:=false` (solo Webots y los plugins, sin `safety_gate`
  que pise el comando), `ros2 topic pub -r 10 /phineas/cmd_vel geometry_msgs/msg/Twist "{linear: {y: 0.1}}"`
  hace que Phineas se desplace hacia **su izquierda** sin girar; lo mismo con `linear.x` (avanza) y
  `angular.z` (gira antihorario).
- Si alguna rueda gira al revés, se corrige solo `wheel_signs` (no la cinemática) y se documenta el valor final.
- Estado (2026-10-12): con el control por teclado, el desplazamiento lateral funciona sin girar; falta repetir
  la prueba por ROS 2 con `cmd_vel`.

## 3. Calibrar rangos HSV y `line_threshold`

Dónde: `hue_ranges`, `s_min`, `v_min` en `config/phineas.yaml` y `config/ferb.yaml`; `line_threshold` en los URDF.

Criterios de aceptación:
- Con fotos de los cubos reales bajo la luz del salón, `detect_cubes` clasifica bien los 6 colores en
  al menos 20 imágenes por color; las imágenes y un test de regresión quedan en `mobile_base/test/`.
- El rojo y el morado no se confunden (sus tonos están cerca en el círculo HSV).
- En Webots, `line_front` es verdadero solo sobre la cinta negra.

## 4. Captura de la cámara real en la Pi 5 con Ubuntu

Dónde: [src/mobile_base/mobile_base/nodes/camera_node.py](../src/mobile_base/mobile_base/nodes/camera_node.py).

Criterios de aceptación:
- La IMX219 por CSI publica `camera/image_raw` a 640×480 `bgr8` y al menos 10 Hz, con marca de tiempo correcta.
- Se documenta la vía elegida (pipeline GStreamer con libcamera, picamera2 o `camera_ros`) y cómo instalarla.

## 5. Localizador ArUco cenital y calibración de la homografía

Dónde: [src/overhead_localization](../src/overhead_localization).

Criterios de aceptación:
- `_grab_frame()` lee la cámara cenital real.
- La homografía se calibra con al menos 4 puntos conocidos de la mesa (`fit_homography`) y se guarda
  en `config/overhead.yaml`.
- Con un robot quieto en 5 posiciones medidas a mano, el error de posición es menor a 2 cm y el de
  rumbo menor a 5°.
- Publica `/phineas/pose` y `/ferb/pose` a 10 Hz o más.

## 6. Prueba en banco del HAT con `lgpio`

Dónde: [src/mobile_base/mobile_base/nodes/hat_motor_driver_node.py](../src/mobile_base/mobile_base/nodes/hat_motor_driver_node.py).

Criterios de aceptación:
- Con el robot levantado (ruedas en el aire), cada motor gira en el sentido esperado al publicar
  avance, lateral y giro en `cmd_vel`; si no, se corrige `wheel_signs` o los pines en el YAML.
- Se confirma el número de `gpio_chip` correcto con `gpioinfo` en Ubuntu 24.04 sobre la Pi 5.
- Si se deja de publicar `cmd_vel`, los motores se detienen en menos de 300 ms.
- Se mide que el voltaje medio en bornes del motor no supera 6 V con la batería cargada.

## 7. Campo de visión de la cámara de Webots (IMX219 de 120°)

Dónde: `Camera "camara"` en [src/multirobot_sim/protos/RecolectorUCAB.proto](../src/multirobot_sim/protos/RecolectorUCAB.proto).

Criterios de aceptación:
- `fieldOfView` pasa de 0,94 rad (OV5647) a unos 2,09 rad (120°), con la altura e inclinación medidas
  en el robot real.
- Se recalibra `capture_cy` del comportamiento con la nueva cámara.

## 8. Marcadores ArUco laterales en los robots

Criterios de aceptación:
- Se decide si hacen falta marcadores laterales (además del superior) para que la cámara cenital o la
  del otro robot los detecten de lado; si se agregan, se usan ids distintos y se documentan aquí y en
  el proto de Webots.
