# Tareas pendientes del equipo

Cada tarea es acotada y tiene un criterio de aceptación verificable. Antes de dar una tarea por
terminada, `powershell -File scripts/test_in_docker.ps1` debe seguir pasando.

## 1. Comportamiento de transporte y entrega (`CARRY`) — implementado

Estado: implementado en la rama `feat/carry-delivery` (ver [odd/tasks/carry-delivery.md](../odd/tasks/carry-delivery.md)).
Se eligió la reversa acotada bajo la llave del cuadro. El cuadro real (azul, lado oeste) se midió el
2026-10-15 y la zona se generalizó a cualquier rumbo de llegada: el robot entra mirando al oeste
(ver [architecture.md](architecture.md#pista-y-zona-de-entrega)). Queda pendiente validar la entrada
al cuadro con la cámara cenital real y la tarea 18.
Criterios originales (escritos para la pista de Webots anterior, con entrada por el norte):

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

## 9. Verificar que el cubo quedó en la pala (captura real, no por tiempo) — implementado en Webots

Estado: implementado en `feat/capture-verification` con la cámara (ventana entre las paredes de la
pala, ver [architecture.md](architecture.md)). En la corrida 10, 2 entregas contadas = 2 cubos en el
cuadro. Falta medir la ventana con la cámara real montada (depende de la tarea 7).
Criterios originales:

Dónde: `BehaviorFSM._capture` en [src/mobile_base/mobile_base/core/behavior.py](../src/mobile_base/mobile_base/core/behavior.py).

Problema observado (Webots, corrida 7, 2026-10-12): `CAPTURE` empuja 1 s y pasa a `CARRY` sin comprobar
nada. Se registraron 5 entregas y en el cuadro quedó 1 cubo; otro terminó fuera de la pista. El conteo
oficial del árbitro hoy no es confiable.

Criterios de aceptación:
- Con la cámara del robot (sin comprar sensores): definir y medir en Webots dónde aparece un cubo que está
  dentro de la U (zona inferior central de la imagen, entre las paredes de la pala). Guardar cuadros de
  ejemplo con `tools/sim_monitor.py --frames-every`.
- `CAPTURE` pasa a `CARRY` solo si el cubo se ve en esa zona; si no, vuelve a `APPROACH`.
- Durante `CARRY`, si el cubo deja de verse en la pala durante más de N s, se abandona el transporte
  (devuelve la llave con `delivered=false`).
- Corrida de 7 min en Webots: entregas contadas = cubos dentro del cuadro (contarlos en la captura final).
- Alternativa con hardware (si se acepta comprar): un FC-51 en la pala, como en el documento v2.

## 10. Polígono de la pista real para la cerca virtual

Dónde: [src/mobile_base/config/track.yaml](../src/mobile_base/config/track.yaml).

Estado (2026-10-15): el polígono ya es el rectángulo interior medido a mano (x ±0,585, y ±0,352, sin el
cuadro azul). Falta comprobarlo con la cámara cenital calibrada.

Criterios de aceptación:
- Con la cámara cenital calibrada (tarea 5), detectar el borde interior de la cinta, pasarlo por la
  homografía y guardar el polígono en metros. Medir también el cuadro y ajustar `zone_*`.
- `test_track.py` sigue pasando con los puntos de inicio, espera y entrada medidos en la pista real.

## 11. Pendientes menores encontrados al actualizar el documento técnico

- La figura f13 (captura de Webots) todavía muestra las palas naranjas: recapturarla.
- La cabecera de [RecolectorUCAB.proto](../src/multirobot_sim/protos/RecolectorUCAB.proto) aún menciona
  SG90, FC-51 y TCS34725 como parte del diseño.
- La cámara simulada sigue con el campo de visión de la OV5647 (tarea 7).

## 12. Cubos capturados mirando al borde (decisión de reglas)

Problema observado (Webots, corrida 10): Ferb capturó 7 cubos cerca del borde norte y los perdió todos.
Al capturar mira hacia afuera; con cubo solo puede girar en arco (radio ~8 cm) y la cerca virtual frena
el avance antes de completar el giro, así que abandona el cubo.

Opciones (requieren saber qué dice el reglamento sobre pisar o cruzar la cinta):
- Permitir que, con cubo, la pala y el frente pasen unos centímetros sobre la cinta mientras el
  centro del robot siga dentro del polígono con margen (cerca virtual evaluada en el centro).
- No intentar capturar cubos a menos de ~12 cm del borde (con la pose y el polígono): se pierden esos cubos.
- Probar en el robot real si un giro en el sitio muy lento conserva el cubo en la U.

## 13. Re-adaptar el sistema a cubos de 2 x 2 cm y al cuadro azul de la pista real

Cambio de requisito (2026-10-13): los cubos reales ahora miden **2 x 2 cm** (antes 3 cm) y el cuadro
de entrega de la pista real es un **cuadrado azul**.

Hecho (2026-10-15, rama `feat/track-v2-fusion-hat`, ver [odd/tasks/track-v2-fusion-hat.md](../odd/tasks/track-v2-fusion-hat.md)):
- Pista medida: rectángulo de 120,6 x 74 cm con el cuadro azul al oeste; `track.yaml`, la zona de
  entrega (generalizada a cualquier rumbo, entrada hacia el oeste) y los puntos de espera actualizados.
- `vision.detect_square` reconoce el cuadro azul (región azul más grande que `square_min_area`) y
  `detect_cubes` ya no lo informa como cubo; al entrar, el robot se alinea con el cuadro que ve.
- Mundo de Webots regenerado desde `tools/sim/generate_track.py`: tablero de 1,82 x 0,94 m, textura,
  14 cubos de 2 cm, marcador ArUco id 10 al oeste del cuadro y salidas mirando al oeste
  (sin correr Webots todavía).

Falta:

- Correr una calibración en `calibracion_pala.wbt` con el cubo de 2 cm y ajustar el ancho de la U de
  la pala impresa si la holgura de ~10 mm por lado degrada la retención (relacionado con la tarea 12).
- Revisar los umbrales de detección: `min_area`/`max_area` en `vision.py` y en los YAML — un cubo de
  2 cm ocupa menos píxeles a la misma distancia; recalibrar con la cámara real (tareas 3 y 7).
- Calibrar con la cámara real `square_min_area` (12000 px por defecto) y el rango azul: el cuadro
  debe verse como cuadro desde el punto de entrada y un cubo azul en la boca de la pala no debe
  superar ese umbral.
- Correr `pista_ucab.wbt` y comprobar la entrega hacia el oeste (no se ejecutó Webots en este cambio).
- Actualizar la figura f13 (ver tarea 17 para el Documento Técnico).

Criterios de aceptación: una corrida en Webots con cubos de 2 cm entrega ≥2 cubos sin perderlos en el
transporte, y en el laboratorio `lab_robot.py chase` detecta el cubo de 2 cm a ≥20 cm sin detectar el
cuadro azul como cubo.

## 14. Driver de la Fusion HAT+ en Ubuntu y mapa de ruedas a puertos

Dónde: la Pi 5 de cada robot (Ubuntu Server 26.04) y `fusion_ports` / `fusion_reversed` en
`config/phineas.yaml` y `config/ferb.yaml`. Procedimiento en
[guia-pruebas-laboratorio.md](guia-pruebas-laboratorio.md#43-driver-de-la-fusion-hat).

Criterios de aceptación:
- El instalador de SunFounder apunta a Raspberry Pi OS: en Ubuntu se instalan antes los *headers* del
  kernel y, si falla la compilación del driver, se recompila a mano. Queda documentado qué funcionó.
- `i2cdetect -y 1` muestra la HAT en 0x17 y `python3 -c "from fusion_hat.motor import Motor"` no falla.
- `lab_robot.py motors` mueve cada puerto M0-M3; se anota qué rueda mueve cada uno y en qué sentido, y
  se actualizan `fusion_ports` (orden fl, fr, rl, rr) y `fusion_reversed` en los dos YAML.

## 15. Montaje de la cámara: altura, retroceso e inclinación

Dónde: `Camera "camara"` en [RecolectorUCAB.proto](../src/multirobot_sim/protos/RecolectorUCAB.proto) y
la ventana de la pala (`scoop_*`) en los YAML.

Criterios de aceptación:
- Se miden en el robot real la altura de la cámara, cuánto queda detrás del frente y su inclinación.
- La cámara se inclina unos 25-30° hacia abajo para que la pala y el cubo en su boca entren en la
  imagen (hoy mira casi horizontal; Webots supone 24° hacia abajo a 11 cm).
- Se actualizan la cámara de Webots, `capture_cy` y la ventana `scoop_*`, y se repite la medición con
  `calibracion_pala.wbt`.

## 16. Margen blanco del tablero

Dónde: `BOARD_X`, `BOARD_Y` en [tools/sim/generate_track.py](../tools/sim/generate_track.py).

Criterios de aceptación:
- Se mide el tablero real y la distancia de la cinta a sus bordes (hoy se supone 1,82 x 0,94 m).
- Se regenera la textura con `python3 tools/sim/generate_track.py` y se ajustan las medidas del tablero
  y la mesa en los dos mundos; las pruebas de `multirobot_sim` siguen pasando.

## 17. Regenerar el Documento Técnico

El .docx/.pdf de la raíz no se puede editar desde el repositorio: hay que regenerarlo desde su fuente
con el hardware nuevo (Fusion HAT+, batería 2S de 7,4 V y 2000 mAh), la pista medida, los cubos de
2 x 2 cm y la detección del cuadro azul.

## 18. La entrega se detiene antes del punto de soltar — resuelto (2026-10-15)

Solución: `drop_tolerance` = 2 cm en el comportamiento (mayor que los 1,5 cm que el filtro predice a la
velocidad de entrada): a menos de 2 cm del punto de soltar, la suelta empieza. En el peor caso el cubo
queda ~2 cm dentro del azul en lugar de 4. Prueba de integración en `test_delivery.py`
(`test_behavior_and_gate_complete_a_delivery_on_the_real_track`): comportamiento + filtro sobre el
polígono real completan la entrega; con tolerancia 0 la misma prueba se queda atascada en x = -0,477.


Observado en una simulación cinemática (comportamiento + filtro de seguridad, sin Webots), con la
geometría nueva y también con la anterior: el filtro predice 0,3 s de avance y la cerca virtual frena
el cuerpo cuando su frente llega a ~1 cm de la frontera, pero el punto de soltar deja el frente a 3 mm.
El robot queda en `DELIVER` sin llegar a `zone_drop_offset` (se detuvo 1,3 cm antes).

Criterios de aceptación:
- Decidir la solución (por ejemplo, dar por alcanzado el punto de soltar cuando el cuerpo ya no puede
  avanzar y el cubo está dentro del azul, o alejar el punto de soltar unos milímetros) con una prueba
  que reproduzca el bloqueo.
- La prueba cinemática llega a soltar y salir del pasillo.

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

## 6. Prueba en banco de los motores (Fusion HAT+)

Dónde: [src/mobile_base/mobile_base/nodes/hat_motor_driver_node.py](../src/mobile_base/mobile_base/nodes/hat_motor_driver_node.py)
(`motor_backend: fusion_hat`). Depende de la tarea 14.

Criterios de aceptación:
- Con el robot levantado (ruedas en el aire), cada motor gira en el sentido esperado al publicar
  avance, lateral y giro en `cmd_vel`; si no, se corrige `fusion_reversed` (o `wheel_signs`) en el YAML.
- El sensor de línea sigue en GPIO5: se confirma el número de `gpio_chip` con `gpioinfo` en Ubuntu
  Server 26.04 sobre la Pi 5.
- Si se deja de publicar `cmd_vel`, los motores se detienen en menos de 300 ms.
- Se mide que el voltaje medio en bornes del motor no supera 6 V con la batería cargada.

## 7. Campo de visión de la cámara de Webots (IMX219 de 120°) — FOV aplicado, falta medir el montaje

Estado: `fieldOfView` = 1,60 rad (pinhole, calculado con la lente de 1,79 mm y el área activa del
IMX219) y `capture_cy` = 0,62 medido en Webots. Falta medir altura e inclinación reales y repetir la
medición con `calibracion_pala.wbt`; la distorsión del gran angular real no se simula.

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
