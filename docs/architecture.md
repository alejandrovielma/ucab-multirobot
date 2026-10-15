# Arquitectura

## Flujo de datos

Cada robot vive en su espacio de nombres (`/phineas` o `/ferb`). La lógica produce un comando
**deseado** (`cmd_vel_nav`); el `safety_gate` es el único que publica el comando que llega a los
motores (`cmd_vel`). Así ninguna lógica nueva (por ejemplo la entrega) puede saltarse las reglas de
seguridad.

```mermaid
flowchart LR
  subgraph HW["Capa de hardware (Webots plugin o nodos reales)"]
    CAM[camera / camara]
    LINE[line_sensor / linea_frente]
    MOT[hat_motor_driver / motores Webots]
  end
  subgraph LOGIC["Lógica por robot (igual en sim y real)"]
    DET[cube_detector]
    BEH[behavior]
    GATE[safety_gate]
  end
  subgraph PERRY["Pi de Perry"]
    LOC[aruco_localizer<br/>o GPS+IMU en sim]
    ARB[zone_arbiter]
  end
  CAM -- camera/image_raw --> DET
  DET -- cubes --> BEH
  LINE -- line_front --> BEH
  LINE -- line_front --> GATE
  LOC -- pose propia y del otro robot --> BEH
  LOC -- pose propia y del otro robot --> GATE
  ARB -- /zone/occupant 2 Hz --> GATE
  ARB -- /zone/occupant --> BEH
  BEH -- /zone/request, /zone/release --> ARB
  BEH -- cmd_vel_nav --> GATE
  BEH -- state --> GATE
  GATE -- cmd_vel --> MOT
```

## Contrato de tópicos

Espacio de nombres `/<robot>`, con robot en {phineas, ferb}:

| Tópico | Tipo | Productor (sim / real) | Consumidor |
|---|---|---|---|
| `camera/image_raw` | sensor_msgs/Image | plugin de Webots / camera_node | cube_detector |
| `line_front` | std_msgs/Bool (true = línea negra) | plugin de Webots / line_sensor_node | safety_gate, behavior |
| `pose` | multirobot_interfaces/RobotPose | plugin de Webots (GPS+InertialUnit, emula la cámara cenital) / overhead_localization | behavior, safety_gate de ambos robots |
| `cmd_vel` | geometry_msgs/Twist | safety_gate | plugin de Webots / hat_motor_driver |
| `cmd_vel_nav` | geometry_msgs/Twist | behavior | safety_gate |
| `cubes` | multirobot_interfaces/CubeDetectionArray (cubos, contenido de la pala y cuadro azul `square_*`) | cube_detector | behavior |
| `state` | multirobot_interfaces/RobotState | behavior | coordinador, el otro robot (y safety_gate para saber si carga un cubo) |
| `/zone/occupant` | std_msgs/String (latido 2 Hz, transient local; vacío = libre) | coordinador | behavior, safety_gate |
| `/zone/request`, `/zone/release` | servicios | coordinador | behavior |

Convenciones: `vx` hacia adelante, `vy` positivo a la izquierda, `wz` positivo antihorario; ruedas en
orden FL, FR, RL, RR. El marco de la mesa es el mismo en la pista real y en Webots: origen en el centro
del rectángulo de cinta, x hacia el este a lo largo del lado de 120,6 cm, y hacia el norte (el cuadro
azul sobresale del lado oeste). Medidas en [Pista y zona de entrega](#pista-y-zona-de-entrega).

## Qué corre dónde

| Máquina | Nodos | Launch |
|---|---|---|
| Pi 5 de Phineas | hat_motor_driver, line_sensor, camera, cube_detector, behavior, safety_gate (`/phineas`) | `robot.launch.py robot:=phineas` |
| Pi 5 de Ferb | los mismos nodos en `/ferb` | `robot.launch.py robot:=ferb` |
| Pi de Perry | zone_arbiter (coordinador), aruco_localizer (cámara cenital) | `perry.launch.py` |
| PC (simulación) | Webots + 2 plugins + lógica de ambos robots + coordinador | `sim.launch.py` |

## Simulación frente a robot real

Solo cambia la capa de hardware; `robot_logic.launch.py` se incluye igual en los dos casos.

| Función | Simulación | Robot real |
|---|---|---|
| Motores | `RecolectorDriver` usa `mobile_base.core.kinematics` y fija velocidades en `motor_*` (máx. 10,5 rad/s) | `hat_motor_driver`: misma cinemática, tope de duty 6 V / V_bat (0,71 con 8,4 V), *watchdog* de 300 ms. Backend `fusion_hat` (por defecto): potencia en % por los puertos M0-M3 de la Fusion HAT+; backend `cokoino_drv8833`: PWM por `lgpio` en los pines IN del HAT anterior |
| Sensor de línea | `DistanceSensor "linea_frente"`, negro si el valor < `line_threshold` | TCRT5000 digital en GPIO5 |
| Cámara | `Camera "camara"` publicada en `bgra8` | `camera_node` (OpenCV; pendiente validar CSI en Ubuntu Server 26.04) |
| Pose | GPS + InertialUnit con ruido gaussiano opcional | ArUco cenital (DICT_4X4_50, id 1 Phineas, id 2 Ferb) |
| Pala | `servo_pala` fijo en 0 | pala impresa fija, sin servo |

## Reglas de seguridad (`safety_gate`)

En este orden, en cada ciclo de 20 Hz:

1. Sin latido de `/zone/occupant` durante más de 2 s: parada total.
2. Pose propia con más de 0,5 s de antigüedad: parada total.
3. Solo traslación hacia adelante: se eliminan `vy` y `vx` negativo (salvo la excepción de entrega, abajo). El robot tiene un único sensor
   de línea delantero, así que desplazarse de lado o retroceder es ciego al borde (en Webots, el
   esquive lateral sacó a los dos robots del tablero). Girar en el sitio sí está permitido.
4. Línea negra al frente: se anula el avance; girar sigue permitido (salvo la excepción de entrega).
5. Otro robot a menos de 60° a cada lado del frente (cono de 120° en total): frena linealmente entre 0,60 m y 0,40 m, se detiene
   por debajo de 0,40 m (las palas se tocan a unos 0,38 m entre centros) y gira alejándose de él.
   Si los dos quedan bloqueados de frente más de 3 s, el robot **sin prioridad** (Ferb) gira 1,5 s
   para salir; si lleva un cubo se queda quieto, porque cualquier maniobra lo sacaría de la pala.
6. Con cubo en la pala: solo giros en arco (avance mínimo 0,05 m/s, giro máximo 0,6 rad/s). Girar en
   el sitio expulsa el cubo de la pala fija (comprobado en Webots). Dos excepciones: con la línea al
   frente manda la línea (no avanza y puede girar en el sitio aunque el cubo se caiga; en Webots, salir
   de la línea en arco con el cubo sacó al robot del tablero), y con otro robot al costado (a 25° o más
   del frente y a 0,30 m o más) se aleja en arco en vez de quedarse congelado. Al llegar a la línea con
   cubo, `behavior` da el transporte por perdido, devuelve la llave si la tenía y vuelve a explorar.

**Excepción de entrega.** Mientras el robot tiene la llave del cuadro (`/zone/occupant` igual a su
nombre) y su propia pose cenital está dentro del pasillo de entrega mirando hacia el cuadro (al oeste
en la pista real, ±60°), puede cruzar con la pala la línea del cuadro y, cuando ya no lleva el cubo,
retroceder a 0,05 m/s como máximo.
El filtro lo decide con entradas que lee él mismo (pose y llave), no con lo que diga `behavior`, así
que un error en el comportamiento no habilita la reversa en otra parte de la pista. El pasillo y el
cuadro están en `zone_*` de los YAML; un test exige que coincidan entre `behavior` y `safety_gate`.

## Verificación de la captura con la cámara

`cube_detector` publica, junto con los cubos, `scoop_color` y `scoop_fill`: la fracción de una ventana
fija entre las paredes de la pala (x ±0,2, y 0,74 a 0,84 de la imagen) que tiene un color de cubo.
Un cubo dentro de la U se ve desde arriba como una franja ancha y baja que el detector de cubos
descarta por forma, así que no sirve buscarlo entre las detecciones. En Webots, con la cámara
IMX219-120 (FOV horizontal 1,60 rad), la ventana mide 0,60 con un cubo dentro y 0,04 con el cubo en
la boca de la pala; el umbral es 0,30. Lo usa `behavior`:

- `CAPTURE` pasa a `CARRY` solo si ve un cubo propio en la pala; si no, vuelve a `APPROACH`.
- Con cubo, si deja de verlo más de 1 s (`scoop_lost_timeout`), abandona el transporte y devuelve la llave.
- Al terminar de retroceder, la entrega cuenta (`delivered=true`) solo si la pala quedó vacía.

El mundo [calibracion_pala.wbt](../src/multirobot_sim/worlds/calibracion_pala.wbt) deja un cubo dentro
de la U de Phineas y otro en la boca de la de Ferb para volver a medir la ventana; las imágenes medidas
están en `mobile_base/test/fixtures/` como prueba de regresión.

## Pista y zona de entrega

Pista real medida el 2026-10-15 (cinta negra de 1,8 cm): rectángulo de 120,6 x 74 cm por el borde
exterior, es decir, borde interior en x ±0,585 m e y ±0,352 m. El cuadro de entrega es **azul sólido**,
de 20,3 x 20,3 cm por fuera, y sobresale del lado oeste a 26 cm de la esquina norte y a 27,8 cm de la
esquina sur; su lado hacia la pista no tiene cinta. Interior azul: x −0,788 a −0,585, y −0,075 a 0,092
(se supone que el azul llega hasta la línea interior de la cinta del rectángulo); centro (−0,6865, 0,0085).
Solo la pala puede entrar al cuadro (Perry trabaja ahí), así que el polígono de la cerca virtual
([track.yaml](../src/mobile_base/config/track.yaml)) es el rectángulo interior **sin** el cuadro.

La zona está generalizada en [core/delivery.py](../src/mobile_base/mobile_base/core/delivery.py) para
cualquier rumbo de llegada. Se define con el centro del cuadro, el rumbo con el que el robot entra
(`zone_heading`, π = oeste en la pista real) y distancias medidas **a lo largo del eje de llegada** desde
la *frontera* (la línea que el cuerpo no puede cruzar, x = −0,585 m, a `zone_square_depth / 2` del
centro). Los puntos del robot son de su centro y se dan como distancia antes de la frontera:

| Parámetro | Valor | Punto en la pista real | Por qué |
|---|---|---|---|
| `zone_drop_offset` | 0,098 m | x −0,487 | el frente del cuerpo (0,095 m) queda 3 mm antes de la frontera; el cubo de 2 cm, contra el fondo de la U (0,1395 m delante del centro), queda en x −0,6265, 4 cm dentro del azul |
| `zone_entry_offset` | 0,298 m | x −0,287 | 0,20 m antes del punto de soltar: ahí empieza `DELIVER` con la llave |
| `zone_exit_offset` | 0,205 m | x −0,38 | al retroceder hasta ahí, la punta de la pala (0,20 m delante del centro) queda al este del cubo |
| `zone_corridor_half_width` | 0,08 m | | el interior azul mide 16,7 cm a lo ancho del eje |

`DeliveryZone` da los puntos del mundo (`entry_point`, `drop_point`, `exit_point`), las coordenadas a lo
largo y a lo ancho del eje de una pose (`along_across`), `in_corridor(x, y)` y `aligned(theta)`. Tanto
`behavior` como `safety_gate` construyen la zona desde los mismos `zone_*` (un test exige que sean
iguales en las dos secciones del YAML y en los dos robots). Los puntos de espera sin llave son
(0, 0,22) para Phineas y (0, −0,22) para Ferb: lejos del pasillo y con espacio para girar en el sitio.

## Entrega del cubo

```mermaid
stateDiagram-v2
    CAPTURE --> CARRY: cubo en la pala
    CARRY --> CARRY: pide /zone/request cada 1 s; sin llave va a su punto de espera, con llave al de entrada
    CARRY --> DELIVER: con la llave, en el punto de entrada y mirando al cuadro
    DELIVER --> CARRY: si se sale del pasillo antes de soltar el cubo
    DELIVER --> DELIVER: avanza por el eje hasta zone_drop_offset (cubo dentro); centra el cuadro azul si la cámara lo ve
    DELIVER --> DELIVER: retrocede hasta zone_exit_offset (la U abierta deja el cubo)
    DELIVER --> EXPLORE: /zone/release delivered=true con el color
```

El robot entra mirando al cuadro (al oeste) y se detiene cuando su centro llega al punto de soltar.
Si el otro robot tiene la llave, espera quieto en su propio punto de espera, lejos de la entrada: en
Webots, esperar en la entrada bloqueó al robot que tenía la llave hasta que esta caducó. El nodo
`behavior` convierte las acciones de la máquina de estados en llamadas asíncronas a los servicios y
toma el conteo oficial (`delivered_total`) de la respuesta de `/zone/release`.

### Detección del cuadro azul

El cuadro es del mismo azul que los cubos de Ferb. `vision.detect_square` toma la región más grande del
rango de tono azul (`square_color`) y la acepta como cuadro si supera `square_min_area` (12000 px por
defecto, varias veces un cubo de 2 cm en la boca de la pala; calibrar con la cámara real).
`detect_cubes` descarta las manchas azules que son el cuadro (de ese tamaño o que se solapan con su
región), así que el cuadro nunca se informa como cubo. `cube_detector` publica `square_visible`,
`square_cx`, `square_cy` y `square_area` en el mismo mensaje `cubes`. Mientras `DELIVER` avanza (no al
retroceder) y el cuadro es visible, `behavior` suma `−square_align_gain · square_cx` (0,8) al giro,
acotado por `carry_max_turn`: corrige el error lateral de la pose cenital con lo que ve el robot.

## Motores: Fusion HAT+

Los robots usan la SunFounder Fusion HAT+. Su microcontrolador (I2C 0x17) genera el PWM de los cuatro
puertos de motor M0-M3; la biblioteca `fusion_hat` los maneja con `Motor('M0').power(-100..100)` (el
ejemplo del fabricante usa `set_power`; se admiten las dos) y `set_is_reverse`. La lógica pura está en
[core/fusion_hat.py](../src/mobile_base/mobile_base/core/fusion_hat.py): rueda → puerto e inversión
(`fusion_ports`, `fusion_reversed`, orden fl, fr, rl, rr; por defecto M0-M3 sin invertir porque el
cableado aún no se identificó) y duty → porcentaje (`duty · 100`, nunca más que el tope ni ±100).
La batería 2S llega a 8,4 V cargada y los motores TT son de 6 V, así que el tope de duty sigue siendo
6,0 / 8,4 = 0,71. Si la biblioteca no está instalada, el nodo registra un error y funciona en vacío.
El backend `cokoino_drv8833` (HAT anterior por `lgpio`) se conserva con `motor_backend`.

Pines que usa la Fusion HAT+: GPIO 2/3 (I2C), 18-21 (audio), 17/4/27/22 (sus IO) y 26 (detección de
apagado). El sensor de línea sigue en GPIO5, que la HAT no usa.

## Pines del HAT Cokoino anterior (BCM, backend `cokoino_drv8833`)

| Señal | Pin | Motor |
|---|---|---|
| nSLEEP | 12, 13 (en alto) | ambos DRV8833 |
| DRV#1 AIN1 / AIN2 | 17 / 27 | M1 delantero izquierdo |
| DRV#1 BIN1 / BIN2 | 22 / 23 | M2 trasero izquierdo |
| DRV#2 AIN1 / AIN2 | 24 / 25 | M3 delantero derecho |
| DRV#2 BIN1 / BIN2 | 26 / 16 | M4 trasero derecho |

Decaimiento rápido: PWM en xIN1 con xIN2 en bajo = adelante; al revés para retroceder.
