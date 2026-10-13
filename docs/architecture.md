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
| `cubes` | multirobot_interfaces/CubeDetectionArray | cube_detector | behavior |
| `state` | multirobot_interfaces/RobotState | behavior | coordinador, el otro robot (y safety_gate para saber si carga un cubo) |
| `/zone/occupant` | std_msgs/String (latido 2 Hz, transient local; vacío = libre) | coordinador | behavior, safety_gate |
| `/zone/request`, `/zone/release` | servicios | coordinador | behavior |

Convenciones: `vx` hacia adelante, `vy` positivo a la izquierda, `wz` positivo antihorario; ruedas en
orden FL, FR, RL, RR. El marco de la mesa en simulación es el marco del mundo de Webots (origen en el
centro del tablero, x a lo largo del lado de 1,93 m).

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
| Motores | `RecolectorDriver` usa `mobile_base.core.kinematics` y fija velocidades en `motor_*` (máx. 10,5 rad/s) | `hat_motor_driver`: misma cinemática, tope de duty 6 V / V_bat, PWM por `lgpio` en los pines IN del HAT, *watchdog* de 300 ms |
| Sensor de línea | `DistanceSensor "linea_frente"`, negro si el valor < `line_threshold` | TCRT5000 digital en GPIO5 |
| Cámara | `Camera "camara"` publicada en `bgra8` | `camera_node` (OpenCV; pendiente validar CSI en Ubuntu) |
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
nombre) y su propia pose cenital está dentro del pasillo de entrega mirando al sur (±60°), puede
cruzar la cinta norte del cuadro y, cuando ya no lleva el cubo, retroceder a 0,05 m/s como máximo.
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

## Entrega del cubo

```mermaid
stateDiagram-v2
    CAPTURE --> CARRY: cubo en la pala
    CARRY --> CARRY: pide /zone/request cada 1 s; sin llave va a su punto de espera, con llave al de entrada
    CARRY --> DELIVER: con la llave, dentro del pasillo y mirando al sur (±60°)
    DELIVER --> CARRY: si se sale del pasillo antes de soltar el cubo
    DELIVER --> DELIVER: avanza al sur cruzando la cinta hasta drop_y (cubo dentro)
    DELIVER --> DELIVER: retrocede hasta exit_y (la U abierta deja el cubo)
    DELIVER --> EXPLORE: /zone/release delivered=true con el color
```

El punto de entrada está al norte del cuadro (`zone_entry_y`); el robot entra mirando al sur y se
detiene cuando su centro llega a `zone_drop_y`, que deja el cubo (0,155 m delante del centro) en el
centro del cuadro. Si el otro robot tiene la llave, espera quieto en su propio punto de espera
(`wait_x`, `wait_y`: Phineas al oeste, Ferb al este), lejos de la entrada: en Webots, esperar en la
entrada bloqueó al robot que tenía la llave hasta que esta caducó. El nodo
`behavior` convierte las acciones de la máquina de estados en llamadas asíncronas a los servicios y
toma el conteo oficial (`delivered_total`) de la respuesta de `/zone/release`.

## Pines del HAT Cokoino (BCM)

| Señal | Pin | Motor |
|---|---|---|
| nSLEEP | 12, 13 (en alto) | ambos DRV8833 |
| DRV#1 AIN1 / AIN2 | 17 / 27 | M1 delantero izquierdo |
| DRV#1 BIN1 / BIN2 | 22 / 23 | M2 trasero izquierdo |
| DRV#2 AIN1 / AIN2 | 24 / 25 | M3 delantero derecho |
| DRV#2 BIN1 / BIN2 | 26 / 16 | M4 trasero derecho |

Decaimiento rápido: PWM en xIN1 con xIN2 en bajo = adelante; al revés para retroceder.
