# Estado del proyecto y próximos pasos

Actualizado: 2026-10-20, después de la primera jornada de laboratorio con el robot completo.
Detalle de cada tarea y su criterio de aceptación: [team-todo.md](team-todo.md).

## 1. Qué funciona hoy en el robot real (Phineas)

| Parte | Estado | Evidencia |
|---|---|---|
| Pi 5 con Ubuntu Server 26.04 | Funciona | Arranca, SSH, `git pull` desde GitHub |
| Driver de la Fusion HAT+ | **Funciona** | HAT en I2C 0x17, módulo `fusion_hat` carga solo al arrancar, `fusion_hat` 1.14.0 importa. Pasos probados en la [guía 4.3](guia-pruebas-laboratorio.md#43-driver-de-la-fusion-hat) |
| Motores y mapa de ruedas | **Funciona** | `motors --battery 8.4`: cada rueda en su puerto y sentido; avanzar, lateral izquierda y girar antihorario correctos. `fusion_ports: [M0, M1, M2, M3]`, `fusion_reversed` todo `false` (sin cambios) |
| Sensor de línea (GPIO5, `gpiochip0`) | **Funciona** | `line`: `blanco` sobre el tablero, `NEGRO (linea)` sobre la cinta. Montado **en el centro del robot** |
| Cámara IMX219 | Funciona | Imagen en vivo; detecta los cubos |
| Detección de colores y cuadro azul | Parcial | Detecta cubos; faltan los datos por color y distancia, y un error que quedó por reportar |
| Pala, manejo en la pista, persecución | Sin probar | Bloques E, F y G de [lab-hoy.md](lab-hoy.md) |
| ROS 2 en la Pi | Sin instalar | Todo lo de hoy fue con `tools/lab/lab_robot.py`, sin ROS |

Lo que salió del laboratorio y ya está en el código:

- **`gpio_chip: 0`** en los YAML: en el kernel 7.0 los 40 pines de la Pi 5 son `gpiochip0`
  (`pinctrl-rp1`, verificado con `gpiodetect`), no `gpiochip4`.
- **Sensor de línea en el centro:** al ver la cinta, el frente del cuerpo (9,5 cm) ya la pasó. Nuevo
  parámetro `line_backoff: 0.14` en `phineas.yaml`: el robot retrocede 14 cm y solo después gira.
  Funciona sin cámara cenital (`blind_reverse: true` deja pasar ese retroceso por el filtro de
  seguridad, nunca con un cubo suelto en la pala).

## 2. Próximos pasos, en orden

1. **Integrar el PR del sensor centrado** (`feat/centered-line-sensor`) antes del próximo laboratorio.
2. **Próxima jornada de laboratorio** (Phineas), con `git pull` en `main`:
   - Mandar el error que salió con la cámara (mensaje completo).
   - Bloque D completo, **guardando la consola en un archivo** para poder pasarla:
     ```bash
     python3 tools/lab/lab_robot.py camera --save-every 5 2>&1 | tee camara-$(date +%H%M).log
     ```
     `tee` muestra la salida en pantalla y a la vez la guarda en el archivo; sirve igual con `line`,
     `drive` y `chase`.
   - Sensor sobre el **cuadro azul**: ¿lee `blanco` o `NEGRO`?
   - Bloques E (pala), F (manejo) y G (persecución, ya con el retroceso de 14 cm).
   - Traer la carpeta `capturas/` y los `.log`.
3. **Calibrar con esos datos** (lo hace quien lleva el software): `capture_cy`, ventana `scoop_*`,
   `min_area` y `square_min_area`, rangos de color.
4. **Ferb:** repetir en su Pi la guía 4.3 (driver), `motors` (su cableado puede ser otro), `line` y la
   posición de su sensor (hoy `line_backoff: 0.0`, `blind_reverse: false` hasta medirla).
5. **ROS 2 Lyrical:** en la laptop de simulación (WSL con Ubuntu 26.04, ver el README) y después en las
   tres Pi; primera prueba en banco con `robot.launch.py require_pose:=false require_heartbeat:=false`.
6. **Cámara cenital** (Perry o dedicada): sin ella no hay pose y no se puede probar la entrega en el
   cuadro azul con el robot real.

## 3. Cómo pueden ayudar los compañeros

Tareas que no necesitan el robot real ni a la persona que lleva el software. Cada una deja un
resultado concreto (un número medido, un archivo, una prueba que pasa) y se integra con un PR.

### Con el robot o la pista (sin programar)

| Tarea | Qué hacer | Resultado esperado | team-todo |
|---|---|---|---|
| Medir el montaje de la cámara | Altura de la lente sobre el piso, distancia de la lente al frente de la pala, inclinación (con un transportador o una app de nivel) | Tres números en cm y grados | 15 |
| Medir el tablero | Largo y ancho del tablero blanco y distancia de la cinta a cada borde | Cuatro números en cm | 16 |
| Medir la pala | Ancho, alto, ángulo de los extremos, distancia del frente del cuerpo al borde de la pala | Un dibujo acotado (foto con medidas sirve) | 21 |
| Fotos de los cubos | 20 fotos por color con la cámara del robot (`camera --save-every 2`), con la luz del laboratorio, a 20-60 cm | Carpeta `capturas/` | 3 |
| Preparar Ferb | Guía 4.2, 4.3 y 4.5 en la Pi de Ferb; `motors --dry-run` sin errores | Ferb listo para `motors` | 14 |

### En Webots (con la simulación en la laptop)

Requisitos: Webots R2025a en Windows y ROS 2 en WSL según el [README](../README.md#instalación).
Mientras no esté instalado ROS 2 Lyrical en WSL se puede seguir usando la WSL con Jazzy que ya existe
(los scripts cargan la versión instalada).

| Tarea | Qué hacer | Resultado esperado | team-todo |
|---|---|---|---|
| Sensor de línea en el centro, en la simulación | En `src/multirobot_sim/worlds/pista_ucab.wbt`, en el `DistanceSensor "linea_frente"` de Phineas (el primero del archivo) cambiar `translation 0.075 0 0.012` por `translation 0 0 0.012` (centro del robot, como en el real), poner `line_backoff: 0.14` y `blind_reverse: true` (ya están en `phineas.yaml`) y correr `ros2 launch bringup sim.launch.py` | Video o capturas: al llegar a la cinta, Phineas retrocede y gira sin sacar el cuerpo de la pista. Anotar si alguna vez la cruza | 19 |
| Probar ROS 2 Lyrical con Webots | Instalar WSL Ubuntu 26.04 + `ros-lyrical-webots-ros2` (README, secciones 2 y 3) y lanzar la simulación | Confirmar que `webots_ros2` de Lyrical funciona con Webots R2025a, o el error exacto | 20 |
| Signos de las ruedas por `cmd_vel` | Tarea 2 de team-todo (`sim.launch.py logic:=false` y `ros2 topic pub`) | Avance, lateral y giro correctos en Webots | 2 |
| Cámara de Webots igual a la real | Con las medidas de la cámara, ajustar la `Camera "camara"` del proto y repetir `calibracion_pala.wbt` | Nuevo `capture_cy` medido en Webots | 15 |
| Pala real en Webots | Con el dibujo de la pala, modelarla en el proto (hoy es una pala en U) y repetir las pruebas de retención (recto, arco, giro) | Tabla: ¿conserva el cubo en cada movimiento? | 21 |

### Programación (con pruebas)

| Tarea | Dónde | team-todo |
|---|---|---|
| Regresión de colores con las fotos reales | `mobile_base/test/` + `hue_ranges` en los YAML | 3 |
| Cámara del nodo de ROS en la Pi (vía que funcionó en Ubuntu) | `camera_node.py` | 4 |
| Localizador ArUco y homografía de la cámara cenital | `src/overhead_localization` | 5 |

Antes de abrir un PR: `powershell -File scripts/test_in_docker.ps1` debe terminar con
`all checks passed`.
