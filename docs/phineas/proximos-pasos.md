# Próximos pasos tras la integración con Ferb-Recolector

Fecha: 2026-11-01. Rama de trabajo: `integracion-ferb` (cortada de `ferb/develop`), integrada en `main`.
Registro detallado de tareas y commits: [odd/tasks/integracion-ferb.md](../../odd/tasks/integracion-ferb.md).

## Qué es este repositorio ahora

La base es el repositorio de Ferb (`notoriussss/Ferb-Recolector`, rama `develop`). Encima se añadió lo
nuestro que no chocaba de ruta y se adaptó lo que sí:

- De ellos, intacto: `vision`, `recolector_common`, `recolector_interfaces`, `recolector_sim`, `fleet_link`,
  `peer_avoidance`, autopilot y teleop web (`tools/teleop_web*`), Docker, scripts, ADRs y CI.
- De nosotros: `coordinator`, `overhead_localization`, `multirobot_sim`, `hardware/pala`, `tools/lab`,
  `tools/sim`, el núcleo de comportamiento (`mobile_base/core`, `mobile_base/nodes`) y nuestros docs
  (`docs/phineas/`).
- Nuestros ejecutables se llaman `phineas_*` y nuestros launch `phineas_*.launch.py`, para no chocar con los suyos.
- Mensajes y topics unificados con los de ellos: `recolector_interfaces/CubeDetectionArray` en `cubes`,
  `recolector_interfaces/LineSensors` en `line`, más un `Scene` aditivo en `cubes/scene`
  (ver [integracion-interfaces.md](integracion-interfaces.md)).

## Qué está verificado y qué no

Verificado (Docker, ROS 2 Lyrical, imagen `recolector:sim`):

- `colcon build` de 10 paquetes y `colcon test` con 529 tests sin fallos.
- Importación de 82 módulos y arranque sin hardware de `phineas_robot.launch.py` para `phineas` y `ferb`.
- Los tests de `tools`: 202 pasan y 1 falla (ver punto 1 de "Avisar a Ferb").
- Su simulación Webots arranca y publica. Nuestra cadena (`phineas_on_ferb_sim.launch.py`) corre sobre ella,
  con `line` activo, y el robot se queda en la mesa.

NO verificado:

- Captura y entrega completas, en simulación y en el robot real.
- Todo lo que depende del hardware: AO analógico, `confirm_reads`, umbrales de la T6, `camera.hfov_rad`.
- Un nodo de línea que muere a mitad de ejecución (solo hay tests unitarios).

## 1. Pruebas en el robot real (Raspberry Pi) — prioridad alta

1. Sensor analógico AO: `lab_robot.py line` con `adc_channel: 0` en la sección `line_sensor`. Esperado (según
   las mediciones de Ferb): unas 285 a 299 cuentas sobre blanco y unas 3383 sobre negro. Depende de que el cable
   AO esté en A0 del HAT: confirmarlo antes.
2. `confirm_reads: 2` en `chase` y `drive`; comprobar que no se pierde la cinta de 19 mm.
3. Valores de la T6 que elegimos nosotros y nadie ha probado: `capture_fails=2`, `capture_fail_window_s=20`,
   `skip_time_s=3`, `line_stuck_escapes=4`, `line_stuck_window_s=30`, `line_stuck_clear_s=1`
   (en `src/mobile_base/config/collector.yaml`). Vigilar que el watchdog no se dispare con el ruido normal del
   sensor y que 3 s de ignorar un color sea suficiente.
4. `line_timeout_s: 0.5` del safety gate (`phineas.yaml`, `ferb.yaml`): comprobar que no dispara falsos
   `line_stale` con el nodo real a 50 Hz.
5. Fijar `camera.hfov_rad` por robot: `phineas.yaml` declara 1.60 (IMX219-120) y el valor por defecto es el de
   Ferb, 0.94. Hoy nuestro comportamiento no usa bearing ni distancia, pero sí lo hace el FSM de Ferb.
6. Correr los tests de `tools/lab` en Linux (necesitan `termios`): `scripts/test_in_docker.sh`.
7. No lanzar a la vez el `cube_detector` de Ferb y nuestro `phineas_cube_detector`: los dos publican en `cubes`.

## 2. Simulación: llegar a una entrega completa — prioridad media

- Hoy la cadena nuestra llega a `APPROACH` pero no se observó ninguna captura. Falta que un cubo del color
  objetivo esté a la vista y que los rangos HSV sirvan en su mundo. Nuestros rangos son los del robot real; los de
  Ferb para la simulación están en `src/vision/config/hsv_ranges.yaml`.
- Nuestro detector reporta las paredes naranja de la propia pala como dos cubos fijos. Con `targets` naranja se
  perseguiría a sí mismo: enmascarar esa zona de la imagen.
- El puente de ellos aproxima el sensor central con los dos sensores delanteros de las esquinas (el proto no
  tiene sensor central). Es algo más conservador que el real.
- Cambio posible en el proto del robot si se quiere un sensor central de verdad (archivo de Ferb: coordinar).

## 3. Avisar a Ferb — cosas que les benefician

1. Su test `tools/test_teleop_web.py::test_each_black_edge_of_the_line_sensor_is_logged_with_its_analog_value`
   falla siempre en Docker, también sobre su `ferb/develop` sin nuestro código. Si su CI lo ejecuta, les sale rojo.
2. Su `docker/Dockerfile` no instala `libsndio7.0` y `webots-bin` sale con código 127 en Ubuntu 26.04. Nuestro
   commit `d4a09dc` lo corrige con una capa aparte.
3. Su puente de simulación nunca publicaba `line`; el commit `bcd0ddc` (solo toca `webots_bridge.py` y un test
   nuevo) lo añade y se les puede pasar tal cual.
4. Un error que descubrimos: suscribirse a `line` con QoS fiable no conecta con un publicador `best effort`
   (el de Ferb lo es). En nuestros nodos lo arregla `8e46043`. Conviene revisar que sus propios suscriptores a
   `line` usen `qos_profile_sensor_data`.

## 4. Deuda técnica

- Lint: el CI de Ferb corre `ruff check` y `ruff format --check`. Nuestro código tenía unos 2977 errores y 79
  archivos por reformatear, así que está en `extend-exclude` de `pyproject.toml`. Limpiarlo módulo por módulo con
  sus tests (docstrings, complejidad máxima 10, 80 columnas, valores mágicos) y borrar su línea de la lista.
- `docs/architecture.md` describe solo la pila de Ferb: añadir nuestros nodos `phineas_*` y el `Scene`.
- Caja del cubo: nuestro detector no conserva la forma, así que se publica un cuadrado de lado `sqrt(area)`
  (marcado `shortcut:` en el código). Mejorar si el FSM de Ferb necesita la caja real.
- Dos pilas en paralelo (la de Ferb y la nuestra `phineas_*`). Decidir a medio plazo cuál queda como la oficial
  del robot, o cómo converger en una sola.
- `behavior_node` aún no avisa si `line` se silencia; el gate sí falla cerrado (`line_stale`). Añadir un aviso
  con límite de frecuencia si se quiere visibilidad en el comportamiento.

## 5. Pospuesto: necesita una fuente de pose

El geofence (`core/track.py`), el planner y el cartógrafo solo funcionan con la pose de la cámara cenital, que
los robots reales no usan por ahora. Siguen en el repositorio, probados, pero sin conectar al FSM de Ferb.
Reabrir si se vuelve a usar `overhead_localization`. Carry-delivery se omitió a propósito: su zona (piso o
balizas), `WAIT_ZONE` y el empuje ciego de Ferb ya lo cubren. La verificación de captura por cámara es opcional
mientras funcionen el `FC-51` y el `TCS34725`.

## 6. Repositorio y CI

- Remoto `ferb` configurado localmente (`https://github.com/notoriussss/Ferb-Recolector.git`) para seguir
  trayendo sus cambios; los historiales son independientes, así que cada traída es un `merge` manual.
- Este repositorio ahora trae el workflow `.github/workflows/ci.yml` de Ferb. Usa un runner `ubuntu-26.04` y el
  contenedor `ros:lyrical`, y ejecuta los tests de `tools`, incluido el que falla de Ferb. Es probable que el CI
  de nuestro remoto salga en rojo hasta resolver el punto 3.1 o adaptar el workflow.
