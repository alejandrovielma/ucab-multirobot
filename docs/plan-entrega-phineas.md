# Plan: corregir la entrega de cubos de Phineas

Rama de trabajo: `feature/phineas-on-ferb-develop` (código base de `ferb/develop` con cambios mínimos para Phineas).
Código en la Pi: `~/ucab-multirobot-ferb`, servidor `http://<ip-de-la-Pi>:8081`, lanzado con `~/run_teleop_mirror.py`.
Fecha de redacción: 2026-11-03.

## 1. Problema

El robot detecta el cubo azul y va por él, pero **no lo deja en la zona de descarga**: se queda a unos centímetros antes de llegar. Hay que corregirlo para que entregue todos los azules y evite los amarillos.

## 2. Qué se sabe (datos medidos)

### 2.1 Geometría de la zona de descarga (medida por el usuario)

- Profundidad (eje de empuje): **13,7 cm** = 1,8 (tira lima de entrada) + 10,1 (azul) + 1,8 (tira lima del fondo).
- Ancho: **16,5 cm**.
- Centro del cuadrilátero: a **6,85 cm del borde cercano**.
- Con el centro del cubo en el centro del papel, su borde lejano queda a **12,35 cm de la lente** (5,5 cm del centro del cubo a la lente + 6,85 cm hasta el borde lejano).

### 2.2 Calibración de cámara (`camera.model` en `tools/teleop_web.yaml`)

Valores actuales: `hfov_rad 1.916`, `height_m 0.058`, `tilt_rad 0.125`. Con cubos puestos a mano:

| Distancia real | Lectura del modelo |
|---|---|
| 15 cm | 14,6 cm |
| 30 cm | 28,6 cm |
| 45 cm | 44,8 cm |

Un ajuste con siete puntos da `hfov 1.895`, `tilt 0.125` (error 5,4 % frente a 5,5 %). No hay que tocar el config. El modelo anterior (`hfov 1.1826`, `tilt 0.1835`) leía el cubo de 45 cm a 30 cm.

### 2.3 Lectura de la zona a 40 cm del centro (centro de la pala a 40 cm del centro del cuadrilátero)

- El detector solo devuelve `gate` (hueco entre las balizas): distancia 26,2 cm, rumbo −0,106 rad. **Sin** borde cercano, borde lejano ni ángulo de la línea rosa.
- Causa: a 40 cm el papel mide ~900 px y `floor_min_area_px` exige 1500. Es por diseño.
- Medido a mano sobre la imagen (colores `area_teal` y `area_lime`): papel entre las filas 238 y 253, es decir, borde cercano a **31,4 cm** y lejano a **49,7 cm** de la lente. Su centro sale a unos **40,5 cm de la lente**.
- Si el centro de la pala fuera el fondo de la banda (4 cm delante de la lente) se esperaría el centro a 44 cm. **Hay un desfase de unos 3,5 cm sin explicar.** Un píxel de fila equivale a 1–1,5 cm a esta distancia.

#### Repetición a 40 cm (2026-11-03, robot recto frente a la zona, tres capturas seguidas)

- Papel entre las filas 239 y 253 en las tres capturas: borde cercano a **31,4 cm** (±1,2 cm por fila) y lejano a **47,9 cm** de la lente. Centro del papel: ~**39,7 cm de la lente**.
- Eso coincide con 40 cm si el centro de la pala está a la altura de la lente. Con ese punto de referencia se esperaría el borde cercano a 33,2 cm y el lejano a 46,9 cm: las dos lecturas quedan a ~1,5 cm, dentro del error de una fila. **El desfase de 3,5 cm desaparece**; era la suposición de que el centro de la pala estaba 4 cm delante de la lente.
- Sigue sin hacerse la comprobación con cinta (etapa 1) y a 25 y 12 cm.
- Detector de zona a 40 cm: `gate` a 26,7–27,2 cm, rumbo −0,07 rad, sin `floor` (papel ~900 px < 1500). Estable en las tres capturas.

#### Lectura a 25 cm del centro (2026-11-03, tres capturas seguidas)

- El detector ya devuelve `floor` (papel de ~3640 px): borde cercano **20,2 cm**, borde lejano **33,2 cm**, rumbo −0,13 rad, **`entry_angle_rad: None`**.
- Esperado con el centro de la pala a 25 cm (lente a la altura de la pala): cercano 18,2 cm y lejano 31,9 cm. El error es de +1,4 a +2,0 cm; a 40 cm era de −0,3 cm. Queda dentro de la tolerancia de colocación a mano (~1 cm) y de una fila de imagen. El modelo de cámara se mantiene.
- **Hallazgo principal:** el papel de la zona **no tiene línea rosa de entrada**, solo tiras lima (visible en la imagen recortada). Con `entry_angle: None` y el borde cercano a más de `entry_hidden_distance_m` (0,15) el autopiloto entra en `CARRY (no entry line)` y **se queda parado para siempre** a ~20 cm del papel. Coincide con el síntoma "se queda a unos centímetros antes de llegar".
- **Corrección aplicada** (commit `c89182e`): `entry_hidden_distance_m: 0.30` (mayor que `deliver_commit_distance_m` 0,20 + histéresis 0,08 = 0,28). Así no espera la línea rosa y entrega recto. Sin la línea el robot no puede comprobar que va recto antes de empujar.

#### Lectura a 12 cm del centro (2026-11-03, tres capturas seguidas)

- `floor` con ~42 280 px: borde cercano **9,4–9,8 cm**, lejano **19,5 cm**, rumbo **−0,34 rad**, `entry_angle_rad: None`.
- Esperado (lente a la altura de la pala): borde cercano de la lámina azul a 12 − 6,85 + 1,8 = 7,0 cm (a 5,2 cm si cuenta la tira lima delantera) y lejano a 18,9 cm. El lejano coincide (+0,6 cm). El **cercano se lee 2,7 cm más lejos que el azul y 4,6 más que la tira lima**: la tira lima delantera queda en las filas bajas de la imagen (388–430), fuera del blob `floor`, que termina en la fila 388.
- **Rumbo −0,34 rad** con el robot centrado: a esta distancia el papel (~570 px de ancho) se sale por el borde derecho de la imagen, y el rumbo (centro del contorno proyectado) se sesga. Es mayor que `drop_align_tolerance_rad` (0,12), así que el autopiloto se desplazaría de lado en `CARRY (center)` antes de empujar. **Sin verificar en movimiento.**

### 2.4 Valores del config contra esa geometría

| Parámetro | Valor | Observación |
|---|---|---|
| `deliver_insert_m` | 0,087 | Asume 1,8 + 13,8/2 = 8,7 cm. Con el papel real de 13,7 cm de fondo el centro está a 6,85 cm: **se pasa 1,85 cm** |
| `deliver_far_stop_m` | 0,12 | Coherente con los 12,35 cm calculados |
| `cube_ahead_m` | 0,055 | Centro del cubo a 5,5 cm de la lente; sin verificar en este robot |
| `deliver_commit_distance_m` | 0,20 | Medido con el modelo de cámara anterior |
| `zone_stop_distance_m` | 0,59 | Idem |
| `entry_hidden_distance_m` | 0,15 | Idem |
| `floor_arrive_v_px` | 370 | Idem |

### 2.5 Fallos ya corregidos en código y todavía no verificados en pista

1. **Empuje de 0 cm por borde lejano truncado.** El log mostró `near edge 0.078 m, far 0.121 m, 0.000 m = 0.00 s`. El borde lejano (12,1 cm) estaba solo 4,3 cm detrás del cercano, cuando el papel mide 13,7 cm de fondo. Corrección: `deliver_far_min_span_m: 0.08` ignora un borde lejano a menos de 8 cm detrás del cercano.
2. **Cubo en la pala visto mal tras el empuje.** Corrección: `assume_in_scoop_distance_m: 0.12`. Un cubo aún visible a menos de 12 cm tras el empuje se da por recogido; entre 12 y 20 cm se vuelve a buscar.
3. **Filtro `beyond_line` descartaba el cubo.** Leía la sombra oscura de la pala (filas bajas de la imagen) como cinta del borde. Corrección: `camera.border.ignore_bottom_px: 70`.
4. **Telemetría** en el log de la Pi: cambios de estado y una línea por empuje de entrega (`delivery: blind push, near edge …, speed …, … m = … s`).

## 3. Condiciones previas para cada prueba

- Batería de la Fusion HAT+ por encima de **7,2 V** (a 6,6 V / 11 % los motores no arrancaban y la Pi se reiniciaba). Comprobar con `grep -E "STATUS|CAPACITY|VOLTAGE_NOW" /sys/class/power_supply/fusion-hat/uevent`.
- Localizar la Pi con `python pi_ip.py` (carpeta padre del repo); la MAC de Phineas es `2C:CF:67:69:48:FA`. No tocar la Pi de Ferb (`2C:CF:67:CA:B5:E5`).
- Servidor arrancado con `cd ~/ucab-multirobot-ferb && python3 ~/run_teleop_mirror.py`, con Auto apagado.
- Pista despejada, sin cubos amarillos ni manos en el encuadre de la cámara.

## 4. Etapas

### Etapa 1. Cerrar el desfase de 3,5 cm

**Objetivo:** saber si el desfase es un offset de la pala o un error del modelo de cámara.

**Medidas con cinta (el usuario):**
1. Distancia de la **lente** de la cámara al borde cercano de la tira lima del papel, con el centro de la pala a 40 cm del centro del cuadrilátero.
2. Distancia del borde cercano al **fondo de la banda** de la pala (punto más profundo, 4 cm delante de la lente según el config).

**Repetir** con el centro de la pala a **25 cm** y a **12 cm** del centro.

**Análisis:**
- Si el error es constante en las tres distancias: es un offset de la pala (cambia el punto de referencia, no el modelo).
- Si el error crece al acercarse: hay un problema del modelo a corto alcance; se vuelve a ajustar con puntos cercanos.

**Salida:** tabla (distancia real, distancia leída) para el borde cercano y el centro.

### Etapa 2. Rango útil de las lecturas de la zona

**Objetivo:** saber desde qué distancia el detector da borde cercano, borde lejano y ángulo de la línea rosa.

**Método:** con el robot recto frente a la zona, capturar la imagen cada 5 cm entre 45 y 10 cm del centro y ejecutar el diagnóstico de zona (`~/zonediag.py <imagen>`). Registrar área del papel (px), `distance_m`, `far_distance_m`, `bearing_rad`, `entry_angle_rad`.

**Criterio:** la distancia a la que el área supera `floor_min_area_px` (1500) y aparecen todos los campos. Se espera entre 25 y 30 cm.

**Salida:** distancia mínima de lectura fiable, y la fila `v` de los bordes a cada distancia.

### Etapa 3. Corregir los parámetros de entrega

**Orden, un cambio por vez**, con las medidas de las etapas 1 y 2:

1. `deliver_insert_m`: dejar el centro del cubo a 6,85 cm del borde cercano. Valor de partida: 0,0685.
2. `deliver_far_stop_m`: comprobar que siga siendo ~0,12 (12,35 cm calculados); ajustar con la medida real de `cube_ahead_m`.
3. `cube_ahead_m`: medir con un cubo en la pala la distancia de su centro a la lente.
4. Revalidar en metros con el modelo nuevo: `deliver_commit_distance_m` (0,20), `zone_stop_distance_m` (0,59), `entry_hidden_distance_m` (0,15) y `floor_arrive_v_px` (370). Para cada uno, calcular a qué distancia física corresponde ahora y compararlo con lo que se quería.

**Archivos:** `tools/teleop_web.yaml` (sección `auto:`); tests en `tools/test_autopilot.py`.

### Etapa 4. Prueba de campo con telemetría

**Montaje:** batería cargada, un solo cubo azul a unos 40 cm de la pala y la zona de descarga de frente. Se activa Auto con `POST /cmd {"auto": true}` y se vigila `~/teleop_ferb.log`. Si algo se sale de lo esperado, se apaga con `{"auto": false}`.

**Qué se lee del log:**
- Secuencia de estados: `SEARCH → APPROACH → CATCH → CARRY → CARRY (align/center/square) → DROP → DROP (back up)`.
- Línea `delivery:` con borde cercano, velocidad y metros ciegos del empuje.
- Si aparece `delivery: cube left outside, not counted`, la entrega falló.

**Tras cada prueba:** medir con cinta dónde quedó el cubo respecto al centro del papel. Cambiar **un solo parámetro** y repetir.

### Etapa 5. Qué hacer según el síntoma

| Síntoma en el log | Posible causa | Acción |
|---|---|---|
| Se queda en `CARRY (no entry line)` | No ve la línea rosa de entrada | Ajustar `entry_hidden_distance_m` o los rangos de `area_pink` |
| Pasa a `SEARCH` con el cubo en la pala | Pierde la zona y la da por perdida (`carry_zone_timeout_s`) | Subir `carry_zone_timeout_s` y `carry_paper_hold_s` |
| Empuje corto y la línea `blind push` muestra metros correctos | La velocidad real es menor que la medida | Compensar la potencia de arranque de motores (`start_power_pct`, ~20 %) |
| Empuje de 0 m o `far edge … stop pushing` demasiado pronto | Borde lejano mal leído | Subir `deliver_far_min_span_m` o ajustar `deliver_far_stop_m` |
| Cubo aún en la pala tras el empuje y vuelve a buscar | `catch_check_distance_m` mayor que `assume_in_scoop_distance_m` | Revisar ambos umbrales contra la lectura real del cubo en la pala |
| El robot no avanza y el estado no cambia | Batería baja | Cargarla y repetir |

### Etapa 6. Criterio de éxito y cierre

- **Éxito:** el centro del cubo queda sobre el papel azul en **3 de 3 entregas seguidas**, confirmado visualmente por el usuario (sin cámara cenital disponible).
- **Cierre:** commit de los parámetros finales en `feature/phineas-on-ferb-develop`, con un mensaje que cite los valores antes y después, y actualización de `docs/hardware.md` con las medidas de la zona (13,7 × 16,5 cm y desglose de tiras).

## 5. Riesgos y puntos sin verificar

- Nada de este plan se ha ejecutado todavía. Los valores esperados son cálculos con las cotas medidas por el usuario.
- El desfase de 3,5 cm podría ser un error del modelo a esa distancia; si lo es, cambia la Etapa 3.
- La batería bajó del 67 % al 47 % en pocos minutos y la Pi se reinició varias veces: hay que vigilar la alimentación.
- Sin imagen cenital (DroidCam fuera de la red), el resultado de cada entrega se juzga con la cámara del robot y la confirmación del usuario.
- Los umbrales en metros se calibraron con modelos de cámara anteriores; hasta revalidarlos pueden dar comportamientos distintos a los esperados.
