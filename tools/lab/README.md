# Pruebas de laboratorio del robot real (sin ROS)

> Procedimiento completo de la jornada (preparar la Pi, orden de las pruebas y tabla de resultados):
> [docs/guia-pruebas-laboratorio.md](../../docs/guia-pruebas-laboratorio.md). Este archivo describe la herramienta.

Herramienta en Python para probar en la Raspberry Pi 5 (Ubuntu Server 26.04, placa de motores
SunFounder Fusion HAT+, batería Li-ion 2S de 7,4 V y 2000 mAh) los motores, el sensor de línea y la cámara,
y ver al robot acercarse solo a los cubos de su color. **No necesita ROS**: usa la misma cinemática,
la misma detección de cubos y la misma máquina de estados que el resto del repositorio
(`src/mobile_base`), así que lo que se valida aquí es lo que después corre en ROS.

Sin pala y sin cámara cenital el robot no sabe dónde está ni puede entregar: en el modo `chase`
explora, se acerca al cubo y **se detiene** cuando el cubo llega a donde estará la boca de la pala.

## Seguridad (antes de encender)

- Trabajar bajo la supervisión del encargado del laboratorio.
- Hacer **todas las conexiones con el equipo apagado** y la batería desconectada. Revisar que no haya
  cables pelados ni contacto entre pines antes de conectar la batería (cortocircuitos).
- **Medir el voltaje de la batería** con el multímetro y pasarlo con `--battery`. Con ese valor se
  limita la potencia para que los motores TT (6 V) nunca reciban más de 6 V de promedio (con 8,4 V el
  tope es 71 %). Usar solo la batería 2S (7,4 V nominal, 8,4 V cargada): la entrada de la Fusion HAT+
  admite de 6,0 a 8,4 V.
- La primera prueba de motores se hace **con el robot levantado** (ruedas en el aire).
- `Ctrl+C` detiene los motores en cualquier modo. En `drive`, si no se pulsa ninguna tecla en 3 s
  el robot se detiene solo. Tener a alguien listo para levantar el robot en `chase`.

## Preparación de la Pi 5

Sistema elegido: **Ubuntu Server 26.04**. Los pasos completos, marcados como *por verificar* en esa
versión, están en la [guía del laboratorio](../../docs/guia-pruebas-laboratorio.md#4-preparar-la-raspberry-pi-antes-de-ir-al-laboratorio):

```bash
sudo apt update
sudo apt install -y python3-lgpio python3-opencv python3-numpy python3-yaml gpiod i2c-tools
```

- **Motores (Fusion HAT+):** hacen falta el driver del kernel y la biblioteca `fusion_hat` de SunFounder.
  Su instalador apunta a Raspberry Pi OS; en Ubuntu instalar antes los *headers* del kernel
  (`sudo apt install linux-headers-$(uname -r)`) y, si falla la compilación del driver,
  `cd ~/fusion-hat/driver && make && sudo make install`. Detalle en la guía, apartado 4.3. Sin la
  biblioteca la herramienta avisa `fusion_hat not available` y los motores quedan en vacío.
- **Sensor de línea:** sigue en GPIO5 con `lgpio` (la Fusion HAT+ no usa ese pin).
- **Cámara:** pendiente de verificar en Ubuntu Server 26.04. La herramienta prueba, en este orden,
  `picamera2`, GStreamer con `libcamerasrc` y V4L2 (una webcam USB con `--camera v4l2` sirve de alternativa).

La placa de motores se elige en el YAML (`motor_backend` de `hat_motor_driver`): `fusion_hat` por
defecto, o `cokoino_drv8833` para el HAT anterior (2 x DRV8833 por `lgpio`, con sus pines `pins_*`).

Copiar el código a la Pi (basta con las carpetas `tools/lab` y `src/mobile_base`):

```bash
git clone https://github.com/alejandrovielma/ucab-multirobot.git
cd ucab-multirobot
```

Si la Pi no tiene acceso al repositorio privado, copiar esas dos carpetas con una memoria USB
respetando la misma estructura.

Todos los comandos se corren desde la raíz del repositorio. Para ver la ayuda:

```bash
python3 tools/lab/lab_robot.py --help
```

## Orden de las pruebas

Anotar el resultado de cada paso (sirve para corregir la configuración después).

### 1. Motores, con las ruedas en el aire

```bash
python3 tools/lab/lab_robot.py motors --battery 8.1
```

Mueve cada rueda 1 s hacia adelante y 1 s hacia atrás y anuncia el puerto que usa, por ejemplo
`rueda fl (puerto M0): adelante` (`fl` delantera izquierda, `fr` delantera derecha, `rl` trasera
izquierda, `rr` trasera derecha); luego mueve el robot completo: avanzar, lateral a la izquierda y
girar antihorario. El aviso `GPIO dry run` es normal con la Fusion HAT+: los motores no usan los GPIO.

El cableado de los puertos todavía no se conoce (por defecto fl M0, fr M1, rl M2, rr M3):

- Anotar qué rueda física mueve cada puerto y si gira hacia atrás cuando el anuncio dice `adelante`.
- Escribir en `hat_motor_driver` de `src/mobile_base/config/phineas.yaml` (o `ferb.yaml`)
  `fusion_ports` con el puerto de cada rueda en el orden `[fl, fr, rl, rr]` y `fusion_reversed` con
  `true` en las ruedas invertidas; repetir la prueba para confirmarlo.
- Si no se mueve nada: revisar que no aparezca `fusion_hat not available`, que `i2cdetect -y 1`
  muestre la HAT en `17` y que la batería esté conectada.

### 1b. Movimiento en el piso, sin el cable de SSH

El cable Ethernet que se usa para el SSH tira del robot y lo desvía. Después de entrar por `ssh`:

```bash
bash tools/lab/prueba_piso.sh            # 0,2 m/s, 15 s para desconectar, 2 s por paso
bash tools/lab/prueba_piso.sh 0.08 15 2  # velocidad, espera y segundos por paso
```

Lanza en segundo plano avanzar, lateral a la izquierda y girar antihorario (sigue aunque se corte el
SSH), dice cuánto se va a mover y el nombre del `.log`. Desconectar el cable durante la cuenta
regresiva, volver a conectarlo cuando el robot se detenga y leer el `.log`. Anotar si avanza recto, si
el lateral es puro y cuánto gira.

### 2. Sensor de línea

```bash
python3 tools/lab/lab_robot.py line
```

Pasar el sensor sobre el tablero blanco y sobre la cinta negra. Debe decir `blanco` y `NEGRO (linea)`.
Si está al revés, cambiar `active_level` en `line_sensor` del YAML (1 a 0). Si no cambia nunca,
ajustar el potenciómetro del módulo o revisar que esté en GPIO5.

### 3. Cámara y detección de cubos

```bash
python3 tools/lab/lab_robot.py camera
```

Abrir en el navegador de una laptop conectada a la misma red `http://IP-DE-LA-PI:8080/`
(la IP se ve con `hostname -I`). Se ve la imagen con un recuadro sobre cada cubo detectado, su color
y su posición (`cx` de -1 a 1, izquierda a derecha; `cy` de 0 a 1, arriba a abajo), y un rectángulo
gris donde se mirará el cubo dentro de la pala. La consola imprime lo mismo una vez por segundo.

- Probar los 6 colores. Si alguno no se detecta o se confunde (rojo y morado son los más cercanos),
  anotarlo: los rangos están en `hue_ranges` de `cube_detector` en el YAML.
- **Calibración importante:** poner un cubo justo donde estará la boca de la pala (unos 20 cm delante
  del centro del robot) y anotar su `cy`. Ese valor, un poco menor, es el `capture_cy` del robot real
  (en Webots es 0,62, pero depende de la altura e inclinación reales de la cámara).

### 4. Manejo con el teclado (en el piso)

```bash
python3 tools/lab/lab_robot.py drive --max-speed 0.05
```

`w`/`s` avanzar/retroceder, `a`/`d` lateral, `q`/`e` girar, espacio parar, `+`/`-` velocidad, `x` salir.
Funciona por SSH. Comprobar que las direcciones coinciden con las teclas.

### 5. Persecución autónoma de cubos

```bash
python3 tools/lab/lab_robot.py chase --max-speed 0.05
```

Con el video en el navegador se ve el estado (`EXPLORE`, `APPROACH`, `CAPTURE`) y la orden a los
motores. El robot explora, gira al ver la línea negra, se acerca al cubo más grande de sus colores
(Phineas: rojo, naranja, amarillo; Ferb: `--robot ferb`) y se detiene al llegar a la boca de la pala.
Si se detiene muy lejos o muy cerca del cubo, repetir con `--capture-cy` usando el valor del paso 3.

## Problemas frecuentes

| Mensaje | Causa probable | Qué hacer |
|---|---|---|
| `fusion_hat not available: motors in dry run` | Falta la biblioteca de la Fusion HAT+ | Instalar el driver y la biblioteca (guía, apartado 4.3) |
| `i2cdetect -y 1` no muestra `17` | HAT mal asentada, apagada o sin driver | Revisar con todo apagado; `dmesg \| grep fusion_hat`, `lsmod \| grep fusion_hat` |
| `lgpio not available: falling back to GPIO dry run` (modos `line` y `chase`) | No está instalado `python3-lgpio` | `sudo apt install python3-lgpio` |
| `no pinctrl-rp1 gpiochip found` | No es una Pi 5 o el kernel numera distinto | `gpiodetect` y pasar `--chip N` del que diga `pinctrl-rp1` |
| `Permission denied` al abrir el gpiochip | El usuario no está en el grupo dueño de `/dev/gpiochip*` | `ls -l /dev/gpiochip*`, agregar el usuario a ese grupo y volver a entrar, o correr con `sudo` |
| `no camera backend worked` | Ninguna vía de cámara funciona todavía en Ubuntu | Revisar el cable CSI (con todo apagado); probar una webcam USB con `--camera v4l2` |
| El navegador no carga el video | Otra red, IP equivocada o puerto ocupado | `hostname -I`, misma red Wi-Fi, o `--port 8081` |

## Para probar en una laptop antes de ir

Sin hardware (`--dry-run`) y con una imagen en lugar de la cámara:

```bash
python3 tools/lab/lab_robot.py motors --dry-run --seconds 0.1
python3 tools/lab/lab_robot.py chase --dry-run --camera file:src/mobile_base/test/fixtures/webots_cube_at_mouth.jpg
```

## Dashboard en el navegador (misión + manual)

Botones de misión (color objetivo, prohibido, neutro; INICIAR / DETENER), cámara del robot, parada de
emergencia y modo manual con WASD (Q/E giran en el sitio), desde un celular o laptop en la misma red. Sin
modelo de lenguaje: es la capa deliberativa de la arquitectura híbrida (decisión 2026-10-30):

```bash
bash tools/lab/prueba_dashboard.sh [segundos] [espera s] [velocidad m/s]
cat prueba-dashboard-*.log      # la direccion completa, con ?t=<token>
```

Corre desacoplado (se puede quitar el cable). Todo comando pasa por el `SafetyGate` (línea, otro robot),
el robot se detiene 0,4 s después del último comando y Esc es una parada que se mantiene hasta pulsar el
botón otra vez. El token evita que otra persona de la red lo maneje.

## Posición estimada y memoria (cartógrafo)

`chase` y `dashboard` estiman dónde está el robot sin cámara cenital (filtro de partículas,
`core/localization.py`): predicen con las órdenes enviadas a los motores y se corrigen con la cinta
(sensor de línea), el cuadro azul y los rectángulos verde y rojo. Recuerdan las zonas vistas, los cubos
objetivo vistos y los toques de cinta (`core/cartographer.py`), y exploran hacia lo no visto girando antes
de la cinta. La línea del video y el estado del dashboard muestran "en (x, y) grados +-cm" cuando la
estimación es confiable, o "buscando" mientras no.

- `--no-map` vuelve a la exploración anterior (choca y gira), para comparar en la pista.
- Antes de confiar en ella hay que medir: la tabla distancia/altura en la imagen de la cámara
  (`CameraModel.cy_table`) y la posición real de los rectángulos (`LANDMARKS`); hoy son supuestos.
- Quieto y viendo solo el cuadro, el error puede llegar a 10-15 cm a lo largo de la línea de vista;
  se corrige al moverse y al tocar la cinta.

## Encontrar la IP de la Raspberry

Sin instalar nada (solo Python 3, en Windows, Linux o macOS), desde el PC conectado a la misma red Wi-Fi
que el robot o a su cable Ethernet:

```bash
python tools/lab/encontrar_pi.py              # nombres (phineas.local...), tabla ARP y barrido de tus redes /24
python tools/lab/encontrar_pi.py --linklocal  # además el cable (169.254.x.x), tarda ~1 minuto
python tools/lab/encontrar_pi.py --ip 10.68.17  # otra red /24 que quieras barrer
```

Muestra los equipos con el puerto 22 abierto y marca con `PI` los que tienen la dirección MAC de una
Raspberry Pi. **No inicia sesión ni usa contraseñas**: después se entra a mano con `ssh <usuario>@<ip>`.
Si no sale nada: esperar ~30 s tras conectar el cable, comprobar que la Pi está encendida y probar
`--linklocal`. Si el PC enruta el tráfico por una VPN o un proxy (Throne, sing-box...), el barrido puede no
llegar a la red local: apagarla un momento solo para buscar la IP.
