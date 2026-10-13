# Pruebas de laboratorio del robot real (sin ROS)

> Procedimiento completo de la jornada (preparar la Pi, orden de las pruebas y tabla de resultados):
> [docs/guia-pruebas-laboratorio.md](../../docs/guia-pruebas-laboratorio.md). Este archivo describe la herramienta.

Herramienta en Python para probar en la Raspberry Pi 5 los motores, el sensor de línea y la cámara,
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
  limita el PWM para que los motores TT (6 V) nunca reciban más de 6 V de promedio. Usar solo una
  batería de 2 celdas (2S, hasta 8,4 V cargada): el DRV8833 del HAT no admite más de 10,8 V.
- La primera prueba de motores se hace **con el robot levantado** (ruedas en el aire).
- `Ctrl+C` detiene los motores en cualquier modo. En `drive`, si no se pulsa ninguna tecla en 3 s
  el robot se detiene solo. Tener a alguien listo para levantar el robot en `chase`.

## Preparación de la Pi 5

Recomendado: **Raspberry Pi OS Bookworm 64 bits**. En ese sistema la cámara CSI (IMX219) funciona con
`picamera2`; en Ubuntu 24.04 no viene lista (Ubuntu soporta la cámara de la Pi oficialmente desde la
25.04), así que ahí la cámara solo funciona con una webcam USB (`--camera v4l2`).

```bash
sudo apt update
sudo apt install -y python3-lgpio python3-opencv python3-numpy python3-yaml python3-picamera2
```

Comprobar la cámara (Raspberry Pi OS):

```bash
rpicam-hello --list-cameras
```

Si no aparece ninguna cámara, editar `/boot/firmware/config.txt`, poner `camera_auto_detect=0` y
agregar `dtoverlay=imx219,cam0` (o `cam1`, según el conector donde esté la cámara); reiniciar y repetir.

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

Mueve cada rueda 1 s hacia adelante y 1 s hacia atrás (`fl` delantera izquierda, `fr` delantera
derecha, `rl` trasera izquierda, `rr` trasera derecha) y luego el robot completo: avanzar, lateral a
la izquierda y girar antihorario.

- Si una rueda gira al revés: cambiar su signo en `wheel_signs` de
  `src/mobile_base/config/phineas.yaml` (orden fl, fr, rl, rr).
- Si se mueve otra rueda que la anunciada: los pines de `pins_fl`, `pins_fr`, `pins_rl`, `pins_rr`
  en el mismo archivo no corresponden al cableado.
- Si no se mueve nada: revisar la batería del HAT, el mensaje `gpiochip N` al arrancar y que no
  aparezca `GPIO dry run`.

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
python3 tools/lab/lab_robot.py drive --max-speed 0.08
```

`w`/`s` avanzar/retroceder, `a`/`d` lateral, `q`/`e` girar, espacio parar, `+`/`-` velocidad, `x` salir.
Funciona por SSH. Comprobar que las direcciones coinciden con las teclas.

### 5. Persecución autónoma de cubos

```bash
python3 tools/lab/lab_robot.py chase --max-speed 0.08
```

Con el video en el navegador se ve el estado (`EXPLORE`, `APPROACH`, `CAPTURE`) y la orden a los
motores. El robot explora, gira al ver la línea negra, se acerca al cubo más grande de sus colores
(Phineas: rojo, naranja, amarillo; Ferb: `--robot ferb`) y se detiene al llegar a la boca de la pala.
Si se detiene muy lejos o muy cerca del cubo, repetir con `--capture-cy` usando el valor del paso 3.

## Problemas frecuentes

| Mensaje | Causa probable | Qué hacer |
|---|---|---|
| `lgpio not available: falling back to GPIO dry run` | No está instalado `python3-lgpio` | `sudo apt install python3-lgpio` |
| `no pinctrl-rp1 gpiochip found` | No es una Pi 5 o el kernel numera distinto | `gpiodetect` y pasar `--chip N` del que diga `pinctrl-rp1` |
| `Permission denied` al abrir el gpiochip | El usuario no está en el grupo `gpio` | `sudo usermod -aG gpio $USER` y volver a entrar, o correr con `sudo` |
| `no camera backend worked` y `picamera2: No module named 'picamera2'` | Falta `python3-picamera2` o no es Raspberry Pi OS | Instalarlo, o usar una webcam USB con `--camera v4l2` |
| `picamera2: ... no cameras available` | La cámara no se detecta | Revisar el cable CSI (con todo apagado) y el `dtoverlay` de arriba |
| El navegador no carga el video | Otra red, IP equivocada o puerto ocupado | `hostname -I`, misma red Wi-Fi, o `--port 8081` |

## Para probar en una laptop antes de ir

Sin hardware (`--dry-run`) y con una imagen en lugar de la cámara:

```bash
python3 tools/lab/lab_robot.py motors --dry-run --seconds 0.1
python3 tools/lab/lab_robot.py chase --dry-run --camera file:src/mobile_base/test/fixtures/webots_cube_at_mouth.jpg
```
