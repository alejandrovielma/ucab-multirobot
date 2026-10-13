# Guía de pruebas en el laboratorio: Raspberry Pi 5 y robot real

Procedimiento completo para preparar la Raspberry Pi 5 y probar el robot real en el laboratorio,
**sin pala y sin cámara cenital**. Se prueba el hardware (motores, sensor de línea, cámara) y que el
robot se acerque solo a los cubos de su color. Seguir los pasos en orden y llenar la
[tabla de resultados](#7-tabla-de-resultados) al final.

Herramienta que se usa: `tools/lab/lab_robot.py`, en Python, **sin ROS**. Usa la misma cinemática,
detección de cubos y máquina de estados que el resto del proyecto. Detalle de cada modo en
[tools/lab/README.md](../tools/lab/README.md).

## Contenido

1. [Qué se prueba y qué no](#1-qué-se-prueba-y-qué-no)
2. [Materiales](#2-materiales)
3. [Reglas de seguridad](#3-reglas-de-seguridad)
4. [Preparar la Raspberry Pi (antes de ir al laboratorio)](#4-preparar-la-raspberry-pi-antes-de-ir-al-laboratorio)
5. [Pruebas en el laboratorio, paso a paso](#5-pruebas-en-el-laboratorio-paso-a-paso)
6. [Al terminar](#6-al-terminar)
7. [Tabla de resultados](#7-tabla-de-resultados)
8. [Problemas frecuentes](#8-problemas-frecuentes)
9. [Referencia: conexiones](#9-referencia-conexiones)

---

## 1. Qué se prueba y qué no

| Se prueba | No se prueba (todavía no existe en el robot) |
|---|---|
| Que cada motor gire en el sentido correcto | Capturar y transportar cubos (falta la pala) |
| El movimiento del robot: avanzar, lateral, girar | Entregar en el cuadro (falta la cámara cenital que da la posición) |
| El sensor de línea sobre blanco y sobre cinta negra | El programa completo con ROS 2 |
| La cámara IMX219 y la detección de los 6 colores | Esquivar al otro robot |
| Que el robot se acerque solo a un cubo de su color y se detenga | |

Sin la cámara cenital el robot no sabe dónde está: en el modo de persecución explora, gira al ver la
línea negra, va hacia el cubo y **se detiene** donde estaría la boca de la pala.

## 2. Materiales

- [ ] Robot armado: Raspberry Pi 5, HAT Cokoino con sus motores, sensor de línea, cámara IMX219 con su
      cable CSI de 22 pines.
- [ ] Tarjeta microSD con **Raspberry Pi OS (Bookworm, 64 bits)**.
- [ ] Batería del HAT **cargada** (2 celdas, 2S) y su cargador.
- [ ] **Multímetro.**
- [ ] Laptop conectada a la **misma red Wi-Fi** que la Pi (para SSH y para ver la cámara en el navegador).
- [ ] Cubos de los 6 colores (rojo, naranja, amarillo, verde, azul, morado).
- [ ] Un trozo de tablero blanco con cinta negra (la pista o una muestra) para el sensor de línea.
- [ ] Algo para levantar el robot y dejar las ruedas en el aire (una caja o un soporte).
- [ ] Cinta métrica o regla.
- [ ] Esta guía y la tabla de resultados (impresas o en la laptop).

## 3. Reglas de seguridad

- Trabajar **bajo la supervisión del encargado del laboratorio**.
- **Toda conexión o desconexión se hace con el equipo apagado y la batería desconectada**: cables de
  motores, sensor, cámara CSI y la propia batería.
- Antes de conectar la batería, revisar que no haya cables pelados, pines doblados ni metal suelto sobre
  el HAT (**cortocircuitos**).
- Usar **solo batería de 2 celdas (2S, hasta 8,4 V cargada)**. El driver del HAT (DRV8833) no admite más
  de 10,8 V, y los motores TT son de 6 V: la herramienta limita el PWM con el voltaje que se le indique.
- La primera prueba de motores se hace **con las ruedas en el aire**.
- `Ctrl+C` detiene los motores en cualquier modo. Durante la persecución autónoma, una persona tiene que
  estar lista para levantar el robot.
- Si algo se calienta, huele a quemado o hace ruido raro: `Ctrl+C`, apagar y desconectar la batería.

## 4. Preparar la Raspberry Pi (antes de ir al laboratorio)

Todo este paso se puede hacer en casa con la Pi conectada a una pantalla o por SSH. Toma unos 20–30 min.

### 4.1 Red y acceso por SSH

1. Conectar la Pi a la red Wi-Fi (desde el escritorio de Raspberry Pi OS o con `sudo nmtui`).
2. Activar SSH: `sudo raspi-config` → *Interface Options* → *SSH* → *Yes*.
3. Ver la IP de la Pi:
   ```bash
   hostname -I
   ```
4. Desde la laptop, en la misma red, entrar por SSH (cambiar `usuario` e `IP`):
   ```bash
   ssh usuario@IP
   ```

Anotar la IP en la tabla de resultados: se usa para ver la cámara en el navegador. En la red del
laboratorio la IP puede ser otra; repetir `hostname -I` al llegar.

### 4.2 Instalar lo necesario

```bash
sudo apt update
sudo apt install -y git python3-lgpio python3-opencv python3-numpy python3-yaml python3-picamera2 gpiod
```

`gpiod` trae el comando `gpiodetect`, útil si hay problemas con los pines.

Comprobar que el usuario puede usar los pines (en Raspberry Pi OS el usuario principal ya está en el
grupo `gpio`):

```bash
groups
```

Si `gpio` no aparece en la lista: `sudo usermod -aG gpio $USER`, cerrar la sesión y volver a entrar.

### 4.3 Cámara

1. **Con la Pi apagada**, conectar el cable CSI de la cámara a uno de los conectores de cámara de la Pi 5
   (el cable de 22 pines del módulo Pi5-IMX219), con los contactos del lado correcto según el manual
   del módulo. Anotar en qué conector quedó (CAM/DISP 0 o 1).
2. Encender y comprobar que la Pi detecta la cámara:
   ```bash
   rpicam-hello --list-cameras
   ```
   Debe aparecer una cámara `imx219`.
3. Si **no aparece**: editar la configuración de arranque,
   ```bash
   sudo nano /boot/firmware/config.txt
   ```
   cambiar `camera_auto_detect=1` por `camera_auto_detect=0` y agregar al final
   `dtoverlay=imx219,cam0` si la cámara está en el conector 0, o `dtoverlay=imx219,cam1` si está en el
   1. Guardar (`Ctrl+O`, `Enter`, `Ctrl+X`), reiniciar con `sudo reboot` y repetir el paso 2.
4. Tomar una foto de prueba:
   ```bash
   rpicam-still -o prueba.jpg
   ```

### 4.4 Descargar el código

El repositorio es privado. Para clonarlo en la Pi hace falta un **token de acceso** de GitHub (la
contraseña de la cuenta no sirve para `git`):

1. En GitHub: foto de perfil → *Settings* → *Developer settings* → *Personal access tokens* →
   *Fine-grained tokens* → *Generate new token*.
2. *Repository access*: solo `ucab-multirobot`. *Permissions* → *Contents*: **Read-only**. Fecha de
   vencimiento corta (por ejemplo 7 días).
3. Copiar el token y, en la Pi:
   ```bash
   cd ~
   git clone https://github.com/alejandrovielma/ucab-multirobot.git
   ```
   Cuando pida *Username*, el usuario de GitHub; cuando pida *Password*, **pegar el token**.

Sin token: copiar las carpetas `tools/lab` y `src/mobile_base` del repositorio a una memoria USB y
pegarlas en la Pi dentro de una carpeta `ucab-multirobot`, respetando la misma estructura.

Para actualizar el código más adelante: `cd ~/ucab-multirobot && git pull`.

### 4.5 Comprobación en seco (sin mover nada)

```bash
cd ~/ucab-multirobot
python3 tools/lab/lab_robot.py motors --dry-run --seconds 0.1
```

Debe imprimir `GPIO dry run` y la lista de pruebas sin errores. Si falla aquí, el problema es de
instalación (paso 4.2), no del robot.

---

## 5. Pruebas en el laboratorio, paso a paso

Todos los comandos se escriben en la sesión SSH, dentro de la carpeta del repositorio:

```bash
cd ~/ucab-multirobot
```

### Paso 0. Medir la batería

1. Con el multímetro en voltaje DC, medir la batería del HAT.
2. Anotar el valor (por ejemplo `8.1`). **Se usa en todos los pasos con motores** como `--battery 8.1`.

| Lectura | Qué significa |
|---|---|
| 7,4 a 8,4 V | Batería 2S en buen estado: seguir |
| Menos de 7,0 V | Batería descargada: cargarla antes de seguir |
| Más de 8,4 V | No es una batería 2S: **no conectar**, avisar al encargado |

### Paso 1. Motores, con las ruedas en el aire

1. Robot **apagado**: conectar la batería, poner el robot sobre el soporte con las ruedas en el aire.
2. Encender la Pi y entrar por SSH.
3. Correr (con el voltaje medido):
   ```bash
   python3 tools/lab/lab_robot.py motors --battery 8.1
   ```
4. La consola anuncia cada prueba. Mirar cada rueda y anotar:

| Prueba anunciada | Qué debe pasar |
|---|---|
| `rueda fl: adelante` | Solo la delantera izquierda gira, hacia adelante |
| `rueda fl: atras` | La misma, hacia atrás |
| `rueda fr` / `rueda rl` / `rueda rr` | Igual con la delantera derecha, trasera izquierda y trasera derecha |
| `robot: avanzar` | Las 4 ruedas giran para avanzar |
| `robot: lateral izquierda` | En diagonal: fl y rr hacia atrás, fr y rl hacia adelante (el robot se movería de lado a la izquierda) |
| `robot: girar antihorario` | Las ruedas de la izquierda hacia atrás, las de la derecha hacia adelante |

**Si algo no coincide:**

- **Una rueda gira al revés:** cambiar su signo en `wheel_signs` de
  `src/mobile_base/config/phineas.yaml` (orden: fl, fr, rl, rr; por ejemplo `[1, -1, 1, 1]`) y repetir.
- **Se mueve otra rueda que la anunciada:** los pares de pines `pins_fl`, `pins_fr`, `pins_rl`,
  `pins_rr` del mismo archivo no corresponden al cableado. Anotar cuál se movió.
- **No se mueve nada:** revisar que al arrancar no diga `GPIO dry run`, que diga `gpiochip N`, y que la
  batería del HAT esté conectada y encendida.

Para editar el archivo: `nano src/mobile_base/config/phineas.yaml` (en Ferb, `ferb.yaml`).

### Paso 2. Sensor de línea

```bash
python3 tools/lab/lab_robot.py line
```

Pasar el sensor (en la parte de abajo, adelante) sobre el blanco y sobre la cinta negra, a la altura a
la que queda con el robot apoyado.

| Debe decir | Si no |
|---|---|
| `blanco` sobre el tablero, `NEGRO (linea)` sobre la cinta | Si está invertido: cambiar `active_level` de `line_sensor` en el YAML (1 por 0) |
| | Si nunca cambia: ajustar el potenciómetro del módulo (el LED del módulo debe cambiar) y revisar que la salida esté en GPIO5 |

`Ctrl+C` para salir.

### Paso 3. Cámara y colores

1. Correr:
   ```bash
   python3 tools/lab/lab_robot.py camera
   ```
2. En la laptop, abrir en el navegador `http://IP:8080/` (la IP del paso 4.1).
3. Se ve la imagen de la cámara con un recuadro de color sobre cada cubo detectado, su color y su
   posición `cx` (-1 izquierda, +1 derecha) y `cy` (0 arriba, 1 abajo). La consola imprime lo mismo una
   vez por segundo y los cuadros por segundo (fps).
4. Poner **un cubo de cada color**, uno por uno, a unos 30 cm delante del robot, y anotar si se detecta
   con el color correcto.
5. Probar también con los 6 cubos a la vez y con la luz normal del laboratorio.

| Revisar | Bien | Anotar si no |
|---|---|---|
| fps en la consola | 10 o más | El valor |
| Cada color | Se detecta con su nombre correcto | Qué color falla y con qué se confunde (rojo y morado son los más cercanos) |
| Tablero y cinta | No aparecen recuadros sobre ellos | Dónde aparecen detecciones falsas |

### Paso 4. Calibración de la cámara (importante para la pala)

Con el modo cámara todavía abierto:

1. Medir la **altura de la cámara** desde la mesa y su **inclinación** aproximada; anotarlas.
2. Poner un cubo **centrado delante del robot**, con su centro a **20 cm del centro del robot** (donde
   quedará la boca de la pala). Anotar su `cy`.
3. Repetir con el cubo a 15 cm y a 25 cm y anotar los `cy`.

El `cy` a 20 cm, un poco menor, será el `capture_cy` del robot real (en Webots es 0,62, pero depende de
cómo quedó montada la cámara).

### Paso 5. Manejo con el teclado (en el piso)

1. Bajar el robot al piso o a la pista, con espacio libre alrededor.
2. Correr:
   ```bash
   python3 tools/lab/lab_robot.py drive --battery 8.1 --max-speed 0.08
   ```
3. Teclas: `w`/`s` avanzar y retroceder, `a`/`d` lateral izquierda y derecha, `q`/`e` girar a la
   izquierda y a la derecha, **espacio para parar**, `+`/`-` velocidad, `x` salir. Si no se pulsa
   ninguna tecla durante 3 s, el robot se detiene solo.
4. Comprobar que cada tecla mueve el robot en la dirección indicada y que avanza recto.

### Paso 6. Persecución autónoma de cubos

1. Poner el robot en la pista y un cubo de su color a 50–80 cm (Phineas: rojo, naranja o amarillo; Ferb:
   verde, azul o morado).
2. Correr (Phineas):
   ```bash
   python3 tools/lab/lab_robot.py chase --battery 8.1 --max-speed 0.08
   ```
   Para Ferb, agregar `--robot ferb`. Con el valor de `cy` del paso 4 se puede ajustar la parada:
   `--capture-cy 0.60` (por ejemplo).
3. Ver el video en `http://IP:8080/`: arriba a la izquierda muestra el estado y la orden a los motores.

| Debe pasar | Si no |
|---|---|
| Sin cubo a la vista: `EXPLORE`, avanza recto | Si no avanza: repetir el paso 1 |
| Al ver la cinta negra: se detiene y gira en el sitio | Si cruza la cinta: repetir el paso 2 |
| Al ver un cubo de su color: `APPROACH`, se orienta y se acerca | Si no lo ve: repetir el paso 3 con ese color |
| Al llegar: `CAPTURE`, se detiene a unos 20 cm del cubo | Si se detiene lejos o lo empuja: ajustar `--capture-cy` (más alto = se acerca más) |
| Ignora los cubos del otro robot | Anotar qué color confunde |

Repetir 3 veces con el cubo en distintas posiciones y anotar cuántas veces llegó bien.

## 6. Al terminar

1. `Ctrl+C` en la herramienta.
2. Apagar la Pi de forma ordenada:
   ```bash
   sudo poweroff
   ```
   Esperar a que el LED verde deje de parpadear.
3. **Después**, desconectar la batería.
4. Si se cambió algún YAML (`wheel_signs`, `active_level`, colores), copiar los valores a la tabla o
   guardar el archivo modificado:
   ```bash
   cd ~/ucab-multirobot && git diff
   ```
5. Enviar la tabla de resultados y, si hubo errores, el mensaje exacto de la consola (una foto de la
   pantalla sirve).

## 7. Tabla de resultados

Copiar y llenar (una por robot).

| Dato | Resultado |
|---|---|
| Robot (Phineas / Ferb) | |
| IP de la Pi | |
| Sistema operativo (`cat /etc/os-release`, línea `PRETTY_NAME`) | |
| `gpiochip` que mostró al arrancar | |
| Voltaje de la batería (V) | |
| Rueda fl: ¿gira y en qué sentido? | |
| Rueda fr | |
| Rueda rl | |
| Rueda rr | |
| `wheel_signs` final | |
| Avanzar / lateral / girar: ¿correcto? | |
| Sensor de línea: blanco / negro correcto, `active_level` final | |
| Cámara: conector (0/1), ¿hizo falta `dtoverlay`? | |
| Cámara: fps | |
| Colores detectados bien (de los 6) / confusiones | |
| Altura de la cámara (cm) e inclinación (°) | |
| `cy` del cubo a 15 / 20 / 25 cm | |
| Manejo con teclado: ¿direcciones correctas? | |
| Persecución: aciertos de 3, `capture_cy` usado | |
| Problemas y mensajes de error | |

## 8. Problemas frecuentes

| Síntoma o mensaje | Causa probable | Qué hacer |
|---|---|---|
| `lgpio not available: falling back to GPIO dry run` | Falta `python3-lgpio` | Paso 4.2 |
| `no pinctrl-rp1 gpiochip found` | La numeración del chip es distinta | `gpiodetect`, buscar la línea `pinctrl-rp1` y agregar `--chip N` con ese número |
| `Permission denied` al abrir el gpiochip | El usuario no está en el grupo `gpio` | Paso 4.2 (o, como último recurso, anteponer `sudo`) |
| `no camera backend worked` con `No module named 'picamera2'` | Falta `python3-picamera2` | Paso 4.2 |
| `no camera backend worked` con mensajes de cámara no disponible | Cable CSI mal conectado o falta el `dtoverlay` | Apagar, revisar el cable; paso 4.3 |
| El navegador no carga `http://IP:8080/` | Otra red, IP equivocada o el modo cámara no está corriendo | `hostname -I`, misma red Wi-Fi; probar `--port 8081` y abrir esa dirección |
| El robot no se detiene en la cinta | Sensor invertido, mal regulado o muy alto | Paso 2 |
| Un motor no gira en ningún paso | Cable del motor suelto o pines cambiados | Apagar, revisar conexión; anotar y seguir con el resto |
| La Pi se reinicia sola al mover los motores | Caída de voltaje (batería baja o mala conexión) | Cargar la batería, revisar conexiones, bajar `--max-speed` |

## 9. Referencia: conexiones

Pines del HAT Cokoino (numeración BCM), según `src/mobile_base/config/phineas.yaml`:

| Señal | Pines | Parámetro del YAML |
|---|---|---|
| nSLEEP de los dos DRV8833 (en alto) | 12, 13 | `nsleep_pins` |
| Motor delantero izquierdo (fl) | 17, 27 | `pins_fl` |
| Motor delantero derecho (fr) | 24, 25 | `pins_fr` |
| Motor trasero izquierdo (rl) | 22, 23 | `pins_rl` |
| Motor trasero derecho (rr) | 26, 16 | `pins_rr` |
| Sensor de línea (salida digital) | 5 | `pin` en `line_sensor` |

Los motores reciben PWM por software (`lgpio`) a 1 kHz, limitado para que el promedio no pase de 6 V
con el voltaje de batería indicado.
