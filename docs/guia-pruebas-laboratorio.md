# Guía de pruebas en el laboratorio: Raspberry Pi 5 y robot real

Procedimiento completo para preparar la Raspberry Pi 5 y probar el robot real en el laboratorio,
**sin pala y sin cámara cenital**. Se prueba el hardware (motores, sensor de línea, cámara) y que el
robot se acerque solo a los cubos de su color. Seguir los pasos en orden y llenar la
[tabla de resultados](#7-tabla-de-resultados) al final.

Hardware actual: Raspberry Pi 5 con **Ubuntu Server 26.04**, placa de motores **SunFounder Fusion HAT+**,
batería **Li-ion 2S de 7,4 V y 2000 mAh**, cámara IMX219 de 120° y un sensor de línea delantero.

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
| Qué rueda mueve cada puerto de motor (M0-M3) y en qué sentido | Capturar y transportar cubos (falta la pala) |
| El movimiento del robot: avanzar, lateral, girar | Entregar en el cuadro (falta la cámara cenital que da la posición) |
| El sensor de línea sobre blanco y sobre cinta negra | El programa completo con ROS 2 |
| La cámara IMX219 y la detección de los 6 colores | Esquivar al otro robot |
| Que el robot se acerque solo a un cubo de su color y se detenga | |

Sin la cámara cenital el robot no sabe dónde está: en el modo de persecución explora, gira al ver la
línea negra, va hacia el cubo y **se detiene** donde estaría la boca de la pala.

## 2. Materiales

- [ ] Robot armado: Raspberry Pi 5, **Fusion HAT+** con los 4 motores en sus puertos M0-M3, sensor de
      línea, cámara IMX219 con su cable CSI de 22 pines.
- [ ] Tarjeta microSD con **Ubuntu Server 26.04** para Raspberry Pi.
- [ ] Batería **Li-ion 2S de 7,4 V y 2000 mAh** (conector XH de 3 hilos) **cargada**, y su cargador.
- [ ] **Multímetro.**
- [ ] Laptop conectada a la **misma red Wi-Fi** que la Pi (para SSH y para ver la cámara en el navegador).
- [ ] Cubos de 2 x 2 cm de los 6 colores (rojo, naranja, amarillo, verde, azul, morado).
- [ ] Un trozo de tablero blanco con cinta negra (la pista o una muestra) para el sensor de línea.
- [ ] Algo para levantar el robot y dejar las ruedas en el aire (una caja o un soporte).
- [ ] Cinta métrica o regla y un transportador (inclinación de la cámara).
- [ ] Esta guía y la tabla de resultados (impresas o en la laptop).

## 3. Reglas de seguridad

- Trabajar **bajo la supervisión del encargado del laboratorio**.
- **Toda conexión o desconexión se hace con el equipo apagado y la batería desconectada**: cables de
  motores, sensor, cámara CSI y la propia batería.
- Antes de conectar la batería, revisar que no haya cables pelados, pines doblados ni metal suelto sobre
  la HAT (**cortocircuitos**).
- Usar **solo la batería 2S** (7,4 V nominal, **8,4 V cargada**): la Fusion HAT+ admite de 6,0 a 8,4 V
  en su entrada y alimenta desde ahí la Pi y los motores. Los motores TT son de 6 V: la herramienta
  limita la potencia con el voltaje que se le indique (`--battery`).
- La primera prueba de motores se hace **con las ruedas en el aire**.
- `Ctrl+C` detiene los motores en cualquier modo. Durante la persecución autónoma, una persona tiene que
  estar lista para levantar el robot.
- Si algo se calienta, huele a quemado o hace ruido raro: `Ctrl+C`, apagar y desconectar la batería.
- Los scripts de instalación que se descargan de internet se **leen antes de ejecutarlos**.

## 4. Preparar la Raspberry Pi (antes de ir al laboratorio)

Todo este paso se puede hacer en casa con la Pi conectada a una pantalla o por SSH. Toma unos 30–45 min
(la compilación del driver de la HAT es lo más lento).

> **Por verificar en Ubuntu Server 26.04.** Esta guía se escribió para Raspberry Pi OS y se adaptó a
> Ubuntu sin haberla probado todavía en 26.04. Lo marcado como *por verificar* hay que confirmarlo la
> primera vez y corregir la guía con lo que funcione (tarea 14 de [team-todo.md](team-todo.md)).

### 4.1 Red y acceso por SSH

1. Configurar el usuario, el Wi-Fi y SSH al grabar la tarjeta, con las opciones de personalización de
   Raspberry Pi Imager (*por verificar* con la imagen de Ubuntu Server 26.04; si no se ofrecen, conectar
   pantalla y teclado la primera vez).
2. Si SSH no responde, desde la consola de la Pi:
   ```bash
   sudo apt install -y openssh-server
   sudo systemctl enable --now ssh
   ```
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
sudo apt install -y git python3-lgpio python3-opencv python3-numpy python3-yaml gpiod i2c-tools
```

- `python3-lgpio` lee el sensor de línea (GPIO5). Los motores ya no usan los GPIO: van por la HAT.
- `gpiod` trae `gpiodetect` y `gpioinfo`; `i2c-tools` trae `i2cdetect` (para ver la HAT en el bus I2C).

Comprobar que el usuario puede abrir los pines:

```bash
ls -l /dev/gpiochip*
groups
```

Si el grupo dueño de `/dev/gpiochip*` no aparece en `groups`, agregar el usuario a ese grupo
(`sudo usermod -aG <grupo> $USER`), cerrar la sesión y volver a entrar.

### 4.3 Driver de la Fusion HAT+

La Fusion HAT+ necesita un driver del kernel y la biblioteca de Python `fusion_hat`. El instalador de
SunFounder **está hecho para Raspberry Pi OS**; en Ubuntu hay reportes de que la compilación del driver
falla si faltan los *headers* del kernel
([foro de SunFounder](https://forum.sunfounder.com/t/cannot-compile-driver-for-fusion-hat-on-pi5-running-ubuntu-24-04-with-install-script/4302)).
Todo este apartado está **por verificar en Ubuntu Server 26.04**.

1. Instalar primero los *headers* del kernel en uso:
   ```bash
   sudo apt install -y linux-headers-$(uname -r)
   ```
2. Descargar el instalador y **leerlo antes de ejecutarlo** (se ejecuta con `sudo`):
   ```bash
   curl -sSL -o install-fusion-hat.sh https://raw.githubusercontent.com/sunfounder/sunfounder-installer-scripts/main/install-fusion-hat.sh
   less install-fusion-hat.sh
   sudo bash install-fusion-hat.sh
   ```
   Es lo mismo que el comando de una línea del fabricante,
   `curl -sSL https://raw.githubusercontent.com/sunfounder/sunfounder-installer-scripts/main/install-fusion-hat.sh | sudo bash`,
   pero permite revisar qué hace antes de darle permisos de administrador.
3. Si la compilación del driver falla, recompilarlo a mano:
   ```bash
   cd ~/fusion-hat/driver && make && sudo make install
   ```
4. Reiniciar (`sudo reboot`) y comprobar:
   ```bash
   uname -a                       # versión del kernel
   cat /etc/os-release            # versión de Ubuntu
   i2cdetect -y 1                 # la HAT debe aparecer en la dirección 0x17
   dmesg | grep fusion_hat        # mensajes del driver
   lsmod | grep fusion_hat        # el módulo del driver está cargado
   python3 -c "from fusion_hat.motor import Motor; print('fusion_hat ok')"
   ```

Anotar en la tabla de resultados la salida de estos comandos si algo falla. Interfaces y pines de la
HAT: [documentación de SunFounder](https://docs.sunfounder.com/projects/fusion-hat/en/latest/hardware/interfaces.html).

### 4.4 Cámara (por verificar en Ubuntu)

1. **Con la Pi apagada**, conectar el cable CSI de la cámara a uno de los conectores de cámara de la Pi 5
   (el cable de 22 pines del módulo Pi5-IMX219), con los contactos del lado correcto según el manual
   del módulo. Anotar en qué conector quedó (CAM/DISP 0 o 1).
2. **Pendiente de verificar en Ubuntu Server 26.04:** todavía no se comprobó qué paquetes dan acceso a la
   cámara CSI en esta versión, así que no se indican aquí. La herramienta prueba, en este orden, tres
   vías: **picamera2**, **GStreamer con `libcamerasrc`** y **V4L2** (una cámara USB sirve como
   alternativa). La primera que entregue imágenes es la que usa; el paso 3 del laboratorio dice cuál.
3. Cuando se encuentre la forma de instalarla, documentarla en esta sección (tarea 4 de
   [team-todo.md](team-todo.md)).

### 4.5 Descargar el código

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

### 4.6 Comprobación en seco (sin mover nada)

```bash
cd ~/ucab-multirobot
python3 tools/lab/lab_robot.py motors --dry-run --seconds 0.1
```

Debe imprimir `backend fusion_hat ... dry run True` y la lista de pruebas con el puerto de cada rueda,
sin errores. Si falla aquí, el problema es de instalación (paso 4.2), no del robot.

---

## 5. Pruebas en el laboratorio, paso a paso

Todos los comandos se escriben en la sesión SSH, dentro de la carpeta del repositorio:

```bash
cd ~/ucab-multirobot
```

### Paso 0. Medir la batería

1. Con el multímetro en voltaje DC, medir la batería (7,4 V nominal, 8,4 V cargada).
2. Anotar el valor (por ejemplo `8.1`). **Se usa en todos los pasos con motores** como `--battery 8.1`:
   con ese valor la herramienta calcula el tope de potencia que mantiene los motores en 6 V o menos
   (con 8,4 V el tope es 6,0 / 8,4 = 71 %).

| Lectura | Qué significa |
|---|---|
| 7,4 a 8,4 V | Batería 2S en buen estado: seguir |
| Menos de 7,0 V | Batería descargada: cargarla antes de seguir |
| Más de 8,4 V | No es una batería 2S: **no conectar**, avisar al encargado |

### Paso 1. Motores: qué rueda mueve cada puerto (ruedas en el aire)

Todavía no se sabe qué rueda está conectada a cada puerto M0-M3. Este paso lo averigua.

1. Robot **apagado**: conectar la batería, poner el robot sobre el soporte con las ruedas en el aire.
2. Encender la Pi y entrar por SSH.
3. Correr (con el voltaje medido):
   ```bash
   python3 tools/lab/lab_robot.py motors --battery 8.1
   ```
   Al arrancar dice `backend fusion_hat` y `dry run False`. El aviso `GPIO dry run` es normal: los
   motores no usan los GPIO de la Pi.
4. La consola anuncia cada prueba con el puerto que mueve, por ejemplo `rueda fl (puerto M0): adelante`.
   Con la configuración inicial, `fl` es M0, `fr` es M1, `rl` es M2 y `rr` es M3. Para cada anuncio,
   mirar **qué rueda física gira y hacia dónde**, y anotarlo:

| Anuncio | Rueda que giró (fl, fr, rl o rr) | ¿Hacia adelante cuando dice `adelante`? |
|---|---|---|
| `rueda fl (puerto M0)` | | |
| `rueda fr (puerto M1)` | | |
| `rueda rl (puerto M2)` | | |
| `rueda rr (puerto M3)` | | |

5. Con esa tabla, editar `hat_motor_driver` en `src/mobile_base/config/phineas.yaml` (en Ferb,
   `ferb.yaml`) con `nano`:
   - `fusion_ports`: en el orden `[fl, fr, rl, rr]`, el puerto que movió cada rueda. Ejemplo: si M0
     movió la trasera derecha, M1 la delantera izquierda, M2 la delantera derecha y M3 la trasera
     izquierda, queda `fusion_ports: [M1, M2, M3, M0]`.
   - `fusion_reversed`: `true` en la posición de cada rueda que giró hacia atrás cuando el anuncio decía
     `adelante`. Ejemplo: `[false, true, false, true]`.
6. Repetir el paso 3: ahora cada anuncio debe mover la rueda que nombra, hacia adelante y luego atrás.
   Después la consola prueba el robot completo:

| Prueba anunciada | Qué debe pasar |
|---|---|
| `robot: avanzar` | Las 4 ruedas giran para avanzar |
| `robot: lateral izquierda` | En diagonal: fl y rr hacia atrás, fr y rl hacia adelante (el robot se movería de lado a la izquierda) |
| `robot: girar antihorario` | Las ruedas de la izquierda hacia atrás, las de la derecha hacia adelante |

**Si no se mueve nada:** revisar que al arrancar no diga `fusion_hat not available` (paso 4.3), que
`i2cdetect -y 1` muestre `17`, y que la batería esté conectada y la HAT encendida.

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
   La consola dice qué vía de cámara funcionó (`camera backend: ...`). Anotarla.
2. En la laptop, abrir en el navegador `http://IP:8080/` (la IP del paso 4.1).
3. Se ve la imagen de la cámara con un recuadro de color sobre cada cubo detectado, su color y su
   posición `cx` (-1 izquierda, +1 derecha) y `cy` (0 arriba, 1 abajo). La consola imprime lo mismo una
   vez por segundo y los cuadros por segundo (fps).
4. Poner **un cubo de cada color**, uno por uno, a unos 30 cm delante del robot, y anotar si se detecta
   con el color correcto.
5. Probar también con los 6 cubos a la vez y con la luz normal del laboratorio.
6. Poner la cámara frente al **cuadro azul** de la pista: **no** debe aparecer como cubo azul.

| Revisar | Bien | Anotar si no |
|---|---|---|
| fps en la consola | 10 o más | El valor |
| Cada color | Se detecta con su nombre correcto | Qué color falla y con qué se confunde (rojo y morado son los más cercanos) |
| Tablero y cinta | No aparecen recuadros sobre ellos | Dónde aparecen detecciones falsas |
| Cuadro azul | No aparece como cubo | Desde qué distancia aparece como cubo |

### Paso 4. Montaje y calibración de la cámara (importante para la pala)

Hoy la cámara mira casi horizontal. Para que la pala y el cubo en su boca entren en la imagen, hay que
inclinarla unos **25-30° hacia abajo** (tarea 15 de [team-todo.md](team-todo.md)).

1. Medir la **altura de la cámara** desde la mesa, **cuánto queda detrás del frente** del robot y su
   **inclinación**; anotarlas antes y después de inclinarla.
2. Con el modo cámara abierto, poner un cubo **centrado delante del robot**, con su centro a **20 cm del
   centro del robot** (donde quedará la boca de la pala). Anotar su `cy`.
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
| Ignora los cubos del otro robot y el cuadro azul | Anotar qué confunde |

Repetir 3 veces con el cubo en distintas posiciones y anotar cuántas veces llegó bien.

## 6. Al terminar

1. `Ctrl+C` en la herramienta.
2. Apagar la Pi de forma ordenada:
   ```bash
   sudo poweroff
   ```
   Esperar a que el LED verde deje de parpadear.
3. **Después**, desconectar la batería.
4. Si se cambió algún YAML (`fusion_ports`, `fusion_reversed`, `active_level`, colores), copiar los
   valores a la tabla o guardar el archivo modificado:
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
| Sistema operativo (`cat /etc/os-release`, línea `PRETTY_NAME`) y kernel (`uname -a`) | |
| Driver de la HAT: ¿hizo falta recompilar? ¿`i2cdetect` muestra 0x17? | |
| `gpiochip` que mostró el modo `line` | |
| Voltaje de la batería (V) | |
| Puerto M0: rueda y sentido | |
| Puerto M1 | |
| Puerto M2 | |
| Puerto M3 | |
| `fusion_ports` y `fusion_reversed` finales | |
| Avanzar / lateral / girar: ¿correcto? | |
| Sensor de línea: blanco / negro correcto, `active_level` final | |
| Cámara: conector (0/1) y vía que funcionó (picamera2, GStreamer o V4L2) | |
| Cámara: fps | |
| Colores detectados bien (de los 6) / confusiones / ¿el cuadro azul aparece como cubo? | |
| Altura de la cámara (cm), retroceso desde el frente (cm) e inclinación (°) | |
| `cy` del cubo a 15 / 20 / 25 cm | |
| Manejo con teclado: ¿direcciones correctas? | |
| Persecución: aciertos de 3, `capture_cy` usado | |
| Problemas y mensajes de error | |

## 8. Problemas frecuentes

| Síntoma o mensaje | Causa probable | Qué hacer |
|---|---|---|
| `fusion_hat not available: motors in dry run` | Falta la biblioteca de la HAT | Paso 4.3 |
| `i2cdetect -y 1` no muestra `17` | La HAT no está bien asentada, está apagada o el driver no cargó | Apagar, revisar la HAT; `dmesg \| grep fusion_hat`, `lsmod \| grep fusion_hat` |
| La compilación del driver falla al instalar | Faltan los *headers* del kernel (Ubuntu) | `sudo apt install linux-headers-$(uname -r)` y `cd ~/fusion-hat/driver && make && sudo make install` |
| `lgpio not available: falling back to GPIO dry run` en el modo `line` | Falta `python3-lgpio` | Paso 4.2 |
| `no pinctrl-rp1 gpiochip found` | La numeración del chip es distinta | `gpiodetect`, buscar la línea `pinctrl-rp1` y agregar `--chip N` con ese número |
| `Permission denied` al abrir el gpiochip | El usuario no está en el grupo dueño de `/dev/gpiochip*` | Paso 4.2 (o, como último recurso, anteponer `sudo`) |
| `no camera backend worked` | Ninguna de las tres vías de cámara funciona todavía en Ubuntu | Paso 4.4; probar con una cámara USB (V4L2) |
| El navegador no carga `http://IP:8080/` | Otra red, IP equivocada o el modo cámara no está corriendo | `hostname -I`, misma red Wi-Fi; probar `--port 8081` y abrir esa dirección |
| El robot no se detiene en la cinta | Sensor invertido, mal regulado o muy alto | Paso 2 |
| Un puerto no mueve ninguna rueda | Cable del motor suelto | Apagar, revisar conexión; anotar y seguir con el resto |
| La Pi se reinicia sola al mover los motores | Caída de voltaje (batería baja o mala conexión) | Cargar la batería, revisar conexiones, bajar `--max-speed` |

## 9. Referencia: conexiones

Pines de la Raspberry Pi (numeración BCM) que usa la **Fusion HAT+**, según la
[documentación de SunFounder](https://docs.sunfounder.com/projects/fusion-hat/en/latest/hardware/interfaces.html):

| Uso | Pines BCM |
|---|---|
| I2C con el microcontrolador de la HAT (dirección 0x17) | 2, 3 |
| Audio | 18, 19, 20, 21 |
| Entradas y salidas propias de la HAT | 17, 4, 27, 22 |
| Detección de apagado | 26 |
| **Sensor de línea** (salida digital, libre: la HAT no lo usa) | **5** (`pin` en `line_sensor`) |

Los motores van a los puertos **M0-M3** de la HAT (conectores XH2.54). Su microcontrolador genera el PWM
(100 Hz por defecto); la herramienta y el nodo `hat_motor_driver` le envían una potencia de -100 a 100 %,
limitada para que el promedio en el motor no pase de 6 V con el voltaje de batería indicado. Qué rueda
va en cada puerto se configura con `fusion_ports` y `fusion_reversed` (paso 1).

El HAT anterior (Cokoino, 2 x DRV8833 por `lgpio`) sigue disponible con
`motor_backend: cokoino_drv8833`; sus pines están en [architecture.md](architecture.md).
