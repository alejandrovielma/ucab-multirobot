"""Control manual del recolector para probar el modelo en Webots.

No es la lógica del robot: sirve para comprobar que motores, pala y sensores
responden como el robot real antes de escribir los nodos de ROS 2.

Teclas (haz clic en la vista 3D primero):
  W / S        avanzar / retroceder
  A / D        moverse de lado a la izquierda / derecha (ruedas mecanum)
  Q / E        girar a la izquierda / derecha
  P            subir / bajar la pala
  + / -        más / menos velocidad
  espacio      detener
"""

import colorsys
import sys

from controller import Keyboard, Robot

# Medidas del chasis Hiwonder (ver RecolectorUCAB.proto)
RADIO_RUEDA = 0.0325       # m
LX = 0.0605                # m, mitad de la distancia entre ejes
LY = 0.0655                # m, mitad de la trocha
VEL_MAX_MOTOR = 10.5       # rad/s, TT 1:120 a ~6 V
LIMITE_2S = 0.7            # con batería 2S se limita al ~70 % para no forzar los TT de 6 V

PALA_ABAJO = 0.0
PALA_ARRIBA = -0.7         # rad (~40 grados)

UMBRAL_LINEA = 120         # TCRT5000 simulado: blanco ~240, linea negra ~0-30


def leer_colores(args):
    for a in args:
        if a.startswith("--colores="):
            return a.split("=", 1)[1].split(",")
    return []


def color_por_tono(r, g, b):
    """Clasificación sencilla por tono, solo para mostrar en consola."""
    h, s, v = colorsys.rgb_to_hsv(r / 255, g / 255, b / 255)
    if s < 0.25 or v < 0.15:
        return "ninguno"
    h *= 360
    if h < 15 or h >= 330:
        return "rojo" if v > 0.45 else "morado"
    if h < 40:
        return "naranja"
    if h < 70:
        return "amarillo"
    if h < 170:
        return "verde"
    if h < 260:
        return "azul"
    return "morado"


def main():
    robot = Robot()
    paso = int(robot.getBasicTimeStep())
    nombre = robot.getName()
    mis_colores = leer_colores(sys.argv[1:])

    motores = {}
    for n in ("fl", "fr", "rl", "rr"):
        m = robot.getDevice(f"motor_{n}")
        m.setPosition(float("inf"))
        m.setVelocity(0.0)
        motores[n] = m
    servo = robot.getDevice("servo_pala")
    servo.setPosition(PALA_ABAJO)

    lineas = {}
    for n in ("fl", "fr", "rl", "rr"):
        s = robot.getDevice(f"linea_{n}")
        s.enable(paso)
        lineas[n] = s
    sonar = robot.getDevice("ultrasonido")
    sonar.enable(paso)
    ir_pala = robot.getDevice("ir_pala")
    ir_pala.enable(paso)
    color_pala = robot.getDevice("color_pala")
    color_pala.enable(paso * 4)
    camara = robot.getDevice("camara")
    camara.enable(paso * 4)
    acel = robot.getDevice("acelerometro")
    acel.enable(paso)
    giro = robot.getDevice("giroscopio")
    giro.enable(paso)

    teclado = Keyboard()
    teclado.enable(paso)

    escala = 0.5
    pala_arriba = False
    p_previa = False
    rumbo = 0.0  # integrado del giroscopio: deriva con el tiempo, como el MPU6050 real
    t_ultimo = 0.0

    print(f"[{nombre}] listo. Mis colores: {', '.join(mis_colores) or 'sin asignar'}")

    while robot.step(paso) != -1:
        vx = vy = w = 0.0
        p_ahora = False
        tecla = teclado.getKey()
        while tecla != -1:
            c = chr(tecla & 0xFF).upper() if (tecla & 0xFF) < 128 else ""
            if c == "W" or tecla == Keyboard.UP:
                vx = 1.0
            elif c == "S" or tecla == Keyboard.DOWN:
                vx = -1.0
            elif c == "A":
                vy = 1.0
            elif c == "D":
                vy = -1.0
            elif c == "Q" or tecla == Keyboard.LEFT:
                w = 1.0
            elif c == "E" or tecla == Keyboard.RIGHT:
                w = -1.0
            elif c == "P":
                p_ahora = True
            elif c == "+":
                escala = min(1.0, escala + 0.01)
            elif c == "-":
                escala = max(0.1, escala - 0.01)
            tecla = teclado.getKey()

        if p_ahora and not p_previa:
            pala_arriba = not pala_arriba
            servo.setPosition(PALA_ARRIBA if pala_arriba else PALA_ABAJO)
        p_previa = p_ahora

        # Cinemática inversa mecanum (rodillos en X). vx, vy en m/s, w en rad/s.
        v_lin = VEL_MAX_MOTOR * LIMITE_2S * RADIO_RUEDA * escala
        vx *= v_lin
        vy *= v_lin
        w *= v_lin / (LX + LY)
        k = LX + LY
        ruedas = {
            "fl": (vx - vy - k * w) / RADIO_RUEDA,
            "fr": (vx + vy + k * w) / RADIO_RUEDA,
            "rl": (vx + vy - k * w) / RADIO_RUEDA,
            "rr": (vx - vy + k * w) / RADIO_RUEDA,
        }
        tope = VEL_MAX_MOTOR * LIMITE_2S
        for n, v in ruedas.items():
            motores[n].setVelocity(max(-tope, min(tope, v)))

        dt = paso / 1000.0
        rumbo += giro.getValues()[2] * dt

        t = robot.getTime()
        if t - t_ultimo >= 0.5:
            t_ultimo = t
            negro = [n for n, s in lineas.items() if s.getValue() < UMBRAL_LINEA]
            img = color_pala.getImage()
            r = g = b = 0
            if img:
                ancho, alto = color_pala.getWidth(), color_pala.getHeight()
                for yy in range(alto):
                    for xx in range(ancho):
                        r += color_pala.imageGetRed(img, ancho, xx, yy)
                        g += color_pala.imageGetGreen(img, ancho, xx, yy)
                        b += color_pala.imageGetBlue(img, ancho, xx, yy)
                npx = ancho * alto
                r, g, b = r // npx, g // npx, b // npx
            hay_cubo = ir_pala.getValue() > 0.5
            color = color_por_tono(r, g, b) if hay_cubo else "-"
            mio = "sí" if color in mis_colores else "no"
            print(
                f"[{nombre}] t={t:5.1f}s vel={escala:.2f} pala={'arriba' if pala_arriba else 'abajo'} "
                f"linea_negra={negro or 'ninguna'} sonar={sonar.getValue():.2f} m "
                f"cubo_en_pala={'sí' if hay_cubo else 'no'} color={color}"
                + (f" (mío: {mio})" if hay_cubo else "")
                + f" rumbo_giro={rumbo * 57.3:6.1f}°"
            )


if __name__ == "__main__":
    main()
