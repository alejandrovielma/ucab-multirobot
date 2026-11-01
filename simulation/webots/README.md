# Collector simulation in Webots

Webots world with the UCAB Robotics track, the 14 cubes and the two collectors (Phineas and Ferb) modelled according to the plan. It is used to test movements, sensors and logic before the robot is built.

Tested with **Webots R2025a**: it loads without errors, the mecanum wheels drive forward, sideways and rotate, the scoop pushes and holds a cube, and the sensors respond.

## Contents

```
webots-recolector/
├── worlds/pista_ucab.wbt              world: table, board, line, square, cubes and 2 robots
├── worlds/textures/pista_ucab.png      black line taken from the track photo (rectified)
├── protos/RecolectorUCAB.proto         the robot (chassis, wheels, scoop and sensors)
├── protos/CuboMadera.proto             2.5 cm wooden cube
├── controllers/recolector_teleop/      keyboard control to test the model
└── vista_cenital.png                   top-down capture of the world
```

## How to open it

1. Install Webots R2025a from https://cyberbotics.com (Windows, Linux or macOS).
2. Install Python 3 if you do not have it. Webots uses it for the controllers; on Windows, check in *Tools → Preferences → Python command* that it points to your `python`.
3. Open `worlds/pista_ucab.wbt` with *File → Open World*.
4. Press ▶ (play). Click on the 3D view and use the keyboard.

| Key | Action |
|---|---|
| W / S | forward / backward |
| A / D | move sideways (left / right) |
| Q / E | rotate left / right |
| P | raise / lower the scoop |
| + / − | more / less speed |

Both robots listen to the same keyboard. To drive only one, set the other's `controller` field to `<none>` in the scene tree. In the Webots console, each robot prints every half second: whether it sees the black line, the ultrasonic distance, whether it has a cube in the scoop, its colour and whether it is one of its colours.

## How faithful it is to the real robot

### Manufacturer data or from the photos
- **Hiwonder mecanum chassis:** 186 × 161 mm, 65 mm wheels, TT 1:120 motors **without encoders** (the model has no position sensors on the wheels, just like the real one).
- **Speed:** ~100 rpm per motor (TT 1:120 at ~6 V) and a 70 % limit because of the 2S battery → ~0.24 m/s at most.
- **Mecanum wheels:** simulated with directional friction at 45° (the same technique as the Webots KUKA youBot). They slip somewhat when rotating, like the real ones.
- **Track:** the black line was taken from the photo by rectifying the perspective. The 19 mm width matches electrical tape.
- **Colour split from the plan:** Phineas `--colors=red,orange,yellow` and Ferb `--colors=green,blue,purple`, passed in `controllerArgs`.

### Estimates (measure and adjust)
| Measurement | Value in the model | Where to change it |
|---|---|---|
| Board | 1.93 × 1.22 m | `PISTA` in the `.wbt` (and the texture) |
| Delivery square | ~24 × 24 cm | comes from the texture |
| Cubes | 2.5 cm, 9.4 g | `CuboMadera.proto` |
| Robot mass | ~0.89 kg in total | `masaCuerpo` (body mass) on the robot |
| TT torque | 0.15 N·m | `torqueMaxMotor` |
| Scoop | 32 mm U, 2.5 mm above the floor | `RecolectorUCAB.proto`, "Pala" (scoop) section |
| Camera | 24° downwards, 11 cm high | `RecolectorUCAB.proto`, `Camera "camara"` |

The board measurements come from comparing the tape width in both directions of the photo. If the real board measures something else, let me know and I will adjust the texture.

## Robot devices

These names (Spanish, kept as is) are the ones the code uses, in Webots and later in ROS 2.

| Name | Real part | What it returns |
|---|---|---|
| `motor_fl`, `motor_fr`, `motor_rl`, `motor_rr` | TT motors through the Cokoino HAT | velocity controlled (rad/s) |
| `servo_pala` (scoop servo) | SG90 | position: 0 = down, −0.7 rad = up |
| `camara` (camera) | OV5647, 640×480, ~54° | image |
| `linea_fl`, `linea_fr`, `linea_rl`, `linea_rr` (line sensors) | TCRT5000 | ~240 on white, 0–30 on the black line |
| `ultrasonido` (ultrasonic) | HC-SR04 | distance in metres (0.02 to 4) |
| `ir_pala` (scoop IR) | FC-51 | 1 if there is a cube in the scoop, 0 otherwise |
| `color_pala` (scoop colour) | TCS34725 | 4×4 image of the cube (average it to get the RGB) |
| `acelerometro`, `giroscopio` (accelerometer, gyroscope) | MPU6050 | m/s² and rad/s; there is no compass, the heading is integrated and drifts |

On top of each robot there is an ArUco marker (DICT_4X4_50: Phineas id 1, Ferb id 2), and next to the square there is another (id 10). The square's marker depends on the rules allowing it.

## What the simulator does not reproduce well
- The exact slipping of the mecanum wheels on the real board.
- The colours under the room's lighting: the HSV ranges are calibrated with the real cubes.
- The battery voltage drop and motor heating.
- The claw robot (Perry) is not modelled; the square stays free.

## Connection with ROS 2
`worlds/pista_ucab_ros.wbt` is the same world with Ferb (our robot) on `<extern>` and Phineas (the other collector) idle. Instead of `webots_ros2`, a small custom bridge is used, `webots_bridge` (`src/recolector_sim`): it publishes `image_raw` and `range`, and drives the wheels from `cmd_vel`, just like the real robot's nodes. That way, what works in simulation moves to the Pi 5 by changing only the nodes that talk to the hardware.

- Everything in Docker: `docker compose up sim` (headless Webots) and open http://localhost:8080.
- Webots on Windows and nodes in Docker: open `pista_ucab_ros.wbt`, press ▶ and run `docker compose up sim-external`.

See `docker/README.md`. If you change `pista_ucab.wbt`, run `python3 tools/generate_ros_world.py`.
