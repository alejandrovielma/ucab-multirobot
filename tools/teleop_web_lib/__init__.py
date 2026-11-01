"""
Modules behind tools/teleop_web.py: browser control, camera video and the
Auto mode, without ROS.

  kinematics   mecanum wheel ratios
  motors       fake, GPIO HAT and Fusion HAT+ motor drivers
  line_sensor  bottom TCRT5000 line sensor
  config       YAML loading and the typed Auto settings
  autopilot    Auto mode state machine (pure logic)
  drive        control thread: manual command, brake timeout, Auto
  frame_source camera backends (fake, picamera2, OpenCV)
  vision       cube and delivery-line detection on top of the camera
  camera       capture thread that keeps the latest JPEG
  web_server   HTTP handler (page, MJPEG stream, /cmd)
"""
