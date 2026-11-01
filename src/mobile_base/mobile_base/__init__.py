"""
Collector mecanum base: motion, safety filters, behaviour and fleet link.

Nodes: behavior_fsm, obstacle_guard, fleet_link, motor_driver, line_sensors,
ultrasonic, scoop_driver, color_sensor. Pure logic lives in modules without
rclpy (approach, collector, speed_limits, fleet, peer_avoidance, kinematics,
scoop_color, mathutil); hardware access in motor_hardware and
sensor_hardware. See docs/architecture.md, section "Code layout".
"""
