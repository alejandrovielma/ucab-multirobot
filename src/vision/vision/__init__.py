"""
Collector camera and colour-based cube detection.

Nodes: camera_node, cube_detector, web_viewer. Pure logic lives in modules
without rclpy (hsv, camera_model, color_detection, teleop, web_server);
see docs/architecture.md, section "Code layout".
"""
