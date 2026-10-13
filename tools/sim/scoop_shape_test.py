"""One scoop shape: spin CCW 5 s, spin CW 5 s, reverse 3 s (no gate). Prints scoop fill after each."""
import sys, time, numpy as np, rclpy, cv2
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from mobile_base.vision import VisionParams, scoop_content
label, out = sys.argv[1], sys.argv[2]
rclpy.init(); n = Node("shape_test"); st = {"img": None}
pub = n.create_publisher(Twist, "/phineas/cmd_vel", 1)
def cb(m):
    ch = 4 if m.encoding == "bgra8" else 3
    st["img"] = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, ch)[:, :, :3].copy()
n.create_subscription(Image, "/phineas/camera/image_raw", cb, 1)
def run(sec, vx=0.0, wz=0.0, tag=""):
    end = time.monotonic() + sec
    while time.monotonic() < end:
        t = Twist(); t.linear.x = vx; t.angular.z = wz; pub.publish(t); rclpy.spin_once(n, timeout_sec=0.05)
    c, f = scoop_content(st["img"], VisionParams()) if st["img"] is not None else ("?", -1)
    if st["img"] is not None: cv2.imwrite(f"{out}/{label}_{tag}.png", st["img"])
    return f
r = [run(2.0, tag="start"), run(5.0, wz=1.2, tag="ccw"), run(1.0, tag="s1"), run(5.0, wz=-1.2, tag="cw"), run(1.0, tag="s2"), run(3.0, vx=-0.04, tag="rev")]
print(f"RESULT {label} start={r[0]:.2f} after_ccw={r[2]:.2f} after_cw={r[4]:.2f} after_reverse={r[5]:.2f}", flush=True)
rclpy.shutdown()
