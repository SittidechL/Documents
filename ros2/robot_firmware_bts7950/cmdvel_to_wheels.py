
#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class CmdVelToWheelRPM(Node):
    def __init__(self):
        super().__init__('cmdvel_to_wheels')

        # ---- พารามิเตอร์ ปรับตามรถจริง ----
        # รัศมีล้อ (เมตร)
        self.declare_parameter('wheel_radius', 0.05)    # ตัวอย่าง: R = 0.05 m (ล้อเส้นผ่านศูนย์กลาง 10 ซม.)
        # ระยะฐานล้อ (เมตร) ระหว่างจุดสัมผัสล้อซ้าย-ขวา
        self.declare_parameter('track_width', 0.30)     # ตัวอย่าง: L = 0.30 m

        # จำกัดรอบสูงสุด (กันตั้งค่ามากเกินไป)
        self.declare_parameter('rpm_limit', 200.0)

        # Topic names
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('left_topic', 'wheel_command_left')
        self.declare_parameter('right_topic', 'wheel_command_right')

        # อ่านพารามิเตอร์
        self.R = float(self.get_parameter('wheel_radius').value)
        self.L = float(self.get_parameter('track_width').value)
        self.RPM_LIMIT = float(self.get_parameter('rpm_limit').value)

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        left_topic    = self.get_parameter('left_topic').value
        right_topic   = self.get_parameter('right_topic').value

        self.sub = self.create_subscription(Twist, cmd_vel_topic, self.cb_cmdvel, 10)
        self.pub_left  = self.create_publisher(Float32, left_topic, 10)
        self.pub_right = self.create_publisher(Float32, right_topic, 10)

        self.get_logger().info(
            f"CmdVelToWheelRPM started. R={self.R:.3f} m, L={self.L:.3f} m, limit={self.RPM_LIMIT} rpm\n"
            f"Sub: {cmd_vel_topic}, Pub: {left_topic}, {right_topic}"
        )

    def cb_cmdvel(self, msg: Twist):
        v = float(msg.linear.x)     # m/s
        w = float(msg.angular.z)    # rad/s

        # คำนวณความเร็วล้อ m/s
        v_L = v - (w * self.L / 2.0)
        v_R = v + (w * self.L / 2.0)

        # แปลงเป็น RPM
        if self.R <= 0.0:
            self.get_logger().warn("wheel_radius <= 0, ignore command")
            return

        rpm_L = (v_L / (2.0 * math.pi * self.R)) * 60.0
        rpm_R = (v_R / (2.0 * math.pi * self.R)) * 60.0

        # จำกัดรอบ
        rpm_L = max(-self.RPM_LIMIT, min(self.RPM_LIMIT, rpm_L))
        rpm_R = max(-self.RPM_LIMIT, min(self.RPM_LIMIT, rpm_R))

        # ส่งไปที่ Teensy
        msgL = Float32(); msgL.data = float(rpm_L)
        msgR = Float32(); msgR.data = float(rpm_R)
        self.pub_left.publish(msgL)
        self.pub_right.publish(msgR)

def main():
    rclpy.init()
    node = CmdVelToWheelRPM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
