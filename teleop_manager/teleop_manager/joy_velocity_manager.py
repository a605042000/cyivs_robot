#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray

class JoyVelocityManager(Node):
    def __init__(self):
        super().__init__('joy_velocity_manager')
        
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.servo_pub = self.create_publisher(Int32MultiArray, '/servo', 10)
        
        # 初始速度放大倍率
        self.linear_scale = 0.4
        self.angular_scale = 0.8
        
        # 步進值
        self.linear_step = 0.05
        self.angular_step = 0.1

        self.lift_angle = 0
        self.gripper_angle = 180
        self.lift_step = 5
        self.min_lift_angle = 0
        self.max_lift_angle = 200
        self.gripper_close_angle = 130
        self.gripper_open_angle = 180
        
        self.last_buttons = [0] * 8
        self.get_logger().info(f'雙模同步控制器已啟動！類比搖桿與十字鍵皆可行駛。')

    def joy_callback(self, msg):
        if len(msg.buttons) < 8 or len(msg.axes) < 8:
            return

        # --- 偵測 L1/L2, R1/R2 增減速度上限 (維持原有機制) ---
        if msg.buttons[4] == 1 and self.last_buttons[4] == 0:
            self.linear_scale += self.linear_step
            self.get_logger().info(f'📈 線性速度上限: {self.linear_scale:.2f} m/s')
        if msg.buttons[6] == 1 and self.last_buttons[6] == 0:
            self.linear_scale = max(0.0, self.linear_scale - self.linear_step)
            self.get_logger().info(f'📉 線性速度上限: {self.linear_scale:.2f} m/s')
        if msg.buttons[5] == 1 and self.last_buttons[5] == 0:
            self.angular_scale += self.angular_step
            self.get_logger().info(f'📈 旋轉速度上限: {self.angular_scale:.2f} rad/s')
        if msg.buttons[7] == 1 and self.last_buttons[7] == 0:
            self.angular_scale = max(0.0, self.angular_scale - self.angular_step)
            self.get_logger().info(f'📉 旋轉速度上限: {self.angular_scale:.2f} rad/s')

        servo_changed = False

        if msg.buttons[0] == 1:
            self.lift_angle = max(self.min_lift_angle, self.lift_angle - self.lift_step)
            servo_changed = True
        if msg.buttons[2] == 1:
            self.lift_angle = min(self.max_lift_angle, self.lift_angle + self.lift_step)
            servo_changed = True
        if msg.buttons[3] == 1 and self.gripper_angle != self.gripper_close_angle:
            self.gripper_angle = self.gripper_close_angle
            servo_changed = True
        if msg.buttons[1] == 1 and self.gripper_angle != self.gripper_open_angle:
            self.gripper_angle = self.gripper_open_angle
            servo_changed = True

        self.last_buttons = list(msg.buttons[:8])

        # --- 核心修改：同步讀取「類比搖桿」與「十字鍵」 ---
        # 1. 線性速度 (前後前進)
        joy_linear = msg.axes[1]  # 左類比搖桿 上/下
        dpad_linear = msg.axes[7] # 十字鍵 上/下
        
        # 2. 旋轉速度 (左右轉彎)
        joy_angular = msg.axes[0] # 左類比搖桿 左/右
        dpad_angular = msg.axes[6]# 十字鍵 左/右

        # 混合邏輯：如果十字鍵有被按住(不等於0)，優先用十字鍵；否則採用類比搖桿的數值
        final_linear_input = dpad_linear if dpad_linear != 0.0 else joy_linear
        final_angular_input = dpad_angular if dpad_angular != 0.0 else joy_angular

        # --- 計算並發布 cmd_vel ---
        twist = Twist()
        twist.linear.x = final_linear_input * self.linear_scale
        twist.angular.z = final_angular_input * self.angular_scale
        
        self.cmd_pub.publish(twist)

        if servo_changed:
            servo_msg = Int32MultiArray()
            servo_msg.data = [self.lift_angle, self.gripper_angle]
            self.servo_pub.publish(servo_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoyVelocityManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
