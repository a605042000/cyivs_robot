#!/usr/bin/env python3
import math
from enum import Enum
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def clamp(value: float, min_value: float, max_value: float) -> float:
	return max(min_value, min(max_value, value))


def limit_rate(target: float, current: float, accel: float, decel: float, dt: float) -> float:
	delta = target - current
	if delta > 0.0:
		max_step = accel * dt
	else:
		max_step = decel * dt
	if abs(delta) <= max_step:
		return target
	return current + math.copysign(max_step, delta)


def normalize_angle(angle: float) -> float:
	while angle > math.pi:
		angle -= 2.0 * math.pi
	while angle < -math.pi:
		angle += 2.0 * math.pi
	return angle


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
	siny_cosp = 2.0 * (w * z + x * y)
	cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
	return math.atan2(siny_cosp, cosy_cosp)


class MotionState(Enum):
	IDLE = 0
	ROTATE_TO_PATH = 1
	TRANSLATE = 2
	FINAL_ROTATE = 3


class RelativeMoveController(Node):
	def __init__(self) -> None:
		super().__init__('relative_move_controller')

		self.declare_parameter('cmd_vel_topic', '/cmd_vel')
		self.declare_parameter('move_topic', '/move')
		self.declare_parameter('odom_topic', '/odom')
		self.declare_parameter('imu_topic', '/imu')
		self.declare_parameter('use_imu_yaw', True)
		self.declare_parameter('imu_yaw_filter_alpha', 0.25)
		self.declare_parameter('imu_rotation_scale', 1.0)

		self.declare_parameter('control_frequency', 50.0)
		self.declare_parameter('max_linear_speed', 0.25)
		self.declare_parameter('max_angular_speed', 1.0)
		self.declare_parameter('linear_kp', 0.9)
		self.declare_parameter('angular_kp', 1.2)
		self.declare_parameter('position_tolerance', 0.01)
		self.declare_parameter('yaw_tolerance', 0.02)
		self.declare_parameter('cross_track_kp', 2.0)
		self.declare_parameter('max_heading_correction', 0.35)
		self.declare_parameter('max_linear_accel', 0.20)
		self.declare_parameter('max_linear_decel', 0.35)
		self.declare_parameter('max_angular_accel', 1.2)
		self.declare_parameter('max_angular_decel', 2.0)

		cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
		move_topic = self.get_parameter('move_topic').value
		odom_topic = self.get_parameter('odom_topic').value
		imu_topic = self.get_parameter('imu_topic').value

		self.control_frequency = float(self.get_parameter('control_frequency').value)
		self.use_imu_yaw = bool(self.get_parameter('use_imu_yaw').value)
		self.imu_yaw_filter_alpha = clamp(float(self.get_parameter('imu_yaw_filter_alpha').value), 0.0, 1.0)
		self.imu_rotation_scale = float(self.get_parameter('imu_rotation_scale').value)
		self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
		self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
		self.linear_kp = float(self.get_parameter('linear_kp').value)
		self.angular_kp = float(self.get_parameter('angular_kp').value)
		self.position_tolerance = float(self.get_parameter('position_tolerance').value)
		self.yaw_tolerance = float(self.get_parameter('yaw_tolerance').value)
		self.cross_track_kp = float(self.get_parameter('cross_track_kp').value)
		self.max_heading_correction = float(self.get_parameter('max_heading_correction').value)
		self.max_linear_accel = float(self.get_parameter('max_linear_accel').value)
		self.max_linear_decel = float(self.get_parameter('max_linear_decel').value)
		self.max_angular_accel = float(self.get_parameter('max_angular_accel').value)
		self.max_angular_decel = float(self.get_parameter('max_angular_decel').value)

		self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
		self.move_sub = self.create_subscription(Vector3, move_topic, self.move_callback, 10)
		self.odom_sub = self.create_subscription(
			Odometry,
			odom_topic,
			self.odom_callback,
			qos_profile_sensor_data,
		)
		self.imu_sub = self.create_subscription(
			Imu,
			imu_topic,
			self.imu_callback,
			qos_profile_sensor_data,
		)

		timer_period = 1.0 / self.control_frequency
		self.control_dt = timer_period
		self.control_timer = self.create_timer(timer_period, self.control_loop)

		self.current_x: Optional[float] = None
		self.current_y: Optional[float] = None
		self.current_odom_yaw: Optional[float] = None
		self.current_imu_yaw: Optional[float] = None

		self.state = MotionState.IDLE
		self.start_yaw = 0.0
		self.rotate_to_path_yaw = 0.0
		self.final_yaw = 0.0
		self.target_x = 0.0
		self.target_y = 0.0
		self.translate_heading_yaw = 0.0
		self.line_start_x = 0.0
		self.line_start_y = 0.0
		self.line_dir_x = 1.0
		self.line_dir_y = 0.0
		self.skip_rotate_to_path = False
		self.prefer_reverse = False
		self.current_cmd_linear_x = 0.0
		self.current_cmd_angular_z = 0.0

		self.get_logger().info('relative_move_controller started. Waiting for /move commands.')

	def get_current_yaw(self) -> Optional[float]:
		if self.use_imu_yaw and self.current_imu_yaw is not None:
			return self.current_imu_yaw
		return self.current_odom_yaw

	def publish_cmd_with_rate_limit(self, target_cmd: Twist) -> None:
		limited_cmd = Twist()
		limited_cmd.linear.x = limit_rate(
			target_cmd.linear.x,
			self.current_cmd_linear_x,
			self.max_linear_accel,
			self.max_linear_decel,
			self.control_dt,
		)
		limited_cmd.angular.z = limit_rate(
			target_cmd.angular.z,
			self.current_cmd_angular_z,
			self.max_angular_accel,
			self.max_angular_decel,
			self.control_dt,
		)
		self.current_cmd_linear_x = limited_cmd.linear.x
		self.current_cmd_angular_z = limited_cmd.angular.z
		self.cmd_pub.publish(limited_cmd)

	def odom_callback(self, msg: Odometry) -> None:
		self.current_x = msg.pose.pose.position.x
		self.current_y = msg.pose.pose.position.y

		q = msg.pose.pose.orientation
		self.current_odom_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

	def imu_callback(self, msg: Imu) -> None:
		q = msg.orientation
		if abs(q.w) < 1e-9 and abs(q.x) < 1e-9 and abs(q.y) < 1e-9 and abs(q.z) < 1e-9:
			return
		raw_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
		if self.current_imu_yaw is None:
			self.current_imu_yaw = raw_yaw
			return
		delta = normalize_angle(raw_yaw - self.current_imu_yaw)
		self.current_imu_yaw = normalize_angle(self.current_imu_yaw + self.imu_yaw_filter_alpha * delta)

	def compute_rotation_cmd(self, yaw_error: float) -> float:
		return clamp(self.angular_kp * yaw_error, -self.max_angular_speed, self.max_angular_speed)

	def move_callback(self, msg: Vector3) -> None:
		current_yaw = self.get_current_yaw()
		if self.current_x is None or self.current_y is None or current_yaw is None:
			self.get_logger().warn('Ignoring /move command because /odom is not ready yet.')
			return

		dx = float(msg.x)
		dy = float(msg.y)
		dz = float(msg.z)
		effective_dz = dz * self.imu_rotation_scale if self.use_imu_yaw else dz

		is_pure_x = abs(dx) > self.position_tolerance and abs(dy) <= self.position_tolerance
		is_pure_y = abs(dy) > self.position_tolerance and abs(dx) <= self.position_tolerance
		self.skip_rotate_to_path = is_pure_x or is_pure_y
		self.prefer_reverse = is_pure_x and dx < 0.0

		self.start_yaw = current_yaw

		world_dx = math.cos(self.start_yaw) * dx - math.sin(self.start_yaw) * dy
		world_dy = math.sin(self.start_yaw) * dx + math.cos(self.start_yaw) * dy

		self.target_x = self.current_x + world_dx
		self.target_y = self.current_y + world_dy
		self.final_yaw = normalize_angle(self.start_yaw + effective_dz)

		distance = math.hypot(world_dx, world_dy)

		if distance > self.position_tolerance and not self.skip_rotate_to_path:
			self.rotate_to_path_yaw = math.atan2(self.target_y - self.current_y, self.target_x - self.current_x)
			self.translate_heading_yaw = self.rotate_to_path_yaw
			self.line_start_x = self.current_x
			self.line_start_y = self.current_y
			self.line_dir_x = math.cos(self.translate_heading_yaw)
			self.line_dir_y = math.sin(self.translate_heading_yaw)
			self.state = MotionState.ROTATE_TO_PATH
		elif distance > self.position_tolerance:
			self.translate_heading_yaw = math.atan2(world_dy, world_dx)
			self.line_start_x = self.current_x
			self.line_start_y = self.current_y
			self.line_dir_x = math.cos(self.translate_heading_yaw)
			self.line_dir_y = math.sin(self.translate_heading_yaw)
			self.state = MotionState.TRANSLATE
		else:
			self.state = MotionState.FINAL_ROTATE

		self.get_logger().info(
			f'New goal: dx={dx:.3f}, dy={dy:.3f}, dz={dz:.3f} -> '
			f'effective_dz={effective_dz:.3f}, '
			f'target=({self.target_x:.3f}, {self.target_y:.3f}), final_yaw={self.final_yaw:.3f}'
		)

	def control_loop(self) -> None:
		current_yaw = self.get_current_yaw()
		if self.current_x is None or self.current_y is None or current_yaw is None:
			return

		if self.state == MotionState.IDLE:
			self.current_cmd_linear_x = 0.0
			self.current_cmd_angular_z = 0.0
			return

		cmd = Twist()

		if self.state == MotionState.ROTATE_TO_PATH:
			yaw_error = normalize_angle(self.rotate_to_path_yaw - current_yaw)
			if abs(yaw_error) <= self.yaw_tolerance:
				self.state = MotionState.TRANSLATE
			else:
				cmd.angular.z = self.compute_rotation_cmd(yaw_error)
				self.publish_cmd_with_rate_limit(cmd)
				return

		if self.state == MotionState.TRANSLATE:
			dx = self.target_x - self.current_x
			dy = self.target_y - self.current_y
			distance = math.hypot(dx, dy)

			if distance <= self.position_tolerance:
				self.state = MotionState.FINAL_ROTATE
			else:
				heading_target = self.translate_heading_yaw
				cross_track_error = -self.line_dir_y * (self.current_x - self.line_start_x) + self.line_dir_x * (self.current_y - self.line_start_y)
				heading_correction = clamp(
					-self.cross_track_kp * cross_track_error,
					-self.max_heading_correction,
					self.max_heading_correction,
				)
				heading_error = normalize_angle(heading_target + heading_correction - current_yaw)
				linear_direction = 1.0

				if self.prefer_reverse:
					reverse_heading_error = normalize_angle(heading_target + math.pi - current_yaw)
					if abs(reverse_heading_error) < abs(heading_error):
						heading_error = reverse_heading_error
						linear_direction = -1.0

				linear_speed = clamp(
					linear_direction * self.linear_kp * distance,
					-self.max_linear_speed,
					self.max_linear_speed,
				)
				angular_speed = clamp(
					self.angular_kp * heading_error,
					-self.max_angular_speed,
					self.max_angular_speed,
				)

				cmd.linear.x = linear_speed
				cmd.angular.z = angular_speed
				self.publish_cmd_with_rate_limit(cmd)
				return

		if self.state == MotionState.FINAL_ROTATE:
			yaw_error = normalize_angle(self.final_yaw - current_yaw)
			if abs(yaw_error) <= self.yaw_tolerance:
				self.state = MotionState.IDLE
				self.current_cmd_linear_x = 0.0
				self.current_cmd_angular_z = 0.0
				self.cmd_pub.publish(Twist())
				self.get_logger().info('Relative move goal completed.')
				return

			cmd.angular.z = self.compute_rotation_cmd(yaw_error)
			self.publish_cmd_with_rate_limit(cmd)


def main(args=None) -> None:
	rclpy.init(args=args)
	node = RelativeMoveController()
	try:
		rclpy.spin(node)
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()
