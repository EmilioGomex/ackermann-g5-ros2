#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

RADIUS = 0.75
LOOKAHEAD_ARC = 0.22
CRUISE_SPEED = 0.16
MAX_CURVATURE = 4.0


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def circle_command(x, y, yaw, cx, cy, progress):
    radial = math.atan2(y - cy, x - cx)
    target_angle = radial + LOOKAHEAD_ARC / RADIUS
    dx = cx + RADIUS * math.cos(target_angle) - x
    dy = cy + RADIUS * math.sin(target_angle) - y
    lateral = -math.sin(yaw) * dx + math.cos(yaw) * dy
    curvature = 2.0 * lateral / max(dx * dx + dy * dy, 1e-8)
    curvature = max(-MAX_CURVATURE, min(MAX_CURVATURE, curvature))
    remaining = RADIUS * max(0.0, math.tau - progress)
    velocity = min(CRUISE_SPEED, max(0.04, 0.8 * remaining))
    return velocity, velocity * curvature


class CircleFollower(Node):
    def __init__(self):
        super().__init__('circle_follower')
        self.declare_parameter('start_delay', 5.0)
        self.delay = max(0.0, self.get_parameter('start_delay').value)
        self.publisher = self.create_publisher(Twist, '/g5/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Odometry, '/g5/odom', self.on_odom, qos_profile_sensor_data
        )
        self.timer = self.create_timer(0.05, self.update)
        self.last_received = None
        self.ready_at = None
        self.pose = None
        self.start_pose = None
        self.center = None
        self.previous_angle = None
        self.progress = 0.0
        self.samples = 0
        self.error_sq_sum = 0.0
        self.max_error = 0.0
        self.finished = False

    def on_odom(self, message):
        pose = message.pose.pose
        q = pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (pose.position.x, pose.position.y, yaw)
        self.last_received = self.get_clock().now()

    def update(self):
        cmd = Twist()
        now = self.get_clock().now()
        if self.finished or self.last_received is None:
            self.publisher.publish(cmd)
            return
        if (now - self.last_received).nanoseconds > 500_000_000:
            self.publisher.publish(cmd)
            return
        if self.ready_at is None:
            self.ready_at = now
            self.get_logger().info(f'Odometry ready. Starting in {self.delay:.1f} s.')
        if (now - self.ready_at).nanoseconds < self.delay * 1e9:
            self.publisher.publish(cmd)
            return

        x, y, yaw = self.pose
        if self.center is None:
            self.start_pose = (x, y)
            self.center = (x - RADIUS * math.sin(yaw), y + RADIUS * math.cos(yaw))
        cx, cy = self.center
        radial = math.atan2(y - cy, x - cx)
        if self.previous_angle is not None:
            self.progress += wrap(radial - self.previous_angle)
        self.previous_angle = radial

        error = abs(math.hypot(x - cx, y - cy) - RADIUS)
        self.samples += 1
        self.error_sq_sum += error * error
        self.max_error = max(self.max_error, error)
        if self.progress >= math.tau:
            self.finished = True
            rms = math.sqrt(self.error_sq_sum / self.samples)
            closure = math.hypot(x - self.start_pose[0], y - self.start_pose[1])
            self.get_logger().info(
                f'Lap complete. Odometry radial RMSE {rms:.4f} m, '
                f'max radial error {self.max_error:.4f} m, closure {closure:.4f} m.'
            )
        else:
            cmd.linear.x, cmd.angular.z = circle_command(x, y, yaw, cx, cy, self.progress)
        self.publisher.publish(cmd)


def main():
    rclpy.init()
    node = CircleFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
