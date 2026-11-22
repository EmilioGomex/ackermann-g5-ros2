#!/usr/bin/env python3

import csv
import math
from bisect import bisect_right
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

LOOKAHEAD = 0.18
MAX_CURVATURE = 4.0


class PathFollower:
    def __init__(self, points):
        self.points = points
        self.distances = [0.0]
        for a, b in zip(points, points[1:]):
            self.distances.append(self.distances[-1] + math.dist(a, b))
        self.progress = 0.0

    def point_at(self, distance):
        i = min(bisect_right(self.distances, distance) - 1, len(self.points) - 2)
        a, b = self.points[i:i + 2]
        fraction = (distance - self.distances[i]) / (self.distances[i + 1] - self.distances[i])
        return tuple(x + fraction * (y - x) for x, y in zip(a, b))

    def update(self, x, y, yaw):
        first = max(0, bisect_right(self.distances, self.progress - 0.08) - 1)
        last = min(len(self.points) - 1, bisect_right(self.distances, self.progress + 0.23))
        error = math.inf
        nearest = self.progress
        for i in range(first, last):
            (ax, ay), (bx, by) = self.points[i:i + 2]
            dx, dy = bx - ax, by - ay
            length = self.distances[i + 1] - self.distances[i]
            fraction = ((x - ax) * dx + (y - ay) * dy) / (length * length)
            fraction = min(1.0, max(0.0, fraction))
            deviation = math.hypot(x - ax - fraction * dx, y - ay - fraction * dy)
            if deviation < error:
                error = deviation
                nearest = self.distances[i] + fraction * length
        self.progress = max(self.progress, nearest)
        remaining = self.distances[-1] - self.progress
        goal_error = math.dist((x, y), self.points[-1])
        if remaining < 0.035 and goal_error < 0.045:
            return 0.0, 0.0, error, goal_error

        tx, ty = self.point_at(self.progress + LOOKAHEAD)
        dx, dy = tx - x, ty - y
        lateral = -math.sin(yaw) * dx + math.cos(yaw) * dy
        curvature = 2.0 * lateral / max(dx * dx + dy * dy, 1e-8)
        curvature = min(MAX_CURVATURE, max(-MAX_CURVATURE, curvature))
        speed = min(0.23 / (1.0 + 0.24 * abs(curvature)), max(0.045, remaining))
        return speed, speed * curvature, error, goal_error


class RobotController(Node):
    def __init__(self):
        super().__init__('ap_follower')
        self.declare_parameter('start_delay', 5.0)
        self.delay = max(0.0, self.get_parameter('start_delay').value)
        self.publisher = self.create_publisher(Twist, '/g5/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Odometry, '/g5/ground_truth', self.on_pose, qos_profile_sensor_data
        )
        self.timer = self.create_timer(0.05, self.update)
        route_file = Path(get_package_share_directory('ackermann_g5')) / 'config' / 'ap_path.csv'
        with route_file.open(newline='') as file:
            self.follower = PathFollower([(float(x), float(y)) for x, y in csv.reader(file)])
        self.pose = None
        self.last_received = None
        self.ready_at = None
        self.samples = 0
        self.error_sq_sum = 0.0
        self.max_error = 0.0
        self.finished = False

    def on_pose(self, message):
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
            self.get_logger().info(f'Gazebo pose ready. Starting in {self.delay:.1f} s.')
        if (now - self.ready_at).nanoseconds < self.delay * 1e9:
            self.publisher.publish(cmd)
            return

        x, y, yaw = self.pose
        speed, turn, error, goal_error = self.follower.update(x, y, yaw)
        if error > 0.30:
            self.finished = True
            self.get_logger().error(f'AP stopped. Path deviation: {error:.3f} m.')
            self.publisher.publish(cmd)
            return
        self.samples += 1
        self.error_sq_sum += error * error
        self.max_error = max(self.max_error, error)
        cmd.linear.x, cmd.angular.z = speed, turn
        if speed == 0.0:
            self.finished = True
            rms = math.sqrt(self.error_sq_sum / self.samples)
            self.get_logger().info(
                f'AP complete. World-frame tracking RMSE {rms:.4f} m, '
                f'max tracking error {self.max_error:.4f} m, endpoint error {goal_error:.4f} m.'
            )
        self.publisher.publish(cmd)


def main():
    rclpy.init()
    node = RobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
