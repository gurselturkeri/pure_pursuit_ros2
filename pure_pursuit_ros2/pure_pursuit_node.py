import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
import matplotlib.pyplot as plt
import math


class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        self.get_logger().info('Pure Pursuit Node Initialized')
        # Declare all parameters
        self.declare_parameter('lookahead_distance', 0.9)
        self.declare_parameter('wheelbase_length', 0.5)
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('pose_topic', '/current_pose')
        self.declare_parameter('path_topic', '/waypoints')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        # Retrieve parameters
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.wheelbase_length = self.get_parameter('wheelbase_length').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        pose_topic = self.get_parameter('pose_topic').value
        path_topic = self.get_parameter('path_topic').value
        cmd_topic = self.get_parameter('cmd_vel_topic').value

        self.get_logger().info(f"Subscribing to pose: {pose_topic}")
        self.get_logger().info(f"Subscribing to path: {path_topic}")
        self.get_logger().info(f"Publishing to cmd_vel: {cmd_topic}")


        self.pose_sub = self.create_subscription(PoseStamped, pose_topic, self.pose_callback, 10)
        self.path_sub = self.create_subscription(Path, path_topic, self.path_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)

        self.current_pose = None
        self.waypoints = []
        self.trajectory = []
        self.original_path = []

        self.fig, self.ax = plt.subplots()
        plt.ion()

    def pose_callback(self, msg):
        self.current_pose = msg
        self.run_pure_pursuit()

    def path_callback(self, msg):
        self.waypoints = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        if not self.original_path:
            self.original_path = self.waypoints.copy()

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def run_pure_pursuit(self):
        if self.current_pose is None or len(self.waypoints) < 2:
            return

        x = self.current_pose.pose.position.x
        y = self.current_pose.pose.position.y
        yaw = self.get_yaw_from_quaternion(self.current_pose.pose.orientation)
        self.get_logger().info(f"Heading: {yaw:.2f} rad / {math.degrees(yaw):.1f} deg")

        lookahead_point = None
        for wx, wy in self.waypoints:
            dx = wx - x
            dy = wy - y
            dist = np.hypot(dx, dy)
            if dist > self.lookahead_distance:
                lookahead_point = (wx, wy)
                break
        if lookahead_point is None:
            self.get_logger().info("No valid lookahead point.")
            return

        lx = np.cos(-yaw) * (lookahead_point[0] - x) - np.sin(-yaw) * (lookahead_point[1] - y)
        ly = np.sin(-yaw) * (lookahead_point[0] - x) + np.cos(-yaw) * (lookahead_point[1] - y)

        curvature = 2 * ly / (self.lookahead_distance ** 2)
        steering_angle = math.atan(curvature * self.wheelbase_length)

        self.trajectory.append((x, y))

        cmd = Twist()
        cmd.linear.x = self.max_linear_speed
        cmd.angular.z = steering_angle
        self.cmd_pub.publish(cmd)

        self.visualize(x, y, yaw, lookahead_point)

    def visualize(self, x, y, yaw, lookahead_point):
        self.ax.clear()

        if self.original_path:
            wp_x, wp_y = zip(*self.original_path)
            self.ax.plot(wp_x, wp_y, 'k--', linewidth=2, label='Path')

        if len(self.trajectory) > 1:
            tx, ty = zip(*self.trajectory)
            self.ax.plot(tx, ty, 'b-', linewidth=1.5, label='Trajectory')

        self.ax.plot(x, y, 'bo', label='Robot')
        self.ax.arrow(x, y, np.cos(yaw), np.sin(yaw), head_width=0.2, color='blue')
        self.ax.plot(lookahead_point[0], lookahead_point[1], 'ro', label='Lookahead')

        self.ax.set_xlim(x - 10, x + 5)
        self.ax.set_ylim(y - 10, y + 5)
        self.ax.set_aspect('equal')
        self.ax.legend()
        self.ax.set_title('Pure Pursuit Visualization')
        plt.draw()
        plt.pause(0.001)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down.")
    finally:
        plt.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
