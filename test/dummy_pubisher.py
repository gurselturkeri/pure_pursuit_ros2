import numpy as np
np.float = float  # fix deprecated alias if needed
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from tf_transformations import quaternion_from_euler
import math


class SimulatedRobot(Node):
    def __init__(self):
        super().__init__('simulated_robot')

        self.pose_pub = self.create_publisher(PoseStamped, '/current_pose', 10)
        self.path_pub = self.create_publisher(Path, '/waypoints', 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0  # in radians

        # Velocity command
        self.v = 0.0
        self.omega = 0.0

        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(0.05, self.update_pose)  # 20 Hz

    def cmd_callback(self, msg):
        self.v = msg.linear.x
        self.omega = msg.angular.z

    def update_pose(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # Update robot pose based on Twist command
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.omega * dt

        # Normalize yaw
        self.yaw = (self.yaw + math.pi) % (2 * math.pi) - math.pi

        # Publish updated pose
        q = quaternion_from_euler(0, 0, self.yaw)
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = now.to_msg()
        pose_msg.pose.position.x = self.x
        pose_msg.pose.position.y = self.y
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]
        self.pose_pub.publish(pose_msg)

        # --- Generate and publish square wave path ---
        path_msg = Path()
        path_msg.header = pose_msg.header

        amplitude = 1.0     # Height of the square wave
        period = 4.0        # Width of each step (x units)
        dx = 0.5            # Step size along x
        steps = 50          # Number of points to generate

        for i in range(steps):
            px = self.x + i * dx
            py = amplitude if int(px // period) % 2 == 0 else -amplitude

            p = PoseStamped()
            p.header = path_msg.header
            p.pose.position.x = px
            p.pose.position.y = py
            p.pose.orientation.w = 1.0
            path_msg.poses.append(p)

        self.path_pub.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimulatedRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
