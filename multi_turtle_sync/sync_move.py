import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time
from threading import Thread


class MultiTurtleCW(Node):
    def __init__(self):
        super().__init__('multi_turtle_cw')

        # Publishers for velocity commands
        self.robot_pubs = [
            self.create_publisher(Twist, f'/robot{i+1}/cmd_vel', 10)
            for i in range(4)
        ]

        # Robot states
        self.robot_yaw = [0.0] * 4
        self.robot_pos_x = [0.0] * 4
        self.robot_pos_y = [0.0] * 4

        # Subscribe to odometry for each robot
        for i in range(4):
            self.create_subscription(
                Odometry,
                f'/robot{i+1}/odom',
                self.create_odom_callback(i),
                10
            )

        # Run callback thread (so odometry updates happen in parallel)
        self.spin_thread = Thread(target=rclpy.spin, args=(self,), daemon=True)
        self.spin_thread.start()

        self.get_logger().info(
            "\nPress [Enter] to move robots one step clockwise.\nPress Ctrl+C to exit.\n"
        )

        try:
            while rclpy.ok():
                input(">>> Press Enter to start CW swap cycle...")
                self.move_robots_cw_once()
        except KeyboardInterrupt:
            self.get_logger().info('Exiting...')

    # ---------------- Odometry callback ----------------
    def create_odom_callback(self, idx):
        def odom_callback(msg):
            q = msg.pose.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.robot_yaw[idx] = math.atan2(siny_cosp, cosy_cosp)
            self.robot_pos_x[idx] = msg.pose.pose.position.x
            self.robot_pos_y[idx] = msg.pose.pose.position.y
        return odom_callback

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    # ---------------- One CW swap cycle ----------------
    def move_robots_cw_once(self):
        rotate_kp = 1.0
        move_kp = 0.5
        rot_threshold = 0.05
        move_threshold = 0.05
        rate = 10  # Hz

        # Snapshot current positions
        start_pos_x = self.robot_pos_x.copy()
        start_pos_y = self.robot_pos_y.copy()

        # Clockwise target indices (1->2->3->4->1)
        #cw_target_idx = [1, 2, 3, 0]
        cw_target_idx = [3, 0, 1, 2]
        target_pos_x = [start_pos_x[j] for j in cw_target_idx]
        target_pos_y = [start_pos_y[j] for j in cw_target_idx]

        # ---- Phase 1: Rotate toward target ----
        self.get_logger().info('Rotating robots toward CW target...')
        rotating = [True] * 4
        while any(rotating) and rclpy.ok():
            for i in range(4):
                if not rotating[i]:
                    continue
                dx = target_pos_x[i] - self.robot_pos_x[i]
                dy = target_pos_y[i] - self.robot_pos_y[i]
                desired_yaw = math.atan2(dy, dx)
                diff = self.normalize_angle(desired_yaw - self.robot_yaw[i])
                twist = Twist()
                if abs(diff) < rot_threshold:
                    rotating[i] = False
                    twist.angular.z = 0.0
                else:
                    twist.angular.z = max(min(rotate_kp * diff, 0.5), -0.5)
                self.robot_pubs[i].publish(twist)
            time.sleep(1.0 / rate)

        # ---- Phase 2: Move toward target positions ----
        self.get_logger().info('Moving robots CW toward target...')
        moving = [True] * 4
        while any(moving) and rclpy.ok():
            for i in range(4):
                if not moving[i]:
                    continue
                dx = target_pos_x[i] - self.robot_pos_x[i]
                dy = target_pos_y[i] - self.robot_pos_y[i]
                dist = math.sqrt(dx * dx + dy * dy)
                twist = Twist()
                if dist <= move_threshold:
                    moving[i] = False
                    twist.linear.x = 0.0
                else:
                    desired_yaw = math.atan2(dy, dx)
                    yaw_error = self.normalize_angle(desired_yaw - self.robot_yaw[i])
                    twist.angular.z = max(min(rotate_kp * yaw_error, 0.4), -0.4)
                    twist.linear.x = max(min(move_kp * dist, 0.25), 0.05)
                self.robot_pubs[i].publish(twist)
            time.sleep(1.0 / rate)

        # ---- Stop all robots briefly ----
        stop = Twist()
        for pub in self.robot_pubs:
            pub.publish(stop)
        time.sleep(0.3)

        self.get_logger().info('CW swap complete. Ready for next input.')


def main(args=None):
    rclpy.init(args=args)
    node = MultiTurtleCW()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

