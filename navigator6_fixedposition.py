#!/usr/bin/env python3
"""
Go-to-goal navigation using /odom for robot state and /beacon/pose for target.

Robot state:
  - /odom (nav_msgs/msg/Odometry)  -> x, y, yaw

Target:
  - /beacon/pose (geometry_msgs/msg/Pose) -> x, y
    IMPORTANT: for correct behavior, beacon pose must be expressed in the SAME frame as /odom.

Command:
  - /myrobot/cmd_vel (geometry_msgs/msg/Twist) -> linear.x and angular.z

Control (didactic, robust):
  - Compute bearing to target: theta_T = atan2(ey, ex)
  - Angular error: e_theta = wrap_to_pi(theta_T - theta)
  - If |e_theta| is large -> rotate in place
  - Else -> drive forward proportional to distance (with saturation)
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Point32



def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Quaternion (ROS ordering x,y,z,w) -> yaw (rad)."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_to_pi(angle: float) -> float:
    """Normalize angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def clip(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def generate_angle(x, y, x_old, y_old):
    return math.atan2(y-y_old, x-x_old)

class GoToBeaconOdom(Node):
    def __init__(self):
        super().__init__("go_to_beacon_odom")

        # -------- Parameters --------
        self.declare_parameter("control_rate_hz", 10.0) # rate of control law

        self.declare_parameter("k_theta", 1.8)          # Angular speed control gain
        self.declare_parameter("k_d", 0.8)              # Linear speed control gain

        self.declare_parameter("v_max", 0.7)            # max linear speed (m/s)
        self.declare_parameter("omega_max", 3.0)        # max angular speed (rad/s)

        self.declare_parameter("goal_tolerance", 0.25)  # m
        self.declare_parameter("theta_align_deg", 30.0) # deg, start driving when within this
        self.declare_parameter("min_data_age_sec", 5.0) # safety watchdog

        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.k_theta = float(self.get_parameter("k_theta").value)
        self.k_d = float(self.get_parameter("k_d").value)
        self.v_max = float(self.get_parameter("v_max").value)
        self.omega_max = float(self.get_parameter("omega_max").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.theta_align = math.radians(float(self.get_parameter("theta_align_deg").value))
        self.min_data_age_sec = float(self.get_parameter("min_data_age_sec").value)

        # -------- I/O --------
        self.cmd_pub = self.create_publisher(Twist, "/groupA/cmd_vel", 10)

        #only one message: we don't need to react to old data
        self.create_subscription(Point32, "/groupA/robot_pos", self.on_odom, 10)
        self.create_subscription(Pose, "/groupA/destination_pose", self.on_beacon_pose, 10)

        # -------- pos 
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        # ------- old pos
        self.robot_x_old = 0.0
        self.robot_y_old = 0.0
        self.robot_theta_old = 0.0
        self.odom_counter = 0

        # -------- target pos
        self.tx = 2
        self.ty = 5
 
        #we will use the following to check for staleness (old messages - do not react to those)
        self.beacon_pose = None
        self.beacon_time = None  
        self.odom_time   = None
        # -------- Timer for control law --------
        self.timer = self.create_timer(1.0 / self.control_rate_hz, self.control_step)

        self.get_logger().info(
            "GoToBeaconOdom running. Subscribing: /odom, /beacon/pose. Publishing: /myrobot/cmd_vel."
        )

    def on_odom(self, msg: Point32) -> None:
        print("odometry being done")
        print(self.odom_counter)
        if self.odom_counter >= 10:
            print("updating old pos")
            self.robot_theta = generate_angle(self.robot_x, self.robot_y, self.robot_x_old, self.robot_y_old)
            self.robot_x_old = self.robot_x
            self.robot_y_old = self.robot_y
            self.robot_theta_old = self.robot_theta
            self.odom_counter = 0

        self.odom_counter += 1
        self.robot_x = msg.x
        self.robot_y = msg.y


        #q = msg.pose.pose.orientation
        #x, y, z, w = q.x, q.y, q.z, q.w

        #self.robot_theta = yaw_from_quaternion(x, y, z, w)
        self.odom_time = self.get_clock().now()

    def on_beacon_pose(self, msg: Pose) -> None:
        self.beacon_pose = msg
        self.beacon_time = self.get_clock().now()

    def publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())
        

    def control_step(self) -> None:
        #print("control step")
        # Need at least one beacon pose + odom update
        # if self.odom_time is None:
        #     return


        # Robot pose 
        rx, ry, theta = self.robot_x, self.robot_y, self.robot_theta

        # Target position  - hacked in.... replace with actual data !!!
        tx = 3
        ty = 4

        # Errors
        ex = tx - rx
        ey = ty - ry
        d = math.sqrt(ex * ex + ey * ey)
        print(f"Distance {d}")
        # Close enough -> stop
        if abs(d) < self.goal_tolerance:
            self.publish_stop()
            self.get_logger().info(f"Reached goal (d={d:.3f} m) -> stop.")
            return

        # Target bearing and heading error
        theta_T = math.atan2(ey, ex)
        e_theta = wrap_to_pi(theta_T - theta)
        print(f"theta: {theta} and e_theta {e_theta}" )
        # Angular control
        omega = clip(self.k_theta * e_theta, -self.omega_max, self.omega_max)

        # Linear control: rotate-in-place until roughly aligned
        if abs(e_theta) > self.theta_align:
            v = 0.0
        else:
            v = clip(self.k_d * d, 0.0, self.v_max)

        # slow down near goal for smooth stop
        slow_radius = 0.4
        if d < slow_radius:
            v *= d / slow_radius

        d_angle = math.sqrt((self.robot_x_old - self.robot_x)**2 + (self.robot_y_old - self.robot_y)**2)

        if d_angle < 0.05 and abs(theta) < 0.06:
            print("go forwared")
            v = 0.2
            omega = 0

        
        # Publish Twist
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(omega)
        #print(f"Publishing {cmd}")
        self.cmd_pub.publish(cmd)



def main(args=None):
    rclpy.init(args=args)
    node = GoToBeaconOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
