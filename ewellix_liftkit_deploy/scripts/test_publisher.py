#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import sys
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class MinimalPublisher(Node):

    def __init__(self, desired_position):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, '/position_trajectory_controller/joint_trajectory', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.desired_position = desired_position

    def timer_callback(self):
        pass

    def pub_message(self):
        msg = JointTrajectory()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        msg.joint_names = ['ewellix_lift_top_joint']

        point = JointTrajectoryPoint()
        j1 = self.desired_position

        point.positions = [j1]
        point.velocities = []
        point.accelerations = []
        point.effort = []
        point.time_from_start.nanosec = 1000000

        msg.points.append( point )

        self.publisher_.publish(msg)

def main(args):
    rclpy.init(args=args)
    print(args[0])
    minimal_publisher = MinimalPublisher(float(args[0]))

    # rclpy.spin(minimal_publisher)
    minimal_publisher.pub_message()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv[1:])
