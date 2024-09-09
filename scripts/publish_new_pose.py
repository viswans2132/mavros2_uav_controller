#!/usr/bin/env python

__author__ = "Viswa Narayanan Sankaranarayanan"
__contact__ = "vissan@ltu.se"

import rclpy
import numpy as np
from tf_transformations import *
import time
import mavros
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

# from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleOdometry, VehicleAttitudeSetpoint, VehicleControlMode, VehicleLocalPosition
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import Thrust, State
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Point, Pose, PoseStamped, Twist, TwistStamped, Vector3
from std_msgs.msg import Int8

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_node')
        self.qos_profile_volatile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.newPosePub = self.create_publisher(PoseStamped, '/shafterx2/reference', self.qos_profile_volatile)

        while True:
            self.loop()

    
    def loop(self):
        newPoseMsg = PoseStamped()
        newPoseMsg.pose.position.x = 0.0
        newPoseMsg.pose.position.y = 0.0
        newPoseMsg.pose.position.z = 1.2
        yaw = -0*np.pi/2
        quat = quaternion_from_euler(0.0, 0.0, yaw)
        newPoseMsg.pose.orientation.x = quat[0]
        newPoseMsg.pose.orientation.y = quat[1]
        newPoseMsg.pose.orientation.z = quat[2]
        newPoseMsg.pose.orientation.w = quat[3]
        self.newPosePub.publish(newPoseMsg)
        time.sleep(0.1)



def main(args=None):
    rclpy.init(args=args)
    publisher_node = PosePublisher()
    print("True")

    rclpy.spin(publisher_node)

    publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
