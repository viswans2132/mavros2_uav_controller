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
from sensor_msgs.msg import Range

# Flight modes class
# Flight modes are activated using ROS2 services
class fcuModes(Node):
    def __init__(self):
        super().__init__('fcu_modes')
        self.node = rclpy.create_node('mode_settings')

    def set_arm(self, requestedValue):
        armService = self.create_client(CommandBool, 'mavros/cmd/arming')
        while not armService.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Arm service not available, waiting again...')
        armingReq = CommandBool.Request()
        armingReq.value = requestedValue
        future = armService.call_async(armingReq)
        
        rclpy.spin_until_future_complete(self, future)
        armingResp = future.result()
        return armingResp.success

    def set_mode(self, requestedMode):
        setStabilizedService = self.create_client(SetMode, 'mavros/set_mode')
        while not setStabilizedService.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Stabilized service not available, waiting again...')

        modeReq = SetMode.Request()
        modeReq.custom_mode = requestedMode
        future = setStabilizedService.call_async(modeReq)
        
        rclpy.spin_until_future_complete(self, future)
        modeResp = future.result()
        return modeResp.mode_sent

    # def setPositionMode(self):
    #     rospy.wait_for_service('mavros/set_mode')
    #     try:
    #         flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
    #         flightModeService(custom_mode='POSCTL')
    #     except rospy.ServiceException as e:
    #         print("service set_mode call failed: %s. Position Mode could not be set.")%e

class OdometryRelay(Node):
    def __init__(self):
        super().__init__('offboard_control')
        qos_profile_transient = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        qos_profile_volatile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        qos_profile_volatile_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        # Nodes
        self.node = rclpy.create_node('odometry_class')

        # Subscribers
        self.odomSub = self.create_subscription(Odometry, '/shafterx2/odometry/imu', self.imu_odometry_callback, qos_profile_volatile)
        self.rangeSub = self.create_subscription(Range, '/sbl', self.range_callback,  qos_profile_volatile)
        #Publishers
        self.odomPub = self.create_publisher(Odometry, '/mavros/odometry/out', qos_profile_volatile_reliable)
        # self.posePub = self.create_publisher(PoseStamped, '/mavros/vision_pose/pose', qos_profile_volatile_reliable)

        self.odomMsg = Odometry()
        self.odomMsg.header.frame_id = 'odom'
        self.odomMsg.child_frame_id = 'base_link'

        self.poseMsg = PoseStamped()
        self.position = np.array([0.0, 0.0, 0.0])
        self.quat = np.array([0.0, 0.0, 0.0, 1.0])

        rate = 30.0

        timerPeriod = 1/rate  # seconds
        self.timer = self.create_timer(timerPeriod, self.publisher_callback)

        
    def range_callback(self, msg):
        [roll, pitch, yaw] = euler_from_quaternion(self.quat)
        self.position[2] = msg.range*np.cos(roll)*np.cos(pitch)

    def imu_odometry_callback(self, msg):
        self.odomMsg.header.stamp = msg.header.stamp
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y
        self.odomMsg.pose.pose.position.x = self.position[0]
        self.odomMsg.pose.pose.position.y = self.position[1]
        self.odomMsg.pose.pose.position.z = self.position[2]
        self.quat = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.odomMsg.pose.pose.orientation.x = self.quat[0]
        self.odomMsg.pose.pose.orientation.y = self.quat[1]
        self.odomMsg.pose.pose.orientation.z = self.quat[2]
        self.odomMsg.pose.pose.orientation.w = self.quat[3]
        R = quaternion_matrix(self.quat)[:-1, :-1]
        # velLinear = R.dot(np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]))
        velLinear = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
        velAngular = np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z])


        self.odomMsg.twist.twist.linear.x = velLinear[0]
        self.odomMsg.twist.twist.linear.y = velLinear[1]
        self.odomMsg.twist.twist.linear.z = velLinear[2]
        self.odomMsg.twist.twist.angular.x = velAngular[0]
        self.odomMsg.twist.twist.angular.y = velAngular[1]
        self.odomMsg.twist.twist.angular.z = velAngular[2]
        self.odomPub.publish(self.odomMsg)
        # self.poseMsg.header.stamp = msg.header.stamp
        # self.poseMsg.pose = msg.pose.pose
        # self.posePub.publish(self.poseMsg)

    def publisher_callback(self):
        pass


        

def main(args=None):
    rclpy.init(args=args)
    odometryRelay = OdometryRelay()
    print("Relaying Odometry from Shafter IMU to MAVROS....")

    rclpy.spin(odometryRelay)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
