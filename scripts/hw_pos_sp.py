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

class OffboardControl(Node):
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
        self.node = rclpy.create_node('control_class')

        # Subscribers
        self.odomSub = self.create_subscription(Odometry, '/mavros/local_position/odom', self.vehicle_odometry_callback, qos_profile_volatile)
        self.relaySub = self.create_subscription(Odometry, '/mavros/odometry/out', self.relay_callback, qos_profile_volatile)
        self.posSpSub = self.create_subscription(PoseStamped, '/shafterx2/reference', self.sp_callback, qos_profile_volatile)
        self.stateSub = self.create_subscription(State, '/mavros/state', self.state_callback, qos_profile_transient)

        #Publishers
        self.posSpPub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', qos_profile_volatile)
        self.attSpPub = self.create_publisher(PoseStamped, '/mavros/setpoint_attitude/attitude', qos_profile_volatile_reliable)
        self.thrSpPub = self.create_publisher(Thrust, '/mavros/setpoint_attitude/thrust', qos_profile_volatile_reliable)
        # self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        # self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        # self.publisher_attitude = self.create_publisher(VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', qos_profile)
        # self.publisher_command = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Constants and cutoff values
        self.gravity = np.array([0, 0, 1.3*9.81])
        self.maxAcc = 5
        self.maxThrottle = 0.7
        timerPeriod = 0.02  # seconds
        self.timer = self.create_timer(timerPeriod, self.cmdloop_callback)


        # initial values for setpoints
        self.curPos = np.zeros((3,))
        self.curVel = np.zeros((3,))
        self.yaw = 0.0
        self.startYaw = 1.0

        # Setpoints
        self.posSp = np.array([-0.0,-0.0, 1.2])
        self.quatSp = np.array([0.0, 0.0, 0.0, 1.0])
        self.velSp = np.array([0.0,0.0,0.0])
        self.yawSp = 0.0
        self.homePos = np.array([0,0,-0.05])

        # Storage Variables
        self.desVel = np.zeros((3,))
        self.errInt = np.zeros((3,))
        self.errVel = np.zeros((3,))
        self.preTime = Clock().now().nanoseconds/1E9
        self.offboardTime = Clock().now().nanoseconds/1E9
        self.dt = timerPeriod
        self.offbCounter = 1

        # Gains
        self.Kpos = np.array([-1.2, -1.2, -1.2])
        self.Kvel = np.array([-0.4, -0.4, -1.5])
        self.Kder = np.array([-0.0, -0.0, -0.3])
        self.Kint = np.array([-0.1, -0.1, -0.4])
        # self.Kder = np.array([-0.06, -0.06, -0.4])
        # self.Kint = np.array([-0.2, -0.2, -0.4])
        self.normThrustConst = 0.05

        # Msg Variables
        # self.data_out = PlotDataMsg()
        self.posSpMsg = PoseStamped()
        self.attSpMsg = PoseStamped()
        self.thrSpMsg = Thrust()


        # Flags
        self.offbFlag = False
        self.armFlag = False
        self.missionFlag = False
        self.relayFlag = False
        self.odomFlag = False
        self.home = False
        self.state = State()

        # # FcuMode
        self.modes = fcuModes()
        offbStatus = self.modes.set_mode('POSITION')

        print("Sleeping")
        time.sleep(2)
        print("Awake")

        

 
    ## Drone State callback
    def state_callback(self, msg):
        self.state = msg
        self.armFlag = msg.armed
        print('Mode: {}'.format(msg.mode))
        if msg.mode == 'OFFBOARD':
            self.offbFlag = True

    def relay_callback(self, msg):
        self.relayFlag = True

    def vehicle_odometry_callback(self, msg):
        self.curPos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        self.curVel = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
        self.curOrien = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.yaw = euler_from_quaternion(self.curOrien)[2]
        self.R = quaternion_matrix(self.curOrien)[:-1, :-1]

        if self.odomFlag == False:
            self.posSp[0] = self.curPos[0]
            self.posSp[1] = self.curPos[1]
            print(self.posSp)
        self.odomFlag = True

    def sp_callback(self, msg):
        # self.posSp = np.array([msg.position.x, msg.position.y, msg.position.z])
        self.posSp[0] = msg.pose.position.x
        self.posSp[1] = msg.pose.position.y
        quat = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        self.yawSp = euler_from_quaternion(quat)[2]
        # print('New setpoint')

    def set_offboard(self):
        pass


    def acc2quat(self,des_a, des_yaw):
        xb_des = np.array([1, 0, 0.0])
        if np.linalg.norm(des_a) == 0.0:
            zb_des = np.array([0,0,1])
        else:    
            zb_des = des_a / np.linalg.norm(des_a)
        yb_des = np.cross(zb_des, xb_des) / np.linalg.norm(np.cross(zb_des, xb_des))
        proj_xb_des = np.cross(yb_des, zb_des) / np.linalg.norm(np.cross(yb_des, zb_des))
       
        rotmat = np.transpose(np.array([proj_xb_des, yb_des, zb_des]))
        return rotmat



    def cmdloop_callback(self):
        if self.odomFlag and self.relayFlag:
            if(self.armFlag == True and self.offbFlag == True):
                norm_distance = np.linalg.norm(self.posSp[:2] - self.curPos[:2])
                posSp_ = np.array([0,0,1.0])
                maxNorm_ = 0.3
                if norm_distance > maxNorm_:
                    posSp_[:2] = self.curPos[:2] + maxNorm_*(self.posSp[:2] - self.curPos[:2])/norm_distance
                else:
                    posSp_[0] = self.posSp[0]
                    posSp_[1] = self.posSp[1]
                errorZ_ = self.posSp[2] - self.curPos[2]
                minErrorZ_ = -0.1
                if errorZ_ < minErrorZ_:
                    posSp_[2] = self.curPos[2] + minErrorZ_
                else:
                    posSp_[2] = self.posSp[2]

                yawSp_ = 0.0
                yawDiff_  = self.yawSp - self.yaw
                yawSum_ = self.yawSp + self.yaw
                if np.absolute(yawDiff_) > np.pi:
                    yawDiff_ = -np.sign(yawDiff_)*(2*np.pi - yawDiff_)
                
                yawDiff_ = np.maximum(-0.1, np.minimum(0.1, yawDiff_))
                yawSp_ = self.yaw + yawDiff_

                self.quatSp = quaternion_from_euler(0.0, 0.0, yawSp_)

                now = self.node.get_clock().now().to_msg()

                self.posSpMsg.header.stamp = now
                self.posSpMsg.pose.position.x = posSp_[0]
                self.posSpMsg.pose.position.y = posSp_[1]
                self.posSpMsg.pose.position.z = posSp_[2]

                self.posSpMsg.pose.orientation.x = self.quatSp[0]
                self.posSpMsg.pose.orientation.y = self.quatSp[1]
                self.posSpMsg.pose.orientation.z = self.quatSp[2]
                self.posSpMsg.pose.orientation.w = self.quatSp[3]



                self.posSpPub.publish(self.posSpMsg)

                

            elif self.offbFlag == True and self.armFlag==False:
                pos_sp = np.array([0.0, 0.0, 0.0])
                now = self.node.get_clock().now().to_msg()


                self.posSpMsg.header.stamp = now
                self.posSpMsg.pose.position.x = pos_sp[0]
                self.posSpMsg.pose.position.y = pos_sp[1]
                self.posSpMsg.pose.position.z = pos_sp[2]

                self.posSpMsg.pose.orientation.x = 0.0
                self.posSpMsg.pose.orientation.y = 0.0
                self.posSpMsg.pose.orientation.z = 0.0
                self.posSpMsg.pose.orientation.w = 1.0


                self.posSpPub.publish(self.posSpMsg)

                # print('Arming')
                # print("Waiting to be armed")
                armStatus = self.modes.set_arm(True) # Here is the arming command
                pass
                # print('Arming Status: {}'.format(armStatus))

             
            else:
                pos_sp = np.array([0.0, 0.0, 0.0])
                now = self.node.get_clock().now().to_msg()


                self.posSpMsg.header.stamp = now
                self.posSpMsg.pose.position.x = pos_sp[0]
                self.posSpMsg.pose.position.y = pos_sp[1]
                self.posSpMsg.pose.position.z = pos_sp[2]

                self.posSpMsg.pose.orientation.x = 0.0
                self.posSpMsg.pose.orientation.y = 0.0
                self.posSpMsg.pose.orientation.z = 0.0
                self.posSpMsg.pose.orientation.w = 1.0


                self.posSpPub.publish(self.posSpMsg)

                

                if self.offbCounter > 30:
                    if self.armFlag == False:
                        armStatus = self.modes.set_arm(True)

                    offbStatus = self.modes.set_mode('OFFBOARD')
                    # print('Offboard flag: {}'.format(self.offbFlag))
                    self.offboardTime = Clock().now().nanoseconds/1E9
                self.offbCounter = self.offbCounter + 1

                # print("OFFBOARD Status: {}".format(offbStatus))

        else:
            print("Odometry not received. Please check the topics")







        

def main(args=None):
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    print("True")

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
