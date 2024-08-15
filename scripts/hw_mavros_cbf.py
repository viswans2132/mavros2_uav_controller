#!/usr/bin/env python

__author__ = "Viswa Narayanan Sankaranarayanan"
__contact__ = "vissan@ltu.se"

import rclpy
import numpy as np
import cvxpy as cp
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
        # self.odomSub = self.create_subscription(Odometry, '/shafterx2/odometry/imu', self.vehicle_odometry_callback, qos_profile_volatile)
        self.posSpSub = self.create_subscription(Int8, '/new_pose', self.sp_position_callback, qos_profile_volatile)
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
        self.posSp = np.array([-0.0,-0.0, 0.8])
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
        self.Kpos = np.array([-0.95, -0.95, -1.2])
        self.Kvel = np.array([-0.4, -0.4, -1.5])
        self.Kder = np.array([-0.0, -0.0, -0.3])
        self.Kint = np.array([-0.05, -0.05, -0.4])
        # self.Kder = np.array([-0.06, -0.06, -0.4])
        # self.Kint = np.array([-0.2, -0.2, -0.4])

        self.Krate = 2.0
        self.Kscale = 2.0
        self.normThrustConst = 0.05

        # Msg Variables
        # self.data_out = PlotDataMsg()
        self.attSpMsg = PoseStamped()
        self.thrSpMsg = Thrust()


        # Flags
        self.offbFlag = False
        self.armFlag = False
        self.missionFlag = False
        self.filterFlag = False
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

    def sp_position_callback(self, msg):
        # self.posSp = np.array([msg.position.x, msg.position.y, msg.position.z])
        self.posSp[0] = 0.0
        self.posSp[1] = -2
        self.filterFlag = True
        print("New setpoint received")

    def set_offboard(self):
        pass

    def safety_filter(self, errPos):
        desVel = self.Kpos * errPos
        if self.filterFlag:
            P = np.eye(3)
            u = cp.Variable(3)

            # R_cd = euler_matrix(self.rp[0], self.rp[1], self.yaw_sp)[:-1, :-1]
            # errPos = R_cd.dot(errPos)

            l = errPos[0]*errPos[0] + errPos[1]*errPos[1]

            if l > 0.01:
                h = errPos[2] - self.Krate*self.Kscale*l*np.exp(-self.Krate*l)

                # self.dataOut.h_func = h


                dhdx = 2*self.Krate*self.Kscale*errPos[0]*(self.Krate*l - 1)*np.exp(-self.Krate*l)
                dhdy = 2*self.Krate*self.Kscale*errPos[1]*(self.Krate*l - 1)*np.exp(-self.Krate*l)
                dhdz = 1

                dhdp = np.array([dhdx, dhdy, dhdz])

                constraints = [dhdp@u >= -1.4*h]
                prob = cp.Problem(cp.Minimize(cp.quad_form(u-desVel, P)), constraints)
                result = prob.solve()

                desVel = u.value
            if np.linalg.norm(errPos[:2]) < 0.15 and errPos[2] < 0.05:
                desVel[2] = -0.1

                if self.curPos[2] < 0.2:
                    desVel[2] = -0.5


        if np.linalg.norm(desVel) > 0.3173:
            desVel = desVel*0.3173/np.linalg.norm(desVel)

        return desVel


    def a_des(self):
        # dt = Clock().now().nanoseconds/1E9 - self.preTime
        # self.preTime = self.preTime + dt
        # if dt > 0.04:
        #     dt = 0.04

        R = np.array([[np.cos(self.yaw), np.sin(self.yaw), 0], [-np.sin(self.yaw), np.cos(self.yaw), 0], [0, 0, 1]])
        # curVel_W = R.T.dot(self.curVel)
        curVel_W = self.curVel

        errPos = self.curPos - self.posSp
        # desVel = self.Kpos * errPos

        desVel = self.safety_filter(errPos)

        derVel = ((self.curVel - desVel) - self.errVel)/self.dt;
        self.errVel = self.curVel - desVel;
        self.errInt = self.errInt + self.errVel*self.dt
        # print(errPos)
        # print(self.errVel)
        maxInt = np.array([2, 2, 6])
        self.errInt = np.maximum(-maxInt, np.minimum(maxInt, self.errInt))

        if self.curPos[2] < 0.2:
            derVel = np.zeros((3,))
            self.errInt[0] = 0.0
            self.errInt[1] = 0.0
            self.Kint[2] = -0.1
        else:            
            self.Kint[2] = -0.5


        desA = np.zeros((3,))


        desA[0] = self.Kvel[0]*self.errVel[0] + self.Kder[0]*derVel[0] + self.Kint[0]*self.errInt[0]
        desA[1] = self.Kvel[1]*self.errVel[1] + self.Kder[1]*derVel[1] + self.Kint[1]*self.errInt[1]
        desA[2] = self.Kvel[2]*self.errVel[2] + self.Kder[2]*derVel[2] + self.Kint[2]*self.errInt[2]

        # print(desA)
        dA = np.zeros((3,))

        dA = R.dot(desA)
        # print(dA)

        # Lines to copy
        maxDes = np.array([0.25, 0.25, 5])
        dA = np.maximum(-maxDes,(np.minimum(maxDes, dA)))

        if np.linalg.norm(dA) > self.maxAcc:
            dA = (self.maxAcc/np.linalg.norm(dA))*dA

        return (dA + self.gravity) 


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
        if self.odomFlag:
            if(self.armFlag == True and self.offbFlag == True):
                desA = self.a_des()

                yaw_rad = 0.0

                yaw_diff = yaw_rad - self.yaw
                yaw_diff = np.maximum(-0.1, np.minimum(0.1, yaw_diff))


                yaw_ref = 0.0



                r_des = self.acc2quat(desA, 0.0)

               
                zb = r_des[:,2]
                thrust = self.normThrustConst * desA.dot(zb)
                quatDes = quaternion_from_euler(-desA[1], desA[0], yaw_ref)
                thrust = np.maximum(-0.8, np.minimum(thrust, 0.8))
                # print(yaw_ref, quatDes[3])

                # print(zb)

                now = self.node.get_clock().now().to_msg()

                self.attSpMsg.header.stamp = now
                self.thrSpMsg.header.stamp = now

                self.attSpMsg.pose.orientation.x = quatDes[0]
                self.attSpMsg.pose.orientation.y = quatDes[1]
                self.attSpMsg.pose.orientation.z = quatDes[2]
                self.attSpMsg.pose.orientation.w = quatDes[3]

                self.thrSpMsg.thrust = thrust
                print("Thrust: {}".format(thrust))
                self.attSpPub.publish(self.attSpMsg)
                self.thrSpPub.publish(self.thrSpMsg)

                

            elif self.offbFlag == True and self.armFlag==False:
                now = self.node.get_clock().now().to_msg()

                self.attSpMsg.header.stamp = now
                self.thrSpMsg.header.stamp = now

                self.attSpMsg.pose.orientation.x = 0.0
                self.attSpMsg.pose.orientation.y = 0.0
                self.attSpMsg.pose.orientation.z = 0.0
                self.attSpMsg.pose.orientation.w = 1.0

                self.thrSpMsg.thrust = 0.5
                self.attSpPub.publish(self.attSpMsg)
                self.thrSpPub.publish(self.thrSpMsg)



                # print('Arming')
                # print("Waiting to be armed")
                armStatus = self.modes.set_arm(True)
                pass
                # print('Arming Status: {}'.format(armStatus))

             
            else:
                now = self.node.get_clock().now().to_msg()

                self.attSpMsg.header.stamp = now
                self.thrSpMsg.header.stamp = now

                self.attSpMsg.pose.orientation.x = 0.0
                self.attSpMsg.pose.orientation.y = 0.0
                self.attSpMsg.pose.orientation.z = 0.0
                self.attSpMsg.pose.orientation.w = 1.0

                self.thrSpMsg.thrust = 0.5
                self.attSpPub.publish(self.attSpMsg)
                self.thrSpPub.publish(self.thrSpMsg)

                

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
