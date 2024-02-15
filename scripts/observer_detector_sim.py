#!/usr/bin/env python3

# python implementation of offboard meaconing attack detector
# observer-based w/ residual testing
__author__  =   "Zhanpeng Yang, Minhyun Cho, Kartik Anand Pant"
__contact__ =   "yang1272@purdue.edu, cho515@purdue.edu, kpant@purdue.edu"

# TODO: handle NED -> ENU transformation (Camera Network)

# load public libraries
import numpy as np
import argparse

# load ROS2 related libraries
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition, VehicleCommand

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import UInt8, Bool

# load private libraries
from offboard_detector.msg import DetectorOutput

class ObserverDetector(Node):

    def __init__(self):

        super().__init__("observer_detector_sim")

        # set publisher and subscriber quality of service profile
        qos_profile_pub =   QoSProfile(
            reliability =   QoSReliabilityPolicy.BEST_EFFORT,
            durability  =   QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history     =   QoSHistoryPolicy.KEEP_LAST,
            depth       =   1
        )

        qos_profile_sub =   QoSProfile(
            reliability =   QoSReliabilityPolicy.BEST_EFFORT,
            durability  =   QoSDurabilityPolicy.VOLATILE,
            history     =   QoSHistoryPolicy.KEEP_LAST,
            depth = 1
        )

        # define subscribers
        self.status_sub         =   self.create_subscription(
            VehicleStatus,
            '/px4_1/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile_sub)

        self.local_pos_sub      =    self.create_subscription(
            VehicleLocalPosition,
            '/px4_1/fmu/out/vehicle_local_position',
            self.local_position_callback,
            qos_profile_sub)
        
        self.true_pos_sub       =   self.create_subscription(
            PoseStamped,
            '/px4_1/detector/in/gz_true_pose_ned',
            self.gz_true_position_callback,
            qos_profile_sub)
        
        # define publishers
        self.detect_meaconing   =   self.create_publisher(
            DetectorOutput,
            '/px4_1/detector/out/observer_output',
             qos_profile_pub)

        # parameters for callback
        self.timer_period_  =   0.02                                                            # seconds
        self.timer_         =   self.create_timer(self.timer_period_, self.cmdloop_callback)

        self.counter_       =   np.uint16(0)                                                    # disable for an experiment
        #0.31
        self.Lobv_          =   np.array([[1.5,0.00,0.00,0.00,0.00,0.00],
                                          [0.00,1.5,0.00,0.00,0.00,0.00],
                                          [0.00,0.00,1.5,0.00,0.00,0.00],
                                          [0.00,0.00,0.00,1.8,0.00,0.00],
                                          [0.00,0.00,0.00,0.00,1.8,0.00],
                                          [0.00,0.00,0.00,0.00,0.00,1.8]],dtype=np.float64)    # observer gain
        
        
        self.F_             =   np.array([[1.00,0.00,0.00,self.timer_period_,0.00,0.00],
                                          [0.00,1.00,0.00,0.00,self.timer_period_,0.00],
                                          [0.00,0.00,1.00,0.00,0.00,self.timer_period_],
                                          [0.00,0.00,0.00,1.00,0.00,0.00],
                                          [0.00,0.00,0.00,0.00,1.00,0.00],
                                          [0.00,0.00,0.00,0.00,0.00,1.00]],dtype=np.float64)    # discrete state transition matrix
        
        self.G_             =   np.array([[np.power(self.timer_period_,2)/2, 0.00, 0.00],
                                          [0.00, np.power(self.timer_period_,2)/2, 0.00],
                                          [0.00, 0.00, np.power(self.timer_period_,2)/2],
                                          [self.timer_period_, 0.00, 0.00],
                                          [0.00, self.timer_period_, 0.00],
                                          [0.00, 0.00, self.timer_period_]],dtype=np.float64)   # discrete input matrix
        
        self.H1_            =   np.array([[1.00,0.00,0.00,0.00,0.00,0.00],
                                          [0.00,1.00,0.00,0.00,0.00,0.00],
                                          [0.00,0.00,1.00,0.00,0.00,0.00],
                                          [0.00,0.00,0.00,1.00,0.00,0.00],
                                          [0.00,0.00,0.00,0.00,1.00,0.00],
                                          [0.00,0.00,0.00,0.00,0.00,1.00]],dtype=np.float64)    # discrete measurement matrix 1
        
        self.H2_            =   np.array([[1.00,0.00,0.00,0.00,0.00,0.00],
                                          [0.00,1.00,0.00,0.00,0.00,0.00],
                                          [0.00,0.00,1.00,0.00,0.00,0.00]],dtype=np.float64)                   # discrete measurement matrix 2

        self.detect_threshold_  =   np.float64(1.00)                                            # detector threshold

        self.xhat_past_     =   np.array([0.00,0.00,0.00,0.00,0.00,0.00],dtype=np.float64)      # xhat(k-1)
        self.xhat_cur_      =   np.array([0.00,0.00,0.00,0.00,0.00,0.00],dtype=np.float64)      # xhat(k)
        self.residual_      =   np.array([0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00],dtype=np.float64)      # y(k)-Hx(k)
        self.residual_pos_      =   np.array([0.00,0.00,0.00,0.00,0.00,0.00],dtype=np.float64)      # y(k)-Hx(k)

        self.atck_detect_   =   False

        # variables for subscribers
        self.local_pos_ned_     =   None
        self.local_vel_ned_     =   None
        self.local_acc_ned_     =   None

        self.gz_true_pos_ned_   =   None
        self.gz_true_pos_ned_f_ =   np.array([0.00,0.00,0.00],dtype=np.float64)      # xhat(k-1)

    # subscriber callback
    def local_position_callback(self,msg):
        self.local_pos_ned_     =   np.array([msg.x,msg.y,msg.z],dtype=np.float64)
        self.local_vel_ned_     =   np.array([msg.vx,msg.vy,msg.vz],dtype=np.float64)
        self.local_acc_ned_     =   np.array([msg.ax,msg.ay,msg.az],dtype=np.float64)

    def vehicle_status_callback(self, msg):
        # print("NAV_STATUS: ", msg.nav_state)
        # print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.nav_state = msg.nav_state

    def gz_true_position_callback(self, msg):
        self.gz_true_pos_ned_   =   np.array([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z],dtype=np.float64)

    # publisher
    def publish_detect_meaconing(self):
        msg                     =   DetectorOutput()
        msg.attack_detect       =   int(self.atck_detect_)
        msg.detector_residual   =   float(np.linalg.norm(self.residual_pos_))
        msg.detector_threshold  =   float(self.detect_threshold_)
        self.detect_meaconing.publish(msg)

    # command loop callback
    def cmdloop_callback(self):
        if (self.local_pos_ned_ is not None) and (self.gz_true_pos_ned_ is not None):

            self.counter_   +=  1   # disable for an experiment

            if self.counter_ <= 1:
                # Initialize
                self.xhat_cur_[0:3]     =   self.local_pos_ned_
                self.xhat_cur_[3:6]     =   self.local_vel_ned_

            elif (self.counter_ > 1) and np.mod(self.counter_,2) != 0:
                # Propagation only
                self.xhat_past_[0:6]    =   self.xhat_cur_[0:6]
                self.xhat_cur_[0:6]     =   np.matmul(self.F_,self.xhat_cur_[0:6])+np.matmul(self.G_,self.local_acc_ned_)

            else:
                # Propagation
                self.xhat_past_[0:6]    =   self.xhat_cur_[0:6]
                self.xhat_cur_[0:6]     =   np.matmul(self.F_,self.xhat_cur_[0:6])+np.matmul(self.G_,self.local_acc_ned_)

                # Filter
                self.gz_true_pos_ned_f_ =   self.gz_true_pos_ned_f_+(1-0.0)*(self.gz_true_pos_ned_-self.gz_true_pos_ned_f_)

                # Update
                self.residual_[0:6]     =   (np.concatenate((self.local_pos_ned_,self.local_vel_ned_))-np.matmul(self.H1_,self.xhat_cur_[0:6]))
                self.residual_[6:9]     =   (self.gz_true_pos_ned_f_-np.matmul(self.H2_,self.xhat_cur_[0:6]))
                self.xhat_cur_[0:6]     =   self.xhat_cur_[0:6]+np.matmul(self.Lobv_,self.residual_[0:6])               # Update using only information coming from the vehicle

                self.residual_pos_[0:3] =   0.2*self.residual_[0:3]
                self.residual_pos_[3:6] =   0.2*self.residual_[6:9]

                print('PX4 local pos: ')
                print(self.local_pos_ned_)
                print('PX4 local vel: ')
                print(self.local_vel_ned_)
                print('PX4 local acc: ')
                print(self.local_acc_ned_)

                if (np.linalg.norm(self.residual_pos_) >= self.detect_threshold_):
                    self.atck_detect_       =   True

                else:
                    self.atck_detect_       =   False

        self.publish_detect_meaconing()
            

def main():
    # parser = argparse.ArgumentParser(description='Delivering parameters for tests')
    # parser.add_argument('--mode','-m',type=int,default=np.uint8(1),help='mode setting')
    # argin = parser.parse_args()
    # offboard_mission = OffboardMission(argin.mode)

    rclpy.init(args=None)

    observer_detector = ObserverDetector()

    rclpy.spin(observer_detector)

    rclpy.shutdown()


if __name__ == '__main__':

    main()
