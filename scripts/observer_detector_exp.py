#!/usr/bin/env python3

# python implementation of offboard meaconing attack detector
# observer-based w/ residual testing
__author__  =   "Zhanpeng Yang, Minhyun Cho, Kartik Anand Pant"
__contact__ =   "yang1272@purdue.edu, cho515@purdue.edu, kpant@purdue.edu"

# TODO: handle NED -> ENU transformation (Camera Network)

# load public libraries
import numpy as np

# load ROS2 related libraries
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from px4_msgs.msg import VehicleOdometry

from geometry_msgs.msg import PoseStamped

# load private libraries
from offboard_detector.msg import DetectorOutput

class ObserverDetector(Node):

    def __init__(self):

        super().__init__("observer_detector_exp")

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
        self.ns = ''

        # define subscribers
        self.local_pos_sub      =    self.create_subscription(
            VehicleOdometry,
            f'{self.ns}/fmu/out/vehicle_odometry',
            self.odom_callback,
            qos_profile_sub)
        
        # enable when doing experiment
        self.mocap_pose_sub_ = self.create_subscription(
            PoseStamped,
            '/drone162/pose',
            self.true_position_callback,
            10
        )

        # disable when doing experiment
        # self.mocap_pose_sub_ = self.create_subscription(
        #     PoseStamped,
        #     f'{self.ns}/detector/in/gz_true_pose_enu',
        #     self.true_position_callback,
        #     10
        
        # define publishers
        self.detect_meaconing   =   self.create_publisher(
            DetectorOutput,
            f'{self.ns}/detector/out/observer_output',
             qos_profile_pub)

        # parameters for callback
        self.timer_period_  =   0.02                                                            # seconds
        self.timer_         =   self.create_timer(self.timer_period_, self.cmdloop_callback)

        self.loop_counter_  =   np.uint16(0) 
        self.entry_execute_ =   np.uint8(0)

        self.Lobv_          =   np.array([[1.12,0.00,0.00],
                                          [0.00,1.12,0.00],
                                          [0.00,0.00,1.12]],dtype=np.float64)                   # observer gain
        
        
        self.F_             =   np.array([[1.00,0.00,0.00],
                                          [0.00,1.00,0.00],
                                          [0.00,0.00,1.00]],dtype=np.float64)                   # discrete state transition matrix
        
        self.G_             =   np.array([[self.timer_period_, 0.00, 0.00],
                                          [0.00, self.timer_period_, 0.00],
                                          [0.00, 0.00, self.timer_period_]],dtype=np.float64)   # discrete input matrix
        
        self.H1_            =   np.array([[1.00,0.00,0.00],
                                          [0.00,1.00,0.00],
                                          [0.00,0.00,1.00]],dtype=np.float64)                   # discrete measurement matrix 1
        
        self.H2_            =   np.array([[1.00,0.00,0.00],
                                          [0.00,1.00,0.00],
                                          [0.00,0.00,1.00]],dtype=np.float64)                   # discrete measurement matrix 2                # discrete measurement matrix 2

        self.detect_threshold_  =   np.float64(1.00)                                            # detector threshold

        self.xhat_past_     =   np.array([0.00,0.00,0.00],dtype=np.float64)                     # xhat(k-1)
        self.xhat_cur_      =   np.array([0.00,0.00,0.00],dtype=np.float64)                     # xhat(k)
        self.residual_      =   np.array([0.00,0.00,0.00,0.00,0.00,0.00],dtype=np.float64)      # y(k)-Hx(k)

        self.atck_detect_   =   False

        # variables for subscribers
        self.local_pos_ned_     =   None
        self.local_vel_ned_     =   None

        self.true_pos_ned_      =   None
        self.true_pos_ned_f_    =   np.array([0.00,0.00,0.00],dtype=np.float64)      # filtered true position

    # subscriber callback
    def odom_callback(self,msg):
        self.local_pos_ned_     =   np.array([msg.position[0],msg.position[1],msg.position[2]],dtype=np.float64)
        self.local_vel_ned_     =   np.array([msg.velocity[0],msg.velocity[1],msg.velocity[2]],dtype=np.float64)

    def true_position_callback(self, msg):
        ##ENU->NED
        self.true_pos_ned_      =   np.array([msg.pose.position.y,msg.pose.position.x,-msg.pose.position.z],dtype=np.float64)

    # publisher
    def publish_detect_meaconing(self):
        msg                     =   DetectorOutput()
        msg.attack_detect       =   int(self.atck_detect_)
        msg.detector_residual   =   float(np.linalg.norm(self.residual_))
        msg.detector_threshold  =   float(self.detect_threshold_)
        self.detect_meaconing.publish(msg)

    # command loop callback
    def cmdloop_callback(self):
        if (self.local_pos_ned_ is not None) and (self.true_pos_ned_ is not None):

            self.loop_counter_   +=  1

            if self.entry_execute_  < 1:
                # Initialize
                self.xhat_cur_[0:3]     =   self.local_pos_ned_
                self.entry_execute_     =   1

            elif (self.loop_counter_ >= 1) and np.mod(self.loop_counter_,1) != 0:                                               # This branch disabled
                # Propagation only
                self.xhat_past_[0:3]    =   self.xhat_cur_[0:3]
                self.xhat_cur_[0:3]     =   np.matmul(self.F_,self.xhat_cur_[0:3])+np.matmul(self.G_,self.local_vel_ned_)
            
            else:
                # Propagation
                self.xhat_past_[0:3]    =   self.xhat_cur_[0:3]
                self.xhat_cur_[0:3]     =   np.matmul(self.F_,self.xhat_cur_[0:3])+np.matmul(self.G_,self.local_vel_ned_)

                # Filter
                self.true_pos_ned_f_    =   self.true_pos_ned_f_+(1-0.0)*(self.true_pos_ned_-self.true_pos_ned_f_)

                # Update
                self.residual_[0:3]     =   (self.local_pos_ned_-np.matmul(self.H1_,self.xhat_cur_[0:3]))
                self.residual_[3:6]     =   (self.true_pos_ned_f_-np.matmul(self.H2_,self.xhat_cur_[0:3]))
                self.xhat_cur_[0:6]     =   self.xhat_cur_[0:3]+np.matmul(self.Lobv_,self.residual_[0:3])                       # Update using only information coming from the vehicle
                self.loop_counter_      =   0

                if (np.linalg.norm(self.residual_) >= self.detect_threshold_):
                    self.atck_detect_       =   True

                else:
                    self.atck_detect_       =   False

        else:
        
            self.entry_execute_     =   0


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
