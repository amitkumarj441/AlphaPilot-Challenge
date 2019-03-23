#!/usr/bin/env python
# -*- coding: utf-8 -*-
''' load libraries '''
import time
from math import pow,atan2,sqrt,sin,cos
from tf.transformations import euler_from_quaternion,quaternion_from_euler
import numpy as np

import rospy

from mav_msgs.msg import RateThrust
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
import sys
import signal

def signal_handler(signal, frame): 
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

global ros_rate
ros_rate = 100

r2d = 180/np.pi
d2r = np.pi/180

class robot():
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        self.local_deg_pub = rospy.Publisher('/uav/input/rateThrust', RateThrust, queue_size=10)
        self.target_gate_pub = rospy.Publisher("/uav/current_target_gate", Int32, queue_size=10)
        self.VO_sub = rospy.Subscriber('/vins_estimator/odometry', Odometry, self.vo_callback)
        self.err_gate_px_sub = rospy.Subscriber("/uav/current_target_gate_error",Int32MultiArray, self.err_gate_px_callback)
        self.clock_sub = rospy.Subscriber('/clock',Clock, self.clock_callback)

        self.rate = rospy.Rate(ros_rate)
        self.deg = RateThrust()
        self.clock = Clock()

        self.ind_wp = 0
        self.err_gate_px = [0, 0]
        self.pre_err_gate_px_comp = [0, 0]
        self.err_gate_px_comp = [0, 0]
        self.err_sum_gate_px = [0, 0]

# body frame
        self.ang_body_x_roll = 0  # (+) right direction
        self.ang_body_y_pitch = 0 # (+) pitch up 
        self.ang_body_z_yaw = 0   # (+) CW
        self.vel_body_x = 0 # Elevator
        self.vel_body_y = 0 # Aileron
        self.vel_body_z = 0 # Altitude
        self.acc_body_x = 0
        self.acc_body_y = 0
        self.acc_body_z = 0

# world frame
        self.pos_world_x = 0
        self.pos_world_y = 0
        self.pos_world_z = 0
        self.vel_world_x = 0
        self.vel_world_y = 0
        self.vel_world_z = 0
        self.acc_world_x = 0
        self.acc_world_y = 0
        self.acc_world_z = 0

# command (body frame)
        self.com_ang_body_x_roll = 0  # (+) right direction
        self.com_ang_body_y_pitch = 0 # (+) pitch up 
        self.com_ang_body_z_yaw = 0   # (+) CW
        self.com_vel_body_x = 0
        self.com_vel_body_y = 0
        self.com_vel_body_z = 0
        self.com_vel_body_r = 0 # degree per second

        self.err_sum_pos_body_x =0
        self.err_sum_pos_body_y =0
        self.err_sum_pos_body_z =0
        self.count_ramp_vel = 0
        self.moving_distance = 0

        Nominal_Gates=[]
        Nominal_Gates.append(rospy.get_param('/uav/Gate10/nominal_location')) # first
        Nominal_Gates.append(rospy.get_param('/uav/Gate21/nominal_location')) # second
        Nominal_Gates.append(rospy.get_param('/uav/Gate2/nominal_location')) # 3rd
        Nominal_Gates.append(rospy.get_param('/uav/Gate13/nominal_location')) # 4th
        Nominal_Gates.append(rospy.get_param('/uav/Gate9/nominal_location')) # 5
        Nominal_Gates.append(rospy.get_param('/uav/Gate14/nominal_location')) # 6
        Nominal_Gates.append(rospy.get_param('/uav/Gate1/nominal_location')) # 7
        Nominal_Gates.append(rospy.get_param('/uav/Gate22/nominal_location')) # 8
        Nominal_Gates.append(rospy.get_param('/uav/Gate15/nominal_location')) # 9
        Nominal_Gates.append(rospy.get_param('/uav/Gate23/nominal_location')) # 10
        Nominal_Gates.append(rospy.get_param('/uav/Gate6/nominal_location')) # 11
        Nominal_Gates=np.array(Nominal_Gates)

        Gates_before_transformed=np.zeros([len(Nominal_Gates),len(Nominal_Gates[0][0])+1])
        self.Gates_after_transformed=np.zeros([len(Nominal_Gates),len(Nominal_Gates[0][0])+1])
        for i in range(0,len(Nominal_Gates)):
            for k in range(0,3): # x, y, z
                for j in range(0,len(Nominal_Gates[0])):
                    Gates_before_transformed[i][k]=Gates_before_transformed[i][k]+Nominal_Gates[i][j][k] # X values sum up
                Gates_before_transformed[i][k]=Gates_before_transformed[i][k]/4
            self.Gates_after_transformed[i,:3]=[Gates_before_transformed[i][1]+23,Gates_before_transformed[i][0]-18,Gates_before_transformed[i][2]-5.3]
            print(Gates_before_transformed)


    def vo_callback(self, msg):
        self.vo = msg.pose.pose.position
        self.vo_vel = msg.twist.twist.linear
        quaternion=msg.pose.pose.orientation
        orientation_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)

    def clock_callback(self,msg):
        self.clock = msg.clock

    def err_gate_px_callback(self, msg):
        self.err_gate_px = msg.data

    def tic(self):
        self.starttime=self.clock
    def toc(self):
        nowtime=self.clock
        return nowtime-self.starttime

    def auto_taking_off(self):
        print("#### Initializing VINS-Mono, taking off #### \n")
        self.tic()
        while self.toc()<rospy.Duration(0.1*0.8,0):
            self.deg.thrust.z = 10.81
            self.deg.header.stamp = rospy.Time.now()
            self.local_deg_pub.publish(self.deg)
            self.rate.sleep()
        while self.toc()<rospy.Duration(0.55*0.8,0):
            self.deg.angular_rates.y=0.2
            self.deg.thrust.z = 12.81
            self.deg.header.stamp = rospy.Time.now()
            self.local_deg_pub.publish(self.deg)
            self.rate.sleep()
        while self.toc()<rospy.Duration(1,0*0.8):
            self.deg.angular_rates.y=0.4
            self.deg.thrust.z = 9.51
            self.deg.header.stamp = rospy.Time.now()
            self.local_deg_pub.publish(self.deg)
            self.rate.sleep()
        while self.toc()<rospy.Duration(1.3*0.8,0):
            self.deg.thrust.z = 9.51
            self.deg.header.stamp = rospy.Time.now()
            self.local_deg_pub.publish(self.deg)
            self.rate.sleep()
        while self.toc()<rospy.Duration(1.8*0.8,0):
            self.deg.angular_rates.y=-0.55
            self.deg.thrust.z = 9.81
            self.deg.header.stamp = rospy.Time.now()
            self.local_deg_pub.publish(self.deg)
            self.rate.sleep()

        while self.toc()<rospy.Duration(2.5*0.8,0):
            print('waiting for a while....')
            self.rate.sleep()
        print('mission start')

def input(rbt):

    max_ang_rate_body_z = 180 * d2r

    max_ang_body_x_mission = 60 * d2r # limitation
    max_ang_body_y_mission = 65 * d2r

    max_vel_body_x_mission = 20 # limitation
    max_vel_body_y_mission = 20
    max_vel_body_z_mission = 20

    waypoint_radius = 0.1 # m

    gain_ang_x_roll_P = 3.0 # 3.0
    gain_ang_y_pitch_P = 3.0 # 3.0
    gain_ang_r_yaw_P = 0.5 # 0.5
    gain_thr_z_altitude_P = 5.5 # 5.0

    gain_vel_x_mission_P = 0.2 # 0.2
    gain_vel_y_mission_P = 0.2 # 0.2

    gain_pos_x_mission_P = 1.0 # 0.8
    gain_pos_y_mission_P = 0.6 # 0.6 # 0.4
    gain_pos_z_mission_P = 1.0 # 1.0

    gain_pos_x_mission_I = 0.2
    gain_pos_y_mission_I = 0.2

    max_err_sum_pos_body_x = 2 # integral windup
    max_err_sum_pos_body_y = 2
    max_err_sum_pos_body_z = 2

    com_vel_waypoint_divide_factor = 6.0

    gain_img_x_P = 16.0#*0.75  # 16.0 roll, y-direction
    gain_img_y_P =  7.0# *0.75 # 7.0 altitude, z-direction

    gain_img_x_I =  0.25#*0.75 # 0.25 roll, y-direction
    gain_img_y_I =  0.5#*0.75  # 0.5 altitude, z-direction

    gain_img_x_D =  0.09#*0.75 # 0.15 roll, y-direction
    gain_img_y_D =  0.1#*0.75  # 0.05 altitude, z-direction

    image_width = 720 #1024 #720.0
    image_height = 540 #768 #540.0

    max_err_gate_px = [2.5 * image_width , 0.2 * image_height] # 2.5, 0.2 integral windup

    err_gate_px_offset_altitude = 130.0#*0.75

##############################################################################################

    rbt.vel_world_x = rbt.vo_vel.x #(rbt.truth.x - rbt.pos_world_x) * ros_rate
    rbt.vel_world_y = -rbt.vo_vel.y #(-rbt.truth.y - rbt.pos_world_y) * ros_rate
    rbt.vel_world_z = rbt.vo_vel.z #(rbt.truth.z - rbt.pos_world_z) * ros_rate
    rbt.pos_world_x = rbt.vo.x #rbt.truth.x
    rbt.pos_world_y = -rbt.vo.y #-rbt.truth.y
    rbt.pos_world_z = rbt.vo.z #rbt.truth.z

##############################################################################################

    waypoints = [
        [   11.0    ,   0   ,   0.5 ,   0   ]   ,   #   10
        
        [rbt.Gates_after_transformed[0][0] - 6,   rbt.Gates_after_transformed[0][1],       rbt.Gates_after_transformed[0][2] + 1.0,   0]    ,   #   0
        [rbt.Gates_after_transformed[0][0],       rbt.Gates_after_transformed[0][1],       rbt.Gates_after_transformed[0][2] - 0.75,  0]    ,   #   0
        [rbt.Gates_after_transformed[0][0] + 6,   rbt.Gates_after_transformed[0][1] + 0.0, rbt.Gates_after_transformed[0][2] - 0.25,  0]    ,   #   0
        
        [rbt.Gates_after_transformed[1][0] - 12,  rbt.Gates_after_transformed[1][1] + 0.5, rbt.Gates_after_transformed[1][2] + 1.5,   0]  ,   #   1
        [rbt.Gates_after_transformed[1][0] + 3.5, rbt.Gates_after_transformed[1][1] + 3.0, rbt.Gates_after_transformed[1][2],         30]   ,   #   1
        [rbt.Gates_after_transformed[1][0] + 6,   rbt.Gates_after_transformed[1][1] - 8,   rbt.Gates_after_transformed[1][2] + 0.5, 120]  ,   #   1
        
        [rbt.Gates_after_transformed[2][0] + 2.5, rbt.Gates_after_transformed[2][1] - 2,   rbt.Gates_after_transformed[2][2] - 0.25, 155]   ,   #   2
        [rbt.Gates_after_transformed[2][0],       rbt.Gates_after_transformed[2][1],       rbt.Gates_after_transformed[2][2] + 0.25, -180]   ,   #   2
        [rbt.Gates_after_transformed[2][0] - 3.5, rbt.Gates_after_transformed[2][1],       rbt.Gates_after_transformed[2][2] + 0.5,   -180] ,   #   3
        
        [rbt.Gates_after_transformed[3][0] + 5,   rbt.Gates_after_transformed[3][1] + 3.5, rbt.Gates_after_transformed[3][2] + 2.0, -180]   ,   #   3
        [rbt.Gates_after_transformed[3][0],       rbt.Gates_after_transformed[3][1] + 1.0, rbt.Gates_after_transformed[3][2] - 0.5,      170]   ,   #   3
        [rbt.Gates_after_transformed[3][0] - 6,   rbt.Gates_after_transformed[3][1] + 0.5, rbt.Gates_after_transformed[3][2] - 0.5,      140]   ,   #   3
        
        [rbt.Gates_after_transformed[4][0] + 9,   rbt.Gates_after_transformed[4][1] - 0,   rbt.Gates_after_transformed[4][2] + 2.5,  160]   ,   #   4
        [rbt.Gates_after_transformed[4][0],       rbt.Gates_after_transformed[4][1],       rbt.Gates_after_transformed[4][2] + 1.0,  175]   ,   #   4
        [rbt.Gates_after_transformed[4][0] - 3,   rbt.Gates_after_transformed[4][1] - 0,   rbt.Gates_after_transformed[4][2] + 0.0, -180]   ,   #   4
        
        [rbt.Gates_after_transformed[5][0] + 3,   rbt.Gates_after_transformed[5][1] - 5,   rbt.Gates_after_transformed[5][2],     -165] ,   #   5
        [rbt.Gates_after_transformed[5][0] + 1,   rbt.Gates_after_transformed[5][1] + 1,   rbt.Gates_after_transformed[5][2],       -150]   ,   #   5
        [rbt.Gates_after_transformed[5][0] - 2,   rbt.Gates_after_transformed[5][1] - 0.5, rbt.Gates_after_transformed[5][2] + 0.25, -130]    ,   #   5
        
        [rbt.Gates_after_transformed[6][0] + 0.5, rbt.Gates_after_transformed[6][1] - 3.5, rbt.Gates_after_transformed[6][2],     -112.5]   ,   #   6
        [rbt.Gates_after_transformed[6][0],       rbt.Gates_after_transformed[6][1],       rbt.Gates_after_transformed[6][2],     -105] ,   #   6
        [rbt.Gates_after_transformed[6][0],       rbt.Gates_after_transformed[6][1] + 4.5, rbt.Gates_after_transformed[6][2] + 2.00, -80]   ,   #   6
        
        [rbt.Gates_after_transformed[7][0] - 2,   rbt.Gates_after_transformed[7][1] - 2,   rbt.Gates_after_transformed[7][2] + 1.00, -70]   ,   #   7
        [rbt.Gates_after_transformed[7][0] - 1,   rbt.Gates_after_transformed[7][1] - 1,   rbt.Gates_after_transformed[7][2] + 0.25, -50]   ,   #   7
        [rbt.Gates_after_transformed[7][0] + 2,   rbt.Gates_after_transformed[7][1] + 1,   rbt.Gates_after_transformed[7][2] + 0.5, -22.5]  ,   #   8
        
        [rbt.Gates_after_transformed[8][0] - 6,   rbt.Gates_after_transformed[8][1],       rbt.Gates_after_transformed[8][2] + 0.25,  0]    ,   #   8
        [rbt.Gates_after_transformed[8][0],       rbt.Gates_after_transformed[8][1],       rbt.Gates_after_transformed[8][2] + 0,  0]   ,   #   8
        [rbt.Gates_after_transformed[8][0] + 6,   rbt.Gates_after_transformed[8][1],       rbt.Gates_after_transformed[8][2] + 1.0,  65]    ,   #   8
        
        [rbt.Gates_after_transformed[9][0] - 12,  rbt.Gates_after_transformed[9][1] - 0.0,  rbt.Gates_after_transformed[9][2] + 2.0,  45]    ,   #   9
        [rbt.Gates_after_transformed[9][0],       rbt.Gates_after_transformed[9][1],       rbt.Gates_after_transformed[9][2] - 0.5,   5]  ,   #   9
        [rbt.Gates_after_transformed[9][0] + 6,   rbt.Gates_after_transformed[9][1],       rbt.Gates_after_transformed[9][2] + 1.0,   0] ,   #   9
        
        [rbt.Gates_after_transformed[10][0] - 6,  rbt.Gates_after_transformed[10][1],      rbt.Gates_after_transformed[10][2] + 2.0,  0]    ,   #   10
        [rbt.Gates_after_transformed[10][0],      rbt.Gates_after_transformed[10][1],      rbt.Gates_after_transformed[10][2] + 0.0,  0]    ,   #   10
        [rbt.Gates_after_transformed[10][0] + 6,  rbt.Gates_after_transformed[10][1],      rbt.Gates_after_transformed[10][2] + 0.0,  0]    ,   #   10
        
        [rbt.Gates_after_transformed[10][0] + 50,  rbt.Gates_after_transformed[10][1] + 20,      rbt.Gates_after_transformed[10][2] + 0,  0] ,   #   10
        [rbt.Gates_after_transformed[10][0] + 100, rbt.Gates_after_transformed[10][1] + 20,      rbt.Gates_after_transformed[10][2] + 0,  0] ,   #   10
    ]
    #print(waypoints)

    gatesFromWaypoints = [
          10,  
          0, 0, 0,  # 0
          1, 1, 1,  # 1
          2, 2, 3,  # 2
          3, 3, 3,  # 3
          4, 4, 4,  # 4
          5, 5, 5,  # 5
          6, 6, 6,  # 6
          7, 7, 8,  # 7
          8, 8, 8,  # 8
          9, 9, 9,  # 9
          10, 10, 10,  # 10
          10, 10
    ]

    tar_vel = np.array([
        7.0,  
        7.0, 7.0, 10.0,  # 0
        9.5, 9.5, 8.5,  # 1
        9.0, 8.5, 9.0,  # 2
        8.5, 8.0, 7.5,  # 3
        9.5, 9.5, 8.5,  # 4
        9.5, 8.5, 7.5,  # 5
        6.5, 6.0, 6.5,  # 6
        7.0, 7.5, 8.0,  # 7
        8.5, 9.5, 10.0,  # 8
        9.5, 9.0, 8.5,  # 9
        9.0, 8.5, 8.0,  # 10
        8.0, 15.0
    ])-1.0

    rbt.target_gate_pub.publish(gatesFromWaypoints[rbt.ind_wp])
    print("waypoints : %.3f, %.3f, %.3f, %.3f"%(waypoints[rbt.ind_wp][0], waypoints[rbt.ind_wp][1], waypoints[rbt.ind_wp][2], waypoints[rbt.ind_wp][3]))

    err_pos_world_x = waypoints[rbt.ind_wp][0] - rbt.pos_world_x
    err_pos_world_y = waypoints[rbt.ind_wp][1] - rbt.pos_world_y
    err_pos_world_z = waypoints[rbt.ind_wp][2] - rbt.pos_world_z
    distance_next_waypoint = sqrt(err_pos_world_x**2 + err_pos_world_y**2 + err_pos_world_z**2)

    angle_btw_waypoints = atan2(waypoints[rbt.ind_wp+1][0] - waypoints[rbt.ind_wp][0], waypoints[rbt.ind_wp+1][1] - waypoints[rbt.ind_wp][1])
    tar_pos_world_x = waypoints[rbt.ind_wp][0] + rbt.moving_distance * sin(angle_btw_waypoints)
    tar_pos_world_y = waypoints[rbt.ind_wp][1] + rbt.moving_distance * cos(angle_btw_waypoints)
    distance_next_waypoint_from_cur_waypoint = sqrt((waypoints[rbt.ind_wp+1][0] - waypoints[rbt.ind_wp][0])**2 + (waypoints[rbt.ind_wp+1][1] - waypoints[rbt.ind_wp][1])**2)
    distance_next_waypoint_from_tar = sqrt((waypoints[rbt.ind_wp+1][0] - tar_pos_world_x)**2 + (waypoints[rbt.ind_wp+1][1] - tar_pos_world_y)**2)
    tar_pos_world_z = waypoints[rbt.ind_wp][2] + (waypoints[rbt.ind_wp+1][2] - waypoints[rbt.ind_wp][2]) * (1 - distance_next_waypoint_from_tar / distance_next_waypoint_from_cur_waypoint)
    err_way_ang_body_z = waypoints[rbt.ind_wp+1][3] - waypoints[rbt.ind_wp][3]
    if err_way_ang_body_z > 180:
        err_way_ang_body_z = err_way_ang_body_z - 360
    if err_way_ang_body_z < -180:
        err_way_ang_body_z = err_way_ang_body_z + 360
    tar_ang_body_z = waypoints[rbt.ind_wp][3] + err_way_ang_body_z * (1 - distance_next_waypoint_from_tar / distance_next_waypoint_from_cur_waypoint)
    print("tar_pos_world_x : %.3f, tar_pos_world_y : %.3f, tar_pos_world_z : %.3f, tar_ang_body_z : %.3f"%(tar_pos_world_x, tar_pos_world_y, tar_pos_world_z, tar_ang_body_z))
    err_tar_pos_world_x = tar_pos_world_x - rbt.pos_world_x
    err_tar_pos_world_y = tar_pos_world_y - rbt.pos_world_y
    err_tar_pos_world_z = tar_pos_world_z - rbt.pos_world_z

    err_pos_body_x = err_tar_pos_world_x * cos(rbt.yaw) - err_tar_pos_world_y * sin(rbt.yaw)
    err_pos_body_y = err_tar_pos_world_x * sin(rbt.yaw) + err_tar_pos_world_y * cos(rbt.yaw)
    err_pos_body_z = err_tar_pos_world_z

    rbt.err_sum_pos_body_x += err_pos_body_x
    rbt.err_sum_pos_body_y += err_pos_body_y

    if rbt.err_sum_pos_body_x  > max_err_sum_pos_body_x:
        rbt.err_sum_pos_body_x = max_err_sum_pos_body_x
    if rbt.err_sum_pos_body_x  < -max_err_sum_pos_body_x:
        rbt.err_sum_pos_body_x = -max_err_sum_pos_body_x
    if rbt.err_sum_pos_body_y  > max_err_sum_pos_body_y:
        rbt.err_sum_pos_body_y = max_err_sum_pos_body_y
    if rbt.err_sum_pos_body_y  < -max_err_sum_pos_body_y:
        rbt.err_sum_pos_body_y = -max_err_sum_pos_body_y
    if rbt.err_sum_pos_body_z  > max_err_sum_pos_body_z:
        rbt.err_sum_pos_body_z = max_err_sum_pos_body_z
    if rbt.err_sum_pos_body_z  < -max_err_sum_pos_body_z:
        rbt.err_sum_pos_body_z = -max_err_sum_pos_body_

    rbt.com_vel_body_x =  gain_pos_x_mission_P * err_pos_body_x + gain_pos_x_mission_I * rbt.err_sum_pos_body_x
    rbt.com_vel_body_y =  gain_pos_y_mission_P * err_pos_body_y + gain_pos_y_mission_I * rbt.err_sum_pos_body_y
    rbt.com_vel_body_z =  gain_pos_z_mission_P * err_pos_body_z

    if rbt.com_vel_body_x  >  max_vel_body_x_mission:
        rbt.com_vel_body_x =  max_vel_body_x_mission
    if rbt.com_vel_body_x  < -max_vel_body_x_mission:
        rbt.com_vel_body_x = -max_vel_body_x_mission
    if rbt.com_vel_body_y  >  max_vel_body_y_mission:
        rbt.com_vel_body_y =  max_vel_body_y_mission
    if rbt.com_vel_body_y  < -max_vel_body_y_mission:
        rbt.com_vel_body_y = -max_vel_body_y_mission
    if rbt.com_vel_body_z  >  max_vel_body_z_mission:
        rbt.com_vel_body_z =  max_vel_body_z_mission
    if rbt.com_vel_body_z  < -max_vel_body_z_mission:
        rbt.com_vel_body_z = -max_vel_body_z_mission

    err_ang_body_z = tar_ang_body_z * d2r - rbt.yaw

    if err_ang_body_z > 180 * d2r:
        err_ang_body_z = err_ang_body_z - 360 * d2r
    if err_ang_body_z < -180 * d2r:
        err_ang_body_z = err_ang_body_z + 360 * d2r

    rbt.com_ang_body_z_yaw =  gain_ang_r_yaw_P * err_ang_body_z
    if (distance_next_waypoint_from_tar < waypoint_radius):
        rbt.ind_wp = rbt.ind_wp + 1
        rbt.count_ramp_vel = 0
        rbt.err_sum_pos_body_x = 0
        rbt.err_sum_pos_body_y = 0
        rbt.err_sum_pos_body_z = 0
        rbt.err_sum_gate_px = [0, 0]
        rbt.moving_distance = 0

    rbt.count_ramp_vel = rbt.count_ramp_vel + 1
    if rbt.count_ramp_vel > ros_rate*3:
        rbt.count_ramp_vel = ros_rate*3
    rbt.moving_distance += tar_vel[rbt.ind_wp] /ros_rate

    # image coordinate - attitude compensation
    angle_btw_center_and_gate = atan2(rbt.err_gate_px[1], rbt.err_gate_px[0])
    distance_btw_center_and_gate = sqrt(rbt.err_gate_px[0]**2 + rbt.err_gate_px[1]**2)
    rbt.err_gate_px_comp[0] = distance_btw_center_and_gate * cos(angle_btw_center_and_gate - rbt.roll)
    rbt.err_gate_px_comp[1] = distance_btw_center_and_gate * sin(angle_btw_center_and_gate - rbt.roll)
    if (rbt.err_gate_px[0] == 0) and (rbt.err_gate_px[1] == 0):
        pass
    else:
        rbt.err_gate_px_comp[1] += err_gate_px_offset_altitude

    rbt.err_sum_gate_px[0] += rbt.err_gate_px_comp[0]
    rbt.err_sum_gate_px[1] += rbt.err_gate_px_comp[1]

    if rbt.err_sum_gate_px[0]  > max_err_gate_px[0]:
        rbt.err_sum_gate_px[0] = max_err_gate_px[0]
    if rbt.err_sum_gate_px[0]  < -max_err_gate_px[0]:
        rbt.err_sum_gate_px[0] = -max_err_gate_px[0]
    if rbt.err_sum_gate_px[1]  > max_err_gate_px[1]:
        rbt.err_sum_gate_px[1] = max_err_gate_px[1]
    if rbt.err_sum_gate_px[1]  < -max_err_gate_px[1]:
        rbt.err_sum_gate_px[1] = -max_err_gate_px[1]

    if (rbt.err_gate_px[0] != 0) or (rbt.err_gate_px[1] != 0):
        rbt.com_vel_body_y /= com_vel_waypoint_divide_factor

    rbt.com_vel_body_y +=  rbt.err_gate_px_comp[0] / image_width * gain_img_x_P + rbt.err_sum_gate_px[0] / image_width * gain_img_x_I + (rbt.err_gate_px_comp[0] - rbt.pre_err_gate_px_comp[0]) / ros_rate / image_width * gain_img_x_D
    rbt.com_vel_body_z += -(rbt.err_gate_px_comp[1] / image_height * gain_img_y_P + rbt.err_sum_gate_px[1] / image_height * gain_img_y_I + (rbt.err_gate_px_comp[1] - rbt.pre_err_gate_px_comp[1]) / ros_rate / image_height * gain_img_y_D)

    rbt.pre_err_gate_px_comp[0] = rbt.err_gate_px_comp[0]
    rbt.pre_err_gate_px_comp[1] = rbt.err_gate_px_comp[1]

    rbt.vel_body_x = rbt.vel_world_x * cos(rbt.yaw) - rbt.vel_world_y * sin(rbt.yaw)
    rbt.vel_body_y = rbt.vel_world_x * sin(rbt.yaw) + rbt.vel_world_y * cos(rbt.yaw)
    rbt.com_ang_body_x_roll  =  gain_vel_y_mission_P * (rbt.com_vel_body_y - rbt.vel_body_y)
    rbt.com_ang_body_y_pitch =  gain_vel_x_mission_P * (rbt.com_vel_body_x - rbt.vel_body_x)

    if rbt.com_ang_body_x_roll   >  max_ang_body_x_mission:
        rbt.com_ang_body_x_roll  =  max_ang_body_x_mission
    if rbt.com_ang_body_x_roll   < -max_ang_body_x_mission:
        rbt.com_ang_body_x_roll  = -max_ang_body_x_mission
    if rbt.com_ang_body_y_pitch  >  max_ang_body_y_mission:
        rbt.com_ang_body_y_pitch =  max_ang_body_y_mission
    if rbt.com_ang_body_y_pitch  < -max_ang_body_y_mission:
        rbt.com_ang_body_y_pitch = -max_ang_body_y_mission

    rbt.deg.angular_rates.x =  gain_ang_x_roll_P * (rbt.com_ang_body_x_roll - rbt.roll)
    rbt.deg.angular_rates.y =  gain_ang_y_pitch_P * (rbt.com_ang_body_y_pitch - rbt.pitch)
    rbt.deg.angular_rates.z =  rbt.com_ang_body_z_yaw * max_ang_rate_body_z
    rbt.deg.thrust.z = 9.81 + gain_thr_z_altitude_P * (rbt.com_vel_body_z - rbt.vel_world_z)

    if abs(rbt.deg.angular_rates.x) < 0.08:
        rbt.deg.angular_rates.x=0
    if abs(rbt.deg.angular_rates.y) < 0.08:
        rbt.deg.angular_rates.y=0
    if abs(rbt.deg.angular_rates.z) < 0.08:
        rbt.deg.angular_rates.z=0    

    rbt.deg.header.stamp = rospy.Time.now()
    rbt.local_deg_pub.publish(rbt.deg)


##############################################################################################
time.sleep(1)
alpha = robot()
time.sleep(5) #wait 1 second to assure that all data comes in
rate = rospy.Rate(ros_rate)

''' main '''
if __name__ == '__main__':
 alpha.auto_taking_off()
 #time.sleep(3.5)
 rospy.sleep(3.5*0.5)
 while 1:
    try:
        input(alpha)
        rate.sleep()
    except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
        pass
