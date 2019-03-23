#!/usr/bin/env python
#Load libraries
import numpy as np
import os
import sys
import cv2
import time

#ROS libraries
import rospy
from flightgoggles.msg import IRMarkerArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32
import sys
import signal

def signal_handler(signal, frame): 
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

## gate_info (gate_id, upper_sequence, lower_sequence) / 0: not change 1: change
gate_marker_seq = [
    [10,1,0],
    [21,0,0],
    [2,1,1],
    [13,1,1],
    [9,0,1],
    [14,1,1],
    [1,0,0],
    [22,0,0],
    [15,0,0],
    [23,1,0],
    [6,1,0]]
img_center_x = int(720 / 2)
img_center_y = int(540 / 2)

def swap(s1, s2):
    temp = s1.copy()
    s1 = s2.copy()
    s2 = temp.copy()
    return s1, s2

class gate_detector:
    def __init__(self):
        # self.image_pub = rospy.Publisher("debug_image",Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/uav/camera/left/image_rect_color", Image, self.image_rect, queue_size=2, buff_size=2**24)
        self.target_gate_sub = rospy.Subscriber("/uav/current_target_gate", Int32, self.target_sub)
        self.ir_sub = rospy.Subscriber("/uav/camera/left/ir_beacons", IRMarkerArray, self.ir_subscribe)
        self.image_pub = rospy.Publisher("/uav/ir_debug_image",Image, queue_size=1)
        self.gate_px_pub = rospy.Publisher("/uav/current_target_gate_error",Int32MultiArray, queue_size=1)
        self.current_target_gate = 0
        self.is_img = 0

        self.center_point = np.empty((2,), dtype='int')
        self.center_point[:] = np.nan

    def target_sub(self, data):
        self.current_target_gate = data.data

    def image_rect(self, data):
        try:
            self.ir_img = self.bridge.imgmsg_to_cv2(data, "rgb8")
            self.is_img = 1
     
        except CvBridgeError as e:
            print(e)

    def ir_subscribe(self, data):
        target_point = np.empty((4,2,))
        target_point[:] = np.nan
        target = "Gate" + str(gate_marker_seq[self.current_target_gate][0])
        is_marker_detected = 0
        marker_point = np.zeros((4))
        tot_point = 0

        # Import marker data to local variable
        for i in range(len(data.markers)):
            if data.markers[i].landmarkID.data == target:
                marker_id = int(data.markers[i].markerID.data)
                target_point[marker_id-1][0] = data.markers[i].x
                target_point[marker_id-1][1] = data.markers[i].y
                marker_point[marker_id-1] = 1
                tot_point += 1
                is_marker_detected = 1

        if(is_marker_detected):
            ## Gate marker sequence align
            if gate_marker_seq[self.current_target_gate][1] == 1 :
                target_point[0], target_point[1] = swap(target_point[0], target_point[1])
                marker_point[0], marker_point[1] = marker_point[1], marker_point[0]
            if gate_marker_seq[self.current_target_gate][2] == 1 :
                target_point[2], target_point[3] = swap(target_point[2], target_point[3])
                marker_point[2], marker_point[3] = marker_point[3], marker_point[2]
        
            ## Gate Center Calculation
            if tot_point == 0:
                self.center_point[0] = img_center_x
                self.center_point[1] = img_center_y
                print("No markers are visible.")
            elif tot_point == 1:
                self.center_point[0] = img_center_x
                self.center_point[1] = img_center_y
                print("Only one marker is visible.")
            elif tot_point == 2:
                if marker_point[0] == 1 and marker_point[1] == 1:
                    self.center_point[0] = int((target_point[0][0] + target_point[1][0]) / 2.0)
                    self.center_point[1] = int(((target_point[0][1] + target_point[1][1]) / 2.0) + ((target_point[1][0] - target_point[0][0]) / 2.0))
                elif marker_point[1] == 1 and marker_point[2] == 1:
                    self.center_point[0] = int(((target_point[1][0] + target_point[2][0]) / 2.0) - ((target_point[2][1] - target_point[1][1]) / 2.0))
                    self.center_point[1] = int((target_point[1][1] + target_point[2][1]) / 2.0)
                elif marker_point[2] == 1 and marker_point[3] == 1:
                    self.center_point[0] = int((target_point[2][0] + target_point[3][0]) / 2.0)
                    self.center_point[1] = int(((target_point[2][1] + target_point[3][1]) / 2.0) - ((target_point[2][0] - target_point[3][0]) / 2.0))
                elif marker_point[3] == 1 and marker_point[0] == 1:
                    self.center_point[0] = int(((target_point[0][0] + target_point[3][0]) / 2.0) + ((target_point[3][1] - target_point[0][1]) / 2.0))
                    self.center_point[1] = int((target_point[0][1] + target_point[3][1]) / 2.0)
                print("Two markers are visible.")
            elif tot_point == 3:
                non_visible_point = np.where(marker_point == 0)
                if non_visible_point[0][0] == 0:
                    self.center_point[0] = int((target_point[1][0] + target_point[3][0]) / 2.0)
                    self.center_point[1] = int((target_point[1][1] + target_point[3][1]) / 2.0)
                elif non_visible_point[0][0] == 1:
                    self.center_point[0] = int((target_point[0][0] + target_point[2][0]) / 2.0)
                    self.center_point[1] = int((target_point[0][1] + target_point[2][1]) / 2.0)
                elif non_visible_point[0][0] == 2:
                    self.center_point[0] = int((target_point[1][0] + target_point[3][0]) / 2.0)
                    self.center_point[1] = int((target_point[1][1] + target_point[3][1]) / 2.0)
                elif non_visible_point[0][0] == 3:
                    self.center_point[0] = int((target_point[0][0] + target_point[2][0]) / 2.0)
                    self.center_point[1] = int((target_point[0][1] + target_point[2][1]) / 2.0)
                print("Three markers are visible.")
            elif tot_point == 4:
                self.center_point[0] = int((target_point[0][0] + target_point[1][0] + target_point[2][0] + target_point[3][0]) / 4.0)
                self.center_point[1] = int((target_point[0][1] + target_point[1][1] + target_point[2][1] + target_point[3][1]) / 4.0)
                print("Four markers are visible.")
        else:
            self.center_point[0] = img_center_x
            self.center_point[1] = img_center_y
        print('X : ' + str(self.center_point[0]) + 'Y : ' + str(self.center_point[1]))
        err_x = self.center_point[0] - img_center_x
        err_y = self.center_point[1] - img_center_y
        self.gate_px_pub.publish(data=(err_x,err_y))
        if self.is_img == 1:
            if (self.center_point[0] == img_center_x) and (self.center_point[1] == img_center_y):
                image_out = cv2.circle(self.ir_img, (self.center_point[0],self.center_point[1]), 10, (0,0,255), -1)
            else:
                image_out = cv2.circle(self.ir_img, (self.center_point[0],self.center_point[1]), 10, (255,0,0), -1)
            image_out = self.bridge.cv2_to_imgmsg(image_out, encoding="rgb8")
            self.image_pub.publish(image_out)

def main(args):
    obj=gate_detector()
    rospy.init_node('gate_detector')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("ShutDown")

if __name__=='__main__':
    main(sys.argv)
