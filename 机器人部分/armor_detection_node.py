#! /home/lyjslay/py3env/bin python
# coding=utf-8
#================================================================
#   Copyright (C) 2019 * Ltd. All rights reserved.
#
#   File name   : armor_detectoion_node.py
#   Author      : Liu Yijun
#   E-Mail      : 2446078134@qq.com
#   Description : Armor Detection ROS Node
#
#================================================================
import sys
import KCF
import time
import rospy
import multiprocessing
from ctypes import c_bool
from multiprocessing import Process, Value, Array
from cv_bridge import CvBridge, CvBridgeError
from detector import *
from tracker import *
from camera import *
from shared_ram import *
from sensor_msgs.msg import Image
from roborts_msgs.msg import GimbalAngle, EnemyDirection
from roborts_msgs.srv import FricWhl, ShootCmd


class ArmorDetectionNode():
    '''Main Process ROS Node
    '''
    def __init__(self):
        
        rospy.init_node('armor_detection_node')
        
        #self.target_bbox = target_bbox
        self.bridge = CvBridge()
        self._ctrlinfo_pub = rospy.Publisher('/cmd_gimbal_angle', 
                                             GimbalAngle, queue_size=1, 
                                             tcp_nodelay=True)
        self._direction_pub = rospy.Publisher('/enemy_direc',
                                              EnemyDirection,queue_size=1, 
                                              tcp_nodelay=True)
        self._image_sub = rospy.Subscriber('/camera/image', 
                                           Image, self._update_images, 
                                           tcp_nodelay=True)
        self._fricwhl_client = rospy.ServiceProxy("/cmd_fric_wheel",FricWhl)
        self._shoot_client = rospy.ServiceProxy("/cmd_shoot",ShootCmd)
        self._can_ctrl = True
        undet_count = 40
        #num_bullets = 100
        self.camera_matrix = np.array(([1750, 0, 356.3],
                                       [0, 1756, 375.9],
                                       [0, 0, 1.0]), dtype=np.double)
        self.dist_coefs = np.array([0, 0, 0, 0, 0], dtype=np.double)
        object_3d_points = np.array(([-72, -32, 0],                 #xmin ymin
                                     [-58, 32, 0],                  #xmin ymax
                                     [58, -32, 0],                  #xmax ymax 
                                     [72, 32, 0]), dtype=np.double) #xmax ymin

        # main process loop
        while not rospy.is_shutdown():
            angle = self.calcu_angle(boundingbox)
            if self._can_ctrl:
                if angle is not None:
                    self._set_fricwhl(True)
                    self._ctrlinfo_pub.publish(angle[0], angle[1])
                    self._direction_pub.publish(direction.value)
                    self._shoot(1,1)
                    rospy.loginfo('pitch '+str(angle[0])+' yaw '+str(angle[1]))
                elif undet_count != 0:
                    self._set_fricwhl(False)
                    self._shoot(0,0)
                    undet_count -= 1
                    self._ctrlinfo_pub.publish(angle[0],angle[1])
                    rospy.loginfo('pitch '+str(angle[0])+' yaw '+str(angle[1]))
                else:
                    self._set_fricwhl(False)
                    self._shoot(0,0)
                    #TODO: define searching mode
                    #searching_mode()
                    rospy.loginfo('searching')
            else:
                rospy.loginfo('decision node needs to control the gimbal')
                
            #rospy.sleep(0.005) # 200Hz frequency

    def _update_images(self, img_msg):
        '''image subscriber callback
        '''
        cv_image = self.bridge.imgmsg_to_cv2(img_msg,"bgr8")
        image_in[:] = cv_image.copy()

    def _update_ctrlpower(self, ctrlpower_msg):
        '''decision node callback
        '''
        self.can_ctrl = ctrlpower_msg.data

    def _set_fricwhl(can_start):
        '''fricwheel service client
        '''
        rospy.wait_for_service("cmd_fric_wheel")
        try:
            resp = fricwhl_client.call(can_start)
            #rospy.loginfo("Message From fricwheelserver:%s"%resp.received)
        except rospy.ServiceException:
            rospy.logwarn("Service call failed")

    def _shoot(shoot_mode, shoot_number):
        '''shoot service client
        '''
        rospy.wait_for_service("cmd_fric_wheel")
        try:
            resp = shoot_client.call(shoot_mode, shoot_number)
            #rospy.loginfo("Message From shootserver:%s"%resp.received)
        except rospy.ServiceException:
            rospy.logwarn("Service call failed")

    #TODO:High accuracy but slow speed
    def calc_xyz_ang(self, roi_orig, box):
        '''calculate the yaw and ptich angle according to lghtbar
        '''
        #frame = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        pnp_mask = cv2.inRange(roi_gray, lower_pnpclr, upper_allclr)
        # find connected region and get its xmin ymin height width area
        _, labels, stats, _ = cv2.connectedComponentsWithStats(pnp_mask)
        # 2 max area is lightbar(except the whole fig)
        stats = stats[stats[:,4].argsort()][-3:-1]
        # sort according to xmin in order to adapt to p1p2p3p4
        stats = stats[stats[:,0].argsort()]
        #print(stats)
        #[box_xmin+roi_xmin,box_ymin+roi_ymin]
        p1 = [box[1] + stats[0][0], box[0] + stats[0][1]] # left top
        p2 = [box[1] + stats[0][0] + stats[0][2], 
              box[0] + stats[0][1] + stats[0][3]] #left bottom
        p3 = [box[1] + stats[1][0], box[0] + stats[1][1]] # right top
        p4 = [box[1] + stats[1][0] + stats[1][2], 
              box[0] + stats[1][1] + stats[1][3]] # right bottom
        #else:
            #print('no enough ponit'+str(nlabels))
            #return float(0.0), float(0.0),float(0.0)
        object_2d_point = np.array((p1, p2, p3, p4), dtype=np.double)
        # calc tvec to get xyz
        found, rvec, tvec = cv2.solvePnP(object_3d_points, object_2d_point, 
                                         camera_matrix, dist_coefs, 
                                         cv2.SOLVEPNP_EPNP)
        # calc pitch
        pitch = float(np.arctan2(tvec[1][0]+OFFSET_Y, tvec[2][0]+OFFSET_Z)) 
        yaw   = float(np.arctan2(tvec[0][0]+OFFSET_X, tvec[2][0]+OFFSET_Z))
        # x,y,z,pitch,yaw
        return tvec[0][0], tvec[1][0], tvec[2][0]

    def calcu_angle(self, bbox):
        if bbox[2] == 0:
            return None
        else:
            # [ymin xmin ymax xmax]
            box = [bbox[1], bbox[0], bbox[1]+bbox[3], bbox[0]+bbox[2]] 
            object_2d_point = np.array(([box[1],box[0]],[box[1],box[2]],
                                        [box[3],box[2]],[box[3],box[0]]),
                                        dtype=np.double)
            _, _, tvec = cv2.solvePnP(self.object_3d_points, object_2d_point, 
                                      self.camera_matrix, self.dist_coefs, 
                                      cv2.SOLVEPNP_EPNP)
            pitch = float(np.arctan2(tvec[1][0], tvec[2][0])) 
            yaw   = float(np.arctan2(tvec[0][0], tvec[2][0]))
            return [pitch, yaw]


if __name__ == '__main__':

    #multiprocessing.set_start_method('spawn')
    # control the sub processes run
    detecting   = Value(c_bool, True)
    initracker  = Value(c_bool, False)
    tracking    = Value(c_bool, False)
    flag        = Value('I', 0)  # num of tracking frames
    direction   = Value('I', 7)  # default direction is tracking
    # ArmorInfo varibles shared by all process
    boundingbox = Array('I', [0, 0, 0, 0]) # unsigned int bbox
    # init image shared memory
    image_in    = empty(640*480*3, 'uint8')
    ENEMY_COLOR = 1 #blue
    # create 2 subprocesses
    detector = Detector('detector', detecting, tracking, initracker, 
                        boundingbox, direction, image_in, ENEMY_COLOR)
    tracker  = Tracker ('tracker', detecting, tracking, initracker, 
                        boundingbox, flag, image_in)
    detector.start()
    tracker.start()

    # create ros node, start main process loop
    armor_detection_node = ArmorDetectionNode()

