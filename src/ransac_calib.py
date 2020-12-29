#!/usr/bin/env python
"""
Zhiang Chen
Dec 2020
T265 external calibration
"""

import rospy
from nav_msgs.msg import Odometry
import numpy as np
import message_filters
import tf
import random
from geometry_msgs.msg._Pose import Pose
from sklearn.linear_model import RANSACRegressor

class Calibrator(object):
    def __init__(self):
        self.samples = []
        self.distance_threshold = 0.1
        self.distance_max = 2.0
        self.angle_threshold = 15/180.*np.pi
        self.previous_vio_rot = 0
        self.sub_gps = message_filters.Subscriber('/gps/odom', Odometry)
        self.sub_vio = message_filters.Subscriber('/vio/odom', Odometry)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.sub_gps, self.sub_vio], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.callback)
        rospy.loginfo("calibrator has been intialized.")
    
    
    def callback(self, gps_data, vio_data):
        gps_pos = self.getPositionFromOdom(gps_data)
        vio_pos = self.getPositionFromOdom(vio_data)
        gps_rot = self.getRotFromOdom(gps_data)
        vio_rot = self.getRotFromOdom(vio_data)
        # append sample
        sample = np.asarray((gps_pos, vio_pos)).transpose()
        if len(self.samples) == 0:
            self.samples.append(sample)
            self.previous_vio_rot = vio_rot
        else:
            previous_sample = self.samples[-1]
            distance = np.linalg.norm(previous_sample - sample)
            if distance < self.distance_threshold:
                return
            
            angle = self.calcRotationDiff(self.previous_vio_rot, vio_rot)
            
            if angle <= self.angle_threshold:
                self.samples.append(sample)
                self.resetDistanceThreshold()
                self.previous_vio_rot = vio_rot
        # estimate transform using RANSAC
        if len(self.samples) % 10 == 0:
            self.estimateTransform()
        
        
    def resetDistanceThreshold(self):
        self.distance_threshold = random.random()*self.distance_max
        print("Next translation goal: ", self.distance_threshold)
    
    def calcRotationDiff(self, r1, r2):
        err_matrix = (np.matmul(r1.transpose(),r2) - np.matmul(r1,r2.transpose()))/2.
        x3 = err_matrix[1, 0]
        x2 = err_matrix[0, 2]
        x1 = err_matrix[2, 1]
        return (x1 + x2 + x3)
    
    def estimateTransform(self):
        # get delta data
        delta_gps = []
        delta_vio = []
        N = len(self.samples)
        for i in range(N-1):
            pos = self.samples[i]
            pos_ = self.samples[i+1]
            delta = pos_ - pos 
            delta_gps.append(delta[:, 0])
            delta_vio.append(delta[:, 1])
            
        # ransac
        # https://scikit-learn.org/stable/modules/generated/sklearn.linear_model.RANSACRegressor.html
        print(delta_gps)
        print(delta_vio)
    
    def getPositionFromOdom(self, odom):
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        z = odom.pose.pose.position.z
        return np.asarray((x, y, z, 1))
    
    def getRotFromOdom(self, odom):
        x = odom.pose.pose.orientation.x
        y = odom.pose.pose.orientation.y
        z = odom.pose.pose.orientation.z
        w = odom.pose.pose.orientation.w
        qua = (x, y, z, w)
        rot = tf.transformations.quaternion_matrix(qua)
        return rot
        
     
        
class Transformer(object):
    def __init__(self):
        trans_vec = (0, 0, 1)
        trans = tf.transformations.translation_matrix(trans_vec)
        quaternion = (0.3826834, 0, 0, 0.9238795)
        rot = tf.transformations.quaternion_matrix(quaternion)
        self.T = np.matmul(trans, rot)
        
        self.sub_vio = rospy.Subscriber('/mavros/odometry/in', Odometry, self.vioCallback, queue_size=1)
        self.pub_vio = rospy.Publisher('/vio/odom', Odometry, queue_size=1)
        
    def vioCallback(self, vio_odom):
        tf_odom = Odometry()
        tf_odom.header = vio_odom.header
        x = vio_odom.pose.pose.position.x
        y = vio_odom.pose.pose.position.y
        z = vio_odom.pose.pose.position.z
        pos = (x, y, z)
        x = vio_odom.pose.pose.orientation.x
        y = vio_odom.pose.pose.orientation.y
        z = vio_odom.pose.pose.orientation.z
        w = vio_odom.pose.pose.orientation.w
        qua = (x, y, z, w)
        trans = tf.transformations.translation_matrix(pos)
        rot = tf.transformations.quaternion_matrix(qua)
        pose = np.matmul(trans, rot)
        tf_pose = np.matmul(self.T, pose)
        tf_pos = tf.transformations.translation_from_matrix(tf_pose)
        tf_qua = tf.transformations.quaternion_from_matrix(tf_pose)
        tf_odom.pose.pose.position.x = tf_pos[0]
        tf_odom.pose.pose.position.y = tf_pos[1]
        tf_odom.pose.pose.position.z = tf_pos[2]
        tf_odom.pose.pose.orientation.x = tf_qua[0]
        tf_odom.pose.pose.orientation.y = tf_qua[1]
        tf_odom.pose.pose.orientation.z = tf_qua[2]
        tf_odom.pose.pose.orientation.w = tf_qua[3]
        self.pub_vio.publish(tf_odom)
        
        
if __name__ == '__main__':
    rospy.init_node('calibrator', anonymous=False)
    vio_Transformer = Transformer()
    calib = Calibrator()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node killed!")