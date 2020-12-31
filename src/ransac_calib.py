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
        self.P_rr = []
        self.p_rr = []
        self.p_cc = []
        self.calib_rot_result = False
        self.calib_trans_result = False
        self.distance_threshold = 0.1
        self.distance_max = 2.0
        self.trans_angle_threshold = 15/180.*np.pi
        self.rot_angle_threshold = 15/180.*np.pi
        self.rot_min = 15/180.*np.pi
        self.rot_max = 70/180.*np.pi
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
        self.calibrateTransform(gps_pos, vio_pos, gps_rot, vio_rot)

    def calibrateTransform(self, gps_pos, vio_pos, gps_rot, vio_rot):
        self.calib_rot_result = True
        # calibrate rotation matrix
        if not self.calib_rot_result:
            R = self.calibrateRotation(gps_pos, vio_pos, gps_rot, vio_rot)
        
        # assume the rotation is known for test purpose
        self.calib_trans_result = False
        trans_vec = (0, 0, 0)
        trans = tf.transformations.translation_matrix(trans_vec)
        rot = tf.transformations.euler_matrix(0.5, 1.0471975512, -0.5)
        T_robot2vio = np.matmul(trans, rot)
        R = np.linalg.inv(T_robot2vio)
        # calibrate translation matrix
        if not self.calib_trans_result:
            trans = self.calibrateTranslation(R, gps_pos, vio_pos, gps_rot, vio_rot)
    
    def calibrateTransform2(self):
        pass
    
    def calibrateRotation(self, gps_pos, vio_pos, gps_rot, vio_rot):
        # append sample
        sample = np.asarray((gps_pos, vio_pos)).transpose()
        if len(self.samples) == 0:
            self.append(sample, gps_rot, gps_pos, vio_pos)
            self.previous_vio_rot = vio_rot
        else:
            previous_sample = self.samples[-1]
            distance = np.linalg.norm(previous_sample - sample)
            if distance < self.distance_threshold:
                return
            
            angle = self.calcRotationDiff(self.previous_vio_rot, vio_rot)
            
            if angle <= self.trans_angle_threshold:
                self.append(sample, gps_rot, gps_pos, vio_pos)
                self.resetDistanceThreshold()
                self.previous_vio_rot = vio_rot
                # estimate rotation using RANSAC
                if len(self.samples) % 10 == 0:
                    R = self.estimateRotation()
                    self.calib_rot_result = True
                    return R
                else:
                    return None
            else:
                return None
    
    def calibrateTranslation(self, R, gps_pos, vio_pos, gps_rot, vio_rot):
        # append sample
        sample = np.asarray((gps_pos, vio_pos)).transpose()
        if len(self.samples) == 0:
            self.append(sample, gps_rot, gps_pos, vio_pos)
            self.previous_vio_rot = vio_rot
        else:
            previous_sample = self.samples[-1]
            angle = self.calcRotationDiff(self.previous_vio_rot, vio_rot)
            
            #print(angle, self.rot_angle_threshold)
            if angle >= self.rot_angle_threshold:
                self.append(sample, gps_rot, gps_pos, vio_pos)
                self.resetRotationThreshold()
                self.previous_vio_rot = vio_rot
                # estimate translation using RANSAC
                if len(self.samples) % 10 == 0:
                    trans = self.estimateTranslation(R)
                    self.calib_trans_result = True
                    return trans
                else:
                    return None
            else:
                return None
        
    def estimateRotation(self):
        # get ransac data
        Y = []
        X = []
        p_gps = np.array([sample[:, 0] for sample in self.samples])
        p_vio = np.array([sample[:, 1] for sample in self.samples])
        N = len(self.samples)
        for i in range(N-1):
            pos = self.samples[i]
            pos_ = self.samples[i+1]
            delta = pos_ - pos 
            delta_gps = delta[:, 0]
            delta_vio = delta[:, 1]
            x1 = np.array((delta_vio[0], delta_vio[1], delta_vio[2], 0, 0, 0, 0, 0, 0, 1, 0, 0))
            y1 = delta_gps[0]
            x2 = np.array((0, 0, 0, delta_vio[0], delta_vio[1], delta_vio[2], 0, 0, 0, 0, 1, 0))
            y2 = delta_gps[1]
            x3 = np.array((0, 0, 0, 0, 0, 0, delta_vio[0], delta_vio[1], delta_vio[2], 0, 0, 1))
            y3 = delta_gps[2]
            X.append(x1)
            X.append(x2)
            X.append(x3)
            Y.append(y1)
            Y.append(y2)
            Y.append(y3)
        X = np.asarray(X)
        Y = np.asarray(Y)
        print("equation number: ", X.shape[0])
        # ransac
        # https://scikit-learn.org/stable/modules/generated/sklearn.linear_model.RANSACRegressor.html
        X = X[:, :-3]
        reg = RANSACRegressor(random_state=0).fit(X, Y)
        params = reg.estimator_.coef_
        inlier_mask = reg.inlier_mask_
        R = params.reshape((3,3))
        # todo: find the closest rotation matrix in SO(3)
        print(R)
        return R
        
    def estimateTranslation(self, R):
        P_rr_n = np.array(self.P_rr)
        p_rr_n = np.array(self.p_rr)
        p_cc_n = np.array(self.p_cc)
        N = P_rr_n.shape[0]
        X = []
        Y = []
        for i in range(N):
            P_rr = P_rr_n[i]
            p_rr = p_rr_n[i]
            p_cc = p_cc_n[i]
            A, r = self.computeTanslationSystemParam(P_rr, p_rr, p_cc, R)
            for j in range(3):
                X.append(A[j, :])
                Y.append(r[j])
            
            inv_P_rr = np.linalg.inv(P_rr)
            trans_vec = (0.1, 0, -0.01)
            trans = tf.transformations.translation_matrix(trans_vec)
            rot = tf.transformations.euler_matrix(0.5, 1.0471975512, -0.5)
            T = np.matmul(trans, rot)
            T_gt = np.linalg.inv(T)
            p_rc = np.matmul(T_gt, p_cc)
            trans_vec = T_gt[:3, 3]
            T0 = tf.transformations.translation_matrix(trans_vec)
            p_const = np.matmul(inv_P_rr, p_rc) - np.matmul(inv_P_rr, p_rr)
            print('left')
            print(np.matmul(np.matmul(np.matmul(P_rr, T0), inv_P_rr), p_rr))
            print('right')
            print(np.matmul(T_gt, p_cc))
            print('p_const')
            print(p_const)
            print(P_rr)
            print(A)
        
        X = np.asarray(X)
        Y = np.asarray(Y)
        print("equation number: ", X.shape[0])
        reg = RANSACRegressor(random_state=0).fit(X, Y)
        params = reg.estimator_.coef_
        print(params)
        return params
            
    def computeTanslationSystemParam(self, P_rr, p_rr, p_cc, R):
        # check out wiki for the translation system: https://github.com/ZhiangChen/gps_vio/wiki/T265-External-Calibration#2-estimating-translation-matrix
        inv_P_rr = np.linalg.inv(P_rr)
        a11 = P_rr[0, 0]
        a12 = P_rr[0, 1]
        a13 = P_rr[0, 2]
        a21 = P_rr[1, 0]
        a22 = P_rr[1, 1]
        a23 = P_rr[1, 2]
        a31 = P_rr[2, 0]
        a32 = P_rr[2, 1]
        a33 = P_rr[2, 2]
        b1 = P_rr[0, 3]
        b2 = P_rr[1, 3]
        b3 = P_rr[2, 3]
        a11_ = inv_P_rr[0, 0]
        a12_ = inv_P_rr[0, 1]
        a13_ = inv_P_rr[0, 2]
        a21_ = inv_P_rr[1, 0]
        a22_ = inv_P_rr[1, 1]
        a23_ = inv_P_rr[1, 2]
        a31_ = inv_P_rr[2, 0]
        a32_ = inv_P_rr[2, 1]
        a33_ = inv_P_rr[2, 2]
        b1_ = inv_P_rr[0, 3]
        b2_ = inv_P_rr[1, 3]
        b3_ = inv_P_rr[2, 3]
        r11 = R[0, 0]
        r12 = R[0, 1]
        r13 = R[0, 2]
        r21 = R[1, 0]
        r22 = R[1, 1]
        r23 = R[1, 2]
        r31 = R[2, 0]
        r32 = R[2, 1]
        r33 = R[2, 2]
        x_cc = p_cc[0]
        y_cc = p_cc[1]
        z_cc = p_cc[2]
        x_rr = p_rr[0]
        y_rr = p_rr[1]
        z_rr = p_rr[2]
        
        left_r1 = a11*b1_ + a12*b2_ + a13*b3_ + b1 + x_rr*(a11*a11_ + a12*a21_ + a13*a31_) + y_rr*(a11*a12_ + a12*a22_ + a13*a32_) + z_rr*(a11*a13_ + a12*a23_ + a13*a33_)
        left_r2 = a21*b1_ + a22*b2_ + a23*b3_ + b2 + x_rr*(a11_*a21 + a21_*a22 + a23*a31_) + y_rr*(a12_*a21 + a22*a22_ + a23*a32_) + z_rr*(a13_*a21 + a22*a23_ + a23*a33_)
        left_r3 = a31*b1_ + a32*b2_ + a33*b3_ + b3 + x_rr*(a11_*a31 + a21_*a32 + a31_*a33) + y_rr*(a12_*a31 + a22_*a32 + a32_*a33) + z_rr*(a13_*a31 + a23_*a32 + a33*a33_)
        right_r1 = r11*x_cc + r12*y_cc + r13*z_cc
        right_r2 = r21*x_cc + r22*y_cc + r23*z_cc
        right_r3 = r31*x_cc + r32*y_cc + r33*z_cc
        
        r1 = right_r1 - left_r1
        r2 = right_r2 - left_r2
        r3 = right_r3 - left_r3
        
        r = np.array((r1, r2, r3))
        A = np.array([[a11-1, a12, a13],[a21, a22-1, a23],[a31, a32, a33-1]])
        return A, r
        
    
    def resetDistanceThreshold(self):
        self.distance_threshold = random.random()*self.distance_max
        print("Next translation goal: ", self.distance_threshold)
        
    def resetRotationThreshold(self):
        r = (random.random() + self.rot_min/np.pi) / (1. + self.rot_min/np.pi + self.rot_max/np.pi)
        self.rot_angle_threshold = r*np.pi/2
        print("Next rotation goal: ", self.rot_angle_threshold/np.pi*180)
    
    def calcRotationDiff(self, r1, r2):
        err_matrix = (np.matmul(r1.transpose(),r2) - np.matmul(r1,r2.transpose()))/2.
        x3 = err_matrix[1, 0]
        x2 = err_matrix[0, 2]
        x1 = err_matrix[2, 1]
        #print(x1, x2, x3)
        return abs(x1) + abs(x2) + abs(x3)
    
    def append(self, sample, R_rr, p_rr, p_cc):
        P_rr = np.zeros((4, 4))
        P_rr[:4, :4] = R_rr
        P_rr[:, 3] = p_rr
        self.P_rr.append(P_rr)
        self.p_rr.append(p_rr)
        self.p_cc.append(p_cc)
        self.samples.append(sample)
        
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
        trans_vec = (0.1, 0, -0.01)
        trans = tf.transformations.translation_matrix(trans_vec)
        rot = tf.transformations.euler_matrix(0.5, 1.0471975512, -0.5)
        self.T_robot2vio = np.matmul(trans, rot)
        print("Ground truth T_vio2robot: ")
        self.T_vio2robot = np.linalg.inv(self.T_robot2vio)
        print(self.T_vio2robot)
        
        self.sub_vio = rospy.Subscriber('/mavros/odometry/in', Odometry, self.vioCallback, queue_size=1)
        self.pub_vio = rospy.Publisher('/vio/odom', Odometry, queue_size=1)
        
    def vioCallback(self, uav_odom):
        tf_odom = Odometry()
        tf_odom.header = uav_odom.header
        x = uav_odom.pose.pose.position.x
        y = uav_odom.pose.pose.position.y
        z = uav_odom.pose.pose.position.z
        pos = (x, y, z)
        x = uav_odom.pose.pose.orientation.x
        y = uav_odom.pose.pose.orientation.y
        z = uav_odom.pose.pose.orientation.z
        w = uav_odom.pose.pose.orientation.w
        qua = (x, y, z, w)
        trans = tf.transformations.translation_matrix(pos)
        rot = tf.transformations.quaternion_matrix(qua)
        uav_pose_r = np.matmul(trans, rot)  # uav's pose in robot coordinates
        
        vio_pose_r = np.matmul(self.T_robot2vio, uav_pose_r)  # vio's pose in robot coordinates
        vio_pose_c = np.matmul(vio_pose_r, self.T_vio2robot)
        
        tf_pos = tf.transformations.translation_from_matrix(vio_pose_c)
        tf_qua = tf.transformations.quaternion_from_matrix(vio_pose_c)
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