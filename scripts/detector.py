#!/usr/bin/env python
# -*- coding:utf8 -*-
import rospy, os, sys, curses, time, cv2, tf
import numpy as np
from numpy import deg2rad
from curses import wrapper
from threading import Thread
from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped, Transform
from sensor_msgs.msg import LaserScan, Imu
from metal_detector_msgs.msg._Coil import Coil
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from tf import transformations
from sklearn.datasets import load_svmlight_file
from sklearn import tree

std = None

transformer = None
transListener = None

coils = Coil()
robotTwist = Twist()
leftCoilPose = PoseStamped()
rightCoilPose = PoseStamped()
minePose = PoseStamped()

robotPose = PoseWithCovarianceStamped()
robotTwistMeasured = Twist()


minePositions = []

SIGNAL_BUFFER_LENGTH = 10
DERIVATIVE_WINDOW_LENGTH = 3
QUEUE_SIZE = 10

coilLeftSignalBuffer = []
coilRightSignalBuffer = []

leftCoilMeans = []
rightCoilMeans = []

leftCoilMedians = []
rightCoilMedians = []

bufferFull = False

distanceFromCenterX = 4.5
distanceFromCenterY = 5.0
robotLength = 0.5

######################### AUXILIARY FUNCTIONS ############################

# Get a transformation matrix from a geometry_msgs/Pose
def matrix_from_pose_msg(pose):
    t = transformations.translation_matrix((pose.position.x, pose.position.y, pose.position.z))
    q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    r = transformations.quaternion_matrix(q)
    return transformations.concatenate_matrices(t, r)

# Get a geometry_msgs/Pose from a transformation matrix
def pose_msg_from_matrix(transformation):
    msg = Pose()
    msg.position.x = transformation[0][3]
    msg.position.y = transformation[1][3]
    msg.position.z = transformation[2][3]
    q = transformations.quaternion_from_matrix(transformation)
    msg.orientation.x = q[0]
    msg.orientation.y = q[1]
    msg.orientation.z = q[2]
    msg.orientation.w = q[3]
    return msg

# Get the distance between two geometry_msgs/Pose
def get_pose_distance(pose1, pose2):
    matrix1 = matrix_from_pose_msg(pose1)
    matrix2 = matrix_from_pose_msg(pose2)
    rospy.loginfo("matrix1: "+str(matrix1))
    rospy.loginfo("matrix2: "+str(matrix2))
    return np.linalg.norm(matrix1[:,3]-matrix2[:,3])


########################## POSE FUNCTIONS ######################

def updateRobotPose(ekfPose):
    global robotPose, robotTwistMeasured, transListener
    robotTwistMeasured = ekfPose.twist
    poseCache = PoseWithCovarianceStamped()
    poseCache.pose = ekfPose.pose
    poseCache.header = ekfPose.header

    now = rospy.Time.now()
    try:
        transListener.waitForTransform('top_plate', 'metal_detector_support', now, rospy.Duration(2.0))    
        (trans,rot) = transListener.lookupTransform('top_plate', 'metal_detector_support', now)
    except:
        return
    
    poseErrorMat = transformations.concatenate_matrices(transformations.translation_matrix(trans), transformations.quaternion_matrix(rot))
    poseMat = matrix_from_pose_msg(poseCache.pose.pose)
    correctedMat = np.dot(poseMat,poseErrorMat)
    
    poseCache.pose.pose=pose_msg_from_matrix(correctedMat)
    tmp = poseCache.pose.pose.position.x
    poseCache.pose.pose.position.x = poseCache.pose.pose.position.y+distanceFromCenterX
    poseCache.pose.pose.position.y = -tmp+distanceFromCenterY
