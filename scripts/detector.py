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
from tf import transformations

std = None

transformer = None
transListener = None

coils = Coil()
robotTwist = Twist()
robotPose = PoseStamped()
leftCoilPose = PoseStamped()
rightCoilPose = PoseStamped()
minePose = PoseStamped()

minePositions = []

SIGNAL_BUFFER_LENGTH = 10
DERIVATIVE_WINDOW_LENGTH = 3 

coilLeftSignalBuffer = []
coilRightSignalBuffer = []

leftCoilMeans = []
rightCoilMeans = []

leftCoilMedians = []
rightCoilMedians = []

bufferFull = False


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
    
    return np.linalg.norm(matrix1[:,3]-matrix2[:,3])

########################## TEMPORARY POSE FUNCTIONS ######################

#REMOVE THESE AFTER NAVIGATOR STARTS PUBLISHING POSE DATA

def updateRobotPose():
    global robotPose

    # This function does not get the true robot pose, but only the pose of 'base_link' in the TF
    # you should replace it by the robot pose resulting from a good localization process 

    robotPose = PoseStamped()
    now = rospy.Time.now()
    # Get left coil position in relation to robot
    try:
        transListener.waitForTransform('minefield', 'base_link', now, rospy.Duration(2.0))    
        (trans,rot) = transListener.lookupTransform('minefield', 'base_link', now)
    except:
        return

    tr2 = transformations.concatenate_matrices(transformations.translation_matrix(trans), transformations.quaternion_matrix(rot))
    robotPose.pose = pose_msg_from_matrix(tr2)

def updateCoilPoseManually(referencePose):
    global transListener, leftCoilPose, rightCoilPose

    now = rospy.Time.now()
    # Get left coil pose in relation to robot
    try:
        transListener.waitForTransform('base_link', 'left_coil', now, rospy.Duration(2.0))    
        (transL,rotL) = transListener.lookupTransform('base_link', 'left_coil', now)
        
        transListener.waitForTransform('base_link', 'right_coil', now, rospy.Duration(2.0))
        (transR,rotR) = transListener.lookupTransform('base_link', 'right_coil', now)
    except:
        return

    localCoil_Mat_L = transformations.concatenate_matrices(transformations.translation_matrix(transL), transformations.quaternion_matrix(rotL))
    localCoil_Mat_R = transformations.concatenate_matrices(transformations.translation_matrix(transR), transformations.quaternion_matrix(rotR))

    # Use reference robot pose
    robot_Mat = matrix_from_pose_msg(referencePose)

    # Compute corrected coil pose
    corrected_Mat_L = np.dot(robot_Mat, localCoil_Mat_L)
    corrected_Mat_R = np.dot(robot_Mat, localCoil_Mat_R)

    leftCoilPose = PoseStamped()
    leftCoilPose.pose = pose_msg_from_matrix(corrected_Mat_L)
    rightCoilPose = PoseStamped()
    rightCoilPose.pose = pose_msg_from_matrix(corrected_Mat_R)

########################## MAIN PROCESSING ###############################

# Wrapper function
def detectorWrapper():
    rospy.loginfo("Wrapper C1 "+bufferFull+" leftCoilMeans length "+len(leftCoilMeans))
    if bufferFull and len(leftCoilMeans) >= DERIVATIVE_WINDOW_LENGTH:
        rospy.loginfo("Wrapper C2")
        if isMine():
            rospy.loginfo("Wrapper C3")
            if isUniqueMine(minePose):
                pubMine = rospy.Publisher('/HRATC_FW/set_mine', PoseStamped)
                pubMine.publish(minePose)
            
                minePositions.append(minePose)
                rospy.loginfo("Wrapper C4")
            
# Detect based on simple heuristic 
#TODO: EXTRACT FEATURES AND USE DECISION TREE TO DETECT
def isMine():
    #FEATURES
    leftCoil = coils.left_coil
    rightCoil = coils.right_coil
    leftCoilMean = leftCoilMeans[-1]
    leftCoilMedian = leftCoilMedians[-1]
    rightCoilMean = rightCoilMeans[-1]
    rightCoilMedian = rightCoilMedians[-1]
    leftCoilStdDev = np.std(coilLeftSignalBuffer)
    rightCoilStdDev = np.std(coilRightSignalBuffer)
    leftMeanRateOfChange = leftCoilMeans[-1] - leftCoilMeans[0]
    rightMeanRateOfChange = rightCoilMeans[-1] - rightCoilMeans[0]
    meansDiffOverSum = (leftCoilMean-rightCoilMean)/(leftCoilMean+rightCoilMean)
    mediansDiffOverSum = (leftCoilMedian-rightCoilMedian)/(leftCoilMedian+rightCoilMedian)
    
    rospy.loginfo("isMineData: "+str(leftCoil)+" "+str(rightCoil))
    
    if coils.left_coil>=0.35:
        minePose=leftCoilPose
        rospy.loginfo("isMine Left True")
        return True
    elif coils.right_coil>=0.35:
        minePose=rightCoilPose
        rospy.loginfo("isMine Right True")
        return True
    else:
        rospy.loginfo("isMine False")
        return False
    
# Check if mine is within range of current known mines
def isUniqueMine(newMine):
    rospy.loginfo("isUniqueMine started")
    for mine in minePositions:
        dist = get_pose_distance(newMine, mine)
        rospy.loginfo("isUniqueMine distance: "+dist)
        if dist<=1:
            return False
    return True

# Mine Detection Callback
def receiveCoilSignal(actualCoil):
    rospy.loginfo("Signal received")
    global coils
    coils = actualCoil
    if len(coilLeftSignalBuffer) <> SIGNAL_BUFFER_LENGTH and len(coilRightSignalBuffer) <> SIGNAL_BUFFER_LENGTH:
        coilLeftSignalBuffer.append(coils.left_coil)
        coilRightSignalBuffer.append(coils.right_coil)
    else:
        bufferFull = True
        coilLeftSignalBuffer.pop(0)
        coilLeftSignalBuffer.append(coils.left_coil)
        coilRightSignalBuffer.pop(0)
        coilRightSignalBuffer.append(coils.right_coil)
        leftCoilMeans.append(np.mean(coilLeftSignalBuffer))
        leftCoilMedians.append(np.median(coilLeftSignalBuffer))
        rightCoilMeans.append(np.mean(coilRightSignalBuffer))
        rightCoilMedians.append(np.median(coilRightSignalBuffer))
        if len(leftCoilMeans) > DERIVATIVE_WINDOW_LENGTH:
            leftCoilMeans.pop(0)
            leftCoilMedians.pop(0)
            rightCoilMeans.pop(0)
            rightCoilMedians.pop(0)
    updateRobotPose()
    updateCoilPoseManually(robotPose.pose)
    detectorWrapper()  

def spin():
    rospy.spin()

if __name__ == '__main__':
    # Initialize client node
    rospy.init_node('detector')
    rospy.loginfo("Node initialized")
    transListener = tf.TransformListener()

    # Subscribing to relevant topics to bring the robot or simulation to live data
    rospy.Subscriber("/coils", Coil, receiveCoilSignal, queue_size = 1)
    rospy.spin()
