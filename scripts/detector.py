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

#LOGFILE = open("temp_data.txt","w")

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

#Simulator values
#distanceFromCenterX = 5.0
#distanceFromCenterY = 4.5
#Testing values?
# distanceFromCenterX = 4.5
# distanceFromCenterY = -5.0

orientationOffset = -0.7
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
    #rospy.loginfo("matrix1: "+str(matrix1))
    #rospy.loginfo("matrix2: "+str(matrix2))
    return np.linalg.norm(matrix1[:,3]-matrix2[:,3])


########################## POSE FUNCTIONS ######################

def updateRobotPose(ekfPose):
    global robotPose, robotTwistMeasured, transListener
    robotTwistMeasured = ekfPose.twist
    poseCache = PoseWithCovarianceStamped()
    poseCache.pose = ekfPose.pose
    poseCache.header = ekfPose.header

    updateCoilPoseManually(poseCache.pose.pose)

    tmp = poseCache.pose.pose.position.x
    poseCache.pose.pose.position.x = -poseCache.pose.pose.position.y
    poseCache.pose.pose.position.y = tmp
    if (poseCache.pose.pose.orientation.z >= np.sin(np.pi/3.0) and poseCache.pose.pose.orientation.w >= -0.5) or (poseCache.pose.pose.orientation.z >= np.sin(np.pi/4.0) and poseCache.pose.pose.orientation.w <= np.cos(np.pi/4.0)):
        poseCache.pose.pose.orientation.z = -poseCache.pose.pose.orientation.z
        poseCache.pose.pose.orientation.w = -poseCache.pose.pose.orientation.w
    #rospy.loginfo(str(robotPose.pose.pose.orientation))
    robotPose = poseCache
        

def updateCoilPoseManually(referencePose):
    global transListener, leftCoilPose, rightCoilPose

    refPoseCache = referencePose
    if (refPoseCache.orientation.z >= np.sin(np.pi/3.0) and refPoseCache.orientation.w >= -0.5) or (refPoseCache.orientation.z >= np.sin(np.pi/4.0) and refPoseCache.orientation.w <= np.cos(np.pi/4.0)):
       refPoseCache.orientation.z = -refPoseCache.orientation.z
       refPoseCache.orientation.w = -refPoseCache.orientation.w

    now = rospy.Time.now()
    # Get left and right coil pose in relation to robot
    try:
        transListener.waitForTransform('base_link', 'left_coil', now, rospy.Duration(2.0))    
        (transL,rotL) = transListener.lookupTransform('base_link', 'left_coil', now)
        
        transListener.waitForTransform('base_link', 'right_coil', now, rospy.Duration(2.0))
        (transR,rotR) = transListener.lookupTransform('base_link', 'right_coil', now)
    except Exception as a:
        rospy.loginfo(str(a))
        return

    localCoil_Mat_L = transformations.concatenate_matrices(transformations.translation_matrix(transL), transformations.quaternion_matrix(rotL))
    localCoil_Mat_R = transformations.concatenate_matrices(transformations.translation_matrix(transR), transformations.quaternion_matrix(rotR))

    # Use reference robot pose
    robot_Mat = matrix_from_pose_msg(refPoseCache)

    # Compute corrected coil pose
    corrected_Mat_L = np.dot(robot_Mat, localCoil_Mat_L)
    corrected_Mat_R = np.dot(robot_Mat, localCoil_Mat_R)

    poseCacheL = PoseStamped()
    poseCacheR = PoseStamped()

    leftCoilPose = PoseStamped()
    leftCoilPose.pose = pose_msg_from_matrix(corrected_Mat_L)
    
    poseCacheL = leftCoilPose
    tmp = poseCacheL.pose.position.x
    poseCacheL.pose.position.x = -poseCacheL.pose.position.y
    poseCacheL.pose.position.y = tmp
    
    leftCoilPose = poseCacheL    

    rightCoilPose = PoseStamped()
    rightCoilPose.pose = pose_msg_from_matrix(corrected_Mat_R)
    
    poseCacheR = rightCoilPose
    tmp = poseCacheR.pose.position.x
    poseCacheR.pose.position.x = -poseCacheR.pose.position.y
    poseCacheR.pose.position.y = tmp
    
    rightCoilPose = poseCacheR

########################## MAIN PROCESSING ###############################

# Wrapper function
def detectorWrapper():
    #rospy.loginfo("Wrapper C1 "+str(bufferFull)+" leftCoilMeans length "+str(len(leftCoilMeans)))
    if bufferFull and len(leftCoilMeans) >= DERIVATIVE_WINDOW_LENGTH:
        #rospy.loginfo("Wrapper C2")
        #isMineNear()
        if isMine():
            #rospy.loginfo("LEFTCOIL"+str(leftCoilPose.pose.position))
            global minePose
            if coils.left_coil > coils.right_coil:
                minePose = leftCoilPose
            else:
                minePose = rightCoilPose
            #rospy.loginfo("Wrapper C3")
            if isUniqueMine(minePose.pose):
                pubMine = rospy.Publisher('/HRATC_FW/set_mine', PoseStamped, queue_size = 1, latch = True)
                pubMine.publish(minePose)
                rospy.loginfo("minePose: "+str(minePose.pose))
                minePositions.append(minePose.pose)
                #rospy.loginfo("Wrapper C4")
            
# Detect based on decision tree
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
    
    coilData = [[leftCoil, rightCoil, leftCoilMean, leftCoilMedian, rightCoilMean, rightCoilMedian, leftCoilStdDev, rightCoilStdDev, leftMeanRateOfChange, rightMeanRateOfChange]]

    #LOGFILE.write(str(mineExists)+" 1:"+str(leftCoil)+" 2:"+str(rightCoil)+" 3:"+str(leftCoilMean)+" 4:"+str(leftCoilMedian)+" 5:"+str(rightCoilMean)+" 6:"+str(rightCoilMedian)+" 7:"+str(leftCoilStdDev)+" 8:"+str(rightCoilStdDev)+" 9:"+str(leftMeanRateOfChange)+" 10:"+str(rightMeanRateOfChange)+"\n")

    prediction = decTree.predict(coilData)
    
    if prediction:
	rospy.loginfo("MINE POSITION: "+str(robotPose.pose.pose.position))
        return True
    else:
        return False

# Check if mine is within range of current known mines
def isUniqueMine(newMine):
    #rospy.loginfo("isUniqueMine started "+str(minePositions))
    if len(minePositions) == 0:
        return True
    for mine in minePositions:
        dist = get_pose_distance(newMine, mine)
        #rospy.loginfo("isUniqueMine distance: "+str(dist))
        if dist<=0.5:
            return False
    return True

# Mine Detection Callback
def receiveCoilSignal(actualCoil):
    global coils, bufferFull
    
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
    #rospy.loginfo("coilSignalBuffer"+str(coilLeftSignalBuffer))
    #rospy.loginfo("derivativeWindow"+str(leftCoilMeans))
    detectorWrapper()  

if __name__ == '__main__':    
     
    # Initialize detector node
    rospy.init_node('detector')
    #rospy.loginfo("Node initialized")
    transListener = tf.TransformListener()

    decTree = tree.DecisionTreeClassifier()
    #decTreeNearing = tree.DecisionTreeClassifier()

    directory = os.path.dirname(os.path.abspath(__file__))
    
    # Train decision trees
    detectionTrainPath = os.path.join(directory, '..', 'data', 'training.txt')
    #nearingTrainPath = os.path.join(directory, '..', 'data', 'nearingtraining.txt')

    #rospy.loginfo("start training trees")
    X_train, y_train = load_svmlight_file(detectionTrainPath)
    decTree = decTree.fit(X_train.toarray(), y_train)

    #X_train_nearing, y_train_nearing = load_svmlight_file(nearingTrainPath)
    #decTreeNearing = decTreeNearing.fit(X_train_nearing.toarray(), y_train_nearing)
    #rospy.loginfo("trained trees")
    
    # Subscribing to relevant topics to bring the robot or simulation to live data
    rospy.Subscriber("/coils", Coil, receiveCoilSignal, queue_size = 1)
    rospy.Subscriber("/odometry/filtered", Odometry, updateRobotPose, queue_size = 1)
    rospy.spin()
