#!/usr/bin/python
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

# read/write stuff on screen
std = None

logfile = open("/home/rcmakers/hratc2017_workspace/coildata.txt", 'w')

radStep = deg2rad(15)
linStep = 0.1
transformer = None
transListener = None

# Robot data
robotTwist = Twist()
robotPose = PoseWithCovarianceStamped()
leftCoilPose = PoseStamped()

#laser information
laserInfo = LaserScan()
laserInfoHokuyo = LaserScan()

#Inertial Unit 
imuInfo = Imu()

#Metal detector data
coils = Coil()

distanceFromCenterX = 5.0
distanceFromCenterY = 5.0

initialPose = PoseWithCovarianceStamped()
initialPoseGet = False

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

######################### GETTING POSES FROM TF ############################

def updateRobotPose(ekfPose):
    global robotPose
    poseCache = ekfPose
   # tmp = -poseCache.pose.pose.position.x
   # poseCache.pose.pose.position.x = poseCache.pose.pose.position.y
   # poseCache.pose.pose.position.y = tmp
   # poseCache.pose.pose.position.x = poseCache.pose.pose.position.x+distanceFromCenterX
   # poseCache.pose.pose.position.y = poseCache.pose.pose.position.y+distanceFromCenterY
   # robotPose = poseCache 
    if initialPoseGet:
        offsetPose = poseCache
        offsetPose.pose.pose.position.x = initialPose.pose.pose.position.x - poseCache.pose.pose.position.x + distanceFromCenterX
        offsetPose.pose.pose.position.y = poseCache.pose.pose.position.y - initialPose.pose.pose.position.y + distanceFromCenterY
        tmp = offsetPose.pose.pose.position.y
        offsetPose.pose.pose.position.y = offsetPose.pose.pose.position.x
        offsetPose.pose.pose.position.x = tmp
        offsetPose.pose.pose.position.z = 0.5
        robotPose = offsetPose
    else:
        global initialPoseGet, initialPose
        initialPoseGet = True
        initialPose = ekfPose
        rospy.loginfo(str(initialPose))
        robotPose = ekfPose
    updateCoilPoseManually(robotPose.pose.pose)

def updateCoilPoseFromTF():
    global transListener, leftCoilPose

    now = rospy.Time.now()
    # Change left coil position into world position
    try:
        transListener.waitForTransform('minefield', 'left_coil', now, rospy.Duration(3.0)) 
        (trans,rot) = transListener.lookupTransform('minefield', 'left_coil', now)
    except:
        return

    tr = transformations.concatenate_matrices(transformations.translation_matrix(trans), transformations.quaternion_matrix(rot))

    leftCoilPose = PoseStamped()
    leftCoilPose.pose = pose_msg_from_matrix(tr)

def updateCoilPoseManually(referencePose):
    global transListener, leftCoilPose

    now = rospy.Time.now()
    # Get left coil pose in relation to robot
    try:
        transListener.waitForTransform('base_link', 'left_coil', now, rospy.Duration(2.0))    
        (trans,rot) = transListener.lookupTransform('base_link', 'left_coil', now)
    except:
        return

    localCoil_Mat = transformations.concatenate_matrices(transformations.translation_matrix(trans), transformations.quaternion_matrix(rot))

    # Use reference robot pose
    robot_Mat = matrix_from_pose_msg(referencePose)

    # Compute corrected coil pose
    corrected_Mat = np.dot(robot_Mat, localCoil_Mat)

    leftCoilPose = PoseStamped()
    leftCoilPose.pose = pose_msg_from_matrix(corrected_Mat)

# Send mine position to HRATC Framework
def sendMine():
    global transListener

    ## You can get the pose of the metal detector from the TF
    ## But this value may be wrong if the robot pose in TF (that is, base_link) is not accurate
#    updateCoilPoseFromTF()

    ## It is better to compute the coil pose in relation to a corrected robot pose
    updateCoilPoseManually(robotPose.pose.pose)

    pubMine  = rospy.Publisher('/HRATC_FW/set_mine', PoseStamped)
    pubMine.publish(leftCoilPose)

######################### CALLBACKS ############################

# Laser range-finder callback
def receiveLaser(LaserNow):
    global laserInfo 
    laserInfo = LaserNow

def receiveLaserHokuyo(LaserNow):
    global laserInfoHokuyo 
    laserInfoHokuyo = LaserNow

# IMU data callback
def receiveImu(ImuNow):
    global imuInfo 
    imuInfo = ImuNow

# Mine Detection Callback, append to signal buffer, get means
def receiveCoilSignal(actualCoil):
    global coils
    coils = actualCoil
    coilLeftSignalBuffer.append(coils.left_coil)
    coilRightSignalBuffer.append(coils.right_coil)
    if len(coilLeftSignalBuffer) > SIGNAL_BUFFER_LENGTH:
        coilLeftSignalBuffer.pop(0)
        coilRightSignalBuffer.pop(0)
        leftCoilMeans.append(np.mean(coilLeftSignalBuffer))
        rightCoilMeans.append(np.mean(coilRightSignalBuffer))
        if len(leftCoilMeans) > DERIVATIVE_WINDOW_LENGTH:
            leftCoilMeans.pop(0)
            rightCoilMeans.pop(0)
    updateCoilPoseManually(robotPose.pose.pose)  

######################### CURSES STUFF ############################

# Printing data on screen
def showStats():

    if std == None:
        return

    std.clear()
    std.addstr(0,0,"Press Esc to Quit...")
    std.addstr(1,0,"Linear:")
    std.addstr(2, 0, "{} \t {} \t {}".format(robotTwist.linear.x,robotTwist.linear.y,robotTwist.linear.z))
    std.addstr(4,0,"Angular:")
    std.addstr(5, 0, "{} \t {} \t {}".format(robotTwist.angular.x,robotTwist.angular.y,robotTwist.angular.z))
    std.addstr(7,0,"Robot Position:")
    std.addstr(8, 0, "{} \t {} \t {}".format(robotPose.pose.pose.position.x, robotPose.pose.pose.position.y, robotPose.pose.pose.position.z))
    std.addstr(9,0,"Coil Position:")
    std.addstr(10, 0, "left: {} \t {} \t {}".format(leftCoilPose.pose.position.x, leftCoilPose.pose.position.y, leftCoilPose.pose.position.z))

    std.addstr(18, 0, "Coils readings: l: {} \t r: {}".format(coils.left_coil, coils.right_coil))
    std.addstr(19, 0, "IMU Quaternion w: {:0.4f} x: {:0.4f} y: {:0.4f} z: {:0.4f} ".format(imuInfo.orientation.w, imuInfo.orientation.x, imuInfo.orientation.y, imuInfo.orientation.z))
    if laserInfo.ranges != []:
        std.addstr(20, 0 , "Laser Readings {} Range Min {:0.4f} Range Max {:0.4f}".format( len(laserInfo.ranges), min(laserInfo.ranges), max(laserInfo.ranges)))
    if laserInfoHokuyo.ranges != []:
        std.addstr(21, 0 , "Laser Hokuyo Readings {} Range Min {:0.4f} Range Max {:0.4f}".format( len(laserInfoHokuyo.ranges), min(laserInfoHokuyo.ranges), max(laserInfoHokuyo.ranges)))
    #std.addstr(22,0, "Mine Detect Label: {}".format(str(mine_detected)))
    std.refresh()

# Basic control
def KeyCheck(stdscr):
    stdscr.keypad(True)
    stdscr.nodelay(True)
    logstr = "" 
    k = None
    global std
    std = stdscr
    #publishing topics
    pubVel   = rospy.Publisher('/p3at/cmd_vel', Twist, queue_size=1)
    global mine_detected
    mine_detected = False
    # While 'Esc' is not pressed
    while k != chr(27):
        if len(leftCoilMeans) >= DERIVATIVE_WINDOW_LENGTH:
            leftCoil = coils.left_coil
            rightCoil = coils.right_coil
            leftCoilMean = leftCoilMeans[-1]
            leftCoilMedian = np.median(coilLeftSignalBuffer)
            rightCoilMean = rightCoilMeans[-1]
            rightCoilMedian = np.median(coilRightSignalBuffer)
            leftCoilStdDev = np.std(coilLeftSignalBuffer)
            rightCoilStdDev = np.std(coilRightSignalBuffer)
            leftMeanRateOfChange = leftCoilMeans[-1] - leftCoilMeans[0]
            rightMeanRateOfChange = rightCoilMeans[-1] - rightCoilMeans[0]
            meansDiffOverSum = (leftCoilMean-rightCoilMean)/(leftCoilMean+rightCoilMean)
            mediansDiffOverSum = (leftCoilMedian-rightCoilMedian)/(leftCoilMedian+rightCoilMedian)
        
            logstr = "0 1:"+str(leftCoil)+" 2:"+str(rightCoil)+" 3:"+str(leftCoilMean)+" 4:"+str(leftCoilMedian)+" 5:"+str(rightCoilMean)+" 6:"+str(rightCoilMedian)+" 7:"+str(leftCoilStdDev)+" 8:"+str(rightCoilStdDev)+" 9:"+str(leftMeanRateOfChange)+" 10:"+str(rightMeanRateOfChange)+" 11:"+str(meansDiffOverSum)+" 12:"+str(mediansDiffOverSum)+"\n"
        
            # Check no key
            try:
                k = stdscr.getkey()
            except:
                k = None

            # Set mine position: IRREVERSIBLE ONCE SET
            if k == "x":
                mine_detected = not mine_detected
                sendMine()

            # Robot movement
            if k == " ":
                robotTwist.linear.x  = 0.0           
                robotTwist.angular.z = 0.0
            if k == "KEY_LEFT":
                robotTwist.angular.z += radStep
            if k == "KEY_RIGHT":
                robotTwist.angular.z -= radStep
            if k == "KEY_UP":
                robotTwist.linear.x +=  linStep
            if k == "KEY_DOWN":
                robotTwist.linear.x -= linStep

            robotTwist.angular.z = min(robotTwist.angular.z,deg2rad(90))
            robotTwist.angular.z = max(robotTwist.angular.z,deg2rad(-90))
            robotTwist.linear.x = min(robotTwist.linear.x,1.0)
            robotTwist.linear.x = max(robotTwist.linear.x,-1.0)
            pubVel.publish(robotTwist)
            
            if(mine_detected):
                logstr = logstr.replace("0", "1", 1)
            logfile.write(logstr)

            showStats()
            time.sleep(0.1)
    
    logfile.close()

    stdscr.keypad(False)
    rospy.signal_shutdown("Shutdown Competitor")

######################### MAIN ############################
    
def StartControl():
    wrapper(KeyCheck)

if __name__ == '__main__':
    # Initialize client node
    rospy.init_node('client')
    
    transListener = tf.TransformListener()

    # Subscribing to all these topics to bring the robot or simulation to live data
    rospy.Subscriber("/coils", Coil, receiveCoilSignal, queue_size = 1)
    rospy.Subscriber("/imu/data", Imu, receiveImu)
    rospy.Subscriber("/scan", LaserScan, receiveLaser)
    rospy.Subscriber("/robot_pose_ekf/odom", PoseWithCovarianceStamped, updateRobotPose) 
    rospy.Subscriber("/scan_hokuyo", LaserScan, receiveLaserHokuyo)

    #Starting curses and ROS
    Thread(target = StartControl).start()
    rospy.spin()
