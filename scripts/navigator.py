#!/usr/bin/python
# -*- coding:utf8 -*-
import rospy, os, sys, curses, time, cv2, tf
import numpy as np
from numpy import deg2rad
from curses import wrapper
from threading import Thread
from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped, Transform
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu
from metal_detector_msgs.msg._Coil import Coil
from tf import transformations
from tf.transformations import euler_from_quaternion
import math

# read/write stuff on screen
std = None
# Robot data
robotTwist = Twist()
robotTwistMeasured = Twist()

#Simulator values
distanceFromCenterX = 5.0
distanceFromCenterY = 4.5
#Testing values?
# distanceFromCenterX = 4.5
# distanceFromCenterY = -5.0

robotLength = 0.5
pubVel  = rospy.Publisher('/p3at/cmd_vel', Twist, queue_size=1)
robotPose = PoseWithCovarianceStamped()
initialPose = PoseWithCovarianceStamped()
stepPose = PoseWithCovarianceStamped()
goalPose = PoseWithCovarianceStamped()
obstaclePose = PoseWithCovarianceStamped()
direction = -1

minePose = PoseStamped()
mines = []
#laser information
laserInfo = LaserScan()
laserInfoHokuyo = LaserScan()

first = True
turnSeq = 0
turnStep = 1
turning = False
obstacleSeq = 0
obstacleStep = 1 
avoidingObstacle = False

mineTriggerDist = 0.7
mapDistanceY = 8.0
slowLinearSpeed = 0.3
angularSpeed = 0.6
fastLinearSpeed = 0.4
turnDistance = 1.5
avoidSideDist = 1
avoidForwardDist = 1.5

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

def get_pose_distance(pose1, pose2):
    matrix1 = matrix_from_pose_msg(pose1)
    matrix2 = matrix_from_pose_msg(pose2)
    rospy.loginfo("matrix1: "+str(matrix1))
    rospy.loginfo("matrix2: "+str(matrix2))
    return np.linalg.norm(matrix1[:,3]-matrix2[:,3])

def getAngle(drivePose):
    (roll, pitch, yaw) = euler_from_quaternion([drivePose.orientation.x, drivePose.orientation.y, drivePose.orientation.z, drivePose.orientation.w])
    if math.degrees(yaw) > 0:
        result = math.degrees(yaw)
    else:
        result = 360 + math.degrees(yaw)
    if result > 358 or result < 2:
        result = 0
    return result

######################### CALLBACKS ############################

# Laser range-finder callback
def receiveLaser(LaserNow):
    global laserInfo 
    laserInfo = LaserNow

def receiveLaserHokuyo(LaserNow):
    global laserInfoHokuyo 
    laserInfoHokuyo = LaserNow

def recieveMine(MineNow):
    global minePose
    global mines
    global avoidingObstacle
    avoidingObstacle = True
    minePose = MineNow
    mines.append(MineNow)

def updateRobotPose(ekfPose):
    global robotPose, robotTwistMeasured, transListener, robotTwist, pubVel, stepPose, initialPose, goalPose
    robotTwistMeasured = ekfPose.twist
    poseCache = PoseWithCovarianceStamped()
    poseCache.pose = ekfPose.pose
    poseCache.header = ekfPose.header
    tmp = poseCache.pose.pose.position.x
    poseCache.pose.pose.position.x = poseCache.pose.pose.position.y+distanceFromCenterX
    poseCache.pose.pose.position.y = -tmp+distanceFromCenterY
    robotPose = poseCache
    logMessage = str(getAngle(robotPose.pose.pose))
    rospy.loginfo("Angle: " + logMessage)
    if first:
        global first
        initialPose = robotPose
        first = False
        stepPose.pose.pose.position.x = robotPose.pose.pose.position.x
        stepPose.pose.pose.position.y = robotPose.pose.pose.position.y-mapDistanceY


    if abs(robotPose.pose.pose.position.y-stepPose.pose.pose.position.y) <= 0.3 or turning:
	rospy.loginfo("turning")
        stepPoseCache = turnAround()
        global avoidingObstacle, obstacleStep, obstacleSeq
        avoidingObstacle = False
        obstacleSeq = 0
        obstacleStep = 1
        if stepPoseCache != None:
            stepPose = stepPoseCache

    
    elif checkMine() or avoidingObstacle:
	if checkMine():
	    rospy.loginfo("avoiding a mine")
	else:
	    rospy.loginfo("avoiding an obstacle")
        global avoidingObstacle
        avoidingObstacle = True
        avoidObstacle()
    elif len(laserInfo.ranges) > 0 and min(laserInfo.ranges[270:550]) <= 0.05:
	    rospy.loginfo("avoiding an obstacle: {} away".format(min(laserInfo.ranges[300:520])))
            global avoidingObstacle
            avoidingObstacle = True
            avoidObstacle()
    else:
	rospy.loginfo("going straight")
        robotTwist.linear.x = fastLinearSpeed
        pubVel.publish(robotTwist)


######################### CURSES STUFF ############################

def checkMine():
    if len(mines) > 0:
        for mine in mines:
            if get_pose_distance(mine.pose,robotPose.pose.pose) <= mineTriggerDist:
                return True
    return False

def avoidObstacle():
    global obstacleSeq, obstacleStep, avoidingObstacle, robotTwist, pubVel, obstaclePose
    rospy.loginfo("obstacleSeq: {} obstacleStep: {} avoidngObstacle:{} obstaclePose x: {} y: {}".format(obstacleSeq, obstacleStep, avoidingObstacle, obstaclePose.pose.pose.position.x, obstaclePose.pose.pose.position.y))
    if obstacleSeq == 1:
        if obstacleStep == 1:
            if getAngle(robotPose.pose.pose) == 0 or getAngle(robotPose.pose.pose) > 270: #turn right
                robotTwist.angular.z = -angularSpeed
                robotTwist.linear.x = 0.0
                pubVel.publish(robotTwist)
            else:
                robotTwist.angular.z = 0.0
                robotTwist.linear.x = 0.0
                pubVel.publish(robotTwist)
                obstaclePose = robotPose
                obstacleStep = 2

        elif obstacleStep == 2:
            if abs(obstaclePose.pose.pose.position.x-robotPose.pose.pose.position.x) < avoidSideDist: #go forward for 1 unit
                robotTwist.linear.x = slowLinearSpeed
                robotTwist.angular.z = 0.0
                pubVel.publish(robotTwist)
            else:
                robotTwist.linear.x = 0.0 
                robotTwist.angular.z = 0.0
                pubVel.publish(robotTwist)
                obstacleStep = 3

        elif obstacleStep == 3:
            if getAngle(robotPose.pose.pose) > 0: #turn left
                robotTwist.angular.z = angularSpeed
                robotTwist.linear.x = 0.0
                pubVel.publish(robotTwist)
            else:
                robotTwist.angular.z = 0.0 
                robotTwist.linear.x = 0.0
                pubVel.publish(robotTwist)
                obstaclePose = robotPose 
                obstacleStep = 4 

        elif obstacleStep == 4:
            if abs(obstaclePose.pose.pose.position.y-robotPose.pose.pose.position.y) < avoidForwardDist: #go forward for 2 unit
                robotTwist.linear.x = slowLinearSpeed
                robotTwist.angular.z = 0.0
                pubVel.publish(robotTwist)
            else:
                robotTwist.linear.x = 0.0 
                robotTwist.angular.z = 0.0
                pubVel.publish(robotTwist)
                obstacleStep = 5

        elif obstacleStep == 5:
            if getAngle(robotPose.pose.pose) < 90: #turn left
                robotTwist.angular.z = angularSpeed
                robotTwist.linear.x = 0.0
                pubVel.publish(robotTwist)
            else:
                robotTwist.angular.z = 0.0
                robotTwist.linear.x = 0.0 
                pubVel.publish(robotTwist)
                obstaclePose = robotPose 
                obstacleStep = 6

        elif obstacleStep == 7:
            if abs(obstaclePose.pose.pose.position.x-robotPose.pose.pose.position.x) < avoidSideDist: #go forward for 1 unit
                robotTwist.linear.x = slowLinearSpeed
                robotTwist.angular.z = 0.0
                pubVel.publish(robotTwist)
            else:
                robotTwist.linear.x = 0.0 
                robotTwist.angular.z = 0.0
                pubVel.publish(robotTwist)
                obstacleStep = 8

        elif obstacleStep == 8:
            if getAngle(robotPose.pose.pose) > 0: #turn right
                robotTwist.angular.z = -angularSpeed
                robotTwist.linear.x = 0.0
                pubVel.publish(robotTwist)
            else:
                robotTwist.angular.z = 0.0
                robotTwist.linear.x = 0.0 
                pubVel.publish(robotTwist)
                obstacleStep = 1
                obstacleSeq = 0
                avoidingObstacle = False
                return

    elif obstacleSeq == 2:
        if obstacleStep == 1:
            if getAngle(robotPose.pose.pose) < 270: #turn left
                robotTwist.angular.z = angularSpeed
                robotTwist.linear.x = 0.0
                pubVel.publish(robotTwist)
            else:    
                robotTwist.angular.z = 0.0
                robotTwist.linear.x = 0.0 
                pubVel.publish(robotTwist)
                obstacleStep = 2
                obstaclePose = robotPose

        elif obstacleStep == 2:
            if abs(obstaclePose.pose.pose.position.x-robotPose.pose.pose.position.x) < avoidSideDist: #go forward for 1 unit
                robotTwist.linear.x = slowLinearSpeed
                robotTwist.angular.z = 0.0
                pubVel.publish(robotTwist)
            else:  
                robotTwist.linear.x = 0.0
                robotTwist.angular.z = 0.0 
                pubVel.publish(robotTwist)
                obstacleStep = 3

        elif obstacleStep == 3:
            if getAngle(robotPose.pose.pose) > 180: #turn right
                robotTwist.angular.z = -angularSpeed
                robotTwist.linear.x = 0.0
                pubVel.publish(robotTwist)
            else:
                robotTwist.angular.z = 0.0
                robotTwist.linear.x = 0.0 
                pubVel.publish(robotTwist)
                obstaclePose = robotPose  
                obstacleStep = 4

        elif obstacleStep == 4:
            if abs(obstaclePose.pose.pose.position.y-robotPose.pose.pose.position.y) < avoidForwardDist: #go forward for 2 unit
                robotTwist.linear.x = slowLinearSpeed
                robotTwist.angular.z = 0.0
                pubVel.publish(robotTwist)
            else:    
                robotTwist.linear.x = 0.0
                robotTwist.angular.z = 0.0 
                pubVel.publish(robotTwist)
                obstacleStep = 5

        elif obstacleStep == 5:
            if getAngle(robotPose.pose.pose) > 90: #turn right
                robotTwist.angular.z = -angularSpeed
                robotTwist.linear.x = 0.0
                pubVel.publish(robotTwist)
            else:   
                robotTwist.angular.z = 0.0
                robotTwist.linear.x = 0.0 
                pubVel.publish(robotTwist)
                obstaclePose = robotPose 
                obstacleStep = 6

        elif obstacleStep == 6:
            if abs(obstaclePose.pose.pose.position.x-robotPose.pose.pose.position.x) < avoidSideDist: #go forward for 1 unit
                robotTwist.linear.x = slowLinearSpeed
                robotTwist.angular.z = 0.0
                pubVel.publish(robotTwist)
            else:  
                robotTwist.linear.x = 0.0 
                robotTwist.angular.z = 0.0
                pubVel.publish(robotTwist)
                obstacleStep = 7

        elif obstacleStep == 7:
            if getAngle(robotPose.pose.pose) < 180: #turn left
                robotTwist.angular.z = angularSpeed
                robotTwist.linear.x = 0.0
                pubVel.publish(robotTwist)
            else:    
                robotTwist.angular.z = 0.0 
                robotTwist.linear.x = 0.0
                pubVel.publish(robotTwist)
                obstacleStep = 1
                obstacleSeq = 0
                avoidingObstacle = False
                return

    elif getAngle(robotPose.pose.pose) == 0:
        obstacleSeq = 1
        return
    elif abs(getAngle(robotPose.pose.pose) - 180) < 2:
        obstacleSeq = 2
        return

def turnAround():
    global turnSeq, turnStep, turning, robotTwist, pubVel, obstaclePose
    turning = True
    if turnSeq == 1:
        if turnStep == 1:
            if getAngle(robotPose.pose.pose) == 0 or getAngle(robotPose.pose.pose) > 270: #turn left 
                robotTwist.angular.z = -angularSpeed
                robotTwist.linear.x = 0.0
                pubVel.publish(robotTwist)
                
            else:
                robotTwist.angular.z = 0.0 
                robotTwist.linear.x = 0.0
                pubVel.publish(robotTwist)
                turnStep = 2
                obstaclePose = robotPose
        elif turnStep == 2:
            if abs(obstaclePose.pose.pose.position.x-robotPose.pose.pose.position.x) < turnDistance: #go forward for 1 unit
                robotTwist.linear.x = slowLinearSpeed
                robotTwist.angular.z = 0.0
                pubVel.publish(robotTwist)
                
            else:
                robotTwist.linear.x = 0.0 
                robotTwist.angular.z = 0.0
                pubVel.publish(robotTwist)
                turnStep = 3
        elif turnStep == 3:
            if getAngle(robotPose.pose.pose) > 180: #turn right
                robotTwist.angular.z = -angularSpeed
                robotTwist.linear.x = 0.0
                pubVel.publish(robotTwist)
                
            else:
                robotTwist.angular.z = 0.0 
                robotTwist.linear.x = 0.0
                pubVel.publish(robotTwist)
                turnStep = 4
                obstaclePose = robotPose  
        elif turnStep == 4:
            stepPose = robotPose
            stepPose.pose.pose.position.y += mapDistanceY
            turnStep = 1
            turnSeq = 0
            turning = False
            return stepPose
    
    elif turnSeq == 2:
        if turnStep == 1:
            if getAngle(robotPose.pose.pose) < 270: #turn left
                robotTwist.angular.z = angularSpeed
                robotTwist.linear.x = 0.0
                pubVel.publish(robotTwist)
            else:
                robotTwist.angular.z = 0.0 
                robotTwist.linear.x = 0.0
                pubVel.publish(robotTwist)
                obstaclePose = robotPose
                turnStep = 2
        elif turnStep == 2:
            if abs(obstaclePose.pose.pose.position.x-robotPose.pose.pose.position.x) < turnDistance: #go forward for 1 unit
                robotTwist.linear.x = slowLinearSpeed
                robotTwist.angular.z = 0.0
                pubVel.publish(robotTwist)
                
            else:
                robotTwist.linear.x = 0.0 
                robotTwist.angular.z = 0.0
                pubVel.publish(robotTwist)
                turnStep = 3
        elif turnStep == 3:
            if getAngle(robotPose.pose.pose) > 0: #turn left
                robotTwist.angular.z = angularSpeed
                robotTwist.linear.x = 0.0
                pubVel.publish(robotTwist)
                
            else:
                robotTwist.angular.z = 0.0 
                robotTwist.linear.x = 0.0
                pubVel.publish(robotTwist)
                turnStep = 4
        elif turnStep == 4:
            stepPose = robotPose
            stepPose.pose.pose.position.y -= mapDistanceY
            turnStep = 1
            turnSeq = 0
            turning = False
            return stepPose
    elif robotPose.pose.pose.position.y < -4.0:
        turnSeq = 1
    elif robotPose.pose.pose.position.y > 4.0:
        turnSeq = 2

######################### MAIN ############################

if __name__ == '__main__':
    # Initialize client node
    rospy.init_node('navigator')

    
    transListener = tf.TransformListener()

    rospy

    # Subscribing to all these topics to bring the robot or simulation to live data
    rospy.Subscriber("/HRATC_FW/set_mine", PoseStamped, recieveMine, queue_size = 1)
    rospy.Subscriber("/scan", LaserScan, receiveLaser)
    rospy.Subscriber("/scan_hokuyo", LaserScan, receiveLaserHokuyo)
    rospy.Subscriber("/odometry/filtered", Odometry, updateRobotPose)
    #Starting curses and ROS
    rospy.spin()




