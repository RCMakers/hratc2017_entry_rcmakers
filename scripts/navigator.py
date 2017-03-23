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

USING_SIM = True

firstInc = True

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

turningError = 3

waypoints = [[0.0,0.0],[11.364583, 4.0], [10.841441, -1.8], [9.754374499999999, -1.7750000000000001], 
             [10.230339749999999, 3.95], [9.0960965, 3.9], [8.667308, -1.75], [7.5802415, -1.725], 
             [7.961853249999999, 3.85], [6.827609999999999, 3.8], [6.493175, -1.7], [5.4061085, -1.675], 
             [5.693366749999999, 3.75], [4.559123499999998, 3.7], [4.319042, -1.65], [3.2319755, -1.625], 
             [3.4248802499999984, 3.65], [2.2906369999999985, 3.6], [2.144909, -1.6], [1.0578424999999996, -1.575], 
             [1.1563937499999977, 3.55], [0.02215049999999863, 3.5], [-0.02922399999999925, -1.55], 
             [-1.1162904999999999, -1.525], [-1.1120927500000022, 3.45], [-2.246336000000003, 3.4], 
             [-2.2033570000000005, -1.5], [-3.2904234999999993, -1.475], [-3.380579250000002, 3.35], 
             [-4.514822500000003, 3.3], [-4.37749, -1.45], [-5.4645565000000005, -1.425], [-5.649065750000002, 3.25], 
             [-6.783309000000003, 3.2], [-6.551622999999999, -1.4], [-7.638689499999998, -1.375], 
             [-7.9175522500000035, 3.15], [-9.051795500000004, 3.1], [-8.725756, -1.35], [-9.8128225, -1.325], 
             [-10.186038750000002, 3.05], [-11.320282, 3.0], [-10.899889, -1.3]]

currentWaypoint = 0
goalAngle = 0

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

def getGoalAngle(index1, index2):
    global waypoints
    if waypoints[index2][0] == waypoints[index1][0]:
        if waypoints[index2][1] > waypoints[index1][1]:
            return 180
        else:
            return 0
    angle = math.degrees(math.atan((waypoints[index2][1]-waypoints[index1][1])/(waypoints[index2][0]-waypoints[index1][0])))
    if waypoints[index2][0] > waypoints[index1][0]:
        return angle + 90
    else:
        return angle + 270

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
    minePoseCache = MineNow
    if USING_SIM:
        tmp = -minePoseCache.pose.position.x
        minePoseCache.pose.position.x = -minePoseCache.pose.position.x
        minePoseCache.pose.position.y = -minePoseCache.pose.position.y
        minePoseCache.pose.position.x = -minePoseCache.pose.position.y
        minePoseCache.pose.position.y = tmp
    avoidingObstacle = True
    minePose = minePoseCache
    mines.append(minePoseCache)

def checkMine():
    if len(mines) > 0:
        for mine in mines:
            if get_pose_distance(mine.pose,robotPose.pose.pose) <= mineTriggerDist:
                return True
    return False

def updateRobotPose(ekfPose):
    global turnStep, robotPose, robotTwistMeasured, transListener, robotTwist, pubVel, stepPose, initialPose, goalPose, waypoints, currentWaypoint, goal, goalAngle
    robotTwistMeasured = ekfPose.twist
    poseCache = PoseWithCovarianceStamped()
    poseCache.pose = ekfPose.pose
    poseCache.header = ekfPose.header
    tmp = poseCache.pose.pose.position.x
    poseCache.pose.pose.position.x = -poseCache.pose.pose.position.y
    poseCache.pose.pose.position.y = tmp
    robotPose = poseCache
    logMessage = str(getAngle(robotPose.pose.pose))
    logMsg2 = str(goalAngle)
    rospy.loginfo("Angle: " + logMessage + " Goalangle: "+logMsg2)
    if first:
        global first
        initialPose = robotPose
        first = False
        #stepPose.pose.pose.position.x = robotPose.pose.pose.position.x
        #stepPose.pose.pose.position.y = robotPose.pose.pose.position.y-mapDistanceY


    '''if abs(robotPose.pose.pose.position.y-stepPose.pose.pose.position.y) <= 0.3 or turning:
	rospy.loginfo("turning")
        stepPoseCache = turnAround()
        global avoidingObstacle, obstacleStep, obstacleSeq
        avoidingObstacle = False
        obstacleSeq = 0
        obstacleStep = 1
        if stepPoseCache != None:
            stepPose = stepPoseCache
    '''
    if math.sqrt(abs((robotPose.pose.pose.position.x - waypoints[currentWaypoint][0])**2+(robotPose.pose.pose.position.y - waypoints[currentWaypoint][1])**2)) < 0.3 and turnStep < 2:
        rospy.loginfo("Arrived to Waypoint " + str(currentWaypoint))
        if firstInc:
            currentWaypoint = currentWaypoint+1
            global firstInc
            firstInc = False
        goalAngle = getGoalAngle(currentWaypoint-1,currentWaypoint)
        rospy.loginfo("goalAngle: "+str(goalAngle))
        if turnStep == 1:
            if abs(goalAngle - getAngle(robotPose.pose.pose)) > turningError:
                robotTwist.angular.z = angularSpeed
                robotTwist.linear.x = 0.0
                pubVel.publish(robotTwist)
            else:
                turnStep = 2

    if math.sqrt(abs((robotPose.pose.pose.position.x - waypoints[currentWaypoint][0])**2+(robotPose.pose.pose.position.y - waypoints[currentWaypoint][1])**2)) > 0.3 and turnStep == 2:
        turnStep = 1
        global firstInc
        firstInc = True

    elif checkMine() or avoidingObstacle:
    	if checkMine():
    	    rospy.loginfo("avoiding a mine")
    	else:
    	    rospy.loginfo("avoiding an obstacle")
            global avoidingObstacle
            avoidingObstacle = True
            avoidObstacle()
    lazors = []
    if len(laserInfo.ranges) > 0:
        for h in laserInfo.ranges[270:550]:
            if h != 0.0:
                lazors.append(h)
    elif len(lazors) > 0 and np.min(lazors) <= 0.05:
	    rospy.loginfo("avoiding an obstacle: {} away".format(min(lazors)))
            global avoidingObstacle
            avoidingObstacle = True
            avoidObstacle()
    else:
	rospy.loginfo("going straight")
        robotTwist.angular.z = 0.0
        robotTwist.linear.x = fastLinearSpeed
        pubVel.publish(robotTwist)


######################### NAVIGATION SEQUENCES ############################

def avoidObstacle():
    global obstacleSeq, obstacleStep, avoidingObstacle, robotTwist, pubVel, obstaclePose
    rospy.loginfo("obstacleSeq: {} obstacleStep: {} avoidngObstacle:{} obstaclePose x: {} y: {}".format(obstacleSeq, obstacleStep, avoidingObstacle, obstaclePose.pose.pose.position.x, obstaclePose.pose.pose.position.y))
    turnAngle = None
    if True:
        if obstacleStep == 1:
            if turnAngle == None:
                turnAngle = getAngle(robotPose.pose.pose) - 90
                if turnAngle < 0:
                    turnAngle += 360
            if abs(turnAngle-getAngle(robotPose.pose.pose)) > turningError: #turn left
                robotTwist.angular.z = angularSpeed
                robotTwist.linear.x = 0.0
                pubVel.publish(robotTwist)
            else:
                robotTwist.angular.z = 0.0
                robotTwist.linear.x = 0.0
                pubVel.publish(robotTwist)
                obstaclePose = robotPose
                obstacleStep = 2
                turnAngle = None

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
            if turnAngle == None:
                turnAngle = getAngle(robotPose.pose.pose) + 90
                if turnAngle > 360:
                    turnAngle -= 360
            if abs(turnAngle-getAngle(robotPose.pose.pose)) > turningError: #turn right
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
            if turnAngle == None:
                turnAngle = getAngle(robotPose.pose.pose) + 90
                if turnAngle > 360:
                    turnAngle -= 360
            if abs(turnAngle-getAngle(robotPose.pose.pose)) > turningError: #turn right
                robotTwist.angular.z = -angularSpeed
                robotTwist.linear.x = 0.0
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
            if turnAngle == None:
                turnAngle = getAngle(robotPose.pose.pose) - 90
                if turnAngle < 0:
                    turnAngle += 360
            if abs(turnAngle-getAngle(robotPose.pose.pose)) > turningError: #turn left
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

    '''elif obstacleSeq == 2:
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
        return'''

'''
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
'''
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
