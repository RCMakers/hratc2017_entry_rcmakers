#!/usr/bin/python
# -*- coding:utf8 -*-
import rospy, os, sys, curses, time, cv2, tf
import numpy as np
from numpy import deg2rad
from curses import wrapper
from threading import Thread
from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped, Transform
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu
from metal_detector_msgs.msg._Coil import Coil
from tf import transformations
from tf.transformations import euler_from_quaternion
import math

USING_SIM = False

minDist = 100000000000

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
pubVel  = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=1)

pubLog = rospy.Publisher('/angleDestlog', String, queue_size=1)
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

mineCoords = []

mineTriggerDist = 0.7
mapDistanceY = 8.0
slowLinearSpeed = 0.3
angularSpeed = 0.6
fastLinearSpeed = 0.4
turnDistance = 1.5
avoidSideDist = 1
avoidForwardDist = 1.5
avoidDist = 1.5
avoidDistFwd = 1.5
avoidDistBack = 0.5

turningError = 3

avoidingMine = False
facingMine = False
obstacleToLeft = False

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

swappedOutWaypoints = []

currentWaypoint = [0.0,0.0]
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
    #rospy.loginfo("matrix1: "+str(matrix1))
    #rospy.loginfo("matrix2: "+str(matrix2))
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

def getGoalAnglePoints(point1, point2):
    if len(point2) < 2:
        return -1
    if point2[0] == point1[0]:
        if point2[1] < point1[1]:
            return 180
        else:
            return 0
    elif point2[1] == point1[1]:
        if point2[0] < point1[0]:
            return 90
        else:
            return 270
    angle = math.degrees(math.atan((point1[0]-point2[0])/(point2[1]-point1[1])))
    if point2[1] > point1[1]:
        if angle <0:
            return angle+360
        else:
            return angle
    else:
        return angle+180

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
    global mineCoords
    mineCoords = []
    if len(mines) > 0:
        for mine in mines:
            if get_pose_distance(mine.pose,robotPose.pose.pose) <= mineTriggerDist:
                mineCoords = [mine.pose.position.x, mine.pose.position.y]
                rospy.loginfo("mine checked at {}: ".format(str(mineCoords)))
                return True
    return False

def updateRobotPose(ekfPose):
    global turnStep, robotPose, robotTwistMeasured, transListener, robotTwist, pubVel, stepPose, goalPose, waypoints, currentWaypoint, goal, goalAngle, avoidingObstacle, swappedOutWaypoints, facingMine, avoidingMine, mineCoords, minDist, obstacleToLeft
    avoidAngle1 = 0
    avoidAngle2 = 0
    robotTwistMeasured = ekfPose.twist
    poseCache = PoseWithCovarianceStamped()
    poseCache.pose = ekfPose.pose
    poseCache.header = ekfPose.header
    tmp = poseCache.pose.pose.position.x
    poseCache.pose.pose.position.x = -poseCache.pose.pose.position.y
    poseCache.pose.pose.position.y = tmp
    robotPose = poseCache
    poseList = [robotPose.pose.pose.position.x, robotPose.pose.pose.position.y]
    logMessage = str(getAngle(robotPose.pose.pose))
    logMsg2 = str(getGoalAnglePoints(poseList, currentWaypoint))
    logString = "Angle: " + logMessage + " Goalangle: "+logMsg2 + " waypoint: "+str(currentWaypoint)
    pubLog.publish(logString)
    rospy.loginfo(logString)
    obstaclePresent = False
    lazors = []
    if len(laserInfo.ranges) > 0:
        startLaser = len(laserInfo.ranges)/2-int((np.pi/4)/laserInfo.angle_increment)
        endLaser = len(laserInfo.ranges)/2+int((np.pi/4)/laserInfo.angle_increment)
        for h in laserInfo.ranges[startLaser:endLaser]:
            if h >= 0.1:
                lazors.append(h)
        minDist = min(lazors)
        if min(lazors) <= 0.8:
            rospy.loginfo("obstacle detected {} away".format(str(min(lazors))))
            for l in range(startLaser, endLaser):
                if laserInfo.ranges[l] == minDist:
                    if l >= len(laserInfo.ranges)/2:
                        obstacleToLeft = True
                        break
                    else:
                        obstacleToLeft = False
                        break
            obstaclePresent = True

    checkMine()
    mineAngle = getGoalAnglePoints(poseList, mineCoords)
    if mineAngle != -1:
        mineAngleMin = mineAngle-90
        mineAngleMax = mineAngle+90
        facingMine = False
        if mineAngleMin < 0:
            mineAngleMin = mineAngleMin +360
            if (getAngle(robotPose.pose.pose) < mineAngleMax or getAngle(robotPose.pose.pose) > mineAngleMin):
                if getAngle(robotPose.pose.pose) > mineAngleMax or getAngle(robotPose.pose.pose) < mineAngle:
                    obstacleToLeft = True
                else:
                    obstacleToLeft = False
                obstaclePresent = True
                facingMine = True
        elif mineAngleMax >= 360:
            mineAngleMax = mineAngleMax-360
            if (getAngle(robotPose.pose.pose) < mineAngleMax or getAngle(robotPose.pose.pose) > mineAngleMin):
                if getAngle(robotPose.pose.pose) < mineAngleMin or getAngle(robotPose.pose.pose) >= mineAngle:
                    obstacleToLeft = False
                else:
                    obstacleToLeft = True
                obstaclePresent = True
                facingMine = True 
        
        else:
            if(getAngle(robotPose.pose.pose) < mineAngleMax and getAngle(robotPose.pose.pose) > mineAngleMin):
                if getAngle(robotPose.pose.pose) >= mineAngle:
                    obstacleToLeft = False
                else:
                    obstacleToLeft = True
                obstaclePresent = True
                facingMine = True
        if obstaclePresent and facingMine:
            if facingMine and (not avoidingMine):
                avoidingMine = True
                
            rospy.loginfo("facing near mine at {}, angle of {}, min {}, max{}".format(str(mineCoords), str(mineAngle), str(mineAngleMin), str(mineAngleMax)))
    if len(swappedOutWaypoints) == 0 and avoidingObstacle:
        avoidingObstacle = False
        currentWaypoint = waypoints[0]



    if (math.sqrt(abs((robotPose.pose.pose.position.x - currentWaypoint[0])**2+(robotPose.pose.pose.position.y - currentWaypoint[1])**2)) < 0.3) or (checkMine() and math.sqrt(abs((robotPose.pose.pose.position.x - currentWaypoint[0])**2+(robotPose.pose.pose.position.y - currentWaypoint[1])**2)) < 0.8):
        rospy.loginfo("Arrived to Waypoint " + str(currentWaypoint))
        if avoidingObstacle:
            if len(swappedOutWaypoints) > 0:
                swappedOutWaypoints.pop(0)
            if len(swappedOutWaypoints) > 0:
                currentWaypoint = swappedOutWaypoints[0]
            else:
                avoidingObstacle = False
                if avoidingMine:
                    avoidingMine = False
                currentWaypoint = waypoints[0]
        else:
            waypoints.pop(0)
            if len(waypoints) > 0:
                currentWaypoint = waypoints[0]
       	    else:
                robotTwist.linear.x = 0.0
                robotTwist.angular.z = 0.0
                pubVel.publish(robotTwist)
                rospy.signal_shutdown("path completed")
                return 0

    elif obstaclePresent and (not avoidingObstacle):
        rospy.loginfo("avoid1")
        if not avoidingObstacle:
            rospy.loginfo("avoid2")
            avoidingObstacle = True
            swappedOutWaypoints = []
            if not obstacleToLeft:
                avoidAngle1 = getAngle(robotPose.pose.pose)+90
                if avoidAngle1 >= 360:
                    avoidAngle1 = avoidAngle1-360
                avoidAngle2 = getAngle(robotPose.pose.pose)-90
                if avoidAngle2 < 0:
                    avoidAngle2 = avoidAngle2+360
            else:
                avoidAngle1 = getAngle(robotPose.pose.pose)-90
                if avoidAngle1 >= 360:
                    avoidAngle1 = avoidAngle1-360
                avoidAngle2 = getAngle(robotPose.pose.pose)+90
                if avoidAngle2 < 0:
                    avoidAngle2 = avoidAngle2+360
            if False:
                avoidWaypoint0 = [robotPose.pose.pose.position.x+np.sin(np.deg2rad(getAngle(robotPose.pose.pose)))*avoidDistBack, robotPose.pose.pose.position.y-np.cos(np.deg2rad(getAngle(robotPose.pose.pose)))*avoidDistBack]
                avoidWaypoint1 = [avoidWaypoint0[0]-np.sin(np.deg2rad(avoidAngle1))*avoidDist, avoidWaypoint0[1]+np.cos(np.deg2rad(avoidAngle1))*avoidDist]
                avoidWaypoint2 = [avoidWaypoint1[0]-np.sin(np.deg2rad(getAngle(robotPose.pose.pose)))*avoidDistFwd*1.5, avoidWaypoint1[1]+np.cos(np.deg2rad(getAngle(robotPose.pose.pose)))*avoidDistFwd*1.5]
            else:
                avoidWaypoint0 = [robotPose.pose.pose.position.x, robotPose.pose.pose.position.y]
                avoidWaypoint1 = [avoidWaypoint0[0]-np.sin(np.deg2rad(avoidAngle1))*avoidDist, avoidWaypoint0[1]+np.cos(np.deg2rad(avoidAngle1))*avoidDist]
                avoidWaypoint2 = [avoidWaypoint1[0]-np.sin(np.deg2rad(getAngle(robotPose.pose.pose)))*avoidDistFwd, avoidWaypoint1[1]+np.cos(np.deg2rad(getAngle(robotPose.pose.pose)))*avoidDistFwd]
            currentWaypoint = avoidWaypoint0
            swappedOutWaypoints = [avoidWaypoint0, avoidWaypoint1, avoidWaypoint2]

    elif abs(getAngle(robotPose.pose.pose)-getGoalAnglePoints(poseList, currentWaypoint)) >= 5:
        robotTwist.linear.x = 0.0
        rightTurnThresh1 = getAngle(robotPose.pose.pose)-180
        leftTurnThresh1 = getAngle(robotPose.pose.pose)+180
        if rightTurnThresh1 < 0:
            rightTurnThresh1 = rightTurnThresh1+360
            if getGoalAnglePoints(poseList, currentWaypoint) < getAngle(robotPose.pose.pose) or getGoalAnglePoints(poseList, currentWaypoint) > rightTurnThresh1:
                robotTwist.angular.z = -angularSpeed
            else:
                robotTwist.angular.z = angularSpeed
        elif leftTurnThresh1 >= 360:
            leftTurnThresh1 = leftTurnThresh1-360
            if getGoalAnglePoints(poseList, currentWaypoint) > getAngle(robotPose.pose.pose) or getGoalAnglePoints(poseList, currentWaypoint) < leftTurnThresh1:
                robotTwist.angular.z = angularSpeed
            else:
                robotTwist.angular.z = -angularSpeed
        else:
            robotTwist.angular.z = angularSpeed
        pubVel.publish(robotTwist)
    else:
	rospy.loginfo("going straight")
        robotTwist.angular.z = 0.0
        robotTwist.linear.x = fastLinearSpeed
        pubVel.publish(robotTwist)

    if obstaclePresent and avoidingObstacle and (len(swappedOutWaypoints) < 2 or (avoidingMine and minDist <= 0.7)):
        if avoidingMine and minDist <= 0.7:
            avoidingMine = False
        swappedOutWaypoints = []
        if not obstacleToLeft:
            avoidAngle1 = getAngle(robotPose.pose.pose)+90
            if avoidAngle1 >= 360:
                avoidAngle1 = avoidAngle1-360
            avoidAngle2 = getAngle(robotPose.pose.pose)-90
            if avoidAngle2 < 0:
                avoidAngle2 = avoidAngle2+360
        else:
            avoidAngle1 = getAngle(robotPose.pose.pose)-90
            if avoidAngle1 >= 360:
                avoidAngle1 = avoidAngle1-360
            avoidAngle2 = getAngle(robotPose.pose.pose)+90
            if avoidAngle2 < 0:
                avoidAngle2 = avoidAngle2+360
        if False: #checkMine():
            avoidWaypoint0 = [robotPose.pose.pose.position.x+np.sin(np.deg2rad(getAngle(robotPose.pose.pose)))*avoidDistBack, robotPose.pose.pose.position.y-np.cos(np.deg2rad(getAngle(robotPose.pose.pose)))*avoidDistBack]
            avoidWaypoint1 = [avoidWaypoint0[0]-np.sin(np.deg2rad(avoidAngle1))*avoidDist, avoidWaypoint0[1]+np.cos(np.deg2rad(avoidAngle1))*avoidDist]
            avoidWaypoint2 = [avoidWaypoint1[0]-np.sin(np.deg2rad(getAngle(robotPose.pose.pose)))*avoidDistFwd*1.5, avoidWaypoint1[1]+np.cos(np.deg2rad(getAngle(robotPose.pose.pose)))*avoidDistFwd*1.5]
        else:
            avoidWaypoint0 = [robotPose.pose.pose.position.x, robotPose.pose.pose.position.y]
            avoidWaypoint1 = [avoidWaypoint0[0]-np.sin(np.deg2rad(avoidAngle1))*avoidDist, avoidWaypoint0[1]+np.cos(np.deg2rad(avoidAngle1))*avoidDist]
            avoidWaypoint2 = [avoidWaypoint1[0]-np.sin(np.deg2rad(getAngle(robotPose.pose.pose)))*avoidDistFwd, avoidWaypoint1[1]+np.cos(np.deg2rad(getAngle(robotPose.pose.pose)))*avoidDistFwd]
        currentWaypoint = avoidWaypoint0
        swappedOutWaypoints = [avoidWaypoint0, avoidWaypoint1, avoidWaypoint2]


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
