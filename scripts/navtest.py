#!/usr/bin/env python
# -*- coding:utf8 -*-
import rospy, os, sys, curses, time, cv2, tf
import numpy as np
import math
from tf.transformations import euler_from_quaternion
from numpy import deg2rad
from curses import wrapper
from threading import Thread
from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped, Transform
from sensor_msgs.msg import LaserScan, Imu
from metal_detector_msgs.msg._Coil import Coil
from tf import transformations
from nav_msgs.msg import Odometry

# read/write stuff on screen
std = None
# Robot data
robotTwist = Twist()
robotTwistMeasured = Twist()
distanceFromCenterX = 5.0
distanceFromCenterY = 4.5
orientationOffset = -0.7
robotLength = 0.5
pubVel   = rospy.Publisher('/p3at/cmd_vel', Twist, queue_size=1)
robotPose = PoseWithCovarianceStamped()
initialPose = PoseWithCovarianceStamped()
stepPose = PoseWithCovarianceStamped()
goalPose = PoseWithCovarianceStamped()
obstaclePose = PoseWithCovarianceStamped()

gridWidth = 1000
gridHeight = 1000
gridStep = 0.1

targetPointList = [[4.0, -4.0],[3.0,4.0],[2.0,-4.0],[1.0,4.0],[0.0,-4.0],[-1.0,4.0],[-2.0,-4.0],[-3.0,4.0],[-4.0,-4.0]]


coils = Coil()

minePose = PoseStamped()
mines = []
#laser information
laserInfo = LaserScan()
laserInfoHokuyo = LaserScan()

arenaWidth = 10
arenaHeight = 10

minAngle = 0.2

columnSpacing = 1.0

columnsStart = 5.0
columnsEnd = -5.0
columnsWidth = 1.0

pointsToVisit = []
pointStep = 2
reachedTargetPoint = False

nearMine = False

targetPointDistance = 1.0
minObstacleDistance = 0.3
minMineDistance = 2.0

obstaclePresent = False
startedTurningAway = False
angleToObstacle = 0.0
minObstacleAngle = 0.3
avoiding = False
tempTargetPoint = Pose()

columnsToCover

minMineAngle = 0.5

pathObstructed = False

targetPoint = Pose()

avoidanceStageOne = False
avoidanceStageTwo = False
avoidanceStageThree = False
avoidanceComplete = False

previousTargetDistance = 0.0

minTargetDistance = 0.2

turning = False

turnDir = "down"

coveringTargetPointX = False
coveringTargetPointY = True
distanceToCoverX = 0.0
distanceToCoverY = 0.0

first = True
reachedEndOfColumn = False

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

def linear_map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min




def angleToPoint(pointPose, drivePose):

    (roll, pitch, yaw) = euler_from_quaternion([drivePose.orientation.x, drivePose.orientation.y, drivePose.orientation.z, drivePose.orientation.w])
    lineAngle = math.atan((pointPose.position.y - drivePose.position.y) / (pointPose.position.x - drivePose.position.x))
    return lineAngle - yaw

######################### CALLBACKS ############################

# Laser range-finder callback
def receiveLaser(LaserNow):
    global laserInfo 
    laserInfo = LaserNow

def receiveLaserHokuyo(LaserNow):
    global laserInfoHokuyo 
    laserInfoHokuyo = LaserNow

def receiveMine(MineNow):
    global minePose, nearMine, mines
    nearMine = True
    minePose = MineNow
    mines.append[MineNow]

def receiveCoil(actualCoil):
    global coils
    coils = actualCoil

def move(x, z):
    twist = Twist()
    twist.linear.x = x
    twist.angular.z = z
    pubVel = rospy.Publisher('/p3at/cmd_vel', Twist, queue_size=1)
    pubVel.publish(robotTwist)


def updateRobotPose(ekfPose):
    global robotPose, robotTwistMeasured, transListener, pointsToVisit, robotTwist
    robotTwistMeasured = ekfPose.twist
    poseCache = PoseWithCovarianceStamped()
    poseCache.pose = ekfPose.pose
    poseCache.header = ekfPose.header

    tmp = poseCache.pose.pose.position.x
    poseCache.pose.pose.position.x = poseCache.pose.pose.position.y+distanceFromCenterX
    poseCache.pose.pose.position.y = -tmp+distanceFromCenterY

    if (poseCache.pose.pose.orientation.z >= np.sin(np.pi/3.0) and poseCache.pose.pose.orientation.w >= -0.5) or (poseCache.pose.pose.orientation.z >= np.sin(np.pi/4.0) and poseCache.pose.pose.orientation.w <= np.cos(np.pi/4.0)):
        poseCache.pose.pose.orientation.z = -poseCache.pose.pose.orientation.z
        poseCache.pose.pose.orientation.w = -poseCache.pose.pose.orientation.w

    robotPose = poseCache

    robotTwist = Twist()
    
    rospy.loginfo("robotPose: "+str(robotPose.pose.pose))

    if get_pose_distance(robotPose.pose.pose, targetPoint) <= minTargetDistance:
        targetPointList.pop(0)

    if len(mines) > 0:
        for mine in mines:
            if get_pose_distance(robotPose.pose.pose, mine.pose) <= minMineDistance:
                global nearMine
                nearMine = True
                move(0.0, 0.0)

    if turning:
        if turnDir == "up":
            if robotPose.pose.pose.orientation.w > -0.1 and robotPose.pose.pose.orientation.w < 0.1 and robotPose.pose.pose.orientation.z < -0.9:
                global turning
                
                turning = False
                robotTwist.linear.x = 0.0
                robotTwist.angular.z = 0.0
            else:
                robotTwist.linear.x = 0.0
                robotTwist.angular.z = -0.5
        elif turnDir == "down":
            if robotPose.pose.pose.orientation.z > -0.1 and robotPose.pose.pose.orientation.z < 0.1 and robotPose.pose.pose.orientation.w > 0.9:
                global turning
                
                turning = False
                robotTwist.linear.x = 0.0
                robotTwist.angular.z = 0.0
            else:
                robotTwist.linear.x = 0.0
                robotTwist.angular.z = -0.5
        elif turnDir == "left":
            if robotPose.pose.pose.orientation.z > -0.8 and robotPose.pose.pose.orientation.z < -0.6 and robotPose.pose.pose.orientation.w > 0.6 and robotPose.pose.pose.orientation.w < 0.8:
                global turning
                
                turning = False
                robotTwist.linear.x = 0.0
                robotTwist.angular.z = 0.0
            else:
                robotTwist.linear.x = 0.0
                robotTwist.angular.z = -0.5
        elif turnDir == "right":
            if robotPose.pose.pose.orientation.z > 0.6 and robotPose.pose.pose.orientation.z < 0.8 and robotPose.pose.pose.orientation.w > 0.6 and robotPose.pose.pose.orientation.w < 0.8:
                global turning
                
                turning = False
                robotTwist.linear.x = 0.0
                robotTwist.angular.z = 0.0
            else:
                robotTwist.linear.x = 0.0
                robotTwist.angular.z = -0.5
        else:
            global turning, turnDir
            turnDir = ""
            turning = False
            robotTwist.linear.x = 0.0
            robotTwist.angular.z = 0.0
        pubVel = rospy.Publisher('/p3at/cmd_vel', Twist, queue_size=1)
        pubVel.publish(robotTwist)
        return
    else:
        if nearMine:
            move(0.0, 0.0)
            move(-0.2, 0.0)
            rospy.sleep(0.5)
            move(0.0, 0.0)
            #leastMineDistance = 100.0
            #closestMine = PoseStamped()
            #for mine in mines:
            #    if get_pose_distance(mine.pose, robotPose.pose.pose) < leastMineDistance:
            #        closestMine.pose.position.x = mine.pose.position.x
            #        closestMine.pose.position.y = mine.pose.position.y
            #        leastMineDistance = get_pose_disance(mine.pose, robotPose.pose.pose)
            #if (robotPose.pose.pose.position.y - targetPoint.position.y) > 0 and closestMine.pose.position.x < robotPose.pose.position.x and closestMine.pose.position.x > targetPoint.position.x:
            #    pointCache = targetPoint
            #    pointCache
            if turnDir != "left":
                global turnDir, turning
                turnDir = "left"
                turning = True
                return
            else:
                global avoiding, tempTargetPoint
                
                tempTargetPoint.position.x = robotPose.pose.pose.position.x-1.0
                tempTargetPoint.position.
            
                
    

if __name__ == '__main__':
    # Initialize client node
    rospy.init_node('navigator')
    
    transListener = tf.TransformListener()
    count = 0
    while count <= arenaHeight:
        count2 = 0
        while count2 <= arenaWidth:
            visitPoint = Pose()
            visitPoint.position.x = float(arenaWidth/2-count2)
            visitPoint.position.y = float(arenaHeight/2-count)
            pointsToVisit.append(visitPoint)
            count2 = count2+pointStep
        count = count+pointStep

    global targetPoint
    targetPoint.position.x = 4.0
    targetPoint.position.y = -4.0

    # Subscribing to all these topics to bring the robot or simulation to live data
    rospy.Subscriber("/HRATC_FW/set_mine", PoseStamped, receiveMine, queue_size = 1)
    rospy.Subscriber("/scan", LaserScan, receiveLaser)
    rospy.Subscriber("/coils", Coil, receiveCoil)
    rospy.Subscriber("/scan_hokuyo", LaserScan, receiveLaserHokuyo)
    rospy.Subscriber("/odometry/filtered", Odometry, updateRobotPose)

    #Starting curses and ROS
    #Thread(target = StartControl).start()
    rospy.spin()
