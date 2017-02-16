#!/usr/bin/python
# -*- coding:utf8 -*-
import rospy, os, sys, curses, time, cv2, tf
import numpy as np
import math
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
robotLength = 0.5
pubVel   = rospy.Publisher('/p3at/cmd_vel', Twist, queue_size=1)
robotPose = PoseWithCovarianceStamped()
initialPose = PoseWithCovarianceStamped()
stepPose = PoseWithCovarianceStamped()
goalPose = PoseWithCovarianceStamped()
obstaclePose = PoseWithCovarianceStamped()

coils = Coil()

minePose = PoseStamped()
mines = []
#laser information
laserInfo = LaserScan()
laserInfoHokuyo = LaserScan()

arenaWidth = 10
arenaHeight = 10

minAngle = 0.2

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
minObstacleAngle = 0.5

minMineAngle = 0.5

pathObstructed = False

targetPoint = Pose()

previousTargetDistance = 0.0


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
	lineAngle = math.atan((pointPose.position.y - drivePose.position.y) / (pointPose.position.x - drivePose.position.x)) 
	if (pointPose.position.x - drivePose.position.x) < 0:
		lineAngle =lineAngle - math.pi
	elif (pointPose.position.x - drivePose.position.x) == 0:
		return 0
	radian = lineAngle-(linear_map(drivePose.orientation.z, -1, 1, -1.5*math.pi, math.pi/2))
	return linear_map(radian, -1.5*math.pi, 0.5*math.pi, -1, 1)
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


def updateRobotPose(ekfPose):
    global robotPose, robotTwistMeasured, transListener, pointsToVisit, robotTwist
    robotTwistMeasured = ekfPose.twist
    poseCache = PoseWithCovarianceStamped()
    poseCache.pose = ekfPose.pose
    poseCache.header = ekfPose.header

    #now = rospy.Time.now()
    #try:
    #    transListener.waitForTransform('top_plate', 'metal_detector_support', now, rospy.Duration(2.0))    
    #    (trans,rot) = transListener.lookupTransform('top_plate', 'metal_detector_support', now)
   # except:
   #     return
    
    #poseErrorMat = transformations.concatenate_matrices(transformations.translation_matrix(trans), transformations.quaternion_matrix(rot))
    #poseMat = matrix_from_pose_msg(poseCache.pose.pose)
    #correctedMat = np.dot(poseMat,poseErrorMat)
    
    #poseCache.pose.pose=pose_msg_from_matrix(correctedMat)
    tmp = poseCache.pose.pose.position.x
    poseCache.pose.pose.position.x = poseCache.pose.pose.position.y+distanceFromCenterX
    poseCache.pose.pose.position.y = -tmp+distanceFromCenterY

    robotPose = poseCache
    #rospy.loginfo("robotPose: "+str(robotPose.pose.pose))
    

    rospy.loginfo("pointsToVisit: "+str(len(pointsToVisit)))
    rospy.loginfo("targetPoint: "+str(targetPoint.position))
    
    robotTwist = Twist()

    if len(mines) > 0:
        for mine in mines:
            if get_pose_distance(robotPose.pose.pose, mine.pose) <= minMineDistance:
                if abs(angleToPoint(mine.pose, robotPose.pose.pose)) <= minMineAngle:
                    global pathObstructed, reachedTargetPoint, nearMine
                    nearMine = True
                    robotTwist.linear.x = 0.0  

    if len(pointsToVisit) == 0:
        pubVel = rospy.Publisher('/p3at/cmd_vel', Twist, queue_size=1)
        robotTwist = Twist()
        pubVel.publish(robotTwist)
        rospy.signal_shutdown("all points visited")

    if nearMine and len(mines) > 0:
        leastMineDistance = 100.0
        closestMine = PoseStamped()
        for mine in mines:
            if get_pose_distance(mine.pose, robotPose.pose.pose) < leastMineDistance:
                closestMine.pose.position.x = mine.pose.position.x
                closestMine.pose.position.y = mine.pose.position.y
                leastMineDistance = get_pose_disance(mine.pose, robotPose.pose.pose)
        if abs(angleToPoint(closestMine.pose, robotPose.pose.pose)) <= minMineAngle or get_pose_distance(closestMine.pose, robotPose.pose.pose) <= minMineDistance:
            robotTwist.linear.x = -0.5
            robotTwist.angular.z = -0.5
        else:
            global nearMine, reachedTargetPoint, pathObstructed
            nearMine = False
            reachedTargetPoint = True
            pathObstructed = True
            

    #rospy.loginfo("distance to target: "+str(get_pose_distance(robotPose.pose.pose, targetPoint)))
    if get_pose_distance(robotPose.pose.pose, targetPoint) <= targetPointDistance:
        global reachedTargetPoint
        reachedTargetPoint = True
    else:
        if abs(angleToPoint(targetPoint, robotPose.pose.pose)) <= minAngle:
            for i in laserInfo.ranges[270:540]:
                if i <= minObstacleDistance:
                    global pathObstructed, reachedTargetPoint, obstaclePresent, startedTurningAway
                    pathObstructed = True
                    reachedTargetPoint = True
                    obstaclePresent = True
                    startedTurningAway = True
                    robotTwist.linear.x = 0.0
        if len(mines) > 0:
            for mine in mines:
                if get_pose_distance(robotPose.pose.pose, mine.pose) <= minMineDistance:
                  if abs(angleToPoint(mine.pose, robotPose.pose.pose)) <= minMineAngle:
                      global pathObstructed, reachedTargetPoint
                      pathObstructed = True
                      reachedTargetPoint = True
                      robotTwist.linear.x = 0.0
                      
        rospy.loginfo("angleToPoint: "+str(angleToPoint(targetPoint, robotPose.pose.pose)))
        if angleToPoint(targetPoint, robotPose.pose.pose) > minAngle:
            robotTwist.angular.z = -0.5
            robotTwist.linear.x = 0.0
        elif angleToPoint(targetPoint, robotPose.pose.pose) < -minAngle:
            robotTwist.angular.z = 0.5
            robotTwist.linear.x = 0.0
        if not pathObstructed:
            if abs(angleToPoint(targetPoint, robotPose.pose.pose)) <= minAngle:
                robotTwist.linear.x = 0.5 
                robotTwist.angular.z = 0.0

    if reachedTargetPoint:
        robotTwist.linear.x = 0.0
        global reachedTargetPoint
        global targetPoint
        targetPoint = Pose()
        leastDistance = 100.0
        leastAngle = 5.0
        for pose in pointsToVisit:
            if get_pose_distance(robotPose.pose.pose, pose) < leastDistance and targetPoint.position.x != pose.position.x and targetPoint.position.y != pose.position.y:
                targetPoint.position.x = pose.position.x
                targetPoint.position.y = pose.position.y
                leastDistance = get_pose_distance(robotPose.pose.pose, targetPoint)
        if not pathObstructed:
            for p in range(0, len(pointsToVisit)-1):
                if targetPoint.position.x == pointsToVisit[p].position.x and targetPoint.position.y == pointsToVisit[p].position.y:
                    pointsToVisit.pop(p)
                    reachedTargetPoint = False
                    break
        if pathObstructed:
            if obstaclePresent:
                if startedTurningAway:
                    global startedTurningAway, angleToObstacle
                    startedTurningAway = False
                    angleToObstacle = robotPose.pose.pose.orientation.z
                    robotTwist.linear.x = 0.0
                    robotTwist.angular.z = 0.5
                else:
                    if (robotPose.pose.pose.orientation.z - angleToObstacle) < minObstacleAngle:
                        robotTwist.linear.x = 0.0
                        robotTwist.angular.z = 0.5
                    else:
                        robotTwist.linear.x = 0.0
                        robotTwist.angular.z = 0.0
                        global obstaclePresent
                        obstaclePresent = False
            else:
                leastDistance = 100.0
                leastAngle = 5.0
                targetPoint = Pose()
                robotTwist.linear.x = 0.0
                robotTwist.angular.z = 0.0
                for pose in pointsToVisit:
                    if get_pose_distance(robotPose.pose.pose, pose) < leastDistance and abs(angleToPoint(pose, robotPose.pose.pose)) < leastAngle:
                        targetPoint.position.x = pose.position.x
                        targetPoint.position.y = pose.position.y
                        leastDistance = get_pose_distance(robotPose.pose.pose, pose)
                        leastAngle = abs(angleToPoint(pose, robotPose.pose.pose))
                global pathObstructed, reachedTargetPoint
                pathObstructed = False
                reachedTargetPoint = False
            
    if robotTwist.linear.x >= 0.5 and (coils.left_coil >= 0.6 or coils.right_coil >= 0.6):
        robotTwist.linear.x = 0.2
                 

    pubVel = rospy.Publisher('/p3at/cmd_vel', Twist, queue_size=1)
    pubVel.publish(robotTwist)

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
    targetPoint.position.x = pointsToVisit[0].position.x
    targetPoint.position.y = pointsToVisit[0].position.y

    # Subscribing to all these topics to bring the robot or simulation to live data
    rospy.Subscriber("/HRATC_FW/set_mine", PoseStamped, receiveMine, queue_size = 1)
    rospy.Subscriber("/scan", LaserScan, receiveLaser)
    rospy.Subscriber("/coils", Coil, receiveCoil)
    rospy.Subscriber("/scan_hokuyo", LaserScan, receiveLaserHokuyo)
    rospy.Subscriber("/odometry/filtered", Odometry, updateRobotPose)

    #Starting curses and ROS
    #Thread(target = StartControl).start()
    rospy.spin()
