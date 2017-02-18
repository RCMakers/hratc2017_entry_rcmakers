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

robotAngle = 0.0

coils = Coil()

minePose = PoseStamped()
mines = []
#laser information
laserInfo = LaserScan()
laserInfoHokuyo = LaserScan()

arenaWidth = 10
arenaHeight = 10

minAngle = 0.3

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

minMineAngle = 0.5

pathObstructed = False

targetPoint = Pose()

previousTargetDistance = 0.0

unmanageable = False


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


def getAngle(drivePose):
    (roll, pitch, yaw) = euler_from_quaternion([drivePose.orientation.x, drivePose.orientation.y, drivePose.orientation.z, drivePose.orientation.w])
    if yaw > 0:
        return yaw
    else:
        return 2*math.pi + yaw

def angleToPoint(pointPose, drivePose):

    #(roll, pitch, yaw) = euler_from_quaternion([drivePose.orientation.x, drivePose.orientation.y, drivePose.orientation.z, drivePose.orientation.w])

    yaw = getAngle(drivePose)
    lineAngle = math.atan((pointPose.position.x - drivePose.position.x) / (pointPose.position.y - drivePose.position.y))
    if (pointPose.position.y-drivePose.position.y) <= 0:
        lineAngle = math.pi-lineAngle
    if linengle < 0:
        lineAngle = lineAngle+math.pi*2
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
    rospy.loginfo("robotPose: "+str(robotPose.pose.pose))
    

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
        return

    if unmanageable:
        if abs(angleToPoint(targetPoint, robotPose.pose.pose)) > minAngle:
            robotTwist.linear.x = 0.0
            robotTwist.angular.z = -1.0
            pubVel = rospy.Publisher('/p3at/cmd_vel', Twist, queue_size =1)
            pubVel.publish(robotTwist)
            return
        else:
            global unmanageable
            unmanageable = False
 


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
            
    rospy.loginfo("distance to target: "+str(get_pose_distance(robotPose.pose.pose, targetPoint)))
    if get_pose_distance(robotPose.pose.pose, targetPoint) <= targetPointDistance:
        global reachedTargetPoint
        reachedTargetPoint = True
    else:
        if True: #abs(angleToPoint(targetPoint, robotPose.pose.pose)) <= minAngle:
            for i in laserInfo.ranges[360:450]:
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

        if abs(angleToPoint(targetPoint, robotPose.pose.pose)) > 1.3:
            global unmanageable
            unmanageable = True
            return
     
       if abs(angleToPoint(targetPoint, robotPose.pose.pose) > minAngle):
            robotTwist.angular.z = 0.5
            robotTwist.linear.x = 0.0
        elif angleToPoint(targetPoint, robotPose.pose.pose) < -minAngle:
            robotTwist.angular.z = -0.5
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
                    angleToObstacle = 2 * math.atan(robotPose.pose.pose.orientation.z / robotPose.pose.pose.orientation.w)
                    robotTwist.linear.x = 0.0
                    robotTwist.angular.z = 0.5
                else:
                    robotYaw = 2 * math.atan(robotPose.pose.pose.orientation.z / robotPose.pose.pose.orientation.w)
                    if abs(robotYaw - angleToObstacle) < minObstacleAngle:
                        rospy.loginfo("angletoobstacle: "+str(abs(robotPose.pose.pose.orientation.z - angleToObstacle)))
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
            
    if robotTwist.linear.x >= 0.5 and (coils.left_coil >= 0.7 or coils.right_coil >= 0.7):
        global nearMine
        nearMine = True
                 
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
