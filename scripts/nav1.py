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
distanceFromCenterY = 5.0
robotLength = 0.5
pubVel   = rospy.Publisher('/p3at/cmd_vel', Twist, queue_size=1)
robotPose = PoseWithCovarianceStamped()
initialPose = PoseWithCovarianceStamped()
stepPose = PoseWithCovarianceStamped()
goalPose = PoseWithCovarianceStamped()
obstaclePose = PoseWithCovarianceStamped()

minePose = PoseStamped()
mines = []
#laser information
laserInfo = LaserScan()
laserInfoHokuyo = LaserScan()

arenaWidth = 10
arenaHeight = 10

minAngle = 0.1

pointsToVisit = []
pointStep = 2
reachedTargetPoint = True

targetPointDistance = 0.2
minObstacleDistance = 0.3
minMineDistance = 0.5

minMineAngle = 0.5

pathObstructed

targetPoint = Pose()


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
    rospy.loginfo("matrix1: "+str(matrix1))
    rospy.loginfo("matrix2: "+str(matrix2))
    return np.linalg.norm(matrix1[:,3]-matrix2[:,3])

def linear_map(x, in_min, in_max, out_min, out_max):
    	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def angleToPoint(pointPose, drivePose):
	lineAngle = math.atan(pointPose.position.y - drivePose.position.y) / (pointPose.position.x - drivePose.position.x)) 
	if (pointPose.position.x - drivePose.position.x) < 0:
		lineAngle =lineAngle - math.pi
	elif (pointPose.position.x - drivePose.position.x) == 0:
		return 0
	radian = lineAngle-(linear_map(drivePose.orientation.z, -1, 1, -1.5*math.pi, math.pi/2)
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
    global minePose
    global mines
    minePose = MineNow
    mines.append[MineNow]

def updateRobotPose(ekfPose):
    global robotPose, robotTwistMeasured, transListener, stepPose
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
    
     
    robotPose = poseCache 
    
    robotTwist = Twist()
    if len(pointsToVisit) == 0:
        pubVel = rospy.Publisher('/p3at/cmd_vel', Twist, queue_size=1)
        robotTwist = Twist()
        pubVel.publish(robotTwist)
        rospy.signal_shutdown("all points visited")   

    if reachedTargetPoint:
        global reachedTargetPoint
        reachedTargetPoint = False
        global targetPoint
        targetPoint = Pose()
        leastDistance = 100.0
        for pose in pointsToVisit:
            if get_pose_distance(robotPose.pose.pose, pose) < leastDistance and targetPoint.position.x != pose.position.x and targetPoint.position.y != pose.position.y:
                targetPoint.position.x = pose.position.x
                targetPoint.position.y = pose.position.y
                leastDistance = get_pose_distance(robotPose.pose.pose, pose)
        if not pathObstructed:
            for pose in pointsToVisit:
                if targetPoint.position.x == pose.position.x and targetPoint.position.y == pose.position.y:
                    pointsToVisit.remove(pose)
        if pathObstructed:
            global pathObstructed
            pathObstructed = False
    elif get_pose_distance(robotPose.pose.pose, targetPoint) <= targetPointDistance:
        global reachedTargetPoint
        reachedTargetPoint = True
    else:
        if abs(angleToPoint(targetPoint, robotPose.pose.pose)) <= minAngle:
            for i in laserInfo.ranges[360:450]:
                if i <= minObstacleDistance:
                    global pathObstructed, reachedTargetPoint
                    pathObstructed = True
                    reachedTargetPoint = True
        if len(mines) > 0:
            for mine in mines:
                if get_pose_distance(robotPose.pose.pose, mine.pose) <= minMineDistance:
                  if abs(angleToPoint(mine.pose, robotPose.pose.pose)) <= minMineAngle:
                      global pathObstructed, reachedTargetPoint
                      pathObstructed = True
                      reachedTargetPoint = True
        if not pathObstructed:
            if angleToPoint(targetPoint, robotPose.pose.pose) > minAngle:
                robotTwist.angular.z = 0.5
            else if angleToPoint(targetPoint, robotPose.pose.pose) < -minAngle:
                robotTwist.angular.z = -0.5
            else:
                robotTwist.linear.x = 0.8
                

    pubVel = rospy.Publisher('/p3at/cmd_vel', Twist, queue_size=1)
    pubVel.publish(robotTwist)
       
######################### CURSES STUFF ############################




######################### MAIN ############################

def spin():
    rospy.spin()
    
def StartControl():
    wrapper(Traverse)

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

    # Subscribing to all these topics to bring the robot or simulation to live data
    rospy.Subscriber("/HRATC_FW/set_mine", PoseStamped, receiveMine, queue_size = 10)
    rospy.Subscriber("/scan", LaserScan, receiveLaser)
    rospy.Subscriber("/scan_hokuyo", LaserScan, receiveLaserHokuyo)
    rospy.Subscriber("/odometry/filtered", Odometry, updateRobotPose)

    #Starting curses and ROS
    #Thread(target = StartControl).start()
    rospy.spin()
