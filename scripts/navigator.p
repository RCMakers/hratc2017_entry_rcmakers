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
# Robot data
robotTwist = Twist()
robotTwistMeasured = Twist()
distanceFromCenterX = 4.5
distanceFromCenterY = 5.0
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
    if getAngle(robotPose.pose.pose) > 355 or getAngle(robotPose.pose.pose) < 5:
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
    minePose = MineNow
    mines.append[MineNow]

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
    rospy.loginfo("Angle: " getAngle(robotPose.pose.pose))
    if first:
        initialPose = robotPose
        first = False
        stepPose.pose.pose.position.x = robotPose.pose.pose.position.x - 10
        stepPose.pose.pose.position.y = robotPose.pose.pose.position.y 

    if get_pose_distance(stepPose, robotPose) <= 0.3 or turning:

        stepPose = turnAround()
    elif checkMine() or avoidingObstacle:
        avoidObstacle()
    elif laserInfoHokuyo.ranges != []:
        if min(laserInfoHokuyo.ranges) <= 0.3:
            avoidObstacle()
    else:
        robotTwist.linear.x = 0.3
        pubVel.publish(robotTwist)


######################### CURSES STUFF ############################

def checkMine():
    for mine in mines:
        if get_pose_distance(mine,robotPose) <= 0.8:
            return True
    return False

def avoidObstacle():
    global obstacleSeq, obstacleStep, avoidingObstacle, robotTwist, pubVel
    avoidingObstacle = True
    if obstacleSeq == 1:
        if obstacleStep == 1:
            if getAngle(robotPose.pose.pose) == 0 or getAngle(robotPose.pose.pose) > 270: #turn right
                robotTwist.angular.z = -0.3
                pubVel.publish(robotTwist)
            else:
                robotTwist.angular.z = 0.0 
                pubVel.publish(robotTwist)
                obstaclePose = robotPose
                obstacleStep = 2

        elif obstacleStep == 2:
            if get_pose_distance(obstaclePose, robotPose) < 1: #go forward for 1 unit
                robotTwist.linear.x = 0.2
                pubVel.publish(robotTwist)
            else:
                robotTwist.linear.x = 0.0 
                pubVel.publish(robotTwist)
                obstacleStep = 3

        elif obstacleStep == 3:
            if getAngle(robotPose.pose.pose) > 0: #turn left
                robotTwist.angular.z = 0.3
                pubVel.publish(robotTwist)
            else:
                robotTwist.angular.z = 0.0 
                pubVel.publish(robotTwist)
                obstaclePose = robotPose 
                obstacleStep = 4 

        elif obstacleStep == 4:
            if get_pose_distance(obstaclePose, robotPose) < 2: #go forward for 2 unit
                robotTwist.linear.x = 0.2
                pubVel.publish(robotTwist)
            else:
                robotTwist.linear.x = 0.0 
                pubVel.publish(robotTwist)
                obstacleStep = 5

        elif obstacleStep == 5:
            if getAngle(robotPose.pose.pose) < 90: #turn left
                robotTwist.angular.z = 0.3
                pubVel.publish(robotTwist)
            else:
                robotTwist.angular.z = 0.0 
                pubVel.publish(robotTwist)
                obstaclePose = robotPose 
                obstacleStep = 6

        elif obstacleStep == 7:
            if get_pose_distance(obstaclePose, robotPose) < 1: #go forward for 1 unit
                robotTwist.linear.x = 0.2
                pubVel.publish(robotTwist)
            else:
                robotTwist.linear.x = 0.0 
                pubVel.publish(robotTwist)
                obstacleStep = 8

        elif obstacleStep == 8:
            if getAngle(robotPose.pose.pose) > 0: #turn right
                robotTwist.angular.z = -0.3
                pubVel.publish(robotTwist)
            else:
                robotTwist.angular.z = 0.0 
                pubVel.publish(robotTwist)
                obstacleStep = 1
                obstacleSeq = 0
                avoidingObstacle = False

    elif obstacleSeq == 2:
        if obstacleStep == 1:
            if getAngle(robotPose.pose.pose) < 270: #turn left
                robotTwist.angular.z = 0.3
                pubVel.publish(robotTwist)
            else:    
                robotTwist.angular.z = 0.0 
                pubVel.publish(robotTwist)
                obstacleStep = 2
                obstaclePose = robotPose

        elif obstacleStep == 2:
            if get_pose_distance(obstaclePose, robotPose) < 1: #go forward for 1 unit
                robotTwist.linear.x = 0.2
                pubVel.publish(robotTwist)
            else:  
                robotTwist.linear.x = 0.0 
                pubVel.publish(robotTwist)
                obstacleStep = 3

        elif obstacleStep == 3:
            if getAngle(robotPose.pose.pose) > 180: #turn right
                robotTwist.angular.z = -0.3
                pubVel.publish(robotTwist)
            else:
                robotTwist.angular.z = 0.0 
                pubVel.publish(robotTwist)
                obstaclePose = robotPose  
                obstacleStep = 4

        elif obstacleStep == 4:
            if get_pose_distance(obstaclePose, robotPose) < 2: #go forward for 2 unit
                robotTwist.linear.x = 0.2
                pubVel.publish(robotTwist)
            else:    
                robotTwist.linear.x = 0.0 
                pubVel.publish(robotTwist)
                obstacleStep = 5

        elif obstacleStep == 5:
            if getAngle(robotPose.pose.pose) > 90: #turn right
                robotTwist.angular.z = -0.3
                pubVel.publish(robotTwist)
            else:   
                robotTwist.angular.z = 0.0 
                pubVel.publish(robotTwist)
                obstaclePose = robotPose 
                obstacleStep = 6

        elif obstacleStep == 6:
            if get_pose_distance(obstaclePose, robotPose) < 1: #go forward for 1 unit
                robotTwist.linear.x = 0.2
                pubVel.publish(robotTwist)
            else:  
                robotTwist.linear.x = 0.0 
                pubVel.publish(robotTwist)
                obstacleStep = 7

        elif obstacleStep == 7:
            if getAngle(robotPose.pose.pose) < 180: #turn left
                robotTwist.angular.z = 0.3
                pubVel.publish(robotTwist)
            else:    
                robotTwist.angular.z = 0.0 
                pubVel.publish(robotTwist)
                obstacleStep = 1
                obstacleSeq = 0
                avoidingObstacle = False

    elif getAngle(robotPose.pose.pose) == 0:
        obstacleSeq = 1
    elif abs(getAngle(robotPose.pose.pose) - 180) < 5:
        obstacleSeq = 2

def turnAround():
    global turnSeq, turnStep, turning, robotTwist, pubVel
    turning = True
    if turnSeq == 1:
        if turnStep == 1:
            if getAngle(robotPose.pose.pose) == 0 or getAngle(robotPose.pose.pose) > 270: #turn left 
                robotTwist.angular.z = -0.3
                pubVel.publish(robotTwist)
                
            else:
                robotTwist.angular.z = 0.0 
                pubVel.publish(robotTwist)
                turnStep = 2
                obstaclePose = robotPose
        elif turnStep == 2:
            if get_pose_distance(obstaclePose, robotPose) < 1: #go forward for 1 unit
                robotTwist.linear.x = 0.2
                pubVel.publish(robotTwist)
                
            else:
                robotTwist.linear.x = 0.0 
                pubVel.publish(robotTwist)
                turnStep = 3
        elif turnStep == 3:
            if getAngle(robotPose.pose.pose) > 180: #turn right
                robotTwist.angular.z = -0.3
                pubVel.publish(robotTwist)
                
            else:
                robotTwist.angular.z = 0.0 
                pubVel.publish(robotTwist)
                turnStep = 4
                obstaclePose = robotPose  
        elif turnStep == 4:
            stepPose = robotPose
            stepPose.pose.position.x += 10
            turnStep = 1
            turnSeq = 0
            turning = False
            return stepPose
    
    elif turnSeq == 2:
        if turnStep = 1:
            if getAngle(robotPose.pose.pose) < 270: #turn left
                robotTwist.angular.z = 0.3
                pubVel.publish(robotTwist)
            else:
                robotTwist.angular.z = 0.0 
                pubVel.publish(robotTwist)
                obstaclePose = robotPose
                turnStep = 2
        elif turnStep == 2:
            if get_pose_distance(obstaclePose, robotPose) < 1: #go forward for 1 unit
                robotTwist.linear.x = 0.2
                pubVel.publish(robotTwist)
                
            else:
                robotTwist.linear.x = 0.0 
                pubVel.publish(robotTwist)
                turnStep = 3
        elif turnStep == 3:
            if getAngle(robotPose.pose.pose) > 0: #turn left
                robotTwist.angular.z = 0.3
                pubVel.publish(robotTwist)
                
            else:
                robotTwist.angular.z = 0.0 
                pubVel.publish(robotTwist)
                turnStep = 4
        elif turnStep == 4:
            stepPose = robotPose
            stepPose.pose.position.x -= 10
            turnStep = 1
            turnSeq = 0
            turning = False
            return stepPose
    elif getAngle(robotPose.pose.pose) == 0:
        turnSeq = 1
    elif abs(getAngle(robotPose.pose.pose) - 180) < 5:
        turnSeq = 2

######################### MAIN ############################

if __name__ == '__main__':
    # Initialize client node
    rospy.init_node('navigator')
    
    transListener = tf.TransformListener()

    # Subscribing to all these topics to bring the robot or simulation to live data
    rospy.Subscriber("/HRATC_FW/set_mine", PoseStamped, recieveMine, queue_size = 10)
    rospy.Subscriber("/scan", LaserScan, receiveLaser)
    rospy.Subscriber("/scan_hokuyo", LaserScan, receiveLaserHokuyo)
    rospy.Subscriber("/locator/odom", PoseWithCovarianceStamped, updateRobotPose)
    #Starting curses and ROS
    rospy.spin()


