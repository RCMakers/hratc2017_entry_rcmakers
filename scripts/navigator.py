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
from nav_msgs.msg import Odometry

# read/write stuff on screen
std = None
# Robot data
robotTwist = Twist()
robotTwistMeasured = Twist()
distanceFromCenterX = 4.5
distanceFromCenterY = 5.0
robotLength = 0.5

pubVel = rospy.Publisher('p3at/cmd_vel', Twist, queue_size=1)

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
    global robotPose, robotTwistMeasured
    robotTwistMeasured = ekfPose.twist
    poseCache = PoseWithCovarianceStamped()
    poseCache.pose = ekfPose.pose
    poseCache.header = ekfPose.header
    tmp = poseCache.pose.pose.position.x
    poseCache.pose.pose.position.x = poseCache.pose.pose.position.y+distanceFromCenterX
    poseCache.pose.pose.position.y = -tmp+distanceFromCenterY
    robotPose = poseCache

######################### CURSES STUFF ############################

def checkMine():
    for mine in mines:
        if get_pose_distance(mine.pose,robotPose.pose.pose) <= 0.8:
            return True
    return False 
def turnAround():
    if abs(robotPose.pose.pose.orientation.z) < 0.3:
        while robotPose.pose.pose.orientation.z > -0.68: #turn left
            robotTwist.angular.z = -0.3
            pubVel.publish(robotTwist)
            #time.sleep(0.1)
        robotTwist.angular.z = 0.0 
        pubVel.publish(robotTwist)


        obstaclePose = robotPose

        while get_pose_distance(obstaclePose.pose.pose, robotPose.pose.pose) < 1: #go forward for 1 unit
            robotTwist.linear.x = 0.2
            pubVel.publish(robotTwist)
            #time.sleep(0.1)
        robotTwist.linear.x = 0.0 
        pubVel.publish(robotTwist)

        while robotPose.pose.pose.orientation.z < 0.96: #turn right
            robotTwist.angular.z = -0.3
            pubVel.publish(robotTwist)
            #time.sleep(0.1)
        robotTwist.angular.z = 0.0 
        pubVel.publish(robotTwist)

        obstaclePose = robotPose  

        stepPose = robotPose
        stepPose.pose.pose.position.x += 10
        return stepPose

    elif abs(robotPose.pose.pose.orientation.z) < 0.7:
        while robotPose.pose.pose.orientation.z < -0.68 or robotPose.pose.pose.orientation.z > 0: #turn left
            robotTwist.angular.z = 0.3
            pubVel.publish(robotTwist)
            #time.sleep(0.1)
        robotTwist.angular.z = 0.0 
        pubVel.publish(robotTwist)


        obstaclePose = robotPose

        while get_pose_distance(obstaclePose.pose.pose, robotPose.pose.pose) < 1: #go forward for 1 unit
            robotTwist.linear.x = 0.2
            pubVel.publish(robotTwist)
            #time.sleep(0.1)
        robotTwist.linear.x = 0.0 
        pubVel.publish(robotTwist)

        while robotPose.pose.pose.orientation.z < -0.02: #turn left
            robotTwist.angular.z = 0.3
            pubVel.publish(robotTwist)
            #time.sleep(0.1)
        robotTwist.angular.z = 0.0 
        pubVel.publish(robotTwist)

        stepPose = robotPose
        stepPose.pose.pose.position.x -= 10
        return stepPose

def avoidObstacle():
    if abs(robotPose.pose.pose.orientation.z) < 0.3: #if going down
        while robotPose.pose.pose.orientation.z > -0.68: #turn right
            robotTwist.angular.z = -0.3
            pubVel.publish(robotTwist)
            #time.sleep(0.1)
        robotTwist.angular.z = 0.0 
        pubVel.publish(robotTwist)

        obstaclePose = robotPose

        while get_pose_distance(obstaclePose.pose.pose, robotPose.pose.pose) < 1: #go forward for 1 unit
            robotTwist.linear.x = 0.2
            pubVel.publish(robotTwist)
            #time.sleep(0.1)
        robotTwist.linear.x = 0.0 
        pubVel.publish(robotTwist)

        while robotPose.pose.pose.orientation.z < -0.01: #turn left
            robotTwist.angular.z = 0.3
            pubVel.publish(robotTwist)
            #time.sleep(0.1)
        robotTwist.angular.z = 0.0 
        pubVel.publish(robotTwist)

        obstaclePose = robotPose  

        while get_pose_distance(obstaclePose.pose.pose, robotPose.pose.pose) < 2: #go forward for 2 unit
            robotTwist.linear.x = 0.2
            pubVel.publish(robotTwist)
            #time.sleep(0.1)
        robotTwist.linear.x = 0.0 
        pubVel.publish(robotTwist)

        while robotPose.pose.pose.orientation.z < 0.68: #turn left
            robotTwist.angular.z = 0.3
            pubVel.publish(robotTwist)
            #time.sleep(0.1)
        robotTwist.angular.z = 0.0 
        pubVel.publish(robotTwist)
        obstaclePose = robotPose 


        while get_pose_distance(obstaclePose.pose.pose, robotPose.pose.pose) < 1: #go forward for 1 unit
            robotTwist.linear.x = 0.2
            pubVel.publish(robotTwist)
            #time.sleep(0.1)
        robotTwist.linear.x = 0.0 
        pubVel.publish(robotTwist)

        while robotPose.pose.pose.orientation.z > 0.01: #turn right
            robotTwist.angular.z = -0.3
            pubVel.publish(robotTwist)
            #time.sleep(0.1)
        robotTwist.angular.z = 0.0 
        pubVel.publish(robotTwist)


    elif abs(robotPose.pose.pose.orientation.z) > 0.7:
        while robotPose.pose.pose.orientation.z < -0.68 or robotPose.pose.orientation.z > 0: #turn left
            robotTwist.angular.z = 0.3
            pubVel.publish(robotTwist)
            #time.sleep(0.1)
        robotTwist.angular.z = 0.0 
        pubVel.publish(robotTwist)

        obstaclePose = robotPose

        while get_pose_distance(obstaclePose, robotPose) < 1: #go forward for 1 unit
            robotTwist.linear.x = 0.2
            pubVel.publish(robotTwist)
            #time.sleep(0.1)
        robotTwist.linear.x = 0.0 
        pubVel.publish(robotTwist)

        while robotPose.pose.pose.orientation.z < 0.96: #turn right
            robotTwist.angular.z = -0.3
            pubVel.publish(robotTwist)
            #time.sleep(0.1)
        robotTwist.angular.z = 0.0 
        pubVel.publish(robotTwist)

        obstaclePose = robotPose  

        while get_pose_distance(obstaclePose, robotPose) < 2: #go forward for 2 unit
            robotTwist.linear.x = 0.2
            pubVel.publish(robotTwist)
            #time.sleep(0.1)
        robotTwist.linear.x = 0.0 
        pubVel.publish(robotTwist)

        while robotPose.pose.pose.orientation.z > 0.68 or robotPose.pose.orientation.z < 0: #turn right
            robotTwist.angular.z = -0.3
            pubVel.publish(robotTwist)
            #time.sleep(0.1)
        robotTwist.angular.z = 0.0 
        pubVel.publish(robotTwist)
        obstaclePose = robotPose 

        while get_pose_distance(obstaclePose, robotPose) < 1: #go forward for 1 unit
            robotTwist.linear.x = 0.2
            pubVel.publish(robotTwist)
            #time.sleep(0.1)
        robotTwist.linear.x = 0.0 
        pubVel.publish(robotTwist)

        while robotPose.pose.pose.orientation.z > 0.96: #turn left
            robotTwist.angular.z = 0.3
            pubVel.publish(robotTwist)
            #time.sleep(0.1)
        robotTwist.angular.z = 0.0 
        pubVel.publish(robotTwist)


# Basic control
def Traverse(stdscr):
    stdscr.keypad(True)
    stdscr.nodelay(True)
    logstr = "" 
    k = None
    direction = -1
    initialPose = robotPose
    stepPose = robotPose
    stepPose.pose.pose.position.x -= 10
    global std
    std = stdscr
    #publishing topics
    pubVel   = rospy.Publisher('/p3at/cmd_vel', Twist, queue_size=1)
    global mine_detected
    mine_detected = False
    # While 'Esc' is not pressed
    while k != chr(27):
        
        if get_pose_distance(goalPose.pose.pose, initialPose.pose.pose) <= 0.3:
            stepPose = turnAround()
        elif checkMine() <= 0.8:
            avoidObstacle()
        else:
            robotTwist.linear.x = 0.3
            pubVel.publish(robotTwist)

            # Check no key
        try:
            k = stdscr.getkey()
        except:
            k = None

            #time.sleep(0.1)

    stdscr.keypad(False)
    rospy.signal_shutdown("Shutdown Competitor")

######################### MAIN ############################

def spin():
    rospy.spin()
    
def StartControl():
    wrapper(Traverse)

if __name__ == '__main__':
    # Initialize client node
    rospy.init_node('navigator')
    
    transListener = tf.TransformListener()

    # Subscribing to all these topics to bring the robot or simulation to live data
    rospy.Subscriber("/HRATC_FW/set_mine", PoseStamped, recieveMine, queue_size = 10)
    rospy.Subscriber("/scan", LaserScan, receiveLaser)
    rospy.Subscriber("/scan_hokuyo", LaserScan, receiveLaserHokuyo)
    rospy.Subscriber("/odometry/filtered", Odometry, updateRobotPose)

    #Starting curses and ROS
    Thread(target = StartControl).start()
    Thread(target = spin).start()


