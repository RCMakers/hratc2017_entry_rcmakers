#!/usr/bin/env python

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
import pickle

directory = os.path.dirname(os.path.abspath(__file__))

PIK = os.path.join(directory, '..', 'boundarypoints.dat')

pointsToAvoid = []

def getPoint(odom):
    pointsToAvoid.append(odom.pose.pose)

def writePickle():
    rospy.loginfo("Writing.")
    with open(PIK,"wb") as f:
        pickle.dump(pointsToAvoid, f)
        f.close()

if __name__ == "__main__":
    rospy.init_node('logger')
    rospy.Subscriber("/RosAria/pose", Odometry, getPoint)
    rospy.spin()
    rospy.on_shutdown(writePickle)
