#!/usr/bin/env python3
from unittest import runner
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from iiwa_msgs.msg import CartesianPose
import datetime
x = 0
y = 0
z = 0
xActual = 0
yActual = 0
zActual = 0
new = False
startingTime = 0

def callback(data):
    global x
    global y
    global z
    global new
    x = data.data[0]
    y = data.data[1]
    z = data.data[2]
    new = True

def callbackActual(data):
    global xActual
    global yActual
    global zActual
    xActual = data.poseStamped.pose.position.x * 1000
    yActual = data.poseStamped.pose.position.y * 1000
    zActual = data.poseStamped.pose.position.z * 1000

def subscriber():

    first = False
    fig = plt.figure()
    global new
    new = False
    # syntax for 3-D projection
    global ax 
    global x
    global y
    global z
    global xActual
    global yActual
    global zActual
    global startingTime

    rospy.init_node('eefVisualizer', anonymous=True)

    print("here")
    rospy.Subscriber("eefGoal", Float32MultiArray, callback)
    rospy.Subscriber("iiwa/state/CartesianPose", CartesianPose, callbackActual)

    fig, (plt1, plt2, plt3) = plt.subplots(3)
    fig.suptitle('X Y Z')


    while(1):
        if(new):
            if(not first):
                first = True
                startingTime = datetime.datetime.now()
                # plt1.scatter(0, x ,color='green')
                # plt1.scatter(0, xActual, color='red')

                # plt2.scatter(0, y ,color='green')
                # plt2.scatter(0, yActual, color='red')

                # plt3.scatter(0, z ,color='green')
                # plt3.scatter(0, zActual, color='red')
            else:
                plt1.scatter((datetime.datetime.now() - startingTime).total_seconds() * 1000, x - xActual ,color='green')
                # plt1.scatter((datetime.datetime.now() - startingTime).total_seconds() * 1000, xActual, color='red')

                plt2.scatter((datetime.datetime.now() - startingTime).total_seconds() * 1000, y-yActual ,color='green')
                # plt2.scatter((datetime.datetime.now() - startingTime).total_seconds() * 1000, yActual, color='red')

                plt3.scatter((datetime.datetime.now() - startingTime).total_seconds() * 1000, z-zActual ,color='green')
                # plt3.scatter((datetime.datetime.now() - startingTime).total_seconds() * 1000, zActual, color='red')
            new = False
            plt.pause(0.05)

    rospy.spin()

if __name__ == '__main__':
    subscriber()
    

    