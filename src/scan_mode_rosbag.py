#!/usr/bin/env python
# https://github.com/arebgun/dynamixel_motor 	libreria dynamixel-ROS
# http://docs.ros.org/kinetic/api/dynamixel_msgs/html/msg/JointState.html 	comandos
# rosrun urg_node urg_node _ip_address:="192.168.0.10" _publish_multiecho:="true"

import rospy
from sensor_msgs.msg import Joy
#from dynamixel_msgs.msg import JointState
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import MultiEchoLaserScan
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist
import tf
import matplotlib.pyplot as plt
import numpy as np
import sys,math,time,os,datetime

import pandas as pd
import csv
import os

# >0 down
# <0 up

# python move_dynamixel [angle]
# python scan_data 45

ang_scan = float(sys.argv[1])
dyn_offset = 0 # 0 just for rosbag
count = 0

class First(): # Crea clase
    def __init__(self):
        rospy.init_node("scan_node_mode2")
        rospy.Subscriber("/joy", Joy, self.readJoy)
        self.pub = rospy.Publisher('/tilt_controller/command', Float64, queue_size=10)
        #rospy.Subscriber('/tilt_controller/state', JointState, self.read_Dynamixel) # 20Hz
        rospy.Subscriber('/echoes', MultiEchoLaserScan, self.read_LiDAR) #40Hz
        rospy.Subscriber('/ekf', Odometry, self.read_odom) #50 Hz
        rospy.Subscriber('/imu_data', Imu, self.read_imu)   # 100 Hz
        self.stop_flag = 0
        self.read_pose = 45.8473*np.pi/180.0                # Just for rosbag
        # By Tila
        self.x = 0
        self.y = 0
        self.z = 0
        ##
        self.moving= 0
        self.fileText=''
        self.lidar_showFLAG=0
        self.move_ini(ang_scan)
        rospy.spin()

    def createFile(self):
        Ruta=os.path.abspath('..')
        self.fileText=Ruta+"/data/"+"RAW_DATA_from_rosbag"+".txt"
        print("Fichero: "+self.fileText+"\n")
        f = open(self.fileText,'w')
        f.close()

    def readJoy(self, data):
        buttons = data.buttons
        axes = data.axes
        if (axes[2]<=-0.9 and buttons[4]==1 and buttons[1]==1):	# LT + LB + B
            self.stop_flag=1
            print("F L A G = 1")
        else:
            self.stop_flag=0


    def read_imu(self,data):
        qx = data.orientation.x
        qy = data.orientation.y
        qz = data.orientation.z
        qw = data.orientation.w
        q = np.array([qx,qy,qz,qw])
        euler = tf.transformations.euler_from_quaternion(q)
        self.imu_roll = euler[0]
        self.imu_pitch  = euler[1]
        self.imu_yaw   = euler[2]

    def read_odom(self,data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.z = data.pose.pose.position.z

    def read_LiDAR(self,data):
        #print dir(data)
        ranges = data.ranges
        N=len(ranges)
        intensities = data.intensities
        delta_angle = data.angle_increment
        max_angle =   data.angle_max
        min_angle =   data.angle_min
        if self.lidar_showFLAG==0:
            print("Data from Hokuyo fw: ")
            print("\t max_angle="  +str(max_angle*180.0/np.pi))
            print("\t min_angle="  +str(min_angle*180.0/np.pi))
            print("\t delta_angle="+str(delta_angle*180.0/np.pi))
            print("\t 2D samples="+str(N))
            self.lidar_showFLAG=1
        theta = np.linspace(min_angle,max_angle,N)
        self.ranges_0 = np.zeros([1,N])
        self.ranges_1 = np.zeros([1,N])
        self.ranges_2 = np.zeros([1,N])
        self.intensities_0 = np.zeros([1,N])
        self.intensities_1 = np.zeros([1,N])
        self.intensities_2 = np.zeros([1,N])
        self.number_of_returns=np.zeros([1,N])
        self.scan_angle=np.zeros([1,N])
        for k in range(0,N-1):
            aux = str(ranges[k]).split("[")
            aux = str(aux[1]).strip("]")
            aux = aux.split(",")
            aux2 = str(intensities[k]).split("[")
            aux2 = str(aux2[1]).strip("]")
            aux2 = aux2.split(",")
            self.scan_angle[0,k]=theta[k]
            if len(aux)==1:
            	self.ranges_0[0,k] = float(aux[0])
            	self.intensities_0[0,k] = (aux2[0])
                self.number_of_returns[0,k]=1
            elif len(aux)==2:
            	self.ranges_0[0,k] = (aux[0])
            	self.ranges_1[0,k] = (aux[1])
            	self.intensities_0[0,k] = (aux2[0])
            	self.intensities_1[0,k] = (aux2[1])
                self.number_of_returns[0,k]=2
            elif len(aux)==3:
            	self.ranges_0[0,k] = (aux[0])
            	self.ranges_1[0,k] = (aux[1])
            	self.ranges_2[0,k] = (aux[2])
            	self.intensities_0[0,k] = (aux2[0])
            	self.intensities_1[0,k] = (aux2[1])
            	self.intensities_2[0,k] = (aux2[2])
                self.number_of_returns[0,k]=3
        self.saveINFO()


    def move_ini(self,ang):
        self.createFile()


    def saveINFO(self):
        N = 1081
        n = (N*6)+7
        #M = np.zeros((num	, n))
        f = open(self.fileText,"a") #opens file with name of "test.txt"
        dataLine=''
        dataLine=dataLine+str(self.read_pose+dyn_offset)+"\t"
        dataLine=dataLine+str(self.x)+"\t"
        dataLine=dataLine+str(self.y)+"\t"
        dataLine=dataLine+str(self.z)+"\t"
        dataLine=dataLine+str(self.imu_pitch)+"\t"
        dataLine=dataLine+str(self.imu_roll)+"\t"
        dataLine=dataLine+str(self.imu_yaw)+"\t"
        N = self.ranges_0.shape[1]
        for i in range(0,N-1):
        	dataLine=dataLine+str(self.ranges_0[0,i])+"\t"
        N = self.ranges_1.shape[1]
        for i in range(0,N-1):
        	dataLine=dataLine+str(self.ranges_1[0,i])+"\t"
        N = self.ranges_2.shape[1]
        for i in range(0,N-1):
            dataLine=dataLine+str(self.ranges_2[0,i])+"\t"
        N = self.intensities_0.shape[1]
        for i in range(0,N-1):
            dataLine=dataLine+str(self.intensities_0[0,i])+"\t"
        N = self.intensities_1.shape[1]
        for i in range(0,N-1):
            dataLine=dataLine+str(self.intensities_1[0,i])+"\t"
        N = self.intensities_2.shape[1]
        for i in range(0,N-1):
            dataLine=dataLine+str(self.intensities_2[0,i])+"\t"
        ##
        for i in range(0,N-1):
            dataLine=dataLine+str(self.number_of_returns[0,i])+"\t"
        for i in range(0,N-1):
            dataLine=dataLine+str(self.scan_angle[0,i])+"\t"
        dataLine=dataLine+"\n"
        f.write(dataLine)
        if (self.stop_flag==1):
        	f.close()
        	print("Scan finished")
        	exit()

if __name__ == '__main__':
    try:
    	cv = First()
    except rospy.ROSInterruptException:
    	print("Interrupted before completion")
