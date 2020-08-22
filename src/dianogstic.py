#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published
## to the 'chatter' topic

import rospy
from std_msgs.msg import String

import numpy as np
import matplotlib.pyplot as plt
import sys

class alphaLiDAR(object):
    def __init__(self):
        self.iL=0
        self.iR=0
        self.wL=0
        self.wR=0
        self.volt=0
        self.radio=7/100.0
        self.alpha_speed=0
        self.time_window=60*2
        self.N=self.time_window*50
        self.counter=0
        self.data=np.zeros([self.N,7])
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber('/WL', String, self.callback_wl)
        rospy.Subscriber('/WR', String, self.callback_wr)
        rospy.Subscriber('/currents', String, self.callback_currents)
        rospy.Subscriber('/voltage', String, self.callback_voltage)
        rospy.spin()
        self.saveLOG()

    def callback_voltage(self,data):
        arreglo=data.data
        arreglo=arreglo.split(":")
        arreglo=arreglo[1][:-3]
        self.volt=float(arreglo)

    def callback_wl(self,data):
        self.wL=float(data.data)

    def callback_wr(self,data):
        self.wR=float(data.data)

    def callback_currents(self,data):
        self.counter+=1
        #
        arreglo = data.data
        arreglo = arreglo.split(",")
        left = arreglo[0]
        left = left.split(":")
        self.iL = float(left[1])
        #
        right = arreglo[1]
        right = right.split(":")
        right = right[1]
        right = right[:-4]
        self.iR = float(right)
        ##
        if self.counter<self.N:
            speed=0.5*(self.wL+self.wR)*self.radio
            self.data[self.counter-1,:] = np.array([self.counter-1,self.iL,self.iR,self.wL,self.wR,speed,self.volt])
        else:
            print("FINISH")
            fig, (ax1, ax2, ax3, ax4) = plt.subplots(4)
            fig.suptitle('citricos ROSBAG')
            ax1.plot(self.data[:,1],label='left current mA')
            ax1.legend(loc='lower right')
            ax1.plot(self.data[:,2],label='right current mA')
            ax1.legend(loc='lower right')
            ax2.plot(self.data[:,6],label='voltage')
            ax2.legend(loc='lower right')
            ax3.plot(self.data[:,3],label='left wheel speed rad/sec')
            ax3.legend(loc='lower right')
            ax3.plot(self.data[:,4],label='right wheel speed rad/sec')
            ax3.legend(loc='lower right')
            ax4.plot(self.data[:,5],label='alphaRover spped m/s')
            ax4.legend(loc='lower right')
            plt.show()
            sys.exit()



if __name__ == '__main__':
    alpha= alphaLiDAR()
    listener()
