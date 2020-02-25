#!/usr/bin/env python

# The MIT License (MIT)
#
# Copyright (c) 2015 Paul Wachendorf <paul.wachendorf@web.de>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""This file holds the all subclasses that make up the robot"""
from math import *
from adafruit_servokit import ServoKit
import time

class body(object):
    """this object holds all state information and computes inverse kinematics for robot body""" 
    def __init__(self, blength, bwidth, boffset):
        """parameters of the robot body"""
        self.length = blength
        self.width = bwidth
        self.offset = boffset

        #initialize body position
        self.bodyPos_x = 0
        self.bodyPos_y = 0
        self.bodyPos_z = 0

        #initialize body rotation
        self.bodyRot_x = 0 #pitch
        self.bodyRot_y = 0 #roll
        self.bodyRot_z = 0 #yaw

        #current states
        self.isMoving = False
        #speed level
        self.speedLevel = 0

        #gaits
        self.gaitType = 0
        self.gaitStep = 0
        self.gaits = {
            'Tripod' : [(1, 0, 1, 0, 1, 0), \
                        (0, 1, 0, 1, 0, 1)], \
            'Ripple' : [(1, 0, 0, 0, 0, 1), \
                        (0, 0, 1, 0, 1, 0), \
                        (0, 1, 0, 1, 0, 0)], \
            'Wave'   : [(1, 0, 0, 0, 0, 0),\
                        (0, 1, 0, 0, 0, 0),\
                        (0, 0, 1, 0, 0, 0),\
                        (0, 0, 0, 1, 0, 0),\
                        (0, 0, 0, 0, 1, 0),\
                        (0, 0, 0, 0, 0, 1)]
                        }
        self.gaitID = 'Tripod' #Tripod is default gait

    def bodyIK(self):
        x = self.bodyPos_x
        y = self.bodyPos_y
        z = self.bodyPos_z
        rx = self.bodyRot_x
        ry = self.bodyRot_y
        rz = self.bodyRot_z

    def gaitSelect(self):
        if self.gaitType < 2:
            self.gaitType += 1
        else:
            self.gaitType = 0

    def gaitSeq(self):
        """build the gait sequence"""
        if self.gaitType == 0:
            self.gaitID = 'Tripod'
        elif self.gaitType == 1:
            self.gaitID = 'Ripple'
        else:
            self.gaitID = 'Wave'


class hexleg(object):
	"""object to calculate inverse kinematics for 3dof hexapod leg and control servo motion"""
	def __init__(self, legIndex, x_init, y_init, z_init, isInverted=False):
		
		self.legIndex = legIndex
		self.x = x_init
		self.y = y_init
		self.z = z_init

		# Parameters for inverse kinematics test
        #length of leg limbs in inches
		self.l_tibia = 7.1
		self.l_femur = 5.2
		self.l_coxa = 3.4
        #initialize angle variable
		self.a_tibia = 0.0
		self.a_femur = 0.0 
		self.a_coxa = 0.0 

		if legIndex <= 2:
			servo_address = 0x40
		else:
			servo_address = 0x60
		self.kit = ServoKit(channels=16, address=servo_address) 

	def inverseKin(self):
		#equations taken from blog post by user downeym here: https://www.robotshop.com/community/forum/t/inverse-kinematic-equations-for-lynxmotion-3dof-legs/21336
		legLength = math.sqrt((self.x**2) + (self.y**2))
		HF = math.sqrt((legLength - self.l_coxa)**2 + (self.z**2))
		A1 = math.degrees(math.atan2(legLength-self.l_coxa, self.z))
		A2 = math.degrees(math.acos((self.l_tibia**2 - self.l_femur**2 - HF**2)/(-2* self.l_femur* HF)))
		B1 = math.degrees(math.acos((HF**2 - self.l_tibia**2 - self.l_femur**2)/(-2* self.l_femur* self.l_tibia)))

		self.a_tibia = 90 - B1 + 90
		self.a_femur = 90 - (A1 + A2) + 90
		self.a_coxa = math.degrees(math.atan2(self.y, self.x)) 

	def servoMove(self):
		if self.legIndex == 0: #Front Leg
   		    self.kit.servo[0].angle = self.a_coxa
            self.kit.servo[1].angle = self.a_femur
            self.kit.servo[2].angle = self.a_tibia
		elif self.legIndex == 1: #Middle Leg
			self.kit.servo[3].angle = self.a_coxa
			self.kit.servo[4].angle = self.a_femur
			self.kit.servo[5].angle = self.a_tibia
		elif self.legIndex == 2: #Back Leg
			self.kit.servo[6] = 90 + self.a_coxa
			self.kit.servo[7] = 90 + self.a_femur
			self.kit.servo[8] = 90 + self.a_tibia

class robotarm(object):
	"""object to store position data and compute movements using inverse kinematics for a 4dof robot arm"""
	def ___init___(self, index, x_init, y_init, z_init):
		self.armindex = index
		#set x, y, and z coordinates
		self.x = x_init
		self.y = y_init
		self.z = z_init
		#arm dimensions in inches
		self.l_forearm = 0
		self.l_upperarm = 0
		self.l_shoulder = 0
		#arm angles in degrees
		self.a_shoulder = 0
		self.a_upperarm = 0
		self.a_forearm = 0
		self.a_wrist = 90
		self.a_gripper = 0

	def inverseKin(self):


	def servoMove(self):

    