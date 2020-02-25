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

import math

from steamcontroller import SteamController, SCButtons

from adafruit_servokit import ServoKit
from subsections import hexleg, body, robotarm
		
class crabBot(object):
	"""object to store callback functions for steam controller gamepad control"""
	#Dimensions of robot parts
	#Body Dimensions (in inches)

	#Leg Dimensions (in inches)
	Tibia_Length = 7.1
	Femur_Length = 5.2
	Coxa_Length = 3.4
	Init_Tibia_Ang = radians(20)
	Init_Femur_Ang = radians(130)
	Init_Coxa_Ang = radians(93)

	#Arm Dimensions (in inches)
	Shoulder_Length = 
	Upperarm_Length = 
	Lowerarm_Length = 
	Init_Shoulder_Ang = radians()
	Init_Upperarm_Ang = radians()
	Init_Lowerarm_Ang = radians()


	

	def __init__(self):
        #instantiate robot body
		self.body = body()

		#leg Parameters
		self.x_init_leg = (self.Coxa_Length + (self.Femur_Length * math.cos(self.Init_Femur_Ang)) + (self.Tibia_Length * math.sin(self.Init_Tibia_Ang))) * math.cos(self.Init_Coxa_Ang) 
		self.y_init_leg = (self.Coxa_Length + (self.Femur_Length * math.cos(self.Init_Femur_Ang)) + (self.Tibia_Length * math.sin(self.Init_Tibia_Ang))) * math.sin(self.Init_Coxa_Ang)  
		self.z_init_leg = (self.Femur_Length * math.sin(self.Init_Femur_Ang)) - (self.Tibia_Length * math.cos(self.Init_Tibia_Ang))

		#instantiate leg classes
		self.FR_leg = hexleg(0, self.x_init_leg, self.y_init_leg, self.z_init_leg)
		self.MR_leg = hexleg(1, self.x_init_leg, self.y_init_leg, self.z_init_leg)
		self.BR_leg = hexleg(2, self.x_init_leg, self.y_init_leg, self.z_init_leg)
		self.FL_leg = hexleg(3, self.x_init_leg, self.y_init_leg, self.z_init_leg, isInverted=True)
		self.ML_leg = hexleg(4, self.x_init_leg, self.y_init_leg, self.z_init_leg, isInverted=True)
		self.BL_leg = hexleg(5, self.x_init_leg, self.y_init_leg, self.z_init_leg, isInverted=True)

		#instantiate arm classes
		self.r_arm = robotarm(0, x_init_arm, y_init_arm, z_init_arm)
		self.l_arm = robotarm(1, x_init_arm, y_init_arm, z_init_arm)
		#pad deadzone
		self.dzone = 1


	def a_pressed(self, btn, pressed):
		#cycle between gaits
		if not pressed:
			self.body.gaitSelect() 
			
	def b_pressed(self, btn, pressed):

		pass

	def x_pressed(self, btn, pressed):
		#lower body incrementally
		pass

	def y_pressed(self, btn, pressed):
		#raise body incrementally
		pass

	def stick_axes(self, x, y):
		#move hexapod at the angle and magnitude of the stick
		mag = math.sqrt(x**2 + y**2)
		ang = math.atan2(y, x)
		self.walk(mag, ang)

	def stick_pressed(self, btn, pressed):
		pass

	def pad_axes(self, pad, dx, dy):
		#change position of arm end effector in XY plane
		#end effector angle does not change

		#scale dx and dy values to be the same order of magnitude as the limb length
		scaleFactor = 0.00018310546
		dx *= scaleFactor
		dy *= scaleFactor
		dx = int(round(dx))
		dy = int(round(dy))
		_moved = math.sqrt(dx**2 + dy**2)

		if _moved > self.dzone:
			if pad == RIGHT:
				self.r_arm.x += dx
				self.r_arm.y += dy
				self.r_arm.inverseKin()
				self.r_arm.servoMove()
			else: 
				self.l_arm.x += dx
				self.l_arm.y += dy
				self.l_arm.inverseKin()
				self.l_arm.servoMove()		

	def pad_pressed(self, pad, dTheta, dr):
		#rotate only about robot wrist
		pass

	def trig_axes(self, pos, value):
		#control grippers

		#scale raw values
		value *= 0.00196078431
		value = int(round(value))

		if pos == RIGHT:
			self.r_arm.a_wrist =
			self.r_arm.inverseKin()
			self.r_arm.servoMove()
		else:
			self.l_arm.a_wrist =
			self.l_arm.inverseKin()
			self.l_arm.servoMove()
		

	def bumper_pressed(self, btn, pressed):
		#move arm in positive z direction while pressed
		if btn == SCButtons.LB:
			while pressed:
				self.l_arm.z += 0.05
		else:
			while pressed:
				self.r_arm.z += 0.05

	def grip_pressed(self, btn, pressed):
		#move arm in negative z direction while pressed
		if btn == SCButtons.LGRIP:
			while pressed:
				self.l_arm.z -= 0.05
		else:
			while pressed:
				self.r_arm.z -= 0.05

	def start_pressed(self, btn, pressed):
		pass

	def back_pressed(self, btn, pressed):
		pass

	def steam_pressed(self, btn, pressed):
		pass

	def walk(self, magnitude, direction):
		"""function that controls the robots movement, magnitude correlates to the speed of movement and direction with the direction of the movement"""
		gait = self.body.gaitID
		gaits = self.body.gaits
		for step in gaits[gait]:
			for legMove in step:
				if legMove:

				else:

	