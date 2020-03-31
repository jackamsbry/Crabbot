#! /usr/bin/python3

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
from adafruit_servokit import ServoKit
import time, math
import numpy as np
from scipy.interpolate import CubicSpline

MAXSERVOSPEED = 0


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
    #Leg Section lengths in mm
    TIBIA_LENGTH = 193.68
    FEMUR_LENGTH = 127.47
    COXA_LENGTH = 83

    def __init__(self, legIndex, ServoKit, isInverted=False):
        self.legIndex = legIndex
        #Set initial values for robot startup
        self.a_coxa = 90
        self.a_femur = 0
        self.a_tibia = 90
        self.isInverted = isInverted
        
        if not isInverted:
            self.servo_index = [self.legIndex*3, self.legIndex*3 + 1, self.legIndex*3 + 2]
        else:
            self.servo_index = [(self.legIndex-3)*3, (self.legIndex-3)*3 + 1, (self.legIndex-3)*3 + 2]

        self.coxa_servo = ServoKit.servo[self.servo_index[0]]
        self.femur_servo = ServoKit.servo[self.servo_index[1]]
        self.tibia_servo = ServoKit.servo[self.servo_index[2]]

        #Initialize variables for inverse kinematics
        self.x = 0
        self.y = 0
        self.z = 0

        self.x_new = 0
        self.y_new = 0
        self.z_new = 0

        self.run_coxa_angle(self.a_coxa)
        self.run_femur_angle(self.a_femur)
        self.run_tibia_angle(self.a_tibia)

    def inverse_kin(self, x, y, z):
        #equations taken from blog post by user downeym here: https://www.robotshop.com/community/forum/t/inverse-kinematic-equations-for-lynxmotion-3dof-legs/21336
        legLength = math.sqrt((x**2) + (y**2))
        HF = math.sqrt((legLength - COXA_LENGTH)**2 + (self.z**2))
        A1 = math.degrees(math.atan2(legLength - COXA_LENGTH, self.z))
        A2 = math.degrees(math.acos((TIBIA_LENGTH**2 - FEMUR_LENGTH**2 - HF**2)/(-2* FEMUR_LENGTH * HF)))
        B1 = math.degrees(math.acos((HF**2 - TIBIA_LENGTH**2 - FEMUR_LENGTH**2)/(-2* FEMUR_LENGTH * TIBIA_LENGTH)))

        a_tibia = 90 - B1 + 90
        a_femur = 90 - (A1 + A2) + 90
        a_coxa = math.degrees(math.atan2(y, x))
        if self.isInverted:
            a_tibia = 180 - a_tibia
            a_femur = 180 - a_femur
            a_coxa = 180 - a_coxa

        return a_tibia, a_femur, a_coxa

    def run_coxa_angle(self, move_angle):
        """Runs to a target angle for a given servo"""
        if self.isInverted:
            new_angle = 180 - move_angle
        else:
            new_angle = move_angle

        self.coxa_servo.angle = new_angle

    def run_femur_angle(self, move_angle):
        """Runs to a target angle for a given servo"""
        if self.isInverted:
            new_angle = 180 - move_angle
        else:
            new_angle = move_angle

        self.coxa_servo.angle = new_angle

    def run_tibia_angle(self, move_angle):
        """Runs to a target angle for a given servo"""
        if self.isInverted:
            new_angle = 180 - move_angle
        else:
            new_angle = move_angle

        self.tibia_servo.angle = new_angle


    def interpolate_time(self, speed, numSteps):
        """interpolates between two delay times for smooth movement using cubic spline intrpolation"""
        time_steps = [0, numSteps/2, numSteps]
        delay_time = [0.005*speed, 0.01*speed, 0.005*speed]
        delayInterpolation = CubicSpline(time_steps, delay_time)
        delay_time = []
        for n in range(numSteps):
            delay_time.append(delayInterpolation(n))
        return delay_time
