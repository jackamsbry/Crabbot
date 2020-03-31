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
    def __init__(self, legIndex, a_coxa, a_femur, a_tibia):
        self.legIndex = legIndex
        
        self.a_coxa = 90
        self.a_femur = 0
        self.a_tibia = 90
        self.angles = [self.a_coxa, self.a_femur, self.a_tibia]
        self.x = 0
        self.y = 0
        self.z = 0
        self.forwardKin(a_coxa, a_femur, a_tibia)

        #Parameters for kinematic model
        #length of leg limbs in inches
        self.l_tibia = 7.1
        self.l_femur = 5.2
        self.l_coxa = 3.4

        if self.legIndex <= 2:
            self.isInverted = False
            servo_address = 0x60
            self.servos = [legIndex * 3, legIndex*3 + 1, legIndex*3 + 2]
        else:
            self.isInverted = True
            servo_address = 0x40
            self.servos = [(legIndex - 3)*3, (legIndex - 3)*3 + 1, (legIndex - 3)*3 + 2]
        if self.isInverted:
            for ang in range(len(self.angles)):
                self.angles[ang] = 180 - self.angles[ang]
        else:
            pass

        self.kit = ServoKit(channels=16, address=servo_address) 

    def forwardKin(self, a_coxa, a_femur, a_tibia):
        self.x = 0
        self.y = 0
        self.z = 0


    def inverse_kin(self, x, y, z):
        #equations taken from blog post by user downeym here: https://www.robotshop.com/community/forum/t/inverse-kinematic-equations-for-lynxmotion-3dof-legs/21336
        legLength = math.sqrt((x**2) + (y**2))
        HF = math.sqrt((legLength - self.l_coxa)**2 + (self.z**2))
        A1 = math.degrees(math.atan2(legLength-self.l_coxa, self.z))
        A2 = math.degrees(math.acos((self.l_tibia**2 - self.l_femur**2 - HF**2)/(-2* self.l_femur* HF)))
        B1 = math.degrees(math.acos((HF**2 - self.l_tibia**2 - self.l_femur**2)/(-2* self.l_femur* self.l_tibia)))

        a_tibia = 90 - B1 + 90
        a_femur = 90 - (A1 + A2) + 90
        a_coxa = math.degrees(math.atan2(y, x))
        if self.isInverted:
            a_tibia = 180 - a_tibia
            a_femur = 180 - a_femur
            a_coxa = 180 - a_coxa

        return a_tibia, a_femur, a_coxa

    def run_angle(self, move_angle, servo_index):
        """Runs to a target angle for a given servo"""
        if self.isInverted:
            new_angle = 180 - move_angle
        else:
            new_angle = move_angle
        current_angle = self.angles[servo_index]
        run_servo = self.servos[servo_index]
        print("Running Angle!")
        for ang in np.linspace(current_angle, new_angle, 100):
            ang = float(ang)
            self.kit.servo[run_servo].angle = ang
            time.sleep(0.001)
        self.servos[servo_index] = new_angle

    def interpolate_move(self, move_position, numSteps):
        """interpolates between servos for smooth movement using cubic spline intrpolation"""
        current_a_coxa = self.a_coxa
        current_a_femur = self.a_femur
        current_a_tibia = self.a_tibia
        new_x = move_position[0]
        new_y = move_position[1] 
        new_z = move_position[2]
        a_tibia, a_femur, a_coxa = self.inverse_kin(new_x, new_y, new_z)
        a_coxa_move = np.linspace(current_a_coxa, a_coxa, numSteps)
        a_femur_move = np.linspace(current_a_femur, a_femur, numSteps)
        a_tibia_move = np.linspace(current_a_tibia, a_tibia, numSteps)
        angle_moves = [a_coxa_move, a_femur_move, a_tibia_move]
        time_steps = [0, numSteps/2, numSteps]
        delay_time = [0.005, 0.0001, 0.005]
        delayInterpolation = CubicSpline(time_steps, delay_time)
        delay_time = []
        for n in range(numSteps):
            delay_time.append(delayInterpolation(n))
        return delay_time, angle_moves

    def run_position(self, move_position):
        
        coxa = self.servos[0]
        femur = self.servos[1]
        tibia = self.servos[2]
        delay_times, angle_moves = self.interpolate_move(move_position, 200)
        coxa_angles = angle_moves[0]
        femur_angles = angle_moves[1]
        tibia_angles = angle_moves[2]
        for step in range(len(delay_times)):
            self.kit.servo[coxa].angle = coxa_angles[step]
            self.kit.servo[femur].angle = femur_angles[step]
            self.kit.servo[tibia].angle = tibia_angles[step]
            time.sleep(delay_times[step])