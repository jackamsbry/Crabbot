#! /usr/bin/python3

from adafruit_servokit import ServoKit
import time, math
import numpy as np
from scipy.interpolate import CubicSpline
from subsections import hexleg, body


class bezier2d():
    def __init__(self):
        self.xpoints = []
        self.ypoints = []

    def addPoint(self, x, y):
        self.xpoints.append(x)
        self.ypoints.append(y)

    def editPoint(self, x, y, point):
        self.xpoints[point] = x
        self.ypoints[point] = y

    def getPos(self, t):
        x = self.xpoints
        y = self.ypoints

        numPoints = len(self.xpoints)

        for i in range(numPoints-1):
            for j in range(numPoints-i-1):
                x[j] = (1-t)*x[j] + t*x[j+1]
                y[j] = (1-t)*y[j] + t*y[j+1]
        position = [x[0], y[0]]
        return position

class crabbot():
    MINTURNRAD = 609.6/2

    def __init__(self, legs, body):
        self.legs = legs
        self.body = body

        self.moveCurve = bezier2d()
        self.pushCurve = bezier2d()
        self.wakeCurve = bezier2d()

        self.curve_init()

        self.turnRad = 100000 #Large number so effectively straight motion for default direction

        #State variables
        self.isMoving = False
    
    def interpolate_time(self, speed, numSteps):
            """interpolates between two delay times for smooth movement using cubic spline intrpolation"""
            time_steps = [0, numSteps/2, numSteps]
            delay_time = [0.005*speed, 0.1*speed, 0.005*speed]
            delayInterpolation = CubicSpline(time_steps, delay_time)
            delay_time = []
            for n in range(numSteps):
                delay_time.append(delayInterpolation(n))
            return delay_time

    def interpolate_step(self, speed, numPoints=25):
        """Interpolates between motors for smooth movement for a single gait step"""
        gait = self.body.gaitCurrent
        numSteps = len(gait)
        stepCurrent = gait[self.body.gaitStep]

        delayTime = self.interpolate_time(speed, numPoints)

        move_t = np.linspace(0, 1, num=numPoints)
        t = np.linspace(0,(1/(numSteps-1)), num=numPoints)
        
        move_pos = []
        push_pos = []

        move_pos = self.moveCurve.getPos(x for x in move_t)
        push_pos = self.pushCurve.getPos(x for x in t)

        self.isMoving = True
        
        for i in range(numPoints):
            for index, leg in enumerate(self.legs):
                if stepCurrent[index] == 1:
                    current_pos = move_pos[i]

                elif stepCurrent[index] == 0:
                    current_pos = push_pos[i]

                
                leg.x_new = leg.x
                leg.y_new = current_pos[0]
                leg.z_new = current_pos[1]

                #Body inverse kinematics
                dLegs = self.body.IKSolve()
                dLeg = dLegs[index]

                #Update leg position with body transformations
                leg.x_new += dLeg[0]
                leg.y_new += dLeg[1]
                leg.z_new += dLeg[2]
                
                #Inverse kinematic calculation for leg position
                a_coxa, a_femur, a_tibia = leg.IKSolve(leg.x_new, leg.y_new, leg.z_new)
                
                #Run the servos to calculated angles
                leg.run_angle(a_coxa, 0)
                time.sleep(delayTime[i])
                leg.run_angle(a_femur, 1)
                time.sleep(delayTime[i])
                leg.run_angle(a_tibia, 2)   
                time.sleep(delayTime[i])
        
        self.isMoving = False

    def interpolate_angle(self, speed, numSteps, joint, angle):
        delayTime = self.interpolate_time(speed, numSteps)

        for leg in self.legs:   
            if joint == 0:
                leg.a_coxa_new = angle
                angles = np.linspace(leg.a_coxa, leg.a_coxa_new, numSteps)
                leg.a_coxa = leg.a_coxa_new
            elif joint == 1:
                leg.a_femur_new = angle
                angles = np.linspace(leg.a_femur, leg.a_femur_new, numSteps)
                leg.a_femur = leg.a_femur_new
            else:
                leg.a_tibia_new = angle
                angles = np.linspace(leg.a_tibia, leg.a_tibia_new, numSteps)
                leg.a_tibia = leg.a_tibia_new

        for i in range(numSteps):
            for leg in legs:
                if joint == 0:
                    leg.run_coxa_angle(angles[i])
                elif joint == 1:
                    leg.run_femur_angle(angles[i])
                else:
                    leg.run_tibia_angle(angles[i])
                time.sleep(delayTime[i])

    def curve_init(self):
        x0 = 25
        x1 = 2.5 * x0
        y = 150
        self.moveCurve.addPoint(x0, 0)
        self.moveCurve.addPoint(x1, 0)
        self.moveCurve.addPoint(0, y)
        self.moveCurve.addPoint(-x1, 0)
        self.moveCurve.addPoint(-x0, 0)

        self.pushCurve.addPoint(-x0, 0)
        self.pushCurve.addPoint(x0, 0)

    def stand_angle(self):
        numPoints = 25
        startAngle = 120

        delayTime = self.interpolate_time(10, numPoints)

        self.isMoving = True

        for index, leg in enumerate(self.legs):
            leg.a_coxa_new = startAngle - index * 30
            leg.a_femur_new = 150
            leg.a_tibia_new = 135
            leg.x_new, leg.y_new, leg.z_new = leg.FKSolve(leg.a_coxa_new, leg.a_femur_new, leg.a_tibia_new)
            xMove = np.linspace(leg.x, leg.x_new, num=numPoints)
            yMove = np.linspace(leg.y, leg.y_new, num=numPoints)
            zMove = np.linspace(leg.z, leg.z_new, num=numPoints)
            for i in range(numPoints):
                #Body inverse kinematics
                dLegs = self.body.IKSolve()
                dLeg = dLegs[index]

                #Update leg position with body transformations
                x = xMove[i] + dLeg[0]
                y = yMove[i] + dLeg[1]
                z = zMove[i] + dLeg[2]

                #Inverse kinematic calculation for leg position
                a_coxa, a_femur, a_tibia = leg.IKSolve(x, y, z)

                #Run the servos to calculated angles
                leg.run_angle(a_coxa, 0)
                leg.run_angle(a_femur, 1)
                leg.run_angle(a_tibia, 2)   
                time.sleep(delayTime[i])
            
        self.isMoving = False
    
    def stand(self):
        pass

    def sit(self, numPoints = 25):
        #Calculate leg end positions
        pass
        


if __name__ == "__main__":
    """Main function for testing servo movement and gaits"""
    #Instantiate servo class for each PCA9685 board
    left_servos = ServoKit(channels=16, address=0x60)
    right_servos = ServoKit(channels=16, address=0x40)

    #Instantiate legs
    lf = hexleg(0, left_servos, isInverted=True)
    lm = hexleg(1, left_servos, isInverted=True)
    lb = hexleg(2, left_servos, isInverted=True)

    rf = hexleg(3, right_servos)
    rm = hexleg(4, right_servos)
    rb = hexleg(5, right_servos)

    legs = [lf, lm, lb, rf, rm, rb]
    
    #Instantiate body
    body = body()

    #Instantiate robot control class
    bot = crabbot(legs, body)
    #Wait for 1.5 seconds so servos are in correct location
    time.sleep(1.5)
    #Set initial leg positions
    bot.stand()
    time.sleep(1)
    print("Done!")
    
        