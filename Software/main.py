#! /usr/bin/python3



from adafruit_servokit import ServoKit
import time, math
import numpy as np
from scipy.interpolate import CubicSpline

class bezier2d():
    def __init__(self):
        self.xpoints = []
        self.ypoints = []

    def addPoint(self, x, y):
        self.xpoints.append(x)
        self.ypoints.append(y)

    def getPos(self, t):
        x = self.xpoints
        y = self.ypoints

        numPoints = length(self.xpoints)

        for i in range(numPoints-1):
            for j in range(numPoints-i-1):
                x[j] = (1-t)*x[j] + t*x[j+1]
                y[j] = (1-t)*y[j] + t*y[j+1]
        position = [x[0], y[0]]
        return position

def interpolate_legs(legs, numSteps=100):
    """Pass in array of leg objects and interpolate movement"""




def main()
    """Main function for testing servo movement and gaits"""
    #Instantiate legs
    lf = hexleg()
    lm = hexleg()
    lb = hexleg()

    rf = hexleg()
    rm = hexleg()
    rb = hexleg()

    legs = [lf, lm, lb, rf, rm, rb]
    #Initialize move parameters and state variables
    stepNum = 0
    isMoving = False
    
    #Move legs to initial positions before wake-up
    







if __name__ == "__main__":
    main()