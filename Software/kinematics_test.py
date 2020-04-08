import math

TIBIA_LENGTH = 193.68
FEMUR_LENGTH = 127.47
COXA_LENGTH = 83

def FKSolve(a_coxa, a_femur, a_tibia):
    #temporary angle for calculation
    temp_ang = a_tibia - (90 - (a_femur - 90))
    print(temp_ang)
    #Convert to radians
    a_coxa = math.radians(a_coxa)
    a_femur = math.radians(a_femur - 90)
    temp_ang = math.radians(temp_ang)
    x = (COXA_LENGTH + (FEMUR_LENGTH * math.cos(a_femur)) + (TIBIA_LENGTH *math.sin(temp_ang))) * math.cos(a_coxa)
    y = (COXA_LENGTH + (FEMUR_LENGTH * math.cos(a_femur)) + (TIBIA_LENGTH *math.sin(temp_ang))) * math.sin(a_coxa)
    z = (TIBIA_LENGTH * math.cos(temp_ang)) - (FEMUR_LENGTH * math.sin(a_femur))  
    return round(x), round(y), round(z)

def IKSolve(x, y, z):
    
    legLength = math.sqrt(x**2 + y**2)
    HF = math.sqrt((legLength - COXA_LENGTH)**2 + z**2)
    A1 = math.atan2(legLength - COXA_LENGTH, z)
    A2 = math.acos((TIBIA_LENGTH**2 - FEMUR_LENGTH**2 - HF**2)/(-2 * FEMUR_LENGTH * HF))
    B1 = math.acos((HF**2 - TIBIA_LENGTH**2 - FEMUR_LENGTH**2)/(-2 * FEMUR_LENGTH * TIBIA_LENGTH))
    C1 = math.atan2(y, x)
    
    #Convert to servo reference frame and solve for final angles
    a_coxa = math.degrees(C1)
    a_femur = math.degrees(A1 + A2) 
    a_tibia = math.degrees(B1)

    return a_coxa, a_femur, a_tibia

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