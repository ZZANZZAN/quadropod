import numpy as np
import matplotlib.pyplot as plt
from numpy.lib.function_base import _parse_input_dimensions
import seaborn
import bezier 
import math 

class Robot: 
    def __init__(self):
        self.MAXX = 23.
        self.MAXY = 30.
        self.bone1 = 11.75
        self.bone2 = 12
        self.bone3 = 7.75
        self.bone4 = 10.50
        self.shouider = 3.25
        self.bodyLength = 27.50
        self.bodyWidth = 14.
        self.t = 10

    def getIntersect(self, curve, array):
        arrayPoint = []
        for i in range(0, self.t+1):
            nodes = np.asfortranarray([
                [array[0]+array[1]*i,array[0]+array[1]*i],
                [0,self.MAXY],
            ])
            curve1 = bezier.Curve.from_nodes(nodes)
            intersections = curve.intersect(curve1)
            s_vals = intersections[0, :]
            points = curve.evaluate_multi(s_vals)
            arrayPoint.append([points[0][0],points[1][0]])
        return arrayPoint    
    #траектория движения ноги
    def trajectotyMove(self, bezierArrayP):
        nodes1 = np.asfortranarray(bezierArrayP)
        curve1 = bezier.Curve.from_nodes(nodes1)
        lenPP = math.sqrt((bezierArrayP[0][0] - bezierArrayP[0][1])**2)
        stepPoint = lenPP/self.t
        arrayPoint = self.getIntersect(curve1, [bezierArrayP[0][0], stepPoint])
        return arrayPoint
    
    def distanceBetweenPoints(self, Point1, Point2):
        return math.sqrt((Point2[0]-Point1[0])*(Point2[0]-Point1[0])+(Point2[1]-Point1[1])*(Point2[1]-Point1[1]))

    def coordinatesMidpoint(self, Point1, Point2):
        return [(Point1[0]+Point2[0])/2,(Point1[1]+Point2[1])/2]

    def angleBetweenVectors(self, Point1, Point2, Shift):
        Point1[0] += Shift[0]
        Point1[1] += Shift[1]
        num1 = Point1[0]*Point2[0]+Point1[1]*Point2[1]
        den1 = math.sqrt(Point1[0]*Point1[0]+Point1[1]*Point1[1])
        den2 = math.sqrt(Point2[0]*Point2[0]+Point2[1]*Point2[1])
        angle = math.acos(num1/den1/den2)
        return math.degrees(angle)

    def sidesTriangleCorners(self, a, b, c):
        alpha = math.acos((b**2+c**2-a**2)/(2*b*c))
        beta = math.acos((a**2+c**2-b**2)/(2*a*c))
        gamma = math.acos((b**2+a**2-c**2)/(2*b*a))
        return math.degrees(alpha), math.degrees(beta), math.degrees(gamma)
    #нахождение углов ноги он начала координат до точки
    def detAngle(self, coordinatPoint):
        angle1 = self.angleBetweenVectors(coordinatPoint,[0,self.MAXY],[0,0])
        a = self.distanceBetweenPoints([0,0],[coordinatPoint[0],coordinatPoint[1]])
        alpha, beta, gamma = self.sidesTriangleCorners(a,self.bone1,self.bone2)
        PointKnee = [0+math.cos(math.degrees(angle1))*self.bone2, 0+math.sin(math.degrees(angle1))*self.bone2]
        Midpoint = self.coordinatesMidpoint(PointKnee, coordinatPoint)
        angle2 = self.angleBetweenVectors(Midpoint,[0,self.MAXY],[2,0])
        a1 = self.distanceBetweenPoints([2,0],Midpoint)
        alpha1, beta1, gamma1 = self.sidesTriangleCorners(a1,self.bone3,self.bone4)
        return angle1, angle2 

    def theoryAngleToServo(alpha, beta):
        return alpha+45, beta-45