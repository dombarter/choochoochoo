# Dom Barter | 2019
# Move To XYA for v5

# NOTE: this version of moveTOXYA does not check for any objects when moving
# NOTE: all coordinates are measured in cm and all angles in degrees

# Steps for use:
# 1. Create new robot class, passing the created drivetrain as a parameter
# 2. Ensure drivetrain configuration is correct for wheel travel and track width
# 3. Call public robot functions

# Imports ---------------------------------------------------------------------

import sys
import vex
from vex import *
import math

# Robot Class -----------------------------------------------------------------

class Robot:

    x = 0 # x coordinate
    y = 0 # y coordinate
    angle = 0 # current angle of the robot

    # object instantiation
    def __init__(self,dt,turn,speed):

        self.drivetrain = dt #sets the drivetrain variable
        self.turnSpeed = turn
        self.movementSpeed = speed

    # move the robot to an x coord, y coord and angle of rotation
    def moveToXYA(self,x,y,angle = None): 

        self.currentX = self.x #grabs current x coordinate
        self.currentY = self.y #grabs current y coordinate

        self.deltaX = float(math.fabs(x - self.currentX)) #calulates distance between two x coordinates
        self.deltaY = float(math.fabs(y - self.currentY)) #calulates distance between two y coordinates

        if self.deltaX == 0 and self.deltaY != 0: #if the robot is only moving in the y axis
            if self.currentY > y: #if the movement is negative
                self.rotateTo(180) #rotate the robot to 180 degrees
            else: #if the movement is positive
                self.rotateTo(0) #rotate the robot to 0 degrees
        elif self.deltaX != 0 and self.deltaY == 0: #if the robot is only moving in the x axis
            if self.currentX > x: # if the movement is negative
                self.rotateTo(-90) #rotate the robot to -90 degrees
            else: #if the movmeent is postiive
                self.rotateTo(90) # rotate the robot to 90 degrees
        else: #if there is movmeent in the x and y axis'
            if x > self.currentX and y > self.currentY: #if the robot is moving into the first quadrant
                self.rotation = math.degrees(math.atan(self.deltaX / self.deltaY)) #calculate the amount of rotation relative to 0
                self.rotateTo(self.rotation) #rotate to this newly calculated angle
            elif x > self.currentX and y < self.currentY: #if the robot is moving into the second quadrant
                self.rotation = 180 - math.degrees(math.atan(self.deltaX / self.deltaY)) #calculate the amount of rotation relative to 0
                self.rotateTo(self.rotation) #rotate to this newly calculated angle
            elif x < self.currentX and y > self.currentY: #if the robot is moving into the fourth quadrant
                self.rotation = math.degrees(math.atan(self.deltaX / self.deltaY)) * -1 #calculate the amount of rotation relative to 0
                self.rotateTo(self.rotation) #rotate to this newly calculated angle
            elif x < self.currentX and y < self.currentY: #if the robot is moving into the third quadrant
                self.rotation = math.degrees(math.atan(self.deltaX / self.deltaY)) - 180 #calculate the amount of rotation relative to 0
                self.rotateTo(self.rotation) #rotate to this newly calculated angle

        self.distance = self.calculateDeltaD(x,y,self.currentX,self.currentY) # calculate the absolute value between the two coordinates
        self.motion = self.moveBy(self.distance) #move by the newly found distance, and specifiying the ignoreCone parmater with a variable

        if self.motion == False: #if the robot was unable to complete its journey
            return False #return that journey was unsuccessful
        else: #otherwsie,
            self.x = x #set new x coordinate
            self.y = y #set new y coordinate
            if angle != None: # if an end rotation angle was specified
                self.rotateTo(angle) #rotate to the specified angle
            return True #return that journey was successful

    # move the robot forwards by a certain distance
    def moveBy(self,distance): 

        self.currentAngle = self.angle #gets current gyro reading

        if distance >= 0:
             dt.start_drive_for(vex.DirectionType.FWD,distance,vex.DistanceUnits.CM,self.movementSpeed,vex.VelocityUnits.PCT)
        else:
            dt.start_drive_for(vex.DirectionType.REV,distance*-1,vex.DistanceUnits.CM,self.movementSpeed,vex.VelocityUnits.PCT)

        self.resolveResult = self.resolveXY(self.x,self.y,distance,self.currentAngle)
        self.x , self.y = self.resolveResult.x , self.resolveResult.y

        return True #robot has been able to reach destination

    # rotate the robot to a certain angle
    def rotateTo(self,degrees): 

        if degrees < -180 or degrees > 180:
            return False #invalid goal angle
        else:
            self.currentAngle = self.angle #grabs the current angle of the robot
            self.goalAngle = round(degrees) #sets the goal degrees to a new variable

            if self.currentAngle < 0 and self.goalAngle == 180: #couteract 180/-180 clash
                self.goalAngle = -180
            elif self.currentAngle > 0 and self.goalAngle == -180:
                self.goalAngle = 180
            elif self.currentAngle == 180 and self.goalAngle < 0:
                self.currentAngle = -180
            elif self.currentAngle == -180 and self.goalAngle > 0:
                self.currentAngle = 180

            self.deltaR = self.goalAngle - self.currentAngle #calculates how far the robot needs to rotate

            #changes the deltaR value to the desired value (avoids 180/-180 cutoff)
            if self.deltaR > 180:
                self.deltaR = (self.deltaR - 360)
            elif self.deltaR < -180:
                self.deltaR = (self.deltaR + 360)

            if self.deltaR >= 0: #turn the robot
                self.drivetrain.start_turn_for(vex.TurnType.RIGHT,self.deltaR,vex.RotationUnits.DEG,self.turnSpeed,vex.VelocityUnits.PCT)
            else:
                self.drivetrain.start_turn_for(vex.TurnType.LEFT,self.deltaR*-1,vex.RotationUnits.DEG,self.turnSpeed,vex.VelocityUnits.PCT)

            self.angle = self.goalAngle #set new angle

            return True

    # function to update distaplacement of the robot by calculating new coordinates
    def resolveXY(self,xCoord,yCoord,distance,rotation): 

        self.distance = distance

        self.radians = math.radians(rotation) #turns the gyro reading into radians

        self.gyro = rotation #keeps the gyro reading in degrees

        self.coordinates = XYCoordinates() #makes a new set of coordinates

        self.ninety = math.radians(90)

        if self.gyro == 0:
            self.coordinates.x = xCoord
            self.coordinates.y = yCoord + self.distance
        elif self.gyro == 180 or self.gyro == -180:
            self.coordinates.x = xCoord
            self.coordinates.y = yCoord - self.distance
        elif self.gyro == 90:
            self.coordinates.x = xCoord + self.distance
            self.coordinates.y = yCoord
        elif self.gyro == -90:
            self.coordinates.x = xCoord - self.distance
            self.coordinates.y = yCoord
        else:
            if self.gyro > 0 and self.gyro < 90: #first quadrant
                self.coordinates.x = xCoord + (math.sin(self.radians) * self.distance)
                self.coordinates.y = yCoord + (math.cos(self.radians) * self.distance)
            elif self.gyro > 0 and self.gyro > 90: #second quadrant
                self.coordinates.x = xCoord + (math.cos(self.radians - self.ninety) * self.distance)
                self.coordinates.y = yCoord - (math.sin(self.radians - self.ninety) * self.distance)
            elif self.gyro < 0 and self.gyro > -90: #fourth quadrant
                self.coordinates.x = xCoord - (math.sin(math.fabs(self.radians)) * self.distance)
                self.coordinates.y = yCoord + (math.cos(math.fabs(self.radians)) * self.distance)
            elif self.gyro < 0 and self.gyro < -90: #third quadrant
                self.coordinates.x = xCoord - (math.cos(math.fabs(self.radians + self.ninety)) * self.distance)
                self.coordinates.y = yCoord - (math.sin(math.fabs(self.radians + self.ninety)) * self.distance)

        self.coordinates.x = round(self.coordinates.x) #create new coordinates
        self.coordinates.y = round(self.coordinates.y)

        return self.coordinates #return values

    # function to return distance between two coordinates
    def calculateDeltaD(self,goalX,goalY,currentX,currentY): #function to return distance between two coordinates
        self.deltaX = math.fabs(currentX - goalX) #caclulates change in x axis
        self.deltaY = math.fabs(currentY - goalY) #caclulates change in y axis
        self.deltaD = math.fabs(math.sqrt((self.deltaX ** 2) + (self.deltaY ** 2))) # performs pythagoras on two values
        return self.deltaD #returns absolute value

# Coordinates Class -----------------------------------------------------------

class XYCoordinates:
    def __init__(self): #intiation function
        self.x = 0 #x coordinate
        self.y = 0 #y coordinate
        return None

# Robot Setup -----------------------------------------------------------------

wheelTravel = 300 # circumference of the wheel (mm)
trackWidth = 300 # width of the chassis (mm)
turnSpeed = 25 # how fast the robot will turn (%)
movementSpeed = 30 # how fast the robot will go forwards and back (%)

brain      = vex.Brain()
controller = vex.Controller(vex.ControllerType.PRIMARY)
leftMotor  = vex.Motor(vex.Ports.PORT14, vex.GearSetting.RATIO18_1, False) #example usage
rightMotor = vex.Motor(vex.Ports.PORT15, vex.GearSetting.RATIO18_1, True) #example usage
dt         = vex.Drivetrain(leftMotor, rightMotor, wheelTravel, trackWidth, vex.DistanceUnits.MM)

robot = Robot(dt,turnSpeed,movementSpeed) #create the robot class

# Main program ----------------------------------------------------------------

while True:

    # Do Something Awesome :-)

    # Example
    robot.moveToXYA(100,100,90) # x = 100, y = 100, a = 90
