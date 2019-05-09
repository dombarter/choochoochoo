# Imports ---------------------------------------------------------------------

import sys
import vex
from vex import *
import math

# Robot Setup -----------------------------------------------------------------

controller  = vex.Controller(vex.ControllerType.PRIMARY)

leftOne = vex.Motor(vex.Ports.PORT1 , vex.GearSetting.RATIO18_1, False)
leftTwo = vex.Motor(vex.Ports.PORT2 , vex.GearSetting.RATIO18_1, True)
rightOne = vex.Motor(vex.Ports.PORT3 , vex.GearSetting.RATIO18_1, True)
rightTwo = vex.Motor(vex.Ports.PORT4 , vex.GearSetting.RATIO18_1, False)

steering = vex.Motor(vex.Ports.PORT10 , vex.GearSetting.RATIO18_1, False)

# Main ------------------------------------------------------------------------

#moves the robot forwards
def moveForwards(time,power):
    #time measured in milliseconds, power measured in 0-100 percentage
    newPower = math.fabs(power)
    leftOne.spin(vex.DirectionType.FWD,newPower,vex.VelocityUnits.PCT)
    leftTwo.spin(vex.DirectionType.FWD,newPower,vex.VelocityUnits.PCT)
    rightOne.spin(vex.DirectionType.FWD,newPower,vex.VelocityUnits.PCT)
    rightTwo.spin(vex.DirectionType.FWD,newPower,vex.VelocityUnits.PCT)
    sys.sleep(time)

#moves the robot forwards
def moveBackwards(time,power):
    #time measured in milliseconds, power measured in 0-100 percentage
    newPower = math.fabs(power)
    leftOne.spin(vex.DirectionType.REV,newPower,vex.VelocityUnits.PCT)
    leftTwo.spin(vex.DirectionType.REV,newPower,vex.VelocityUnits.PCT)
    rightOne.spin(vex.DirectionType.REV,newPower,vex.VelocityUnits.PCT)
    rightTwo.spin(vex.DirectionType.REV,newPower,vex.VelocityUnits.PCT)
    sys.sleep(time)

def haltAll():
    leftOne.stop(vex.BrakeType.BRAKE)
    leftTwo.stop(vex.BrakeType.BRAKE)
    rightOne.stop(vex.BrakeType.BRAKE)
    rightTwo.stop(vex.BrakeType.BRAKE)
    #steering.stop(vex.BrakeType.HOLD)

def moveLeft(value):
    # steering.spin(vex.DirectionType.FWD,5,vex.VelocityUnits.PCT)
    steering.rotate_to(-0.75*value,vex.RotationUnits.DEG,5,vex.VelocityUnits.PCT,False)

def moveRight(value):
    # steering.spin(vex.DirectionType.REV,5,vex.VelocityUnits.PCT)
    steering.rotate_to(-0.75*value,vex.RotationUnits.DEG,5,vex.VelocityUnits.PCT,False)

while True:

    moveLeft(controller.axis4.value())
    if controller.axis2.value() > 2:
        moveLeft(controller.axis4.value())
        if controller.axis4.value() < -2:
            moveForwards(0,controller.axis2.value())
            moveLeft(controller.axis4.value())
        elif controller.buttonRight.pressing():
            moveForwards(0,controller.axis2.value()) 
            moveRight(controller.axis4.value())
        else:
            moveForwards(0,controller.axis2.value())

    elif controller.axis2.value() < -2:
        moveLeft(controller.axis4.value())
        if controller.axis4.value() < -2:
            moveBackwards(0,controller.axis2.value())
            moveLeft(controller.axis4.value())
        elif controller.axis4.value() > 2:
            moveBackwards(0,controller.axis2.value()) 
            moveRight(controller.axis4.value())
        else:
            moveBackwards(0,controller.axis2.value())

    elif controller.axis4.value() < -2:
            moveLeft(controller.axis4.value())
    elif controller.axis4.value() > 2:
            moveRight(controller.axis4.value())
    else:
        moveLeft(controller.axis4.value())
        haltAll()


