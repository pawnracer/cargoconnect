#!/usr/bin/env pybricks-micropython

"""
This program uses the basic driving functions to do missions. Add more functions as needed.
"""
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from base import baseMovement


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.




# Write your program here.
class Missions(object):
    def __init__(self, gyro, leftMotor, rightMotor, inverted):
        self.gyro = gyro
        self.leftMotor = leftMotor
        self.rightMotor = rightMotor
        self.inverted = inverted
        self.bm=baseMovement(gyro=self.gyro, leftMotor=self.leftMotor, rightMotor=self.rightMotor, inverted=self.inverted)
    def test(self):
        self.gyro.reset_angle(0)
        self.bm.gyroSmartTurn(leftDegPerSec=200, rightDegPerSec=-200, targetAngle=90, selfAdjust=1)
        wait(200)
        self.bm.gyroSmartTurn(leftDegPerSec=-200, rightDegPerSec=200, targetAngle=0, selfAdjust=1)
        wait(200)
        self.bm.gyroDriveMm(distance=400, speed=100)
        wait(200)
        self.bm.gyroDriveMm(distance=-400, speed=-100)
        wait(200)
        self.bm.gyroDriveMmEase(distance=1000, speed=200)
        wait(200)
        self.bm.gyroDriveMmEase(distance=1000, speed=-200)
