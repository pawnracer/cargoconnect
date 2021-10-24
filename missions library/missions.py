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
from Base import Base


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.




# Write your program here.
class Missions:
    def __init__(self, gyro, leftMotor, rightMotor, medMotor1, medMotor2, inverted):
        self.gyro = gyro
        self.leftMotor = leftMotor
        self.rightMotor = rightMotor
        self.inverted = inverted
        self.m1=medMotor1
        self.m2=medMotor2
        self.b=Base(gyro=self.gyro, leftMotor=self.leftMotor, rightMotor=self.rightMotor, wheel_diameter=92.5, axle_track=115, inverted=self.inverted)
    def test(self):
        self.gyro.reset_angle(0)
        self.b.gyroSmartTurn(leftDegPerSec=200, rightDegPerSec=-200, targetAngle=90, selfAdjust=1)
        wait(300)
        self.b.gyroSmartTurn(leftDegPerSec=-200, rightDegPerSec=200, targetAngle=0, selfAdjust=1)
        wait(300)
        self.b.gyroDriveMm(distance=400, speed=100)
        wait(300)
        self.b.gyroDriveMm(distance=-400, speed=-100)
        wait(300)
        self.b.gyroDriveMmEase(distance=1000, speed=200)
        wait(300)
        self.b.gyroDriveMmEase(distance=1000, speed=-200)
    def parking(self):
        self.gyro.reset_angle(0)
        self.b.gyroTankTurn(200,-200, 55)
        distance=900
        speed=500
        self.b.gyroDriveMm(distance, speed)
        self.b.gyroTankTurn(-200,200, -55)
        distance=330
        speed=200
        self.b.gyroDriveMm(distance, speed)
        self.b.gyroTankTurn(-200,200, -90)
        distance=200
        speed=50
        self.b.gyroDriveMm(distance, speed)
        distance=-40
        speed=10
