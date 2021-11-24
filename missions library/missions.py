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
        wait(300)
        self.m1.run_target(speed=200, target_angle=200)
        wait(300)
        self.m1.run_target(speed=200, target_angle=0)
        wait(300)
        self.m2.run_target(speed=200, target_angle=200)
        wait(300)
        self.m2.run_target(speed=200, target_angle=0)
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
    def m_11(self):
        """ package delivery """

        """mission logic
            1. drive gyro straight x distance from home
            2. turn right by 90 degrees
            3. drive gyro straight y distance
            4. turn left 90 degrees
            5. drive gyro straight z distance to drop package
            5. drive backwards z distance
            6. turn left 90 degrees
            7. drive gyro straight y distance  
        """
        x = -400
        y = -50
        z = -30

        self.gyro.reset_angle(0)

        self.b.gyroDriveMm(distance=x, speed=100)
        self.b.gyroSmartTurn(leftDegPerSec=-100, rightDegPerSec=100, targetAngle=90, selfAdjust=1)
        self.b.gyroDriveMm(distance=y, speed=100)
        self.b.gyroSmartTurn(leftDegPerSec=-100, rightDegPerSec=100, targetAngle=0, selfAdjust=1)
        self.b.gyroDriveMm(distance=z, speed=100)
        self.b.gyroDriveMm(distance=-z, speed=100)
        self.b.gyroSmartTurn(leftDegPerSec=-100, rightDegPerSec=100, targetAngle=-90, selfAdjust=1)    
        self.b.gyroDriveMm(distance=-y, speed=100)
