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
    def __init__(self, gyro, leftMotor, rightMotor, medMotor1, inverted):
        self.gyro = gyro
        self.leftMotor = leftMotor
        self.rightMotor = rightMotor
        self.inverted = inverted
        self.m1=medMotor1
        self.b=Base(gyro=self.gyro, leftMotor=self.leftMotor, rightMotor=self.rightMotor, wheel_diameter=92.5, axle_track=115, inverted=self.inverted)
    def test(self):
        self.gyro.reset_angle(0)
        self.b.gyroSmartTurn(leftDegPerSec=100, rightDegPerSec=-100, targetAngle=90, selfAdjust=1)
        wait(300)
        self.b.gyroSmartTurn(leftDegPerSec=-100, rightDegPerSec=100, targetAngle=0, selfAdjust=1)
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
        x = -1085 #1150
        y = -50
        z = -200
       
        self.gyro.reset_angle(0)
       
        self.b.gyroDriveMm(distance=x, speed=200)
        """ turn right 90 degrees """
        
        self.b.gyroSmartTurn(leftDegPerSec=-100, rightDegPerSec=100, targetAngle=-45, selfAdjust=1)
        self.b.gyroDriveMm(distance=100, speed=100)
        #moving back
        self.b.gyroDriveMm(distance=-150, speed=-50)
        #right turn
        self.b.gyroSmartTurn(leftDegPerSec=100, rightDegPerSec=-100, targetAngle=75, selfAdjust=1)
        #drive
        self.b.gyroDriveMm(distance=280, speed=100)
        #left turn
        self.b.gyroSmartTurn(leftDegPerSec=-100, rightDegPerSec=100, targetAngle=-40, selfAdjust=1)
        #final turn
        self.b.gyroDriveMm(distance=200, speed=100)
        wait(1000)
        self.b.gyroDriveMm(distance=-200, speed=-100)
        #left turn
        self.b.gyroSmartTurn(leftDegPerSec=-100, rightDegPerSec=100, targetAngle=-120, selfAdjust=1)
        #drive
        self.b.gyroDriveMm(distance=275, speed=100)
        #left turn
        self.b.gyroSmartTurn(leftDegPerSec=-100, rightDegPerSec=100, targetAngle=-25, selfAdjust=1)
        #drive home
        self.b.gyroDriveMm(distance=x, speed=200)
    def turbine_blade(self):
        self.b.gyroDriveMm(distance=600, speed=200)
        self.b.gyroDriveMm(distance=210, speed=-200)
        self.b.gyroDriveMm(distance=260, speed=200)
        self.b.gyroDriveMm(distance=1000, speed=-500)
    def platooning(self):
        self.b.gyroDriveMm(373.25, 250)
        self.b.gyroTankTurn(100, -100, 87, selfAdjust = 500)
        self.b.gyroDriveMmEase(1000, 300)
        wait(5)
        self.b.gyroDriveMm(-200, -200)
        self.b.gyroTankTurn(-45, 45, -30)
        self.b.gyroDriveMm(780, 240)
        self.b.gyroTankTurn(50, -50, 20)
        self.gyro.reset_angle(0)
        self.b.gyroDriveMm(500, 150)
        self.b.gyroDriveMm(-300, -90)
        self.b.gyroTankTurn(90, -90, 45)
        self.gyro.reset_angle(0)
        self.b.gyroDriveMm(360, 175)
        self.b.gyroTankTurn(180, -180, 100)
        self.gyro.reset_angle(0)
        self.b.gyroTankTurn(-180, 180, -125)
        self.gyro.reset_angle(0)
        self.b.gyroDriveMm(500, 200)
        self.b.gyroDriveMm(150, 500)
        self.b.gyroTankTurn(45, -45, 5)
        self.gyro.reset_angle(0)
        self.b.gyroDriveMm(-200, -100)
        self.b.gyroTankTurn(-90, 90, -95)
        self.b.gyroDriveMm(350, 200)
        self.b.gyroTankTurn(90, -90, 90)
        self.gyro.reset_angle(0)
        self.b.gyroDriveMm(100, 100)
        self.b.gyroTankTurn(1000, -1000, 90)
        self.gyro.reset_angle(0)
    def Cargo_door_and_Mission06(self):
        self.gyro.reset_angle(0)
        self.b.gyroDriveMm(200,200)
        self.b.gyroDriveMm(650,300)    
        self.b.gyroTankTurn(200,-200,85)
        self.b.gyroDriveMm(300,300)
        self.b.gyroDriveMm(-100,-200)
        self.b.gyroTankTurn(-200,200,-70)
        self.b.gyroDriveMm(-150,-200)
        self.b.gyroTankTurn(-200,200,-42)   
        self.b.gyroDriveMm(100,100)
        self.b.gyroDriveMm_with_medium_motors(-130,-50,-100,300)
        self.medMotor1.run_time(100,1700)
        self.b.gyroDriveMm(50,100)
        self.b.gyroTankTurn(200,-200,50)
        self.b.gyroDriveMm(-300,-300)
        self.medMotor1.run_time(-100,1700)
        self.b.gyroTankTurn(-400,400,-120)
        self.medMotor1.run_time(100,1700)
        self.gyro.reset_angle(0)
        self.b.gyroTankTurn(200,-200,150)
        self.b.gyroDriveMm(250,200) 
        self.medMotor1.run_time(-100,2000)
        self.b.gyroTankTurn(-200,200,-90)
        self.medMotor1.run_time(100,1700)
        self.b.gyroDriveMm(200,100)
        self.b.gyroDriveMm(-400,-200)
        self.b.gyroTankTurn(200,-200,60)
        self.b.gyroDriveMm(300,200)
    def m_9_10(self):
        #going to the upper part of the board
        self.b.gyroSmartTurn(leftDegPerSec=-100, rightDegPerSec=100, targetAngle=-90, selfAdjust=1)
        self.b.gyroDriveMm(distance=410, speed=100)
        self.b.gyroSmartTurn(leftDegPerSec=100, rightDegPerSec=-100, targetAngle=90, selfAdjust=1)
        self.b.gyroDriveMm(distance=1500, speed=100)
        self.b.gyroSmartTurn(leftDegPerSec=100, rightDegPerSec=-100, targetAngle=90, selfAdjust=1)
        #picking up the containor
        self.m1.run(speed=-200)
        self.m1.run_until_stalled(-100, duty_limit=50)
        self.m1.reset_angle(0)
        self.b.gyroDriveMm(distance=400, speed=100)
        self.m1.run_target(300, 800)
        self.b.gyroDriveMm(distance=-400, speed=-100)
        self.m1.reset_angle(0)
        #going home
        self.b.gyroDriveMm(distance=200, speed=-50)
        self.b.gyroSmartTurn(leftDegPerSec=100, rightDegPerSec=-100, targetAngle=180, selfAdjust=1)
        self.b.gyroDriveMm(distance=100, speed=100)
        self.b.gyroSmartTurn(leftDegPerSec=-100, rightDegPerSec=100, targetAngle=-90, selfAdjust=1)
        self.b.gyroDriveMm(distance=1700, speed=100)
