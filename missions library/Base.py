#!/usr/bin/env pybricks-micropython
"""
This program contains basic gyro driving function such as gyroTankTurn and gyroDriveCm.
"""
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
class Base(DriveBase):
    def __init__(self, gyro, leftMotor, rightMotor, wheel_diameter, axle_track, fm=fm, inverted):
        """
        IMPORTANT:
        if you motors are inverted, set inverted to True
        """
        self.gyro = gyro
        self.leftMotor = leftMotor
        self.rightMotor = rightMotor
        self.inverted = inverted
        self.medMotor1 = fm
        super().__init__(leftMotor, rightMotor, wheel_diameter, axle_track)
    def gyroDrive(self, PROPORTIONAL_GAIN, init_angle, distance, speed):
        direction = 1 if self.inverted else -1
        try: 
            angle_correction = direction * (abs(speed)/speed) * PROPORTIONAL_GAIN * self.gyro.angle()+ init_angle
            self.drive(speed*direction*-1, angle_correction)
        except:
            return
    def gyroTankTurn(self, leftDegPerSec, rightDegPerSec, targetAngle, stopType = Stop.COAST, selfAdjust = 5):
        """
        Drive each motor at a specific speed until a target angle has been met. In case of overshooting, we self adjust to what the angle should be.
        Parameters:
        leftDegPerSec(int): In degrees, how fast the left motor moves
        rightDegPerSec(int): In degrees, how fast the right motor moves       
        targetAngle(int): The angle that the robot will turn to
        stopType(string): Whether the robot will brake or it will coast
        selfAdjust(int): In degrees, if the overshoot is more than the selfAdjust angle, then we turn back
        
        Returns:
        None
    
        """
        direction = -1 if self.inverted else 1
        currAngle = self.gyro.angle()
        isTurnRight = currAngle < targetAngle
        self.leftMotor.run(leftDegPerSec*direction)
        self.rightMotor.run(rightDegPerSec*direction)
        while ((targetAngle > currAngle) and isTurnRight) or ((targetAngle < currAngle) and not isTurnRight):
            currAngle = self.gyro.angle()
        self.leftMotor.stop(stopType)
        self.rightMotor.stop(stopType)
        if selfAdjust == False: # skip selfAdjust because we sometimes don't need accuracy
            return

        # for coast, wait for motors to settle before checking for overshoot
        if(stopType == Stop.COAST):
            for i in range(0, 2):
                wait(100)
        currAngle = self.gyro.angle()
        angleDelta = currAngle - targetAngle   
        if abs(angleDelta) < selfAdjust:
            return
        # there is no under-turn, so there is no point in checking
        isOver = (isTurnRight and angleDelta > 0) or (not isTurnRight and angleDelta < 0)
        # self-adjust at a lower speed for accuracy
        if isOver: # only self-adjust once to avoid recursion (infinite loop)
            self.gyroTankTurn(leftDegPerSec * (-0.5), rightDegPerSec * (-0.5), targetAngle, Stop.BRAKE, False)
        else:
            return
    def gyroSmartTurn(self, leftDegPerSec, rightDegPerSec, targetAngle, stopType = Stop.BRAKE, selfAdjust = 5):
        """
        Same as gyroTankTurn, but near the end of its turn, the robot slows to avoid overshooting. This allows us to turn at a higher speed
        while still being accurate
        Parameters:
        leftDegPerSec(int): In degrees, how fast the left motor moves
        rightDegPerSec(int): In degrees, how fast the right motor moves       
        targetAngle(int): The angle that the robot will turn to
        stopType(string): Whether the robot will brake or it will coast
        selfAdjust(int): In degrees, if the overshoot is more than the selfAdjust angle, then we turn back
        
        Returns:
        None
    
        """
        midAngle = self.gyro.angle() + (targetAngle - self.gyro.angle()) * 0.80
        self.gyroTankTurn(leftDegPerSec, rightDegPerSec, midAngle, Stop.COAST, False)
        self.gyroTankTurn(leftDegPerSec/2, rightDegPerSec/2, targetAngle, stopType, selfAdjust)
    def gyroDriveMm(self, distance, speed):
        """
        Drives toward a direction for a specific distance, in millimeters, using a gyro.
        Parameters:
        distance(float): In mm, how far we drive for
        speed(int): Speed in mm/sec
        Returns:
        None
        """
        
        self.reset()
        self.gyro.reset_angle(0)
        angle_correction = -1 * (0.011*speed) * self.gyro.angle()
        PROPORTIONAL_GAIN = 0.011*speed
        while abs(self.distance())<abs(distance):
            self.gyroDrive(PROPORTIONAL_GAIN, 0, distance, speed)
        self.stop()
    def gyroDriveMmEase(self, distance, speed):
        """
        Drives toward a direction for a specific distance, in millimeters while easing, using a gyro.
        Parameters:
        distance(float): In mm, how far we drive for
        speed(int): Speed in mm/sec
        Returns:
        None
        """
        self.reset()
        self.gyro.reset_angle(0)
        PROPORTIONAL_GAIN = 0.011*speed
        rate=speed/(distance*0.2)
        for i in range(0, speed, round(rate)):
            PROPORTIONAL_GAIN = i*0.011
            self.gyroDrive(PROPORTIONAL_GAIN, 0, distance*0.2, i)
            wait(5)
        self.gyroDrive(PROPORTIONAL_GAIN, 0, distance*0.6, speed)
        for i in reversed(range(0, speed, round(rate))):
            PROPORTIONAL_GAIN = i*0.011
            self.gyroDrive(PROPORTIONAL_GAIN, 0, distance*0.2, i)
            wait(5)
        self.stop()
    def gyroDriveMm_with_medium_motors(self, distance, speed,speedofmotor, time):
        """
        Drives toward a direction for a specific distance, in millimeters, using a gyro.
        Parameters:
        distance(float): In mm, how far we drive for
        speed(int): Speed in mm/sec
        Returns:
        None
        """
        
        self.reset()
        self.gyro.reset_angle(0)
        angle_correction = -1 * (0.011*speed) * self.gyro.angle()
        PROPORTIONAL_GAIN = 0.011*speed
        while abs(self.distance())<abs(distance):
            self.gyroDrive(PROPORTIONAL_GAIN, 0, distance, speed)
            self.medMotor1.run_time(speedofmotor, time)
        self.stop()
