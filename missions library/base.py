#!/usr/bin/env pybricks-micropython

"""
This program contains basic gyro driving function such as gyroTankTurn and gyroDriveCm..
"""
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile





class baseMovement(object):
    def __init__(self, gyro, leftMotor, rightMotor, inverted):
        """
        IMPORTANT:
        if you motors are inverted, set inverted to True
        """
        self.gyro = gyro
        self.leftMotor = leftMotor
        self.rightMotor = rightMotor
        self.inverted = inverted

        

    def gyroTankTurn(self, leftDegPerSec, rightDegPerSec, targetAngle, stopType = Stop.COAST, selfAdjust = 5):
        """
        Drive each motor at a specific speed until a target angle has been met. In case of overshooting, we self adjust to what the angle should be.
        Parameters:
        leftDegPerSec(int): In degrees, how fast the left motor moves
        rightDegPerSec(int): In degrees, how fast the right motor moves       
        targetAngle(int): The angle that the robot will turn to
        stopType(string): Whether the robot will break or it will coast
        selfAdjust(int): In degrees, if the overshoot is more than the selfAdjust angle, then we turn back
        
        Returns:
        None
    
        """
        
        currAngle = self.gyro.angle()
        isTurnRight = currAngle < targetAngle
        if self.inverted==False:
            self.leftMotor.run(leftDegPerSec)
            self.rightMotor.run(rightDegPerSec)
        elif self.inverted==True:
            self.leftMotor.run(leftDegPerSec*-1)
            self.rightMotor.run(rightDegPerSec*-1)
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
        stopType(string): Whether the robot will break or it will coast
        selfAdjust(int): In degrees, if the overshoot is more than the selfAdjust angle, then we turn back
        
        Returns:
        None
    
        """
        midAngle = self.gyro.angle() + (targetAngle - self.gyro.angle()) * 0.80
        self.gyroTankTurn(leftDegPerSec, rightDegPerSec, midAngle, Stop.COAST, False)
        self.gyroTankTurn(leftDegPerSec/2, rightDegPerSec/2, targetAngle, stopType, selfAdjust)


    def gyroDriveCm(self, distance, speed, ease, startSpeed=0):
        """
        Drives toward a direction for a specific distance, in centimeters, using a gyro.

        Parameters:
        distance(float): In cm, how far we drive for
        speed(int): Speed in mm/sec
        ease(bool): Whether to ease or not
        startSpeet(int): The speed to start at if you are easing.

        Returns:
        None
        """
        self.robot = DriveBase(self.leftMotor, self.rightMotor, wheel_diameter=92.5, axle_track=115)
        self.robot.reset()
        self.gyro.reset_angle(0)
        PROPORTIONAL_GAIN = 0.011*speed
        deltaspeed=speed-startSpeed
        rate=deltaspeed/(distance*0.2)
        if ease == True:
            for i in range(startSpeed, speed, round(rate)):
                PROPORTIONAL_GAIN = i*0.011
                if self.inverted==False:
                    angle_correction = -1 * PROPORTIONAL_GAIN * self.gyro.angle()
                    self.robot.drive(i, angle_correction)
                elif self.inverted==True:
                    angle_correction = PROPORTIONAL_GAIN * self.gyro.angle()
                    self.robot.drive(i*-1, angle_correction)
                wait(5)
        if self.inverted==False:
            while self.robot.distance() < distance*0.6:
                angle_correction = -1 * PROPORTIONAL_GAIN * self.gyro.angle()
                self.robot.drive(speed, angle_correction)
                wait(5)
        elif self.inverted==True:
            while self.robot.distance()*-1 < distance*0.6:
                angle_correction = PROPORTIONAL_GAIN * self.gyro.angle()
                self.robot.drive(speed*-1, angle_correction)
                wait(5)
        if ease == True:
            for i in reversed(range(startSpeed, speed, round(rate))):
                PROPORTIONAL_GAIN = i*0.011
                if self.inverted==False:
                    angle_correction = -1 * PROPORTIONAL_GAIN * self.gyro.angle()
                    self.robot.drive(i, angle_correction)
                elif self.inverted==True:
                    angle_correction = PROPORTIONAL_GAIN * self.gyro.angle()
                    self.robot.drive(i*-1, angle_correction)
                wait(5)
        self.robot.stop()

        

    def gyroDriveCmBackward(self, distance, speed, ease, startSpeed=0):
        """
        Drives toward a direction for a specific distance, backwards, in centimeters, using a gyro.

        Parameters:
        distance(float): In cm, how far we drive for
        speed(int): Speed in mm/sec
        ease(bool): Whether to ease or not
        startSpeet(int): The speed to start at if you are easing.

        Returns:
        None
        """
        self.robot = DriveBase(self.leftMotor, self.rightMotor, wheel_diameter=92.5, axle_track=115)
        self.robot.reset()
        self.gyro.reset_angle(0)
        PROPORTIONAL_GAIN = 0.011*speed
        deltaspeed=speed-startSpeed
        rate=deltaspeed/(distance*0.2)
        if ease == True:
            for i in range(startSpeed, speed, round(rate)):
                PROPORTIONAL_GAIN = i*0.011
                if self.inverted==False:
                    angle_correction = -1 * PROPORTIONAL_GAIN * self.gyro.angle()
                    self.robot.drive(i*-1, angle_correction)
                elif self.inverted==True:
                    angle_correction = PROPORTIONAL_GAIN * self.gyro.angle()
                    self.robot.drive(i, angle_correction)
                wait(5)
        if self.inverted==False:
            while self.robot.distance() > distance*-0.6:
                angle_correction = -1 * PROPORTIONAL_GAIN * self.gyro.angle()
                self.robot.drive(speed*-1, angle_correction)
                wait(5)
        elif self.inverted==True:
            while self.robot.distance()*-1 > distance*-0.6:
                angle_correction = PROPORTIONAL_GAIN * self.gyro.angle()
                self.robot.drive(speed, angle_correction)
                wait(5)
        if ease == True:
            for i in reversed(range(startSpeed, speed, round(rate))):
                PROPORTIONAL_GAIN = i*0.011
                if self.inverted==False:
                    angle_correction = -1 * PROPORTIONAL_GAIN * self.gyro.angle()
                    self.robot.drive(i*-1, angle_correction)
                elif self.inverted==True:
                    angle_correction = PROPORTIONAL_GAIN * self.gyro.angle()
                    self.robot.drive(i, angle_correction)
                wait(5)
        self.robot.stop()
  
