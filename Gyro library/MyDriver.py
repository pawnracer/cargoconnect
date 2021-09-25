#!/usr/bin/env pybricks-micropython
from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import (Port, Stop, Direction, Button, Color,
                                 SoundFile, ImageFile, Align)
from pybricks.tools import print, wait, StopWatch
from pybricks.robotics import DriveBase
import array as arr
import hardware
from MyButtons import checkIfAbort, abort
import math

leftMotor = hardware.leftMotor
rightMotor = hardware.rightMotor
gyro = hardware.gyro
backupGyro = hardware.backupGyro
rampGyro = hardware.rampGyro
# class inheritance: MyDriver extends DriveBase from the ev3 library
# delegate the driving tasks to the MyDriver class
# easy access to both drive motors
class MyDriver(DriveBase):
    # constructor
    def __init__(self):
        super().__init__(leftMotor, rightMotor, hardware.Dimensions.WHEEL_DIAMETER, hardware.Dimensions.AXLE_TRACK)
        self.resetGyro()
    
    def printAngle(self):
        print("gyro = " + str(gyro.angle()))
    
    def resetGyro(self, angle = 0):
        """
        Resets all gyro to input angle (except for rampGyro).

        Parameters:
        angle(int): Resets gyro and backupgyro to this value.

        Returns:
        None
    
        """
        gyro.resetAngle(angle)
        backupGyro.resetAngle(angle)
        rampGyro.resetAngle(0) # we never want to set the ramp gyro to anything but 0

    def switchGyro(self):
        """
        Switches to backupGyro.

        Parameters:
        None

        Returns:
        None
    
        """
        brick.sound.beep(1000, 250)
        global gyro # must assign to global so that it changes the global variable, not just in the function
        gyro = backupGyro # swap gyro
    
    def stop(self, stopType = Stop.COAST):
        super().stop(stopType)
        if(stopType == Stop.BRAKE):
            super().stop(stopType)

    def steeringCalc(self, targetAngle, adjFactor):
        """
        Calculates the adjustment for the robot to turn from the current angle to the target angle, the further from the target we are,the
        more adjustment there is.

        Parameters:
        targetAngle(int): In degrees, the target direction
        adjFactor(float): How fast the adjustment is

        Returns:
        steering(float): Steering factor for DriveBase
    
        """
        angle = gyro.angle()
        angleDiff = targetAngle - angle
        steering = angleDiff * adjFactor
        return steering

    def gyroDriveTime(self, targetAngle, adjFactor, msTime, mmPerSec):
        """
        Drives toward a direction for a specific amount of time, in milliseconds, using a gyro.

        Parameters:
        targetAngle(int): In degrees, the target direction
        adjFactor(float): How fast the adjustment is
        msTime(int): In ms, how long we drive for
        mmPerSec(int): Speed in mm/sec

        Returns:
        None
    
        """
        watch = StopWatch()
        watch.reset() # reset watch to 0
        watch.resume()
        while watch.time() <= msTime:
            steering = self.steeringCalc(targetAngle, adjFactor)
            super().drive(mmPerSec, steering)
            checkIfAbort()
        self.stop()
    
    # drive toward a direction for a specific distance using a gyro
    def gyroDriveCM(self, targetAngle, adjFactor, distanceInCM, mmPerSec):
        """
        Drives toward a direction for a specific distance, in centimeters, using a gyro.

        Parameters:
        targetAngle(int): In degrees, the target direction
        adjFactor(float): How fast the adjustment is
        distanceInCM(float): In cm, how far we drive for
        mmPerSec(int): Speed in mm/sec

        Returns:
        None
    
        """
        leftMotor.reset_angle(0)
        rightMotor.reset_angle(0)
        while abs(self.travelDistanceCM()) < abs(distanceInCM):
            steering = self.steeringCalc(targetAngle, adjFactor)
            super().drive(mmPerSec, steering)
            checkIfAbort()
        self.stop(Stop.BRAKE)

    # slowDownPart is the percentage of the distanceInCM in which the robot will start slowing down
    def advGyroDriveCM(self, targetAngle, adjFactor, distanceInCM, mmPerSecStart, mmPerSecPeak, mmPerSecEnd, slowDownPart = 20):
        """
        Drives toward a direction for a specific distance, in centimeters, using a gyro. Has accelaration and deccelaration to reduce skidding.

        Parameters:
        targetAngle(int): In degrees, the target direction
        adjFactor(float): How fast the adjustment is, the larger the number, the bigger the response is, typically 1 or 2
        distanceInCM(float): In cm, how far we drive for
        mmPerSecStart(int): The starting speed in mm/sec
        mmPerSecPeak(int): The peak speed in mm/sec
        mmPerSecEnd(int): The end speed in mm/sec
        slowDownPart(int): The percentage of the drive that robot deccelarates to the end speed

        Returns:
        None
    
        """
        leftMotor.reset_angle(0)
        rightMotor.reset_angle(0)
        speed = mmPerSecStart
        # distance where we begin to slow down
        startRampDownDistance = distanceInCM * (100 - slowDownPart) / 100
        while abs(self.travelDistanceCM()) < abs(distanceInCM):
            if speed < mmPerSecPeak and self.travelDistanceCM() < startRampDownDistance:
                speed += (hardware.Dimensions.SPEED_CHANGE)
                if(speed > mmPerSecPeak):
                    speed = mmPerSecPeak
            elif speed > mmPerSecEnd and self.travelDistanceCM() >= startRampDownDistance: 
                speed -= (hardware.Dimensions.SPEED_CHANGE)
                if(speed < mmPerSecEnd):
                    speed = mmPerSecEnd
            steering = self.steeringCalc(targetAngle, adjFactor)
            super().drive(speed, steering)
            checkIfAbort()
        self.stop(Stop.BRAKE)

    def advGyroDriveTime(self, targetAngle, adjFactor, msTime, mmPerSecStart, mmPerSecPeak, mmPerSecEnd, slowDownTime = 2000):
        """
        Drives toward a direction for a specific amount of time, in ms, using a gyro. Has accelaration and deccelaration to reduce skiddign.

        Parameters:
        targetAngle(int): In degrees, the target direction
        adjFactor(float): How fast the adjustment is, the larger the number, the bigger the response is, typically 1 or 2
        msTime(int): In ms, how long we drive for
        mmPerSecStart(int): The starting speed in mm/sec
        mmPerSecPeak(int): The peak speed in mm/sec
        mmPerSecEnd(int): The end speed in mm/sec
        slowDownPart(int): The percentage of the drive that robot deccelarates to the end speed

        Returns:
        None
    
        """
        speed = mmPerSecStart
        startRampDownTime = msTime - slowDownTime
        watch = StopWatch()
        watch.reset() # reset watch to 0
        watch.resume()
        while watch.time() <= msTime:
            if speed < mmPerSecPeak and watch.time() < startRampDownTime:
                speed += hardware.Dimensions.SPEED_CHANGE
                if(speed > mmPerSecPeak):
                    speed = mmPerSecPeak
            elif speed>mmPerSecEnd and watch.time() >= startRampDownTime: 
                speed -= hardware.Dimensions.SPEED_CHANGE
                if(speed < mmPerSecEnd):
                    speed = mmPerSecEnd
            steering = self.steeringCalc(targetAngle, adjFactor)
            super().drive(speed, steering)
            checkIfAbort()
        self.stop(Stop.BRAKE)

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
        currAngle = gyro.angle()
        isTurnRight = currAngle < targetAngle
        leftMotor.run(leftDegPerSec)
        rightMotor.run(rightDegPerSec)
        while ((targetAngle > currAngle) and isTurnRight) or ((targetAngle < currAngle) and not isTurnRight):
            currAngle = gyro.angle()
            checkIfAbort()
        self.stop(stopType)
        if selfAdjust == hardware.Dimensions.NO_TURN_ADJUSTMENT: # skip selfAdjust because we sometimes don't need accuracy
            return
        # for coast, wait for motors to settle before checking for overshoot
        if(stopType == Stop.COAST):
            for i in range(0, 2):
                wait(100)
                checkIfAbort()
        currAngle = gyro.angle()
        angleDelta = currAngle - targetAngle
        if abs(angleDelta) < selfAdjust:
            return

        # there is no under-turn, so there is no point in checking
        isOver = (isTurnRight and angleDelta > 0) or (not isTurnRight and angleDelta < 0)
        # self-adjust at a lower speed for accuracy
        if isOver: # only self-adjust once to avoid recursion (infinite loop)
            self.gyroTankTurn(leftDegPerSec * (-0.5), rightDegPerSec * (-0.5), targetAngle, Stop.BRAKE, hardware.Dimensions.NO_TURN_ADJUSTMENT)

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
        midAngle = gyro.angle() + (targetAngle - gyro.angle()) * 0.65
        self.gyroTankTurn(leftDegPerSec, rightDegPerSec, midAngle, Stop.COAST, hardware.Dimensions.NO_TURN_ADJUSTMENT)
        self.gyroTankTurn(leftDegPerSec/2, rightDegPerSec/2, targetAngle, stopType, selfAdjust)

    def driveStraight(self, msTime, mmPerSec):
        """
        Drive each motor at a specific speed until a target angle has been met. In case of overshooting, we self adjust to what the angle should be.

        Parameters:
        msTime(int): In ms, how long we drive for
        mmPerSec(int): Speed in mm/sec 
        
        Returns:
        None
    
        """
        adjFactor = 1 if mmPerSec < 100 else mmPerSec/100
        leftMotor.reset_angle(0)
        rightMotor.reset_angle(0)
        watch = StopWatch()
        watch.reset() # reset watch to 0
        watch.resume()
        while watch.time() <= msTime:
            leftAngle = leftMotor.angle()
            rightAngle = rightMotor.angle()
            steering = (rightAngle - leftAngle) * adjFactor
            super().drive(mmPerSec, steering)
            checkIfAbort()
        self.stop()

    def advDriveStraightTime(self, msTime, mmPerSecStart, mmPerSecPeak, mmPerSecEnd, slowDownTime = 2000):
        """
        A more advanced version of driveStraight, which can speed up and slow down, and also has an optional time in milliseconds for slowdown 
        because some missions may want to have extra time for slow speed to finsih with in the mission.

        Parameters:
        msTime (int) : In ms, how long we drive for
        mmPerSecStart (int): The start speed
        mmPerSecPeak (int) : The peak speed
        mmPerSecEnd (int) : The end speed 
        
        Returns:
        None
    
        """
        speed = mmPerSecStart
        startRampDownTime = msTime - slowDownTime
        leftMotor.reset_angle(0)
        rightMotor.reset_angle(0)
        watch = StopWatch()
        watch.reset() # reset watch to 0
        watch.resume()
        while watch.time() <= msTime:
            leftAngle = leftMotor.angle()
            rightAngle = rightMotor.angle()
            leftAngle = (leftAngle + leftMotor.angle()) / 2
            if speed < mmPerSecPeak and watch.time() < startRampDownTime:
                speed += hardware.Dimensions.SPEED_CHANGE
                if(speed > mmPerSecPeak):
                    speed = mmPerSecPeak
            elif speed>mmPerSecEnd and watch.time() >= startRampDownTime: 
                speed -= hardware.Dimensions.SPEED_CHANGE
                if(speed < mmPerSecEnd):
                    speed = mmPerSecEnd
            adjFactor = 1 if speed < 90 else speed/100 * 1.1
            steering = (rightAngle - leftAngle) * adjFactor
            super().drive(speed, steering)
            checkIfAbort()
        self.stop()

    def advDriveStraightCM(self, distanceInCM, mmPerSecStart, mmPerSecPeak, mmPerSecEnd, slowDownPart = 20):
        """
        Similar to advDriveStraightTime (has acceleration and decceleration) but using distance in CM for stop. Has an optional percentage 
        of distanceInCM at the end to start slowdown.

        Parameters:
        distanceInCM(float): In cm, how far we drive for
        mmPerSecStart(int): The starting speed in mm/sec
        mmPerSecPeak(int): The peak speed in mm/sec
        mmPerSecEnd(int): The end speed in mm/sec
        slowDownPart(int): The percentage of the drive that robot deccelarates to the end speed
        
        Returns:
        None
    
        """
        leftMotor.reset_angle(0)
        rightMotor.reset_angle(0)
        speed = mmPerSecStart
        # distance where we begin to slow down
        startRampDownDistance = distanceInCM * (100 - slowDownPart) / 100
        while abs(self.travelDistanceCM()) < abs(distanceInCM):
            if speed < mmPerSecPeak and self.travelDistanceCM() < startRampDownDistance:
                speed += (hardware.Dimensions.SPEED_CHANGE)
                if(speed > mmPerSecPeak):
                    speed = mmPerSecPeak
            elif speed > mmPerSecEnd and self.travelDistanceCM() >= startRampDownDistance: 
                speed -= (hardware.Dimensions.SPEED_CHANGE)
                if(speed < mmPerSecEnd):
                    speed = mmPerSecEnd
            adjFactor = 1 # if speed < 90 else speed/100 * 1.1
            steering = (rightMotor.angle() - leftMotor.angle()) * adjFactor
            super().drive(speed, steering)
            checkIfAbort()
        self.stop(Stop.BRAKE)

    def gyroDriveRamp(self, targetAngle, mmPerSecStart, mmPerSecPeak, isGoingForFlags):
        """
        Only for driving up the ramp with the option to go all the way up for the flags. 

        Parameters:
        mmPerSecStart: Start speed
        mmPerSecPeak: Peak speed
        isGoingForFlags: True - go all the way up for the flags. False - go on the ramp and stop. 

        Returns:
        None
    
        """
        wait(100)
        rampGyro.resetAngle()
        wait(100)
        speed = mmPerSecStart
        # drive for 1.5 secs to get on the ramp before checking whether we are going for the flags
        rampTime = 1500
        timeToStop = False
        adjFactor = 1
        watch = StopWatch()
        watch.reset() # reset watch to 0
        watch.resume()
        while watch.time() <= rampTime or not(timeToStop):
            if speed < mmPerSecPeak:
                speed += hardware.Dimensions.SPEED_CHANGE
                if(speed > mmPerSecPeak):
                    speed = mmPerSecPeak
            steering = self.steeringCalc(targetAngle, adjFactor)
            super().drive(speed, steering)
            # wait(20)
            checkIfAbort()
            # if we are going for the flags, we detect the flat part of the top of the ramp
            # otherwise, we stop on the ramp without the flags

            # based on the gyro position, the reading of flat is 0 and on the ramp is negative
            # therefore, we use -3 to detect when we are very close to the top
            # so that the front of the robot is on the flags
            timeToStop = rampGyro.angle() > -3 if isGoingForFlags else True
        self.stop(Stop.BRAKE) # make sure to lock the wheels

    def travelDistanceCM (self):
        """
        Finds the distance traveled by both wheels and then finds the average of the two

        Parameters:
        None

        Returns:
        dAverage(float) : In cm, the average distance of both wheels in centimeters 
        """
        angLeft = leftMotor.angle()
        angRight =  rightMotor.angle()
        dLeft = angLeft / 360 * hardware.Dimensions.WHEEL_DIAMETER * math.pi
        dRight = angRight / 360 * hardware.Dimensions.WHEEL_DIAMETER * math.pi
        dAverage = (dLeft + dRight) / 20 # it is 20 instead of 2 because angLeft and angRight are in mm and they need to be in cm
        return dAverage
    
    def initTest(self):
        """
        Reports low battery, prints out the gyro angle once every second for 3 seconds so that we can see if the gyro is stable

        Parameters:
        None

        Returns:
        None
        """
        brick.display.clear()
        batt = brick.battery.voltage()
        if batt < 8000:
            brick.sound.file(SoundFile.GAME_OVER) # blocking
        brick.display.text("Battery = " + str(batt/1000) + "V")

        gyros = [gyro, backupGyro, rampGyro]
        for i in range(0,3):
            if i > 0:
                wait(1000)
            for g in gyros:
                g.storeAngle()
        for g in gyros:
            g.reportDrifting()
            
