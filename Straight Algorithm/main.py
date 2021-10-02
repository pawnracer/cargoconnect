#!/usr/bin/env pybricks-micropython
"""
Drives the robot 400 millimeters forward, and then 400 millimeters backward
"""
from ucollections import namedtuple
import urandom

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, ColorSensor, GyroSensor
from pybricks.parameters import Port, Color, ImageFile, SoundFile
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase

# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Initialize the motors.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=92.5, axle_track=115)

gyro_sensor = GyroSensor(Port.S4) # Assumes gyro is connected to port 4

distance = 400 #how many mm to drive
robotSpeed = 100 # mm/sec

robot.reset()
gyro_sensor.reset_angle(0)

PROPORTIONAL_GAIN = 1.1
while robot.distance() < distance:
    angle_correction = -1 * PROPORTIONAL_GAIN * gyro_sensor.angle()
    robot.drive(robotSpeed, angle_correction)
    wait(5)
distance=0 #setting distance to 0 because we want the robot to stop at zero
robot.stop()
while robot.distance() > distance:
    reverseSpeed = -1 * robotSpeed
    angle_correction = -1 * PROPORTIONAL_GAIN * gyro_sensor.angle()
    robot.drive(reverseSpeed, angle_correction)
robot.stop()
