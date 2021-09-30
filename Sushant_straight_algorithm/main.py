#!/usr/bin/env pybricks-micropython

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
amotor=Motor(Port.A)
up=0
threshold=170
threshold2=190
up_angle=180
speed=100
speed2=-100
amotor.run_until_stalled(-100)
amotor.reset_angle(0)
# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=92.5, axle_track=115)

gyro_sensor = GyroSensor(Port.S4) # Assumes gyro is connected to port 4

distance = 900 # millimetres
robotSpeed = 100 # mm/sec

robot.reset()
gyro_sensor.reset_angle(0)

PROPORTIONAL_GAIN = 1.1
robot.turn(90)
if distance > 0: # move forwards
    while robot.distance() < distance:
        angle_correction = -1 * PROPORTIONAL_GAIN * gyro_sensor.angle()+90
        robot.drive(robotSpeed, angle_correction)
        wait(5)
    distance=-100
    robot.stop()
    amotor.run_target(speed,up_angle)
    while robot.distance() > distance:
        reverseSpeed = -1 * robotSpeed
        angle_correction = -1 * PROPORTIONAL_GAIN * gyro_sensor.angle()
        robot.drive(reverseSpeed, angle_correction)
        if amotor.angle()<=threshold:
            amotor.run_target(speed,up_angle)
        elif amotor.angle()>=threshold2:
            amotor.run_target(speed2,up_angle)
        wait(5)
robot.stop()
