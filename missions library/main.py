#!/usr/bin/env pybricks-micropython

#This program calls the functions defined in the missions.py

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from missions import Missions


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()
gs=GyroSensor(Port.S4)
lm=Motor(Port.B)
rm=Motor(Port.C)
fm=Motor(Port.A)
m=Missions(gyro=gs, leftMotor=lm, rightMotor=rm, medMotor1=fm, inverted=True)

# Write your program here.
while True:
    if Button.UP in ev3.buttons.pressed():
        ev3.screen.clear()
        ev3.screen.print("Turbine Blade")
        m.turbine_blade()
    if Button.LEFT in ev3.buttons.pressed():
        ev3.screen.clear()
        ev3.screen.print("Package Delevery")
        m.m_11()
    if Button.RIGHT in ev3.buttons.pressed():
        ev3.screen.clear()
        ev3.screen.print("Platooning")
        m.platooning()
    if Button.DOWN in ev3.buttons.pressed():
        ev3.screen.clear()
        ev3.screen.print("Cargo door and Mission06")
        m.Cargo_door_and_Mission06()
    ev3.screen.clear()
