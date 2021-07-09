#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.iodevices import Ev3devSensor

ev3 = EV3Brick()
leftMotor = Motor(Port.B, positive_direction = Direction.COUNTERCLOCKWISE)
rightMotor =  Motor(Port.C)

while True:
  ev3.screen.print(f"leftMotor: {leftMotor.angle()}")
  ev3.screen.print(f"rightMotor: {rightMotor.angle()}")
  if Button.CENTER in ev3.buttons.pressed():
    leftMotor.reset_angle(0)
    rightMotor.reset_angle(0)