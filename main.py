#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, ColorSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.iodevices import Ev3devSensor

import math, time

from pid import *

ev3 = EV3Brick()
Houses = [[], [], []]

try: 
  #frontClaw = Motor(Port.A)
  #backClaw = Motor(Port.D)
  leftMotor = Motor(Port.B, positive_direction = Direction.COUNTERCLOCKWISE)
  rightMotor =  Motor(Port.C)

  #htFront = Ev3devSensor(Port.S1) 
  #htSide = Ev3devSensor(Port.S2)
  colLeft = ColorSensor(Port.S3)
  colRight = ColorSensor(Port.S4)
  
except:
  pass

stopwatch = StopWatch()
base = Base(leftMotor, rightMotor, colLeft, colRight)

LineTrack = PID_LineTrack(base, 0.3, 0, 5, 40)
Straight = PID_Straight(base, 0.5, 0, 10)


ev3.speaker.beep()

base.reset()
while rightMotor.angle() < 100:
  Straight.move(80)
base.stop()

while colRight.reflection() > 10:
  LineTrack.move(80)
base.stop()

base.reset()
while rightMotor.angle() < 200:
  Straight.move(80)
base.stop()



  