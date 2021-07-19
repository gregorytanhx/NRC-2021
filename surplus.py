#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, ColorSensor, GyroSensor)
from pybricks.nxtdevices import ColorSensor as nxtColorSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.iodevices import Ev3devSensor

import math, time
from scan import *
from surplus import *
from pid import *

def checkSurplus(ev3ColSensor, GyroStraight, degrees):
  surplus = False
  if ev3ColSensor.reflection() > 20:
    GyroStraight.base.run_target(-30, degrees)
    return True
  else:
    return False

def collectSurplus(base, ev3ColSensor, GyroStraight, GyroTurn, degrees):
  GyroStraight.gyro.reset_angle(0)
  while ev3ColSensor.reflection() > 1:   
    GyroStraight.move(-40)
  #GyroStraight.base.stop()
  while ev3ColSensor.reflection() < 20:   
    GyroStraight.move(-40)
  #GyroStraight.base.stop()
  while ev3ColSensor.reflection() > 1:   
    GyroStraight.move(-40)
  GyroStraight.base.hold()
  # while ev3ColSensor.reflection() > 1:   
  #   GyroStraight.move(-30)
  # base.stop()
  GyroTurn.turn(90)
  GyroStraight.gyro.reset_angle(0)

 
  while base.colRight.reflection() > 15:
    GyroStraight.move(-30)
  base.stop()
  while base.colRight.reflection() < 80:
    GyroStraight.move(-30)
  base.stop()
  while base.colRight.reflection() > 15:
    GyroStraight.move(30)
  base.stop()
  
  PID_LineSquare(base, direction = -1)
  base.stop()

  base.frontClaw.run_time(CorrectSpeed(100), 1100)
  base.frontClaw.reset_angle(0)
  base.reset()
  while base.leftMotor.angle() < degrees:
    GyroStraight.move(30)
  base.hold()

  base.frontClaw.run_target(CorrectSpeed(-50), -418)
  base.frontClaw.hold()  
  