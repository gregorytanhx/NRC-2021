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
from pid import *

ev3 = EV3Brick()
Houses = [[], [], []]
surplus = None

frontClaw = Motor(Port.A)
backClaw = Motor(Port.D)
leftMotor = Motor(Port.B, positive_direction = Direction.COUNTERCLOCKWISE)
rightMotor =  Motor(Port.C)

ev3Col = Ev3devSensor(Port.S1)
ev3ColSensor = ColorSensor(Port.S1)

#nxtCol = nxtColorSensor(Port.S1) 
#HTCol = Ev3devSensor(Port.S1) 
gyro = GyroSensor(Port.S2)
colLeft = ColorSensor(Port.S3)
colRight = ColorSensor(Port.S4)

stopwatch = StopWatch()
base = Base(leftMotor, rightMotor, colLeft, colRight)

LineTrack = PID_LineTrack(base, 0.16, 0, 5, 50)
GyroStraight = PID_GyroStraight(base, 1.2, 0, 5, gyro)
GyroTurn = PID_GyroTurn(base, 1.09, 0.0002, 2, gyro)

gyro.reset_angle(0)

start = stopwatch.time()
base.reset()

def checkHouse1():
  while leftMotor.angle() < 200:
    GyroStraight.move(100)
  base.reset()
  while leftMotor.angle() < 500:
    LineTrack.move(colRight, 50)
  # base.stop()
  while colLeft.reflection() > 15:
    LineTrack.move(colRight, 50)
  base.stop()
  while colLeft.reflection() < 80:
    GyroStraight.move(40)
  base.stop() 
  while colLeft.reflection() > 15:
    GyroStraight.move(-40)
  base.stop()  
  base.reset()
  while leftMotor.angle() < 300:
    GyroStraight.move(50)
  GyroTurn.turn(-90)
  gyro.reset_angle(0)
  # wall align
  base.run_time(-100, 1.2)
  gyro.reset_angle(0)

  scanHouseEV3(Houses[0], ev3Col, GyroStraight, 50, ev3)
  leftMotor.brake()
  rightMotor.brake()

  while colRight.reflection() < 80:
    GyroStraight.move(-40)
  base.stop()
  PID_LineSquare(base, direction = -1)
  leftMotor.hold()
  PID_SingleMotorTurn(rightMotor, gyro, -180)

  gyro.reset_angle(0)
  base.reset()
  GyroTurn.turn(180)
  
  gyro.reset_angle(0)
  PID_AngleOffSet(base, gyro, 45)
  base.reset()
  while leftMotor.angle() < 300:
    LineTrack.move(colRight, 50)
    
  while colLeft.reflection() > 15:
    LineTrack.move(colRight, 50)

  while colLeft.reflection() < 80:
    LineTrack.move(colRight, 50)
  base.stop()
  while colRight.reflection() > 15:
    LineTrack.move(colLeft, 50, side = -1)
  base.hold()
 
  while colRight.reflection() < 80:
    base.run(-40, -40)





def collectGreen():
  while colRight.reflection() > 15:
    LineTrack.move(colLeft, 30, side = -1)
  base.hold()
  base.run_target(-30, -50)
  #main()
  #
  # grab first two
  GyroTurn.turn(-90)
  while colRight.reflection() < 80:
    base.run(-40, -40)
  base.hold()
  PID_LineSquare(base, direction = -1)
  gyro.reset_angle(0)
  base.reset()
  base.run_target(50, 200)
  backClaw.run_target(CorrectSpeed(-50), -227)
  while colRight.reflection() < 70:
    base.run(-30, -30)
  base.hold()


  backClaw.run_time(CorrectSpeed(100), 1200)
  while colRight.reflection() < 70:
    base.run(40, 40)
  base.hold()
  while colRight.reflection() > 40:
    base.run(40, 40)
  base.hold()

  # turn to grab second 2
  GyroTurn.turn(90)
  gyro.reset_angle(0)
  base.reset()
  while leftMotor.angle() < 240:
    LineTrack.move(colLeft, 30, side = -1)
  base.stop()



  GyroTurn.turn(-90)
  while colRight.reflection() < 70:
    base.run(-40, -40)
  base.hold()
  PID_LineSquare(base, direction = -1)
  gyro.reset_angle(0)
  base.reset()
  base.reset()
  base.run_target(50, 200)

  backClaw.run_target(CorrectSpeed(-50), -230)
  while colRight.reflection() < 80:
    base.run(-30, -30)
  base.hold()


  backClaw.run_target(CorrectSpeed(50), 100)
  GyroTurn.turn(-90)
  

checkHouse1()
if checkSurplus():
  surplus = Color.YELLOW
collectSurplus()
collectGreen()
if surplus is None:
  if checkSurplus():
    surplus = Color.GREEN
    collectSurplus()
  else:
    surplus = Color.BLUE






# # 


# base.reset()
# while rightMotor.angle() < 100:
#   GyroStraight.move(80)
# base.stop()

# while colRight.reflection() > 10:
#   LineTrack.move(80)
# base.stop()

# base.reset()
# while rightMotor.angle() < 200:
#   GyroStraight.move(80)
# base.stop()



  