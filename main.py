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

ev3 = EV3Brick()
Houses = [[], [], []]
surplus = None

numYellow = 4
numBlue = 4
numGreen = 4
numSurplus = 4


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
base = Base(leftMotor, rightMotor, colLeft, colRight, frontClaw, backClaw)

LineTrack = PID_LineTrack(base, 0.2, 0, 5, 50)
GyroStraight = PID_GyroStraight(base, 1.2, 0, 5, gyro)
GyroTurn = PID_GyroTurn(base, 1.1, 0.0001, 2.5, gyro)

start = stopwatch.time()

# battery alert
print(ev3.battery.voltage())
if ev3.battery.voltage() <= 7400:
  print('LOW BATTERY')
  ev3.speaker.beep()

lastError = 0

def scanHouseEV3(house, sensor, speed):
  base.reset()
  lastError = 0
  kp, ki, kd = GyroStraight.kp, GyroStraight.ki, GyroStraight.kd
  gyroPID = PID(kp, ki, kd)
  while colRight.reflection() > 15:
    detected = False
    gyroPID.update(gyro.angle(), kp, ki, kd)
    base.run(speed - pid.correction, speed + pid.correction)
    r, g, b = sensor.read('RGB-RAW')
    if r + g + b > 15:
      detected = True
     
      if r - b >= 3 and r - g >= 3:
        house.append(Color.YELLOW)
      elif b - r >= 3 and b - g >= 3:
        house.append(Color.BLUE)
      elif g - r >= 3 and g - b >= 3:
        house.append(Color.GREEN) 
      else:
        detected = False
        
    if detected:
      while r + g + b > 12:
        r, g, b = sensor.read('RGB-RAW')
        gyroPID.update(gyro.angle(), kp, ki, kd)
        base.run(speed - gyroPID.correction, speed + gyroPID.correction)
      detected = False

  print(house)
  base.stop()

def checkSurplus(degrees):
  base.reset()
  detected = False
  while leftMotor.angle() >= degrees:
    base.run(-30, -30)
    print(ev3ColSensor.reflection() > 20)
    if ev3ColSensor.reflection() > 20:
      detected = True
  base.stop()

  return detected

def collectSurplus(degrees):
  gyro.reset_angle(0)
  GyroStraight.move(-40, condition = lambda: ev3ColSensor.reflection() > 1)
  GyroStraight.move(-40, condition = lambda: ev3ColSensor.reflection() < 20)
  base.stop()
  GyroStraight.move(-40, condition = lambda: ev3ColSensor.reflection() > 1)
  base.hold()
  # while ev3ColSensor.reflection() > 1:   
  #   GyroStraight.move(-30)
  # base.stop()
  GyroTurn.turn(90)
  gyro.reset_angle(0)

  GyroStraight.move(-30, condition = lambda: colRight.reflection() > 15)
  base.hold()
  GyroStraight.move(30, condition = lambda: colRight.reflection() < 60)
  base.hold()

  
  PID_LineSquare(base, direction = -1)
  base.stop()
  
  frontClaw.run_time(CorrectSpeed(100), 1100)
  frontClaw.reset_angle(0)
  base.reset()
  GyroStraight.move(-30, condition = lambda: leftMotor.angle() < degrees)
  base.hold()
  wait(1000)

  frontClaw.run_target(CorrectSpeed(-50), -418)
  frontClaw.hold()   

def checkHouse1():
  gyro.reset_angle(0)
  base.reset()
  GyroStraight.move(50, condition = lambda: leftMotor.angle() <= 200)
  base.stop()
  base.reset()
  LineTrack.move(colRight, 50, condition = lambda: leftMotor.angle() <= 500)
  LineTrack.move(colRight, 50, condition = lambda: colLeft.reflection() > 15)
  base.hold()
  base.reset()
  GyroStraight.move(50, condition = lambda:  leftMotor.angle() <= 300)
  base.hold()
  GyroTurn.turn(-90)
  gyro.reset_angle(0)
  # wall align
  base.run_time(-100, 1.2)
  gyro.reset_angle(0)

  scanHouseEV3(Houses[0], ev3Col, 50)
  base.hold()

  
  # while colRight.reflection() <60:
  #   GyroStraight.move(40)
  # base.stop()
  PID_LineSquare(base, direction = -1)
  leftMotor.hold()
  PID_SingleMotorTurn(rightMotor, gyro, -179)
  base.stop()
  
def collectGreen(degrees):
  LineTrack.move(colLeft, 30, side = -1, condition = lambda:  colRight.reflection() > 15)
  base.hold()

  GyroStraight.move(-30, condition = colRight.reflection() > 15)
  base.hold()
  #main()
  #
  # grab first two
  GyroTurn.turn(-90)
  while colRight.reflection() <60:
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

  LineTrack.move(colLeft, 30, side = -1, condition = lambda: leftMotor.angle() <= degrees)
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
  while colRight.reflection() <60:
    base.run(-30, -30)
  base.hold()

  backClaw.run_target(CorrectSpeed(50), 100)
  GyroTurn.turn(-90)
  
def goHouse2():
  GyroTurn.turn(-180, kp = 1.13)
  gyro.reset_angle(0)
  base.reset()

  LineTrack.move(colRight, 50, condition = leftMotor.angle() <= 1200)
  base.stop()

  LineTrack.move(colLeft, 70, kp = 0.4, kd = 5.5, side = -1, condition = lambda: colRight.reflection() > 15)
  base.hold()


def returnHouse1():
  pass

def depositHouse(house):
  if numSurplus > 0:    
    pass

def goHouse3():
  pass

def collectYellow():
  pass

def collectBlue():
  pass

def depositBattery(side = 1):
  pass

def solarPanels():
  # linetrack to intersection
  while colLeft.reflection() > 15:
    base.run(40, 0)
  base.stop()
  while colLeft.reflection() > 15:
    base.run(-40, 0)
  base.stop()
  while colLeft.reflection() < 60:
    base.run(-40, 0)
  base.stop()
  wait(100)
  base.reset()
  LineTrack.move(colRight, 40, condition = lambda: colLeft.reflection() > 15)
  base.stop()

  LineTrack.move(colRight, 40, condition = lambda: colLeft.reflection() < 60)
  base.stop()
  wait(100)
  LineTrack.move(colLeft, 40, side = -1, condition = lambda: colRight.reflection() > 15)
  LineTrack.move(colLeft, 40, side = -1, condition = lambda: colRight.reflection() < 60)
  base.hold()

  
  GyroTurn.turn(90)
  gyro.reset_angle(0)
  base.reset() 
  # push solar panels
  LineTrack.move(colLeft, 50, condition = lambda: leftMotor.angle() <= 500)
  base.hold()
  base.reset()
  GyroStraight.move(40, condition = lambda: leftMotor.angle() <= 150)
  base.hold()
  base.reset()
  GyroStraight.move(-80, condition = lambda: leftMotor.angle() >= -300)
  base.hold()
  GyroTurn.turn(180)
  
  
def main():
  checkHouse1()
  print(ev3ColSensor.reflection())
  if checkSurplus(-140):
    surplus = Color.YELLOW
    collectSurplus(385)
    GyroStraight.move(-40, condition = lambda: colRight.reflection() > 15)
    base.stop()
    base.reset()
    GyroStraight.move(40, condition = lambda: leftMotor.angle() <= 100)
    base.hold()

    GyroTurn.turn(90, kp = 0.7, kd = 1)
    gyro.reset_angle(0)
  else:
    gyro.reset_angle(0)
    base.reset()
    GyroTurn.turn(180)
    
    gyro.reset_angle(0)
    PID_AngleOffSet(base, gyro, 45)
    base.reset()
  
  
  LineTrack.move(colRight, 50, condition = lambda: leftMotor.angle() <= 300)  
  LineTrack.move(colRight, 50, condition = lambda: colLeft.reflection() > 15)
  LineTrack.move(colRight, 50, condition = lambda: colLeft.reflection() < 60)
  base.hold()
  LineTrack.move(colLeft, 50, side = -1, collection = lambda: colRight.reflection() > 15)
  base.hold()

  while colRight.reflection() <60:
    base.run(-40, -40)
  collectGreen(225)
  # collect surplus 
  if surplus is None:
    if checkSurplus(-50):
      surplus = Color.GREEN
      collectSurplus(180)
    else:
      surplus = Color.BLUE
      goHouse2()
  
  solarPanels()
  returnHouse1()
  depositHouse(Houses[0])
  goHouse2()
  scanHouseEV3(Houses[1], ev3Col, 50)
  depositHouse(Houses[1])
  goHouse3()
  scanHouse3(Houses[2], ev3Col, 50)
  depositHouse(Houses[2])
  # always deposit two surplus into battery storage from claw
  depositBattery()
  if Color.YELLOW in Houses[2]:
    collectYellow()
    depositeHouse(Houses[2])
    collectBlue()
  elif Color.BLUE in Houses[2]:
    collectBlue()
    depositHouse(Houses[2])
    collectYellow()
  else:
    collectYellow()
    collectBlue()
    
  depositBattery()
  if Color.BLUE or Color.YELLOW in Houses[1]:
    depositHouse(Houses[1])
    pass
  if Color.BLUE or Color.YELLOW in Houses[0]:
    depositHouse(Houses[0])
  returnBase()


#main()
# # 
main()
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



  