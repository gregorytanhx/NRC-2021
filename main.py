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
from helper import *
from pid import *

ev3 = EV3Brick()
Houses = [[], [], []]
surplus = None

numYellow = 4
numBlue = 4
numGreen = 4
numSurplus = 4

frontClaw = Claw(Port.A)
backClaw = Claw(Port.D)
leftMotor = Motor(Port.B, positive_direction = Direction.COUNTERCLOCKWISE)
rightMotor =  Motor(Port.C)

ev3Col = Ev3devSensor(Port.S1)
ev3ColSensor = ColorSensor(Port.S1)

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
   
  lastError = 0
  kp, ki, kd = GyroStraight.kp, GyroStraight.ki, GyroStraight.kd
  gyroPID = PID(kp, ki, kd)
  while colRight.reflection() > 15:
    detected = False
    gyroPID.update(gyro.angle(), kp, ki, kd)
    base.run(speed - gyroPID.correction, speed + gyroPID.correction)
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
   
  detected = False
  speed = -30
  kp, ki, kd = GyroStraight.kp, GyroStraight.ki, GyroStraight.kd
  gyroPID = PID(kp, ki, kd)
  while leftMotor.angle() >= degrees:
    gyroPID.update(gyro.angle(), kp, ki, kd)
    base.run(speed - gyroPID.correction, speed + gyroPID.correction)
    if ev3ColSensor.reflection() >= 5:
      detected = True
  base.stop()

  return detected

def collectSurplus(degrees, blue = False):
  # align robot using surplus
  if not blue:
    GyroStraight.move(-40, condition = lambda: ev3ColSensor.reflection() > 1)
    GyroStraight.move(-40, condition = lambda: ev3ColSensor.reflection() < 10)
    base.stop()
    GyroStraight.move(-40, condition = lambda: ev3ColSensor.reflection() > 1)
    base.hold()
    
    GyroTurn.turn(90)
    gyro.reset_angle(0)

  GyroStraight.move(-30, condition = lambda: colRight.reflection() > 15)
  base.hold()
  GyroStraight.move(30, condition = lambda: colRight.reflection() < 80)
  base.hold()
  PID_LineSquare(base, direction = -1)
  gyro.reset_angle(0)
  base.hold()
  
  # open claw
  frontClaw.run_time(100, 1200)
   
  GyroStraight.move(30, condition = lambda: leftMotor.angle() <= degrees)
  base.hold()

  frontClaw.run_target(-50, -490)
  frontClaw.hold()   

def checkHouse1():
  gyro.reset_angle(0)
   
  GyroStraight.move(70, condition = lambda: leftMotor.angle() <= 200)
  base.stop()
   
  LineTrack.move(colRight, 50, condition = lambda: leftMotor.angle() <= 500)
  LineTrack.move(colRight, 50, condition = lambda: colLeft.reflection() > 15)
  base.hold()
   
  GyroStraight.move(50, condition = lambda:  leftMotor.angle() <= 310)
  base.hold()
  GyroTurn.turn(-90)
  
  # wall align
  base.run_time(-100, 1.2)
  gyro.reset_angle(0)

  scanHouseEV3(Houses[0], ev3Col, 50)
  base.hold()
  
  GyroStraight.move(40, condition = lambda: colRight.reflection() < 80)
  base.stop()
  PID_LineSquare(base, direction = -1)
  leftMotor.hold()
  PID_SingleMotorTurn(rightMotor, gyro, -178)
  base.hold()
  gyro.reset_angle(0)
  
def collectGreen(degrees):  
  LineTrack.move(colRight, 30, condition = lambda: leftMotor.angle() <= 300)  
  LineTrack.move(colRight, 50, condition = lambda: colLeft.reflection() > 15)
  LineTrack.move(colRight, 50, condition = lambda: colLeft.reflection() < 70)
  base.hold()
  LineTrack.move(colLeft, 50, side = -1, condition = lambda: colRight.reflection() > 15)
  base.hold()

  while colRight.reflection() < 60:
    base.run(-40, -40)
    
  LineTrack.move(colLeft, 30, side = -1, condition = lambda: colRight.reflection() > 15)
  base.hold()

  GyroStraight.move(-30, condition = lambda: colRight.reflection() < 80)
  base.hold()

  # grab first two
  GyroTurn.turn(-90)
  while colRight.reflection() < 60:
    base.run(-40, -40)
  base.hold()
  
  PID_LineSquare(base, direction = -1)
  gyro.reset_angle(0)
   
  base.run_target(40, 200)
  backClaw.run_target(-40, -210)
  GyroStraight.move(-30, condition = lambda: colRight.reflection() < 70)
  GyroStraight.move(-30, condition = lambda: colRight.reflection() > 15)
  base.hold()

  backClaw.run_time(100, 1200)
  GyroStraight.move(40, condition = lambda: colRight.reflection() < 70)
  GyroStraight.move(40, condition = lambda: colRight.reflection() > 40)
  base.hold()

  # turn to grab second 2
  GyroTurn.turn(90)
   

  LineTrack.move(colLeft, 30, side = -1, condition = lambda: leftMotor.angle() <= degrees)
  base.hold()

  GyroTurn.turn(-90)
  while colRight.reflection() < 70:
    base.run(-40, -40)
  base.hold()
  
  PID_LineSquare(base, direction = -1)
  
  gyro.reset_angle(0)
   
 
  GyroStraight.move(50, condition = lambda: leftMotor.angle() < 200)
  base.hold()
  backClaw.run_target(-50, -210)
  GyroStraight.move(-30, condition = lambda: colRight.reflection() < 60)
  base.hold()
  backClaw.run_target(50, 50)
  GyroTurn.maxSpeed = 50
  GyroTurn.turn(-90)

def returnHouse1():
  pass

def depositHouse(house, time, houseNum):
  global numGreen, numBlue, numYellow, numSurplus
  
  if time == 1:
    # first visit to house
    # deposit surplus and green
    numCol = numGreen
    RingCol = Color.GREEN
    if len(house) == 1: # surplus to be deposited
      if houseNum == 1:
        GyroTurn.turn(-90)
      else:
        GyroTurn.turn(90)
       
      if houseNum == 1:
        LineTrack.move(colRight, 35, condition = lambda: leftMotor.angle() < 300)
        base.hold()
        
      frontClaw.run_target(-70, -300)
      GyroStraight.move(40, condition = lambda: colLeft.color() != Color.RED)
      GyroStraight.move(40, condition = lambda: leftMotor.angle() < 100)  
      GyroStraight.move(-40, condition = lambda: leftMotor.angle() > -300)
      base.hold()
      
      GyroTurn.turn(-180)
       
      gyro.reset_angle(0)
      frontClaw.run_target(50, 300)
      
  else:
    numCol = numBlue
    RingCol = Color.BLUE
    if Color.YELLOW in house:
      tmp = 1
      if len(house) == 2:
        if house[0] == Color.YELLOW and house[1] == Color.YELLOW:
          tmp = 2
      if houseNum == 1:
        GyroTurn.turn(90)
      
      if tmp == 2:
        frontClaw.run_target(70, 200)
        GyroStraight.move(-40, condition = lambda: leftMotor.angle() > -200)
        base.hold()
        frontClaw.run_target(-70, -500)
      
      if tmp == 1 and numYellow == 2:
        LineTrack.move(colRight, 35, condition = lambda: leftMotor.angle() < 100)
        base.hold()
        frontClaw.run_target(70, 200)
        GyroStraight.move(-40, condition = lambda: leftMotor.angle() > -200)
        base.hold()
        
      else:
        LineTrack.move(colRight, 35, condition = lambda: leftMotor.angle() < 100)
        base.hold()
        frontClaw.run_target(-70, -300)
        GyroStraight.move(40, condition = lambda: colLeft.color() != Color.RED)
         
        GyroStraight.move(40, condition = lambda: leftMotor.angle() < 100)
        base.hold()
         
        GyroStraight.move(-40, condition = lambda: leftMotor.angle() > -200)
        base.hold()
  
  if RingCol in house: # green/blue to be deposited
    tmp = 1
    if len(house) == 2:
      if house[0] == RingCol and house[1] == RingCol:
        tmp = 2

      if houseNum == 1 and tmp == 2:
        GyroTurn.turn(90)
        
      elif tmp == 2:
        GyroTurn.turn(-90)
      
    GyroStraight.move(-40, condition = lambda: leftMotor.angle() > -50)
    base.hold()

    backClaw.run_target(-20, -140)
        
    if numCol == 4 and tmp == 1:
      GyroStraight.move(30, condition = lambda: leftMotor.angle() < 60) 
      numCol -= 2       
    else:
      GyroStraight.move(30, condition = lambda: leftMotor.angle() < 200)
      numCol -= 4
    
    base.hold()
    backClaw.run_target(40, 100)
    if numCol == 0:
      GyroTurn.maxspeed = 100

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
  gyro.reset_angle(0)
  while colLeft.reflection() > 15:
    base.run(40, 0)
  base.hold()
  while gyro.angle() > 0:
    base.run(0, 40)
  base.hold()
  
   
  LineTrack.move(colRight, 30, condition = lambda: leftMotor.angle() < 200)
  base.stop()
  LineTrack.move(colLeft, 40, side = -1, condition = lambda: colRight.reflection() > 15)
  LineTrack.move(colLeft, 40, side = -1, condition = lambda: leftMotor.angle() < 90 )
   
  base.hold()
  
  GyroTurn.turn(90)
  gyro.reset_angle(0)
    
  # push solar panels
  LineTrack.move(colRight, 40, condition = lambda: leftMotor.angle() < 600)
  base.hold()
   
  GyroStraight.move(30, condition = lambda: colLeft.reflection() > 15 and colRight.reflection() > 15)
  GyroStraight.move(30, condition = lambda: colLeft.reflection() < 80 and colRight.reflection() < 80)
  GyroStraight.move(30, condition = lambda: colLeft.reflection() > 76 and colRight.reflection() > 76)
  base.hold()
   
  GyroStraight.move(-60, condition = lambda: leftMotor.angle() >= -600)
  base.hold()
  GyroTurn.turn(180)
  
def main():
  checkHouse1()
  if checkSurplus(-140):
    surplus = Color.YELLOW
    collectSurplus(370)
    GyroStraight.move(-40, condition = lambda: colRight.reflection() > 15)
     
    GyroStraight.move(40, condition = lambda: leftMotor.angle() <= 100)
    base.hold()
    GyroTurn.turn(90)

  else:
    gyro.reset_angle(0)
     
    GyroTurn.turn(180)

    PID_AngleOffSet(base, gyro, 45)
     
    
  collectGreen(225)
  # collect surplus 
  if surplus is None:
    if checkSurplus(-50):
      surplus = Color.GREEN
      collectSurplus(180)
    else:
      surplus = Color.BLUE
      GyroTurn.turn(-180)
      gyro.reset_angle(0)
       
      LineTrack.move(colRight, 60, condition = lambda: leftMotor.angle() < 1000)
      LineTrack.move(colLeft, 60, condition = lambda: colRight.reflection() > 15)
      base.hold()
      GyroStraight.move(-40, colRight.reflection() < 80)
      base.hold()
      GyroTurn.turn(-90)
      collectSurplus(1000, blue = True)
      
  
  solarPanels()
  returnHouse1()
  depositHouse(Houses[0], 1, 1)
  GyroStraight.move(40, condition = lambda: colRight.reflection() > 15)
  GyroStraight.move(40, condition = lambda: colRight.reflection() < 80)
  base.hold()
  GyroTurn.turn(-90)
  scanHouseEV3(Houses[1], ev3Col, 50)
  depositHouse(Houses[1], 1, 2)
  goHouse3()
  scanHouse3(Houses[2], ev3Col, 50)
  depositHouse(Houses[2], 1, 3)
  # always deposit two surplus into battery storage from claw
  depositBattery()
  if Color.YELLOW and Color.BLUE in Houses[2]:
    collectYellow()
    collectBlue()
    depositeHouse(Houses[2], 2, 3)
  elif Color.YELLOW in Houses[2]:
    collectYellow()
    depositeHouse(Houses[2], 2, 3)
    collectBlue()
  elif Color.BLUE in Houses[2]:
    collectBlue()
    depositHouse(Houses[2], 2, 3)
    collectYellow()
  else:
    collectYellow()
    collectBlue()
    
  depositBattery()
  if Color.BLUE or Color.YELLOW in Houses[1]:
    depositHouse(Houses[1], 2, 2)
    
  if Color.BLUE or Color.YELLOW in Houses[0]:
    depositHouse(Houses[0], 2, 1)
  returnBase()

frontClaw.run_time(-100, 1000)
# frontClaw.reset_angle(0)
wait(1000)
frontClaw.run_target(80,350)
#depositHouse([Color.YELLOW, Color.YELLOW], 2, 2)
 