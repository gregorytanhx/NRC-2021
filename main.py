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

try:
  frontClaw = Motor(Port.A)
  backClaw = Motor(Port.D)
except:
  frontClaw = None
  backClaw = None
leftMotor = Motor(Port.B, positive_direction = Direction.COUNTERCLOCKWISE)
rightMotor =  Motor(Port.C)

# ev3Col = Ev3devSensor(Port.S1)
# ev3ColSensor = ColorSensor(Port.S1)

#nxtCol = nxtColorSensor(Port.S1) 
#HTCol = Ev3devSensor(Port.S1) 
# gyro = GyroSensor(Port.S2)
try:
  colLeft = ColorSensor(Port.S3)
except: 
  colLeft = None
colRight = ColorSensor(Port.S4)

stopwatch = StopWatch()
base = Base(leftMotor, rightMotor, colLeft, colRight, frontClaw, backClaw)

LineTrack = PID_LineTrack(base, 0.16, 0, 5, 50)
# GyroStraight = PID_GyroStraight(base, 1.2, 0, 5, gyro)
# GyroTurn = PID_GyroTurn(base, 1.09, 0.0002, 2, gyro)

#gyro.reset_angle(0)
#base.reset()
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
  gyroPID = pid(kp, ki, kd)
  while colRight.reflection() > 15:
    detected = False
    pid.update(gyro.angle(), kp, ki, kd)
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
        pid.update(gyro.angle(), kp, ki, kd)
        base.run(speed - pid.correction, speed + pid.correction)
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
  wait(1000)
  return detected

def collectSurplus(degrees):
  gyro.reset_angle(0)
  
  while ev3ColSensor.reflection() > 1:   
    GyroStraight.move(-40)
  #GyroStraight.base.stop()
  while ev3ColSensor.reflection() < 20:   
    GyroStraight.move(-40)
  base.stop()
  while ev3ColSensor.reflection() > 1:   
    GyroStraight.move(-40)
  base.hold()
  # while ev3ColSensor.reflection() > 1:   
  #   GyroStraight.move(-30)
  # base.stop()
  GyroTurn.turn(90)
  gyro.reset_angle(0)

 
  while colRight.reflection() > 15:
    GyroStraight.move(-30)
  base.stop()
  while colRight.reflection() < 80:
    GyroStraight.move(-30)
  base.stop()
  while colRight.reflection() > 15:
    GyroStraight.move(30)
  base.stop()
  while colRight.reflection() < 80:
    GyroStraight.move(30)
  base.stop()
  
  PID_LineSquare(base, direction = -1)
  base.stop()
  
  frontClaw.run_time(CorrectSpeed(100), 1100)
  frontClaw.reset_angle(0)
  base.reset()
  while leftMotor.angle() < degrees:
    GyroStraight.move(30)
  base.hold()
  wait(1000)

  frontClaw.run_target(CorrectSpeed(-50), -418)
  frontClaw.hold()   

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
  while leftMotor.angle() < 295:
    GyroStraight.move(50)
  GyroTurn.turn(-90)
  gyro.reset_angle(0)
  # wall align
  base.run_time(-100, 1.2)
  gyro.reset_angle(0)

  scanHouseEV3(Houses[0], ev3Col, 50)
  base.hold()

  
  # while colRight.reflection() < 80:
  #   GyroStraight.move(40)
  # base.stop()
  PID_LineSquare(base, direction = -1)
  leftMotor.hold()
  PID_SingleMotorTurn(rightMotor, gyro, -178)
  base.stop()
  

def collectGreen(degrees):
  while colRight.reflection() > 15:
    LineTrack.move(colLeft, 30, side = -1)
  base.hold()
  while colRight.reflection() > 15:
    GyroStraight.move(-30)
  base.hold()
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
  while leftMotor.angle() < degrees:
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
  
def goHouse2():
  GyroTurn.turn(-180, kp = 1.13)
  gyro.reset_angle(0)
  base.reset()
  while leftMotor.angle() < 1200:
    LineTrack.move(colRight, 50)
  base.stop()

  while colRight.reflection() > 15:
    LineTrack.move(colLeft, 70, kp = 0.4, kd = 5.5, side = -1)
  base.stop()

  while col.reflection() < 80:
    GyroStraight.move(40)
  base.stop() 
  while colLeft.reflection() > 15:
    GyroStraight.move(-40)
  base.stop()

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
  while colLeft.reflection() < 80:
    base.run(-40, 0)
  base.stop()
  wait(100)
  base.reset()
  while colLeft.reflection() > 15:
    LineTrack.move(colRight, 40)
  base.stop()
  while colLeft.reflection() < 80:
    LineTrack.move(colRight, 40)
  base.stop()
  wait(100)
  while colRight.reflection() > 15:
    LineTrack.move(colLeft, 40, side = -1)
  base.stop()

  while colRight.reflection() < 80:
    LineTrack.move(colLeft, 40, side = -1)
  base.stop()

  
  GyroTurn.turn(90)
  gyro.reset_angle(0)
  base.reset() 
  # push solar panels
  while leftMotor.angle() < 500:
    LineTrack.move(colLeft, 50)
  base.stop()
  base.reset()
  while leftMotor.angle() < 150:
    GyroStraight.move(40)
  base.stop()
  
  base.reset()
  while leftMotor.angle() > - 300:
    GyroStraight.move(-80)
  base.stop()
  GyroTurn.turn(180)
  
  
def main():
  frontClaw.run_time(-300, 1000)
  checkHouse1()
  
  print(ev3ColSensor.reflection())
  if checkSurplus(-140):
    surplus = Color.YELLOW
    collectSurplus(385)
    while colRight.reflection() > 15:
      GyroStraight.move(-40)
    base.stop()
    base.reset()
    while leftMotor.angle() <= 100:
      GyroStraight.move(40)
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
main()
lastError = 0
start = stopwatch.time()
kp = 0.15
kd = 5
threshold = 50
start = stopwatch.time()
cnt = LineTrack.move(colRight, 50, endCondition = lambda: Button.UP in ev3.buttons.pressed())
print((stopwatch.time() - start)/cnt)
  #PID_LineSquare(base, direction = -1)
# base.frontClaw.run_time(CorrectSpeed(80), 1000)

#main()





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



  