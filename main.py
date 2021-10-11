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
import sys
from helper import *
from pid import *

# declare global variables
Houses = [[], [], []]
numYellow = 4
numBlue = 4
numGreen = 4
numSurplus = 4

# initialise ev3
ev3 = EV3Brick()

# initialise motors
frontClaw = FrontClaw(Port.A)
backClaw = BackClaw(Port.D)
leftMotor = Motor(Port.B, positive_direction = Direction.COUNTERCLOCKWISE)
rightMotor =  Motor(Port.C)

# initialise sensors

ev3Col = Ev3devSensor(Port.S1)
ev3ColSensor = ColorSensor(Port.S1)
gyro = GyroSensor(Port.S2)
colLeft = ColorSensor(Port.S3)
colRight = ColorSensor(Port.S4)

base = Base(leftMotor, rightMotor, colLeft, colRight, frontClaw, backClaw)

# set up defaults for PID functions
LineTrack = PID_LineTrack(base, 0.16, 0, 5, 50)
GyroStraight = PID_GyroStraight(base, 1.2, 0, 5, gyro)
GyroTurn = PID_GyroTurn(base, 1.0, 0.0002, 2.3, gyro)

# battery alert
print(ev3.battery.voltage())
if ev3.battery.voltage() <= 7400:
  print('LOW BATTERY')
  ev3.speaker.beep()
  sys.exit()

def scanHouseEV3(house, sensor, speed, condition):   
  # initialise pid for gyrostraight
  kp, ki, kd = GyroStraight.kp, GyroStraight.ki, GyroStraight.kd
  gyro.reset_angle(0)
  gyroPID = PID(kp, ki, kd)
  
  while condition():
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
    
    # once an indicator has been detected, move until nothing is detected
    if detected:
      while r + g + b > 12:
        r, g, b = sensor.read('RGB-RAW')
        gyroPID.update(gyro.angle(), kp, ki, kd)
        base.run(speed - gyroPID.correction, speed + gyroPID.correction)
      detected = False

  print(house)
  base.hold()

def checkSurplus(degrees):
  # reverse for certain amount of degrees to check if surplus is present
  speed = -40
  kp, ki, kd = GyroStraight.kp, GyroStraight.ki, GyroStraight.kd
  gyroPID = PID(kp, ki, kd)
  base.reset()
  detected = False
  while leftMotor.angle() >= degrees:
    # r, g, b = ev3Col.read('RGB-RAW')
    gyroPID.update(gyro.angle(), kp, ki, kd)
    base.run(speed - gyroPID.correction, speed + gyroPID.correction)
    if ev3ColSensor.reflection() > 0:
      detected = True
  base.hold()
  return detected

def collectSurplus(degrees, col):
  base.reset()
  frontClaw.run_target(60, 200)
  frontClaw.reset(1500)
  if col != Color.BLUE:
    GyroStraight.move(-30, condition = lambda: leftMotor.angle() > -160)
    GyroTurn.turn(90)
    gyro.reset_angle(0)

  GyroStraight.move(-40, condition = lambda: colRight.reflection() < 70)
  base.hold()

  PID_LineSquare(base, direction = -1)
  gyro.reset_angle(0)
  base.reset()

  #move forward to collect surplus
  GyroStraight.move(30, condition = lambda: leftMotor.angle() < degrees)
  base.hold()
  # grab front surplus using claw
  frontClaw.run_target(-40, -700)
  base.reset()
  GyroStraight.move(40, condition = lambda: leftMotor.angle() < 350)
  base.hold()
  frontClaw.run_target(30, 220)
  
def collectGreen():  
  backClaw.run_time(100, 1200, wait = False)
  LineTrack.move(colRight, 50, side = -1, condition = lambda: colLeft.color() != Color.BLACK)
  base.reset()
  LineTrack.move(colRight, 40, side = -1, condition = lambda: leftMotor.angle() < 210)
  base.hold()

  GyroTurn.turn(-90)
  while colRight.reflection() < 60:
    base.run(-40, -40)
  base.hold()
  
  PID_LineSquare(base, direction = -1)
  gyro.reset_angle(0)
   
  # move forward, lower claw then reverse to collect green
  base.run_target(40, 200)
  backClaw.mid()
  GyroStraight.move(-30, condition = lambda: colRight.reflection() < 70)
  GyroStraight.move(-30, condition = lambda: colRight.reflection() > 15)
  base.hold()

  # reset claw again
  backClaw.run_time(100, 1200, wait = False)
  
  # linetrack and turn to grab other 2 green
  GyroStraight.move(40, condition = lambda: colRight.reflection() < 70)
  GyroStraight.move(40, condition = lambda: colRight.reflection() > 40)
  base.hold()
  GyroTurn.turn(90)
  base.reset()
  GyroStraight.move(40, condition = lambda: leftMotor.angle() <= 360)
  base.hold()

  GyroTurn.turn(-90)
  while colRight.reflection() < 70:
    base.run(-40, -40)
  base.hold()
  
  PID_LineSquare(base, direction = -1)

  gyro.reset_angle(0)
  base.reset()
  GyroStraight.move(40, condition = lambda: leftMotor.angle() < 200)
  base.hold()
  backClaw.mid()
  GyroStraight.move(-40, condition = lambda: colRight.reflection() < 60)
  base.hold()
  backClaw.run_target(50, 80)
  
  # cap speed of turns after grabbing green to stop them from jerking
  GyroTurn.maxSpeed = 40
  GyroTurn.turn(-90)
  wait(2000)

def returnHouse1():
  pass

def depositHouse(house, time, houseNum):
  global numGreen, numBlue, numYellow, numSurplus
  
  if time == 1:
    # first visit to house deposits surplus and green
    numCol = numGreen
    RingCol = Color.GREEN
    if len(house) == 1: # surplus to be deposited
      # only 2 surplus ever need to be deposited
      
      if houseNum == 1:
        GyroTurn.turn(-90)
      else:
        GyroTurn.turn(90)
             
      # lift claw, move forward then reverse to deposit surplus from within catchment area
      frontClaw.run_target(-70, -300)
      LineTrack.move(colRight, 40, condition = lambda: colLeft.color() != Color.RED)
      base.reset()
      GyroStraight.move(40, condition = lambda: leftMotor.angle() < 100)  
      base.reset()
      GyroStraight.move(-40, condition = lambda: leftMotor.angle() > -300)
              
  else:
    # second visit to house deposits yellow and blue
    numCol = numBlue
    RingCol = Color.BLUE
  
    if Color.YELLOW in house: 
      # check number of yellow in house
      tmp = 1
      if len(house) == 2:
        if house[0] == Color.YELLOW and house[1] == Color.YELLOW:
          tmp = 2
      if houseNum == 1:
        GyroTurn.turn(-90)
      if houseNum == 2:
        GyroTurn.turn(90)
      
      if tmp == 2:
        # deposit both yellow
        frontClaw.reset(1000)
        LineTrack.move(colRight, 50, condition = lambda: colLeft.color() != Color.RED)
        base.hold()
      
      elif tmp == 1 and numYellow == 2:
        # deposit yellow from claw
        LineTrack.move(colRight, 40, condition = lambda: colLeft.color() != Color.RED)
        base.hold()
        frontClaw.run_target(70, 200)
        
      else:
        # deposit yellow from catchment area
        base.reset()
        frontClaw.run_target(-70, -300)
        LineTrack.move(colRight, 40, condition = lambda: colLeft.color() != Color.RED)
        base.reset()
        GyroStraight.move(40, condition = lambda: leftMotor.angle() < 100)
        base.hold()

      base.reset()
      GyroStraight.move(-40, condition = lambda: leftMotor.angle() > -200)
      base.hold()
      
  if RingCol not in house:
    if houseNum ==  1 or houseNum ==  2:
      GyroStraight.move(-40, condition = lambda: colLeft.reflection() > 15 or colRight.reflection() > 15)
      base.hold()
      if houseNum ==  1:
        GyroTurn.turn(-90)
      elif houseNum ==  2: 
        GyroTurn.turn(90)
        
    else:
      GyroStraight.move(-40, condition = lambda: leftMotor.angle() > -500)
      base.hold()
      GyroTurn.turn(180)
    
    frontClaw.run_target(30, 300, wait=False)
  
  else: 
    # green/blue to be deposited
    tmp = 1
    
    # check number of green/blue indicators for house
    if len(house) == 2:
      if house[0] == RingCol and house[1] == RingCol:
        tmp = 2

    if (time == 1 and len(house) == 2) or (time == 2 and Color.YELLOW not in house):
      if houseNum == 1:
        GyroTurn.turn(90)
        
      elif houseNum == 2 or (houseNum == 3 and time == 1):
        GyroTurn.turn(-90)
        
    else:
      GyroTurn.turn(180)      
    
    GyroStraight.move(-40, condition = lambda: colRight.color() != Color.BLACK and colLeft.color() != Color.BLACK)
    base.hold()
    
    base.reset()
    GyroStraight.move(-40, condition = lambda: leftMotor.angle() > -120)
    base.hold()
    
    backClaw.run_target(-20, -130)
    base.reset()
    # reverse more if 2 ring blocks have to be deposited
    
    if numCol == 4 and tmp == 1:
      GyroStraight.move(20, condition = lambda: leftMotor.angle() < 80) 
      if RingCol == Color.GREEN:
        numGreen -= 2
      else:
        numBlue -= 2
    else:
      GyroStraight.move(30, condition = lambda: leftMotor.angle() < 150)
      if RingCol == Color.GREEN:
        numGreen -= 4
      else:
        numBlue -= 4
        

    base.hold()
    backClaw.run_target(20, 130)

    if houseNum == 1 or houseNum == 2:
      GyroStraight.move(40, condition = lambda: colLeft.color() != Color.BLACK and colRight.color() != Color.BLACK)
      base.hold()
      base.reset()
      GyroStraight.move(40, condition = lambda: leftMotor.angle() < 150)
      base.hold()
      if houseNum == 1:
        GyroTurn.turn(90)
      else:
        GyroTurn.turn(-90)
         
    # if no more ring blocks are on the bot, reset the maximum turn speed
    if numCol == 0:
      GyroTurn.maxspeed = 100

def collectBlue():
  backClaw.reset(500, speed = 80)
  PID_LineSquare(base)
  gyro.reset_angle(0)
  base.reset()
  GyroStraight.move(-40, condition =  lambda: colRight.color() != Color.WHITE)
  base.hold()
  GyroTurn.turn(-90)
  backClaw.run_target(-50, -230)
  base.reset()
  GyroStraight.move(-20, condition = lambda: leftMotor.angle() > -80)
  base.hold()
  backClaw.run_target(-50, -30)
  base.reset()
  GyroStraight.move(-10, condition = lambda: leftMotor.angle() > -15)
  base.hold()
  backClaw.reset(500, speed = 80)
  base.reset()
  GyroStraight.move(50, condition = lambda: leftMotor.angle() < 95)
  base.hold()
  GyroTurn.turn(90)
  GyroStraight.move(40, condition =  lambda: colRight.color() != Color.BLACK)
  base.reset()
  GyroStraight.move(40, condition =  lambda: leftMotor.angle() < 250)
  base.hold()
  GyroTurn.turn(-90)
  backClaw.run_target(-50, -230)
  base.reset()
  GyroStraight.move(-10, condition = lambda: leftMotor.angle() > -50)
  base.hold()
  backClaw.run_target(50, 90)
    
def depositBatteryFront():
  frontClaw.run_target(-50, -300)
  base.reset()
  GyroStraight.move(40, condition = lambda: leftMotor.angle() < 230)
  base.hold()
  frontClaw.run_target(50, 300)
  GyroStraight.move(-40, condition = lambda: colRight.color() != Color.BLACK or colLeft.color() != Color.BLACK)
  base.hold()

def depositBatteryBack():
  GyroTurn.turn(180)
  base.reset()
  GyroStraight.move(-40, condition = lambda: leftMotor.angle() > -150)
  base.hold()
  wait(50)
  backClaw.run_target(-50, -100)
  base.reset()
  GyroStraight.move(40, condition = lambda: leftMotor.angle() < 40)
  base.hold()
  backClaw.run_target(50, 100)
  
def collectYellow():
  # push solar panels
  base.reset()
  frontClaw.reset(1500)
  frontClaw.run_target(-50, -390)
  frontClaw.hold()
  LineTrack.move(colRight, 40, condition = lambda: leftMotor.angle() < 200)
  while colLeft.color() != Color.BLACK or colRight.color() != Color.BLACK:
    base.run(20, 20)
  base.reset()
  while leftMotor.angle() < 100:
    base.run(20, 20)
  base.hold()
  frontClaw.reset(1500, dir = -1, speed = 40)
  
  # track to first 2 yellow and grab with claw
  GyroTurn.turn(-90)
  base.reset()
  LineTrack.move(colRight, 40, condition = lambda: leftMotor.angle() < 500)
  GyroStraight.move(40, condition = lambda: leftMotor.angle() < 600)
  base.hold()
  frontClaw.reset(1500)
  GyroTurn.turn(90)
  base.reset()
  GyroStraight.move(20, condition = lambda: leftMotor.angle() < 30)
  base.hold()
  frontClaw.run_target(-50, -470)
  base.reset()
  GyroStraight.move(-40, condition = lambda: leftMotor.angle() > -60)
  base.hold()
    
  # collect next 2 in catchment area
  GyroTurn.turn(90)
  LineTrack.move(colLeft, 50, condition = lambda: colRight.color() != Color.BLACK)
  base.reset()  
  LineTrack.move(colLeft, 40, condition = lambda: leftMotor.angle() < 550)
  GyroStraight.move(40, condition = lambda: leftMotor.angle() < 750)
  base.hold()
  frontClaw.run_target(-40, -250)
  GyroTurn.turn(-90)
  base.reset()
  GyroStraight.move(40, condition = lambda: leftMotor.angle() < 200)
  base.hold()
  frontClaw.run_target(30, 250)
  GyroTurn.turn(90)
  GyroStraight.move(60, condition=lambda: colRight.color() != Color.BLACK)
  base.hold()
  
def getExtra():
  cols = {Color.YELLOW: 0, Color.GREEN: 0, Color.BLUE: 0}
  for house in Houses:
    for col in house:
      cols[col] += 1
  
  for key in cols:
    if cols[key] == 1:
      return key
  
def main():
  surplus = None
  gyro.reset_angle(0)
  base.reset()
  GyroStraight.move(70, condition = lambda: leftMotor.angle() < 300)   
  LineTrack.move(colRight, 60, side = -1, condition = lambda: colLeft.color() != Color.BLACK)   
  base.hold()
  base.reset()
  GyroStraight.move(50, condition = lambda: leftMotor.angle() < 305)
  base.hold()
  GyroTurn.turn(-90)
  
  # wall align and scan indicators 
  base.run_time(-100, 800)
  gyro.reset_angle(0)  
  scanHouseEV3(Houses[0], ev3Col, 50, lambda: colRight.color() != Color.BLACK)
  PID_LineSquare(base, direction=-1)
  gyro.reset_angle(0)
  PID_SingleMotorTurn(rightMotor, gyro, -178)

  
  # if yellow surplus is present, collect it 
  # move toward green energy
  if checkSurplus(-100):
    surplus = Color.YELLOW
    collectSurplus(200, Color.YELLOW)
    base.reset()
    GyroStraight.move(-40, condition = lambda: colRight.color() != Color.BLACK)
    base.hold()
    base.reset()
    GyroStraight.move(40, condition = lambda: leftMotor.angle() < 200)
    base.hold()
    GyroTurn.turn(90)

  else:
    gyro.reset_angle(0)
    GyroTurn.turn(179)
    PID_AngleOffSet(base, gyro, 30)
    
  # collect green energy
  collectGreen()
  
  # collect green surplus if present, else go collect blue surplus
  if surplus is None:
    if checkSurplus(-200):
      surplus = Color.GREEN
      collectSurplus(210, Color.GREEN)
      PID_SingleMotorTurn(leftMotor, gyro, -90, direction = -1)
      base.hold()  
      wait(1000)
    else:
      surplus = Color.BLUE
      GyroTurn.turn(180)
      gyro.reset_angle(0)
      base.reset()
      LineTrack.move(colLeft, 60, condition = lambda: colRight.color() != Color.BLACK)
      base.hold()
      base.reset()
      GyroStraight.move(-30, condition = lambda: leftMotor.angle() > -150)
      base.hold()
      GyroTurn.turn(-90)
      collectSurplus(610, Color.BLUE)

      GyroStraight.move(-60, condition = lambda: colLeft.color() != Color.WHITE)
      base.hold()
      GyroTurn.turn(-90)
      
    LineTrack.move(colRight, 60, side = -1,  condition = lambda: colLeft.color() != Color.BLACK)
    LineTrack.move(colRight, 70, side = -1, condition = lambda: colLeft.color() != Color.WHITE)
      
  else:
    gyro.reset_angle(0)
    LineTrack.move(colRight, 70, side = -1,  condition = lambda: leftMotor.angle() < 500)

  LineTrack.move(colRight, 70, side = -1, condition = lambda: colLeft.color() != Color.BLACK)
  base.reset() 
  LineTrack.move(colRight, 40, side = -1,  condition = lambda: leftMotor.angle() < 90)
  base.hold()
  
  # start first round of deposition (green + surplus)
  depositHouse(Houses[0], 1, 1)
  
  LineTrack.move(colRight, 70, side = -1, condition = lambda: colLeft.color() != Color.BLACK)
  LineTrack.move(colRight, 70, side = -1, condition = lambda: colLeft.color() != Color.WHITE)
  LineTrack.move(colRight, 70, side = -1, condition = lambda: colLeft.color() != Color.BLACK)
  base.reset()
  LineTrack.move(colRight, 60, side = -1, condition = lambda: leftMotor.angle() < 650)
  base.hold()
  PID_AngleOffSet(base, gyro, 80)
  base.hold()
  GyroTurn.maxSpeed = 40
  PID_LineSquare(base, direction = -1)
  gyro.reset_angle(0)
  base.reset()
  GyroStraight.move(-50, condition = lambda: leftMotor.angle() > -300)
  base.hold()
  scanHouseEV3(Houses[1], ev3Col, 50)  
  GyroStraight.move(40, condition = lambda: colLeft.reflection() > 76)
  base.hold()
  depositHouse(Houses[1], 1, 2)
  
  LineTrack.move(colLeft, 50, condition = lambda: colRight.color() != Color.BLACK)
  base.reset()
  LineTrack.move(colLeft, 40, condition = lambda: leftMotor.angle() < 100)
  base.hold()
  GyroTurn.turn(90)
  LineTrack.move(colRight, 50, condition = lambda: colLeft.color() != Color.RED)
  base.hold()
  base.reset()
  GyroTurn.move(-40, condition = lambda: leftMotor.angle() > -150)
  base.hold()
  GyroTurn.turn(-90)
  base.reset()
  GyroStraight.move(-40, condition = lambda: leftMotor.angle() > -50)
  base.hold()
  PID_LineSquare(base, direction = -1)
  gyro.reset_angle(0)
  base.reset()
  GyroStraight.move(-50, condition = lambda: leftMotor.angle() > -300)
  base.hold()
  scanHouseEV3(Houses[2], ev3Col, 50, lambda: colLeft.reflection() > 15)
  GyroStraight.move(40, condition = lambda: colLeft.reflection() > 76)
  base.hold()
  depositHouse(Houses[2], 1, 3)

  # always deposit two surplus into battery storage from claw, deposit any remaining green
  extraCol = getExtra()
  depositBatteryFront()
  if extraCol == Color.GREEN:
    depositBatteryBack()
  
  # collect yellow and blue energy
  collectBlue()  
  collectYellow()
  if Color.YELLOW or Color.BLUE in Houses[2]:
    depositHouse(Houses[2], 2, 3)
  
  # based on houses, determine which energy is extra and deposit it
  if extraCol == Color.YELLOW:
    depositBatteryFront()
  elif extraCol == Color.BLUE:
    depositBatteryBack()
  
  # clear remaining houses 
  if Color.BLUE or Color.YELLOW in Houses[1]:
    depositHouse(Houses[1], 2, 2)
    
  if Color.BLUE or Color.YELLOW in Houses[0]:
    depositHouse(Houses[0], 2, 1)
    
  # go back to base
  returnBase()
  



frontClaw.reset(1500, dir = -1)
backClaw.reset(1000)
main()

#PID_SingleMotorTurn(leftMotor, gyro, -90, direction = -1)

#
# # backClaw.defaultPos()
# # wait(2000)
# depositHouse( [Color.BLUE, Color.GREEN], 2, 1,)

# base.reset()
# LineTrack.move(colRight, 50, condition=lambda: leftMotor.angle() < 400)
# GyroStraight.move(40, condition = lambda: colRight.color() != Color.BLACK or colLeft.color() != Color.BLACK)
# base.hold()
# wait(1000)

# base.reset()
# LineTrack.move(colRight, 70, side = -1,  condition = lambda: leftMotor.angle() < 500)
# LineTrack.move(colRight, 70, side = -1, condition = lambda: colLeft.color() != Color.BLACK)
# base.hold()
# wait(1000) 
# wait(4000)
# backClaw.run_target(-20, -140)
# base.reset()
# wait(3000)
# GyroStraight.move(20, condition = lambda: leftMotor.angle() < 200)
# base.hold()
# backClaw.run_target(20, 140)


# scanHouseEV3(Houses[1], ev3Col, 50, lambda: colRight.reflection() > 15)  
# base.hold()
# GyroStraight.move(40, condition = lambda: colRight.reflection() < 80)
# GyroStraight.move(40, condition = lambda: colRight.reflection() > 40)
# base.hold()
# depositHouse(Houses[1], 1, 2)
# # SHOULD SHIFT SOLAR PANELS TO PART FOR COLLECTING YELLOW FOR BETTER ROUTING