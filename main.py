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
GyroTurn = PID_GyroTurn(base, 0.9, 0.0001, 1.5, gyro)

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
    if r + g + b >= 15:
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
  frontClaw.reset()
  if col != Color.BLUE:
    GyroStraight.move(-30, condition = lambda: leftMotor.angle() > -160)
    GyroTurn.turn(89)
    gyro.reset_angle(0)

  GyroStraight.move(-40, condition = lambda: colLeft.color() != Color.BLACK)
  base.hold()
  wait(100)
  base.reset()
  GyroStraight.move(40, condition = lambda: leftMotor.angle() < 50)
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
  LineTrack.move(colRight, 40, side = -1, condition = lambda: colLeft.color() != Color.BLACK)
  base.reset()
  LineTrack.move(colRight, 40, side = -1, condition = lambda: leftMotor.angle() < 225)
  base.hold()

  GyroTurn.turn(-89)
  GyroStraight.move(-40, condition = lambda: colRight.color() != Color.WHITE)
  base.hold()
  
  PID_LineSquare(base, direction = -1)
  gyro.reset_angle(0)
   
  # move forward, lower claw then reverse to collect green
  base.run_target(40, 200)
  backClaw.mid()
  GyroStraight.move(-30, condition = lambda: colRight.color() != Color.WHITE)
  GyroStraight.move(-30, condition = lambda: colRight.color() != Color.BLACK)
  base.hold()

  # reset claw again
  backClaw.run_time(100, 1200, wait = False)
  
  # linetrack and turn to grab other 2 green
  base.reset()
  GyroStraight.move(40, condition = lambda: leftMotor.angle() < 150)
  base.hold()
  GyroTurn.turn(89)
  base.reset()
  GyroStraight.move(40, condition = lambda: leftMotor.angle() < 355)
  base.hold()

  GyroTurn.turn(-89)

  gyro.reset_angle(0)
  base.reset()
  GyroStraight.move(40, condition = lambda: leftMotor.angle() < 120)
  base.hold()
  backClaw.mid()
  base.reset()
  GyroStraight.move(-30, condition = lambda: leftMotor.angle() > -130)
  base.hold()
  base.reset()
  # GyroStraight.move(30, condition = lambda: leftMotor.angle() < 20)
  # base.hold()
  backClaw.run_target(30, 80)
  base.reset()
  # GyroStraight.move(-30, condition = lambda: leftMotor.angle() > -20)
  # base.hold()
  # cap speed of turns after grabbing green to stop them from jerking
  GyroTurn.maxSpeed = 40
  
def depositHouse(house, time, houseNum):
  # TO DO 
  # add variable degree for cases when bot starts closer to house
  global numGreen, numBlue, numYellow, numSurplus
  
  if time == 1:
    # first visit to house deposits surplus and green
    numCol = numGreen
    RingCol = Color.GREEN
    if len(house) == 1: # surplus to be deposited
      # only 2 surplus ever need to be deposited
      
      if houseNum == 1:
        GyroTurn.turn(-89)
      else:
        GyroTurn.turn(89)
             
      # lift claw, move forward then reverse to deposit surplus from within catchment area
      frontClaw.run_target(-70, -300)
      GyroStraight.move(40, condition = lambda: colLeft.color() != Color.RED)
      base.reset()
      GyroStraight.move(40, condition = lambda: leftMotor.angle() < 100)  
      base.reset()
      GyroStraight.move(-40, condition = lambda: leftMotor.angle() > -350)
      base.hold()
              
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
        GyroTurn.turn(-89)
      if houseNum == 2:
        GyroTurn.turn(89)
      
      if tmp == 2:
        # deposit both yellow
        frontClaw.dc()
        wait(1000)
        GyroStraight.move(50, condition = lambda: colLeft.color() != Color.RED)
        base.hold()
      
      elif tmp == 1 and numYellow == 2:
        # deposit yellow from claw
        GyroStraight.move(40, condition = lambda: colLeft.color() != Color.RED)
        base.hold()
        frontClaw.run_target(70, 200)
        
      else:
        # deposit yellow from catchment area
        base.reset()
        frontClaw.run_target(-70, -300)
        GyroStraight.move(40, condition = lambda: colLeft.color() != Color.RED)
        base.reset()
        GyroStraight.move(40, condition = lambda: leftMotor.angle() < 100)
        base.hold()

      base.reset()
      GyroStraight.move(-40, condition = lambda: leftMotor.angle() > -200)
      base.hold()
      
  if RingCol not in house:
    if houseNum ==  1 or houseNum ==  2:
      GyroStraight.move(-40, condition = lambda: colLeft.color() != Color.BLACK or colRight.color() != Color.BLACK)
      base.hold()
      base.reset()
      GyroStraight.move(40, condition = lambda: leftMotor.angle() < 150)
      base.hold()
      if houseNum ==  1:
        GyroTurn.turn(-89)
      elif houseNum ==  2: 
        GyroTurn.turn(89)
        
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
        GyroTurn.turn(89)
        
      elif houseNum == 2 or (houseNum == 3 and time == 1):
        GyroTurn.turn(-89)
        
    else:
      GyroTurn.turn(180)      
    
    base.reset()
    if houseNum == 1 or (houseNum == 2 and time == 2):
      GyroStraight.move(-40, condition = lambda: leftMotor.angle() > -200)
    else:
      GyroStraight.move(-40, condition = lambda: leftMotor.angle() > -50)
    base.hold()
    
    backClaw.run_target(-20, -130)

    base.reset()
    # go more if 2 ring blocks have to be deposited
    if numCol == 4 and tmp == 1:
      GyroStraight.move(20, condition = lambda: leftMotor.angle() < 80) 
      if RingCol == Color.GREEN:
        numGreen -= 2
      else:
        numBlue -= 2
    else:
      GyroStraight.move(20, condition = lambda: leftMotor.angle() < 170)
      if RingCol == Color.GREEN:
        numGreen = 0
      else:
        numBlue = 0

    base.hold()
    
    backClaw.run_target(20, 130)
    GyroStraight.move(-40, condition = lambda: leftMotor.angle() > 0)
    base.hold()
    if houseNum == 1 or houseNum == 2:
      GyroStraight.move(40, condition = lambda: colLeft.color() != Color.BLACK or colRight.color() != Color.BLACK)
      base.hold()
      base.reset()
      GyroStraight.move(40, condition = lambda: leftMotor.angle() < 90)
      base.hold()
      if houseNum == 1:
        GyroTurn.turn(89)
      else:
        GyroTurn.turn(-89)
         
    # if no more ring blocks are on the bot, reset the maximum turn speed
    if numCol == 0:
      GyroTurn.maxspeed = 100

def collectBlue():
  # line track to intersection at house 2 
  GyroStraight.move(50, condition=lambda: colRight.color() != Color.BLACK)
  base.hold()
  base.reset()
  GyroStraight.move(40, condition = lambda: leftMotor.angle() < 150)
  base.hold()
  GyroTurn.turn(89)
  LineTrack.move(colRight, 40, side = -1, condition =  lambda: colLeft.color() != Color.BLACK)
  base.hold()
  base.reset()
  GyroStraight.move(-40, condition = lambda: leftMotor.angle() > -80)
  base.hold()
  PID_SingleMotorTurn(base, gyro, -89, 0, 1)
  base.reset()
  LineTrack.move(colRight, 45, side = -1, condition = lambda: leftMotor.angle() < 200)
  LineTrack.move(colRight, 45, side = -1, condition =  lambda: colLeft.color() != Color.BLACK)
  base.hold()
  base.reset()
  GyroStraight.move(-40, condition = lambda: leftMotor.angle() > -75)
  base.hold()
  GyroTurn.turn(-89)
  base.reset()
  
  # move to blue energy area
  GyroStraight.move(50, condition = lambda: colRight.color() != Color.BLACK)
  base.hold()
  base.reset()
  GyroStraight.move(-40, condition = lambda: leftMotor.angle() > -40)
  base.hold()
  backClaw.dc(speed = 80)
  PID_LineSquare(base)
  gyro.reset_angle(0)
  
  
  # collect blue energy
  GyroStraight.move(-40, condition =  lambda: colRight.color() != Color.WHITE)
  base.hold()
  GyroTurn.turn(-89)
  backClaw.run_target(-50, -230)
  base.reset()
  GyroStraight.move(-20, condition = lambda: leftMotor.angle() > -80)
  base.hold()
  backClaw.run_target(-50, -30)
  base.reset()
  GyroStraight.move(-10, condition = lambda: leftMotor.angle() > -30)
  base.hold()
  backClaw.run_target(50, 200)
  base.reset()
  GyroStraight.move(50, condition = lambda: leftMotor.angle() < 95)
  base.hold()
  GyroTurn.turn(89)
  GyroStraight.move(40, condition =  lambda: colRight.color() != Color.BLACK)
  base.reset()
  GyroStraight.move(40, condition =  lambda: leftMotor.angle() < 250)
  base.hold()
  GyroTurn.turn(-89)
  backClaw.run_target(-30, -175)
  base.reset()
  GyroStraight.move(-5, condition = lambda: leftMotor.angle() > -60)
  base.hold()
  backClaw.run_target(30, 90)
    
def depositBatteryFront():
  frontClaw.run_target(-50, -300)
  base.reset()
  GyroStraight.move(40, condition = lambda: leftMotor.angle() < 165)
  base.hold()
  frontClaw.run_target(50, 300)
  
  GyroStraight.move(-10, condition = lambda: colRight.color() != Color.BLACK or colLeft.color() != Color.BLACK)
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

  frontClaw.dc()
  wait(1500)
  frontClaw.run_target(-40, -440)
  frontClaw.hold()
  base.reset()
  LineTrack.move(colRight, 40, condition = lambda: leftMotor.angle() < 300)
  base.hold()
  GyroStraight.move(15, condition = lambda: colLeft.color() != Color.BLACK or colRight.color() != Color.BLACK)
  GyroStraight.move(15, condition = lambda: colLeft.color() != Color.WHITE or colRight.color() != Color.WHITE)
  base.reset()
  frontClaw.run_target(40, 10)
  GyroStraight.move(15, condition = lambda: leftMotor.angle() < 50)
  base.hold()
  wait(100)

  frontClaw.run_target(40,150)
  base.reset()
  

  frontClaw.reset(dir = -1, deg = 500)
  
  # track to first 2 yellow and grab with claw
  GyroTurn.turn(-89)
  base.reset()
  LineTrack.move(colRight,  40, side = -1, condition = lambda: leftMotor.angle() < 500)
  GyroStraight.move(40, condition = lambda: leftMotor.angle() < 610)
  base.hold()
  frontClaw.reset()
  GyroTurn.turn(89)
  base.reset()
  GyroStraight.move(20, condition = lambda: leftMotor.angle() < 70)
  base.hold()
  frontClaw.run_target(-40, -470)
  base.reset()
  GyroStraight.move(-40, condition = lambda: leftMotor.angle() > -30)
  base.hold()
    
  # collect next 2 in catchment area
  GyroTurn.turn(89)
  LineTrack.move(colLeft, 50, condition = lambda: colRight.color() != Color.BLACK)
  base.reset()  
  LineTrack.move(colLeft, 40, condition = lambda: leftMotor.angle() < 500)
  GyroStraight.move(40, condition = lambda: leftMotor.angle() < 750)
  base.hold()
  frontClaw.run_target(-40, -260)
  GyroTurn.turn(-90)
  base.reset()
  GyroStraight.move(40, condition = lambda: leftMotor.angle() < 200)
  base.hold()
  frontClaw.run_target(30, 260)
  GyroTurn.turn(89)
  
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
  LineTrack.move(colRight, 55, side = -1, condition = lambda: colLeft.color() != Color.BLACK)   
  base.hold()
  base.reset()
  GyroStraight.move(40, condition = lambda: leftMotor.angle() < 305)
  base.hold()
  GyroTurn.turn(-89)
  
  # wall align and scan indicators 
  base.run_time(-100, 800)
  gyro.reset_angle(0)  
  scanHouseEV3(Houses[0], ev3Col, 50, lambda: colRight.color() != Color.BLACK)
  PID_LineSquare(base, direction=-1)
  gyro.reset_angle(0)
  PID_SingleMotorTurn(base, gyro, -179, 0.06, 1)

  
  # if yellow surplus is present, collect it 
  # move toward green energy
  if checkSurplus(-160):
    surplus = Color.YELLOW
    collectSurplus(190, Color.YELLOW)
    base.reset()
    GyroStraight.move(-40, condition = lambda: colRight.color() != Color.BLACK)
    base.hold()
    base.reset()
    GyroStraight.move(40, condition = lambda: leftMotor.angle() < 210)
    base.hold()
    GyroTurn.turn(89)

  else:
    gyro.reset_angle(0)
    GyroTurn.turn(179)
    PID_AngleOffSet(base, gyro, 30)
    
  # collect green energy
  collectGreen()
  
  # collect green surplus if present, else go collect blue surplus
  if surplus is None:
    GyroTurn.turn(-89)
    if checkSurplus(-170):
      surplus = Color.GREEN
      collectSurplus(30, Color.GREEN)
      base.reset()
      GyroStraight.move(-40, condition=lambda: leftMotor.angle() > -60)
      base.hold()
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
      GyroTurn.turn(-89)
      collectSurplus(430, Color.BLUE)

      GyroStraight.move(-60, condition = lambda: colLeft.color() != Color.BLACK)
      base.hold()
      GyroStraight.move(40, condition = lambda: colLeft.color() != Color.BLACK)
      base.hold()
      base.reset()
      GyroStraight.move(40, condition = lambda: leftMotor.angle() < 150)
      base.hold()
    

  # check whether to deposit in house 1
  if Color.GREEN in Houses[0] or len(Houses[0]) == 1: 
    if surplus == Color.BLUE:
      
      GyroTurn.turn(-89)
      LineTrack.move(colRight, 60, side = -1,  condition = lambda: colLeft.color() != Color.BLACK)
      LineTrack.move(colRight, 70, side = -1, condition = lambda: colLeft.color() != Color.WHITE)
    else:
      if surplus == Color.GREEN:
        PID_SingleMotorTurn(base, gyro, -90, 1, 0)
        base.hold()  
      elif surplus == Color.YELLOW:
        GyroTurn.turn(-89)
      
      gyro.reset_angle(0)
      LineTrack.move(colRight, 70, side = -1,  condition = lambda: leftMotor.angle() < 500)
    
    LineTrack.move(colRight, 70, side = -1, condition = lambda: colLeft.color() != Color.BLACK)
    base.reset() 
    LineTrack.move(colRight, 40, side = -1,  condition = lambda: leftMotor.angle() < 90)
    base.hold()

    depositHouse(Houses[0], 1, 1)
    
    # return to house 2 intersection
    LineTrack.move(colRight, 70, side = -1, condition = lambda: colLeft.color() != Color.BLACK)
    LineTrack.move(colRight, 70, side = -1, condition = lambda: colLeft.color() != Color.WHITE)
    
  
  else:
    # turn to face house 2 from green surplus area if not blue surplus
    if surplus == Color.GREEN:
      PID_SingleMotorTurn(base, gyro, 89, 0, 1)
      base.hold()  
    elif surplus == Color.YELLOW:
      base.reset()
      GyroStraight.move(40, condition = lambda: leftMotor.angle() < 80)
      base.hold()
      GyroTurn.turn(89)
       
        
  # scan house 2
  if surplus != Color.BLUE:
    LineTrack.move(colRight, 70, side = -1, condition = lambda: colLeft.color() != Color.BLACK)
    base.reset()
    LineTrack.move(colRight, 60, side = -1, condition = lambda: leftMotor.angle() < 650)
    base.hold()
    PID_AngleOffSet(base, gyro, 80)
    base.hold()
    GyroStraight.move(-40, condition = lambda: colLeft.color() != Color.WHITE)
    
    base.hold()
    PID_LineSquare(base, direction = -1)
    gyro.reset_angle(0)
    base.reset()
    GyroStraight.move(-50, condition = lambda: leftMotor.angle() > -300)
    base.hold()
    
  else:
    PID_SingleMotorTurn(base, gyro, 89, 0, 1)
    
  scanHouseEV3(Houses[1], ev3Col, 50, lambda: colRight.color() != Color.BLACK)  
  base.reset()
  GyroStraight.move(40, condition = lambda: leftMotor.angle() < 150)
  base.hold()
  
  # deposit at house 2 
  if Color.GREEN in Houses[1] or len(Houses[1]) == 1:
    depositHouse(Houses[1], 1, 2)
  else:
    GyroTurn.turn(-89)
    LineTrack.move(colLeft, 40, condition = lambda: colRight.color() != Color.BLACK())
    base.reset()
    LineTrack.move(colLeft, 40, condition = lambda: leftMotor.angle() < 100)
    GyroTurn.turn(-89)
    
  
  # move to house 3 and scan
  LineTrack.move(colLeft, 50, condition = lambda: colRight.color() != Color.BLACK)
  base.reset()
  LineTrack.move(colLeft, 40, condition = lambda: leftMotor.angle() < 100)
  base.hold()
  GyroTurn.turn(89)
  LineTrack.move(colRight, 50, condition = lambda: colLeft.color() != Color.RED)
  base.hold()
  base.reset()
  GyroTurn.move(-40, condition = lambda: leftMotor.angle() > -150)
  base.hold()
  GyroTurn.turn(-89)
  base.reset()
  GyroStraight.move(-40, condition = lambda: leftMotor.angle() > -50)
  base.hold()
  PID_LineSquare(base, direction = -1)
  gyro.reset_angle(0)
  base.reset()
  GyroStraight.move(-50, condition = lambda: leftMotor.angle() > -300)
  base.hold()
  scanHouseEV3(Houses[2], ev3Col, 50, lambda: colLeft.color() != Color.BLACK)
  base.reset()
  GyroStraight.move(40, condition = lambda: leftMotor.angle() < 150)
  base.hold()
  
  # deposit at house 3
  if Color.GREEN in Houses[2] or len(Houses[2]) == 1:
    GyroTurn.turn(-89)
    depositHouse(Houses[2], 1, 3)
  else:
    GyroTurn.turn(-89)
    GyroStraight.move(40, condition = lambda: colRight.color() != Color.BLACK)
    base.reset()
    GyroStraight.move(40, condition = lambda: leftMotor.angle() < 110)
    GyroTurn.turn(-89)
      
  LineTrack.move(colLeft, 50, condition = lambda: colRight.color() != Color.BLACK)
  base.hold()
  
  # always deposit two surplus into battery storage from claw, deposit any remaining green
  # based on houses, determine which energy is extra  
  extraCol = getExtra()
  depositBatteryFront()
  if extraCol == Color.GREEN:
    depositBatteryBack()
    GyroTurn.turn(-89)
  else:
    # single motor turn to avoid hitting wall of battery area
    base.reset()
    GyroStraight.move(-40, condition = lambda: leftMotor.angle() > -80)
    base.hold()
    PID_SingleMotorTurn(base, gyro, 89, 1, 0)
    
  LineTrack.move(colLeft, 60, condition = lambda: colRight.color() != Color.BLACK)
  base.hold()
  base.reset()
  LineTrack.move(colLeft, 40, condition = lambda: leftMotor.angle() < 90)
  base.hold()
  GyroTurn.turn(89)
  
  # collect yellow and blue energy
  collectYellow()
  collectBlue()  
 
  # deposit at house 3 again
  
  GyroStraight.move(60, condition = lambda: colRight.color() != Color.BLACK or colLeft.color() != Color.BLACK)
  base.reset()
  GyroStraight.move(40, condition = lambda: leftMotor.angle() < 150)
  base.hold()
  
 
  if Color.YELLOW or Color.BLUE in Houses[2]:
    GyroTurn.turn(89)
    # add condition to turn based on whether blue is in the house
    depositHouse(Houses[2], 2, 3)
  else:
    GyroTurn.turn(-89)
    
  LineTrack.move(colRight, 40, side = -1, condition =  lambda: colLeft.color() != Color.BLACK)
  base.hold()
 
  if extraCol == Color.YELLOW:
    depositBatteryFront()
  elif extraCol == Color.BLUE:
    depositBatteryBack()

  if Color.BLUE or Color.YELLOW in Houses[1]:
    if extraCol == Color.YELLOW:
      GyroStraight.move(-40, condition = lambda: leftMotor.angle() > -80)
      base.hold()
      PID_SingleMotorTurn(base, gyro, 89, 1, 0)
    elif extraCol == Color.BLUE:
      GyroTurn.turn(89)
    base.reset()      
    LineTrack.move(colRight, 45, side = -1, condition = lambda: leftMotor.angle() < 700)
    base.hold()
    GyroTurn.turn(89)
 
    depositHouse(Houses[1], 2, 2)
    
    LineTrack.move(colRight, 60, side = -1, condition = lambda: colRight.color() != Color.BLACK)
    LineTrack.move(colRight, 60, side = -1, condition = lambda: colRight.color() != Color.WHITE)
    
    
  elif Color.BLUE or Color.YELLOW in Houses[0]:
    if extraCol == Color.YELLOW:
      GyroStraight.move(-40, condition = lambda: leftMotor.angle() > -80)
      base.hold()
      PID_SingleMotorTurn(base, gyro, -89, 1, 0)
    elif extraCol == Color.BLUE:
      GyroTurn.turn(-89)
      
  LineTrack.move(colRight, 60, side = -1, condition = lambda: colRight.color() != Color.BLACK)
  if Color.BLUE or Color.YELLOW in Houses[0]:
    base.reset()
    LineTrack.move(colRight, 50, side = -1, condition = lambda: leftMotor.angle() < 700)
    base.hold()
    GyroTurn.turn(-89)
    depositHouse(Houses[0], 2, 1)
  else:
    LineTrack.move(colRight, 60, side = -1, condition = lambda: colRight.color() != Color.WHITE)
    LineTrack.move(colRight, 60, side = -1, condition = lambda: colRight.color() != Color.BLACK)


    
  # go back to base
  returnBase()
    

# backClaw.defaultPos()
# wait(5000)
GyroTurn.maxSpeed = 40
numGreen = 2
depositHouse([Color.GREEN, Color.YELLOW], 1, 1)

wait(1000)
# frontClaw.dc(dir=-1)
# backClaw.dc()
# wait(1500)
# main()
# collectYellow()

# ADD 3 HOLE BEAMS TO BUMPER