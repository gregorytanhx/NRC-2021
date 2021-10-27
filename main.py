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
clock = StopWatch()

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
LineTrack = PID_LineTrack(base, 0.2, 0, 6, 40)
GyroStraight = PID_GyroStraight(base, 1, 0, 5, gyro)
GyroStraightDeg = PID_GyroStraightDegrees(base, 2.5, 0, 5, gyro)
GyroTurn = PID_GyroTurn(base, 0.9, 0.0001, 1.8, gyro)


surplus = None

# battery alert
print(ev3.battery.voltage())
if ev3.battery.voltage() <= 7400:
  print('LOW BATTERY')
  ev3.speaker.beep()
  sys.exit()


def debug_LineSquare():
  GyroStraight.move(-90, lambda: colLeft.color() != Color.WHITE)
  base.hold()
  PID_LineSquare(base, direction = -1)
  gyro.reset_angle(0)
  base.hold()
  base.reset()
  GyroStraight.move(55, lambda: leftMotor.angle() < 200)
  base.hold()
  wait(1000)
  
def scanHouseEV3(house, sensor, speed, condition):   
  # initialise pid for gyrostraight
  kp, ki, kd = GyroStraight.kp, GyroStraight.ki, GyroStraight.kd
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
  speed = -60
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
  if col == Color.BLUE:
    GyroTurn.turn(180)
    # start opening claw
    frontClaw.dc()
    LineTrack.move(colLeft, 65, lambda: colRight.color() != Color.BLACK)
    base.hold()
    base.reset()
    wait(100)
    GyroStraight.move(-90, lambda: leftMotor.angle() > -120)
    base.hold()
    GyroTurn.turn(-89)
    
  else:    
    base.reset()
    frontClaw.reset()
    GyroStraight.move(-55, lambda: leftMotor.angle() > -160)
    GyroTurn.turn(89)
    gyro.reset_angle(0)
    GyroStraight.move(-55, lambda: colLeft.color() != Color.BLACK)
    base.hold()
    wait(100)
    base.reset()
    GyroStraight.move(55, lambda: leftMotor.angle() < 60)
    base.hold()
    wait(100)
    
  PID_LineSquare(base, direction = -1)
  gyro.reset_angle(0)
  base.reset()
  wait(100)

  #move forward to collect surplus
  GyroStraight.move(90, lambda: leftMotor.angle() < degrees)
  base.hold()
  
  # grab front surplus using claw
  frontClaw.run_target(95, -700)
 
  base.reset()
  GyroStraight.move(90, lambda: leftMotor.angle() < 400)
  base.hold()
  frontClaw.run_target(90,230)
  frontClaw.dc(dir = -1, speed = 20)
  base.reset()
  
  if col == Color.YELLOW:    
    GyroStraight.move(-90, lambda: colRight.color() != Color.BLACK)
    base.hold()
    base.reset()
    GyroStraight.move(90, lambda: leftMotor.angle() < 205)
    base.hold()
    GyroTurn.turn(89)
    
  elif col == Color.GREEN:
    GyroStraight.move(-90, lambda: colLeft.color() != Color.BLACK)
    base.hold()
    wait(100)
    base.reset()
    GyroStraight.move(90, lambda: leftMotor.angle() < 100)
    base.hold()
  
  elif col == Color.BLUE:
    GyroStraight.move(-90, lambda: leftMotor.angle() > -720)
    base.hold()

def collectGreen():  
  backClaw.run_time(100, 1000, wait = False)
  LineTrack.move(colRight, 55, lambda: colLeft.color() != Color.BLACK, side = -1)
  LineTrack.move(colRight, 55, lambda: colLeft.color() != Color.WHITE, side = -1)
  base.hold()
  base.reset()
  LineTrack.move(colRight, 55, lambda: leftMotor.angle() < 218, side = -1)
  base.hold()
  
  GyroTurn.turn(-89)
  GyroStraight.move(-90, lambda: colRight.color() != Color.WHITE)
  base.hold()
  wait(100)
  PID_LineSquare(base, direction = -1)
  gyro.reset_angle(0)
  wait(100)
  base.reset()
  GyroStraight.move(95, lambda: leftMotor.angle() < 100)
  base.hold()
  backClaw.run_angle(70,-185)
  GyroStraight.move(-90, lambda: colRight.color() != Color.WHITE)
  GyroStraight.move(-90, lambda: colRight.color() != Color.BLACK)
  base.hold()
  # reset claw again
  backClaw.run_time(100, 1000, wait = False)
  
  # linetrack and turn to grab other 2 green
  base.reset()
  GyroStraight.move(90, lambda: leftMotor.angle() < 150)
  base.hold()
  GyroTurn.turn(89)
  base.reset()
  GyroStraight.move(90, lambda: leftMotor.angle() < 340)
  base.hold()

  GyroTurn.turn(-89)
  backClaw.run_target(-70,-185)
  base.reset()
  GyroStraight.move(-55, lambda: colLeft.color() != Color.WHITE)
  base.hold()
  base.reset()
  # GyroStraight.move(30, lambda: leftMotor.angle() < 20)
  # base.hold()
  backClaw.run_target(55, 40)
  base.reset()
  base.hold()
  # cap speed of turns after grabbing green to stop them from jerking
  GyroTurn.maxSpeed = 50
  
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
      frontClaw.run_target(-70, -300, wait = False)
      GyroStraight.move(95, lambda: colLeft.color() != Color.RED)
      base.reset()
      GyroStraight.move(95, lambda: leftMotor.angle() < 80)  
      base.reset()
      GyroStraight.move(-90, lambda: leftMotor.angle() > -350)
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
      else:
        GyroTurn.turn(89)
      
      if tmp == 2:
        # deposit both yellow
        frontClaw.dc()
        wait(1000)
        GyroStraight.move(90, lambda: colLeft.color() != Color.RED)
        base.hold()
      
      elif tmp == 1 and numYellow == 2:
        # deposit yellow from claw
        GyroStraight.move(90, lambda: colLeft.color() != Color.RED)
        base.hold()
        frontClaw.run_target(90, 200)
        
      else:
        # deposit yellow from catchment area
        base.reset()
        frontClaw.run_target(-90, -300)
        GyroStraight.move(90, lambda: colLeft.color() != Color.RED)
        base.reset()
        GyroStraight.move(90, lambda: leftMotor.angle() < 100)
        base.hold()

      base.reset()
      GyroStraight.move(-90, lambda: leftMotor.angle() > -200)
      base.hold()
      
  if RingCol not in house:
    if houseNum ==  1 or houseNum ==  2:
      GyroStraight.move(-90, lambda: colLeft.color() != Color.BLACK or colRight.color() != Color.BLACK)
      base.hold()
      base.reset()
      GyroStraight.move(90, lambda: leftMotor.angle() < 80)
      base.hold()
      if houseNum ==  1:
        GyroTurn.turn(-89)
      elif houseNum ==  2: 
        GyroTurn.turn(89)
        
    else:
      GyroStraight.move(-90, lambda: leftMotor.angle() > -500)
      base.hold()
      GyroTurn.turn(180)
    
    frontClaw.run_target(90,300, wait = False)
    #frontClaw.dc(dir = -1, speed = 20)
  
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
        
      else:
        GyroTurn.turn(-89)
        
    else:
      GyroTurn.turn(180)      
    
    base.reset()
    if houseNum == 1 or (houseNum == 2 and time == 2):
      GyroStraight.move(-90, lambda: leftMotor.angle() > -200)
    else:
      GyroStraight.move(-90, lambda: leftMotor.angle() > -50)
    base.hold()
    
    if RingCol == Color.GREEN:
      if numGreen == 4:
        deg = 105
      else:
        deg = 110
    if RingCol == Color.BLUE:
      if numBlue == 4:
        deg = 105
      else:
        deg = 110
    
    backClaw.run_target(-30, -deg)
    print(tmp)
    base.reset()
    # go more if 4 ring blocks have to be deposited
    if numCol == 4 and tmp == 2:
      GyroStraight.move(55, lambda: leftMotor.angle() < 170) 
      if RingCol == Color.GREEN:
        numGreen = 0
      else:
        numBlue = 0
    else:
      GyroStraight.move(55, lambda: leftMotor.angle() < 105)
      if RingCol == Color.GREEN:
        numGreen -= 2
      else:
        numBlue -= 2

    base.hold()

    backClaw.run_target(55, deg)
    GyroStraight.move(-90, lambda: leftMotor.angle() > 50)
    base.hold()
    if houseNum == 1 or houseNum == 2:
      GyroStraight.move(90, lambda: colLeft.color() != Color.BLACK or colRight.color() != Color.BLACK)
      base.hold()
      base.reset()
      GyroStraight.move(90, lambda: leftMotor.angle() < 150)
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
  backClaw.run_time(100, 1000, wait = False)
  LineTrack.move(colRight, 95, lambda: leftMotor.angle() < 200, side = -1)
  LineTrack.move(colRight, 95, lambda: colLeft.color() != Color.BLACK, side = -1)
  base.hold()
  base.reset()
  GyroStraight.move(-90, lambda: leftMotor.angle() > -85)
  base.hold()
  GyroTurn.turn(-90)
  base.reset()
  GyroStraight.move(-90, lambda: leftMotor.angle() > -100)
  base.hold()
  wait(100)
  PID_LineSquare(base, direction = -1)
  gyro.reset_angle(0)
  base.reset()
  
  # move to blue energy area
  base.reset()
  GyroStraight.move(90, lambda: leftMotor.angle() < 300)
  GyroStraight.move(90, lambda: colLeft.color() != Color.WHITE)
  base.hold()
  
  
  # collect first 2 blue energy
  GyroStraight.move(-90, lambda: colRight.color() != Color.WHITE)
  base.hold()
  GyroTurn.turn(-89)
  backClaw.run_target(-70,-230)
  base.reset()
  GyroStraight.move(-20, lambda: leftMotor.angle() > -100)
  base.hold()
  backClaw.run_angle(-70,-40)
  base.reset()
  GyroStraight.move(-10, lambda: leftMotor.angle() > -35)
  base.hold()
  backClaw.run_time(100, 1000)
  base.reset()
  GyroStraight.move(90, lambda: leftMotor.angle() < 120)
  base.hold()
  
  # collect next 2
  GyroTurn.turn(89)
  GyroStraight.move(90, lambda: colRight.color() != Color.BLACK)
  base.reset()
  GyroStraight.move(90, lambda: leftMotor.angle() < 250)
  base.hold()
  GyroTurn.turn(-89)
  backClaw.run_target(-30, -230, wait = False)
  base.reset()
  GyroStraight.move(-5, lambda: leftMotor.angle() > -90)
  base.hold()
  backClaw.run_target(55, 80)
  GyroTurn.maxSpeed = 50
  
  GyroStraight.move(95, lambda: colRight.color() != Color.BLACK)
  base.hold()
  base.reset()
  GyroStraight.move(90, lambda: leftMotor.angle() < 120)
  base.hold()
    
def collectYellow():
  # line track to intersection
  frontClaw.dc()
  LineTrack.move(colLeft, 90, lambda: colRight.color() != Color.BLACK)
  base.hold()
  base.reset()
  
  GyroStraight.move(90, lambda: leftMotor.angle() < 110)
  base.hold()
  GyroTurn.turn(89)
  # push solar panels
  
  frontClaw.hold()
  frontClaw.run_target(-55, -405)
  base.reset()
  LineTrack.move(colRight, 35, lambda: leftMotor.angle() < 500, threshold = 45)
  base.hold()
  gyro.reset_angle(0)
  GyroStraight.move(30, lambda: colLeft.color() != Color.BLACK and colRight.color() != Color.BLACK)
  GyroStraight.move(30, lambda: colLeft.color() != Color.WHITE and colRight.color() != Color.WHITE)
  base.hold()
  base.reset()
  GyroStraight.move(55, lambda: leftMotor.angle() < 50)
  base.hold()
  frontClaw.run_target(90,120) 

  base.reset()
  # GyroStraight.move(-15, lambda: leftMotor.angle() > -20)
  # base.hold()

  frontClaw.run_target(-100, -400)
  
  # track to first 2 yellow and grab with claw
  GyroTurn.turn(-89)
  frontClaw.dc()
  base.reset()
  LineTrack.move(colRight, 70, lambda: leftMotor.angle() < 500, side = -1)
  GyroStraight.move(90, lambda: leftMotor.angle() < 620)
  base.hold()
  GyroTurn.turn(89)
  base.reset()
  GyroStraight.move(90, lambda: leftMotor.angle() < 50)
  base.hold()
  frontClaw.run_target(-95,-460)
  base.reset()
  frontClaw.dc(speed = 20, dir = -1)
  GyroStraight.move(-55, lambda: leftMotor.angle() > -40)
  base.hold()
    
  # collect next 2 in catchment area
  GyroTurn.turn(89)
  LineTrack.move(colLeft, 90, lambda: colRight.color() != Color.BLACK)
  base.reset()  
  LineTrack.move(colLeft, 55, lambda: leftMotor.angle() < 500)
  gyro.reset_angle(0)
  GyroStraight.move(55, lambda: leftMotor.angle() < 750)
  base.hold()
  frontClaw.run_target(-70,-250)
  GyroTurn.turn(-89)
  base.reset()
  GyroStraight.move(55, lambda: leftMotor.angle() < 180)
  base.hold()
  frontClaw.run_target(90,255)
  frontClaw.dc(speed = 20, dir = -1)
  base.reset()
  GyroStraight.move(-95, lambda: leftMotor.angle() > -350)
  base.hold()
  GyroTurn.turn(-89)
  base.reset()
  GyroStraight.move(-90, lambda: leftMotor.angle() > - 680)
  base.hold()
  wait(100)
  PID_LineSquare(base, direction = -1)
  gyro.reset_angle(0)
  wait(100)
  base.reset()
  GyroStraight.move(-90, lambda: leftMotor.angle() > - 900)
  base.hold()
  
  
  # GyroStraight.move(95, lambda: colRight.color() != Color.BLACK)
  # base.hold()
  # base.reset()
  # GyroStraight.move(55, lambda: leftMotor.angle() < 110)
  # base.hold()
  # GyroTurn.turn(89)
  # base.reset()
  # LineTrack.move(colRight, 70, lambda: leftMotor.angle() < 670,side = -1)
  # base.hold()
  # PID_SingleMotorTurn(base, gyro, -89, 0, 1)
  # base.reset()
  
def depositBatteryFront():
  frontClaw.run_target(-70,-300)
  base.reset()
  GyroStraight.move(55, lambda: leftMotor.angle() < 165)
  base.hold()
  frontClaw.run_target(90,300)
  
  GyroStraight.move(-10, lambda: colRight.color() != Color.BLACK or colLeft.color() != Color.BLACK)
  base.hold()

def depositBatteryBack():
  GyroTurn.turn(180)
  base.reset()
  GyroStraight.move(-55, lambda: leftMotor.angle() > -150)
  base.hold()
  wait(50)
  backClaw.run_target(-70,-100)
  base.reset()
  GyroStraight.move(55, lambda: leftMotor.angle() < 40)
  base.hold()
  backClaw.run_target(90,100)
 
def depositBattery(time, extraCol):
  base.reset()
  LineTrack.move(colLeft, 90, lambda: leftMotor.angle() < 400)
  LineTrack.move(colLeft, 20, lambda: colRight.color() != Color.BLACK)
  base.hold()
  
  if time == 1:
    depositBatteryFront()
    if extraCol == Color.GREEN:
      depositBatteryBack()
      GyroTurn.turn(-89)
    else:
      # single motor turn to avoid hitting wall of battery area
      base.reset()
      GyroStraight.move(-90, lambda: leftMotor.angle() > -80)
      base.hold()
      PID_SingleMotorTurn(base, gyro, 89, 1, 0)
  else:
    if extraCol == Color.YELLOW:
      depositBatteryFront()
    elif extraCol == Color.BLUE:
      depositBatteryBack()
     
def getExtra():
  cols = {Color.YELLOW: 0, Color.GREEN: 0, Color.BLUE: 0}
  for house in Houses:
    for col in house:
      cols[col] += 1
  
  for key in cols:
    if cols[key] == 1:
      return key
  
def checkHouse1():

  GyroStraight.move(100, lambda: leftMotor.angle() < 300)   
  LineTrack.move(colRight, 65, lambda: colLeft.color() != Color.BLACK, side = -1)   
  base.hold()
  base.reset()
  GyroStraight.move(55, lambda: leftMotor.angle() < 300)
  base.hold()
  GyroTurn.turn(-89)
  
  # wall align and scan indicators 
  base.run_time(-100, 500)
  gyro.reset_angle(0)  
  scanHouseEV3(Houses[0], ev3Col, 95, lambda: colRight.color() != Color.BLACK)
  PID_SingleMotorTurn(base, gyro, -179, 0.06, 1)

def checkHouse2():
  # scan house 2
  if surplus == Color.BLUE and Color.GREEN not in Houses[0] and len(Houses[0]) != 1:
    # if house 1 has nothing to be deposited, go to house 2 directly from blue surplus area
    PID_SingleMotorTurn(base, gyro, 89, 0, 1)    
  
  else:
    LineTrack.move(colRight, 90, lambda: colLeft.color() != Color.BLACK, side = -1)
    LineTrack.move(colRight, 90, lambda: colLeft.color() != Color.WHITE, side = -1)
    base.reset()
    LineTrack.move(colRight, 90, lambda: leftMotor.angle() < 700, side = -1)
    base.hold()
    gyro.reset_angle(0)
    PID_AngleOffSet(base, gyro, 80)
    base.hold()
    GyroStraight.move(-55, lambda: colLeft.color() != Color.WHITE)
    base.hold()
    PID_LineSquare(base, direction = -1)
    gyro.reset_angle(0)
    base.reset()
    GyroStraight.move(-95, lambda: leftMotor.angle() > -300)
    base.hold()
    wait(10)    
    
  scanHouseEV3(Houses[1], ev3Col, 95, lambda: colRight.color() != Color.BLACK)  
  base.reset()
  GyroStraight.move(90, lambda: leftMotor.angle() < 120)
  base.hold()
  
  # deposit at house 2 
  if Color.GREEN in Houses[1] or len(Houses[1]) == 1:
    depositHouse(Houses[1], 1, 2)
  else:
    GyroTurn.turn(-89)
    LineTrack.move(colLeft, 95, lambda: colRight.color() != Color.BLACK)
    base.reset()
    LineTrack.move(colLeft, 90, lambda: leftMotor.angle() < 100)
    GyroTurn.turn(-89)
       
def checkHouse3():
  # move to house 3 
  LineTrack.move(colLeft, 95, lambda: colRight.color() != Color.BLACK)
  base.reset()
  LineTrack.move(colLeft, 90, lambda: leftMotor.angle() < 100)
  base.hold()
  GyroTurn.turn(89)
  LineTrack.move(colRight, 90, lambda: colLeft.color() != Color.RED)
  base.hold()
  base.reset()
  GyroTurn.move(-95, lambda: leftMotor.angle() > -120)
  base.hold()
  GyroTurn.turn(-89)
  base.reset()
  GyroStraight.move(-55, lambda: leftMotor.angle() > -50)
  base.hold()
  
  # scan house 3
  PID_LineSquare(base, direction = -1)
  gyro.reset_angle(0)
  base.reset()
  GyroStraight.move(-95, lambda: leftMotor.angle() > -300)
  base.hold()
  scanHouseEV3(Houses[2], ev3Col, 95, lambda: colLeft.color() != Color.BLACK)
  base.reset()
  GyroStraight.move(90, lambda: leftMotor.angle() < 120)
  base.hold()
  
  # deposit at house 3
  if Color.GREEN in Houses[2] or len(Houses[2]) == 1:
    depositHouse(Houses[2], 1, 3)
  else:
    GyroTurn.turn(-89)
  GyroTurn.maxSpeed = 50
  
def returnBase():
  if Color.BLUE or Color.YELLOW in Houses[0]:
    base.reset()
    LineTrack.move(colLeft, 95, lambda: leftMotor.angle() < 900)
    base.hold()
    GyroTurn.turn(-89)
    depositHouse(Houses[0], 2, 1)
    frontClaw.dc(dir = -1)
    backClaw.run_time(100, 500)
    base.run_time(-100, 1000)
    GyroTurn.turn(87)
    
  else:
    frontClaw.dc(dir = -1)
    backClaw.run_time(100, 500, wait = False)
    LineTrack.move(colLeft, 90, lambda: colRight.color() != Color.WHITE)
    LineTrack.move(colLeft, 90, lambda: colRight.color() != Color.BLACK)
    base.hold()
    base.reset()
    GyroStraight.move(90, lambda: leftMotor.angle() < 100)
    base.hold()
    GyroTurn.turn(-92)
    
  base.run_time(-100, 1500)  

def returnHouse1():
  global surplus
  if surplus == Color.BLUE:
    GyroTurn.turn(-89)   
  else:
    GyroTurn.turn(-89)      
    
  LineTrack.move(colRight, 90, lambda: colLeft.color() != Color.BLACK, side = -1)
  LineTrack.move(colRight, 95, lambda: colLeft.color() != Color.WHITE, side = -1)
  LineTrack.move(colRight, 95, lambda: colLeft.color() != Color.BLACK, side = -1)
  base.hold()
  base.reset() 

  GyroStraight.move(55, lambda: leftMotor.angle() < 150)
  base.hold()

  depositHouse(Houses[0], 1, 1)
  
  # return to house 2 intersection
  LineTrack.move(colRight, 95, lambda: colLeft.color() != Color.BLACK, side = -1)
  LineTrack.move(colRight, 90, lambda: colLeft.color() != Color.WHITE, side = -1)  

def main():
  global surplus
  gyro.reset_angle(0)
  base.reset()
  checkHouse1()
  
  # if yellow surplus is present, collect it 
  # move toward green energy
  if checkSurplus(-140):
    surplus = Color.YELLOW
    collectSurplus(190, Color.YELLOW)

  else:
    gyro.reset_angle(0)
    GyroTurn.turn(179)
    PID_AngleOffSet(base, gyro, 25)
    
  # collect green energy
  collectGreen()
  
  # collect green surplus if present, else go collect blue surplus
  if surplus is None:
    GyroTurn.turn(-89)
    if checkSurplus(-170):
      surplus = Color.GREEN
      collectSurplus(25, Color.GREEN)
      
    else:
      surplus = Color.BLUE
      collectSurplus(415, Color.BLUE)
      
  # check whether to deposit in house 1
  if Color.GREEN in Houses[0] or len(Houses[0]) == 1: 
    returnHouse1()
  else:
    # turn to face house 2 from green surplus area if not blue surplus
    if surplus == Color.GREEN:
      PID_SingleMotorTurn(base, gyro, 89, 1, 0.3)
    elif surplus == Color.YELLOW:
      base.reset()
      GyroStraight.move(55, lambda: leftMotor.angle() < 80)
      base.hold()
      GyroTurn.turn(89)
        
  checkHouse2()
  checkHouse3()
  # based on houses, determine which energy is extra  
  extraCol = getExtra()
  # always deposit two surplus into battery storage from claw, deposit any remaining green

  depositBattery(1, extraCol)
  
  # collect yellow and blue energy
  collectYellow()
  collectBlue()  
 
  # deposit at house 3 again
  if Color.YELLOW or Color.BLUE in Houses[2]:
    GyroTurn.turn(89)
    # add condition to turn based on whether blue is in the house
    depositHouse(Houses[2], 2, 3)
  else:
    GyroTurn.turn(-89)
    
  depositBattery(2, extraCol)
   # go back to house 2 if needed
  if Color.BLUE or Color.YELLOW in Houses[1]:
    if extraCol == Color.YELLOW:
      GyroStraight.move(-55, lambda: leftMotor.angle() > -80)
      base.hold()
      PID_SingleMotorTurn(base, gyro, -89, 0, 1)
    elif extraCol == Color.BLUE:
      GyroTurn.turn(89)
    base.reset()      
    LineTrack.move(colRight, 45, lambda: leftMotor.angle() < 700, side = -1)
    base.hold()
 
    depositHouse(Houses[1], 2, 2)
    LineTrack.move(colLeft, 95, lambda: colRight.color() != Color.BLACK)
    LineTrack.move(colLeft, 90, lambda: colRight.color() != Color.WHITE)
    
    
  elif Color.BLUE or Color.YELLOW in Houses[0]:
    if extraCol == Color.YELLOW:
      GyroStraight.move(-55, lambda: leftMotor.angle() > -80)
      base.hold()
      PID_SingleMotorTurn(base, gyro, 89, 1, 0)
    elif extraCol == Color.BLUE:
      GyroTurn.turn(-89)
      
  LineTrack.move(colLeft, 90, lambda: colRight.color() != Color.BLACK)
  LineTrack.move(colLeft, 90, lambda: colRight.color() != Color.WHITE)
  # deposit last energy and return to base
  returnBase()
  



# frontClaw.dc(dir=-1)
# backClaw.dc()
# wait(1200)
# ev3.speaker.beep()
# frontClaw.hold()
# backClaw.hold()
# main()
# gyro.reset_angle(0)
# GyroStraight.move(90, lambda: True)
# base.hold()

while True:
  base.run(90,70)
  print(leftMotor.speed())
# wait(1000)
# while True: 
#   print(gyro.angle())

# collectYellow()
# wait(10000)
#backClaw.defaultPos()

# RE RUN COLLECT YELLOW AND BLUE

# fix deposit house abit
