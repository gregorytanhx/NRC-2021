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
surplus = None


# initialise ev3
ev3 = EV3Brick()
clock = StopWatch()

# initialise motors
frontClaw = FrontClaw(Port.A)
backClaw = BackClaw(Port.D)
leftMotor = Motor(Port.B, positive_direction = Direction.COUNTERCLOCKWISE)
rightMotor =  Motor(Port.C)

# unlock speed limit
leftMotor.control.limits(1500)
rightMotor.control.limits(1500)

# initialise sensors

ev3Col = Ev3devSensor(Port.S1)
ev3ColSensor = ColorSensor(Port.S1)
gyro = GyroSensor(Port.S2)
colLeft = ColorSensor(Port.S3)
colRight = ColorSensor(Port.S4)

base = Base(leftMotor, rightMotor, colLeft, colRight, frontClaw, backClaw)

# set up defaults for PID functions
LineTrack = PID_LineTrack(base, 0.15, 0, 10, 40)
GyroStraight = PID_GyroStraight(base, 1.2, 0, 5, gyro)
GyroStraightDeg = PID_GyroStraightDegrees(base, 1.2, 0, 5, gyro)
GyroTurn = PID_GyroTurn(base, 0.9, 0.0001, 1.8, gyro)

# battery alert
print(ev3.battery.voltage())
if ev3.battery.voltage() <= 7500:
  print('LOW BATTERY')
  ev3.speaker.beep()
  sys.exit()

def print_degrees():
  while True:
    print(leftMotor.angle())

def debug_LineSquare():
  GyroStraight.move(-50, lambda: colLeft.color() != Color.WHITE)
  base.hold()
  PID_LineSquare(base, direction = -1)
  gyro.reset_angle(0)
  base.hold()
  base.reset()
  GyroStraight.move(40, lambda: leftMotor.angle() < 200)
  base.hold()
  wait(1000)
  
def scanHouseEV3(house, sensor):   
  # initialise pid for gyrostraight
  kp, ki, kd = GyroStraight.kp, GyroStraight.ki, GyroStraight.kd
  gyroPID = PID(kp, ki, kd)
  target = 320
  speed = 90
  maxSpeed = 90
  minSpeed = 30
  rate = 2 * maxSpeed / (target * 0.04)
  base.reset()
  while colRight.color() != Color.BLACK:
    detected = False
    gyroPID.update(gyro.angle(), kp, ki, kd)
    angle = base.leftMotor.angle()
    if abs(abs(angle) - abs(target)) <= 80 * maxSpeed / 40:
      
      if abs(speed) > minSpeed:
        speed = (abs(speed) - rate) *  speed/abs(speed) 
      if speed < minSpeed:
        speed = minSpeed
    
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
        angle = base.leftMotor.angle()
        if abs(abs(angle) - abs(target)) <= 100 * maxSpeed / 40:
          
          if abs(speed) > minSpeed:
            speed = (abs(speed) - rate) *  speed/abs(speed) 
          if speed < minSpeed:
            speed = minSpeed

        base.run(speed - gyroPID.correction, speed + gyroPID.correction)
      detected = False

  print(house)
  base.hold()

def checkSurplus(degrees):
  # reverse for certain amount of degrees to check if surplus is present
  speed = -50
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
    base.reset()
    LineTrack.move(colLeft, 85, lambda: colRight.color() != Color.BLACK, target = 1000)
    base.hold()
    base.reset()
    
    GyroStraightDeg.move(-70, -100)
    base.hold()
    GyroTurn.turn(-89)
    
  else:    
    base.reset()
    frontClaw.reset()
    GyroStraightDeg.move(-70, -160)
    base.hold()
    GyroTurn.turn(89)
    GyroStraightDeg.move(-40, -100)
    base.hold()
    wait(100)
    
  PID_LineSquare(base, direction = -1)
  gyro.reset_angle(0)
  base.reset()
  wait(100)

  #move forward to collect surplus
  GyroStraightDeg.move(85, degrees)
  base.hold()
  
  # grab front surplus using claw
  frontClaw.run_target(60, -700)
 
  base.reset()
  GyroStraightDeg.move(80, 400)
  base.hold()
  frontClaw.run_target(50, 230)
  frontClaw.dc(dir = -1, speed = 20)
  base.reset()
  
  if col == Color.YELLOW:    
    GyroStraightDeg.move(-80, -(degrees + 200))
    base.hold()
    GyroTurn.turn(89)
    
  elif col == Color.GREEN:
    GyroStraightDeg.move(-80,  -(degrees + 300))
    base.hold()
  
  elif col == Color.BLUE:
    GyroStraightDeg.move(-90, -720)
    base.hold()

def collectGreen():  
  backClaw.run_time(100, 1000, wait = False)
  base.reset()
  LineTrack.move(colRight, 60, lambda: colLeft.color() != Color.BLACK, side = -1, target = 500)
  base.reset()

  LineTrack.move(colRight, 40, lambda: leftMotor.angle() < 225, side = -1, target = 220)
  base.hold()
  wait(100)
  gyro.reset_angle(0)
  

  backClaw.run_angle(50, -185, wait = False)
  GyroTurn.turn(-89)
  GyroStraight.move(-50, lambda: colRight.color() != Color.WHITE)
  GyroStraight.move(-40, lambda: colRight.color() != Color.BLACK)
  base.hold()
  # reset claw again
  backClaw.run_time(100, 1000, wait = False)
  
  # linetrack and turn to grab other 2 green
  base.reset()
  GyroStraightDeg.move(80, 200)
  base.hold()
  GyroTurn.turn(89)
  base.reset()
  GyroStraightDeg.move(80, 345)
  base.hold()
  backClaw.run_target(-50, -185, wait = False)
  GyroTurn.turn(-89)

  base.reset()
  GyroStraightDeg.move(-60, -80)
  base.hold()
  base.reset()
  # GyroStraight.move(30, lambda: leftMotor.angle() < 20)
  # base.hold()
  backClaw.run_target(50, 40)
  base.reset()
  base.hold()
  # cap speed of turns after grabbing green to stop them from jerking
  GyroTurn.maxSpeed = 40
  
def depositHouse(house, time, houseNum):
  
  # TO DO 
  # add variable degree for cases when bot starts closer to house
  global numGreen, numBlue, numYellow, numSurplus, surplus
  
  if time == 1:
    # first visit to house deposits surplus and green
    numCol = numGreen
    RingCol = Color.GREEN
    tmp = 1
    if len(house) == 1 and house[0] == surplus:
        tmp = 2
        
    if len(house) == 1 or (houseNum == 2 and tmp == 1 and numSurplus == 4) : # surplus to be deposited
      # only 2 surplus ever need to be deposited
      # unless house 2
      if houseNum == 1:
        GyroTurn.turn(-89)
      else:
        GyroTurn.turn(89)
             
      # lift claw, move forward then reverse to deposit surplus from within catchment area
      frontClaw.run_target(-50, -300, wait = False)
      GyroStraight.move(60, lambda: colLeft.color() != Color.RED)
      base.reset()
      GyroStraightDeg.move(60, 80)  

      numSurplus -= 2
      
    elif (houseNum == 2 and tmp == 1 and numSurplus == 2):
      # deposit yellow from claw
      GyroStraight.move(60, lambda: colLeft.color() != Color.RED)
      base.hold()
      frontClaw.run_target(70, 200)
      numSurplus = 0
      
    elif (houseNum == 2 and tmp == 2):
      frontClaw.dc()
      GyroStraight.move(60, lambda: colLeft.color() != Color.RED)
      base.hold()
      numSurplus = 0
      
    base.reset()
    GyroStraightDeg.move(-80, -350)
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
        GyroStraight.move(60, lambda: colLeft.color() != Color.RED)
        base.hold()
        numYellow = 0
      
      elif tmp == 1 and numYellow == 2:
        # deposit yellow from claw
        GyroStraight.move(60, lambda: colLeft.color() != Color.RED)
        base.hold()
        frontClaw.run_target(70, 200)
        numYellow = 0
        
      else:
        # deposit yellow from catchment area
        base.reset()
        frontClaw.run_target(-50, -300, wait = False)
        GyroStraight.move(60, lambda: colLeft.color() != Color.RED)
        base.reset()
        GyroStraightDeg.move(70, 80)
        base.hold()
        numYellow -= 2

      base.reset()
      GyroStraightDeg.move(-80, -350)
      base.hold()
      
  if RingCol not in house:
    if houseNum ==  1 or houseNum ==  2:
      GyroStraight.move(-60, lambda: colLeft.color() != Color.BLACK or colRight.color() != Color.BLACK)
      base.hold()
      base.reset()
      GyroStraightDeg.move(70, 80)
      base.hold()
      if houseNum ==  1:
        GyroTurn.turn(-89)
      elif houseNum ==  2: 
        GyroTurn.turn(89)
        
    else:
      GyroStraightDeg.move(-80, -500)
      base.hold()
      GyroTurn.turn(180)
    if houseNum != 1:
      # lower claw if not house 1
      frontClaw.run_target(50, 300, wait = False)
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
      GyroStraightDeg.move(-80, -200)
    else:
      GyroStraight.move(-70, -50)
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
      GyroStraightDeg.move(70, 170) 
      if RingCol == Color.GREEN:
        numGreen = 0
      else:
        numBlue = 0
    else:
      GyroStraightDeg.move(70, 105)
      if RingCol == Color.GREEN:
        numGreen -= 2
      else:
        numBlue -= 2

    base.hold()

    backClaw.run_target(30, deg)
    GyroStraightDeg.move(-60, 50)
    base.hold()
    if houseNum == 1 or houseNum == 2:
      GyroStraight.move(50, lambda: colLeft.color() != Color.BLACK or colRight.color() != Color.BLACK)
      base.hold()
      base.reset()
      GyroStraightDeg.move(70, 150)
      base.hold()
      if houseNum == 1:
        GyroTurn.turn(89)
      else:
        GyroTurn.turn(-89)
         
    # if no more ring blocks are on the bot, reset the maximum turn speed
    if numCol == 0:
      GyroTurn.maxspeed = 100

def collectBlue():

  PID_LineSquare(base, direction = -1, leeway = 2)
  gyro.reset_angle(0)
  wait(100)
  base.reset()
  backClaw.hold()
  backClaw.run_target(-50, -230, wait = False)
  GyroStraightDeg.move(-95, -850)
  base.hold()  
  base.reset()
  GyroStraightDeg.move(-60, -150)
  base.hold()
  backClaw.run_angle(-50, -40)
  base.reset()
  GyroStraight.move(-10, lambda: leftMotor.angle() > -35)
  base.hold()
  backClaw.run_time(100, 1000)
  base.reset()
  GyroStraightDeg.move(70, 120)
  base.hold()
  
  # collect next 2
  GyroTurn.turn(89)
  GyroStraight.move(50,  lambda: colRight.color() != Color.BLACK)
  base.reset()
  GyroStraightDeg.move(70, 250)
  base.hold()
  GyroTurn.turn(-89)
  backClaw.run_target(-30, -230, wait = False)
  base.reset()
  GyroStraight.move(-5, lambda: leftMotor.angle() > -90)
  base.hold()
  backClaw.run_target(30, 80)
  GyroTurn.maxSpeed = 40
  
  GyroStraightDeg.move(95, 900)
  base.hold()
    
def collectYellow():
  # line track to intersection
  frontClaw.dc()
  LineTrack.move(colLeft, 70, lambda: colRight.color() != Color.BLACK, target = 800)
  base.reset()  
  LineTrack.move(colLeft, 50, lambda: leftMotor.angle() < 140, target = 140)
  base.hold()
  GyroTurn.turn(89)
  # push solar panels
  
  frontClaw.hold()
  frontClaw.run_target(-40, -405, wait = False)
  base.reset()
  LineTrack.move(colRight, 40, lambda: leftMotor.angle() < 500, threshold = 45, target = 500)
  base.hold()
  gyro.reset_angle(0)
  GyroStraight.move(30, lambda: colLeft.color() != Color.BLACK and colRight.color() != Color.BLACK)
  GyroStraight.move(30, lambda: colLeft.color() != Color.WHITE and colRight.color() != Color.WHITE)
  base.hold()
  base.reset()
  GyroStraightDef.move(40, lambda: leftMotor.angle() < 50)
  base.hold()
  frontClaw.run_target(70, 120) 

  base.reset()
  # GyroStraight.move(-15, lambda: leftMotor.angle() > -20)
  # base.hold()

  frontClaw.run_target(-100, -400)
  
  # track to first 2 yellow and grab with claw
  GyroTurn.turn(-89)
  frontClaw.dc()
  base.reset()
  LineTrack.move(colRight, 70, lambda: leftMotor.angle() < 500, side = -1)
  GyroStraightDeg.move(70, 620)
  base.hold()
  GyroTurn.turn(89)
  base.reset()
  GyroStraightDeg.move(50, 50)
  base.hold()
  frontClaw.run_target(-60, -460)
  base.reset()
  frontClaw.dc(speed = 20, dir = -1)
  GyroStraightDeg.move(-50, -40)
  base.hold()
    
  # collect next 2 in catchment area
  GyroTurn.turn(89)
  base.reset()
  LineTrack.move(colLeft, 60, lambda: colRight.color() != Color.BLACK, target = 600)
  base.reset()
  LineTrack.move(colLeft, 60, lambda: leftMotor.angle() < 600)
  gyro.reset_angle(0)
  GyroStraightDeg.move(70, 730)
  base.hold()

  base.hold()
  frontClaw.run_target(-50, -250)
  GyroTurn.turn(-89)
  base.reset()
  GyroStraight.move(40, lambda: leftMotor.angle() < 180)
  base.hold()
  frontClaw.run_target(50, 255)
  frontClaw.dc(speed = 20, dir = -1)
  base.reset()
  GyroStraight.move(-60, lambda: leftMotor.angle() > -350)
  base.hold()
  GyroTurn.turn(-89)
  base.reset()
  GyroStraight.move(-70, lambda: leftMotor.angle() > - 680)
  base.hold()
  wait(100)
  PID_LineSquare(base, direction = -1)
  gyro.reset_angle(0)
  wait(100)
  base.reset()
  GyroStraight.move(-70, lambda: leftMotor.angle() > - 900)
  base.hold()
  LineTrack.move(colLeft, 60, lambda: colRight.color() != Color.BLACK, target = 500)
  base.reset()
  LineTrack.move(colLeft, 60, lambda: leftMotor.angle() < 600)
  GyroStraightDeg.move(70, 740)
  base.hold()
  GyroTurn.turn(-90)
  base.reset()
  GyroStraightDeg.move(60, 180)
  base.hold()
  #frontClaw.run_target(50, 255)
  frontClaw.dc(speed = 20, dir = -1)
  base.reset()
  backClaw.run_time(100, 1000, wait = False)
  GyroStraightDeg.move(-95, -360)
  base.hold()
  GyroTurn.turn(-90)
  base.reset()
  GyroStraightDeg.move(-95, -500, deccel = False)
  GyroStraightDeg.move(-95, -750, condition = lambda: colLeft.color() != Color.BLACK)
  base.hold()

  
  
  # GyroStraight.move(60, lambda: colRight.color() != Color.BLACK)
  # base.hold()
  # base.reset()
  # GyroStraight.move(40, lambda: leftMotor.angle() < 110)
  # base.hold()
  # GyroTurn.turn(89)
  # base.reset()
  # LineTrack.move(colRight, 50, lambda: leftMotor.angle() < 650, side = -1)
  # base.hold()
  # PID_SingleMotorTurn(base, gyro, -89, 0, 1)
  # base.reset()
  
def depositBatteryFront():
  frontClaw.run_target(-50, -300)
  base.reset()
  GyroStraightDeg.move(70, 165)
  base.hold()
  frontClaw.run_target(50, 300)
  
  GyroStraight.move(-10, lambda: colRight.color() != Color.BLACK or colLeft.color() != Color.BLACK)
  base.hold()

def depositBatteryBack():
  GyroTurn.turn(180)
  base.reset()
  GyroStraightDeg.move(-70, -150)
  base.hold()
  wait(50)
  backClaw.run_target(-50, -100)
  base.reset()
  GyroStraightDeg.move(60, 40)
  base.hold()
  backClaw.run_target(50, 100)
 
def depositBattery(time, extraCol):
  base.reset()
  LineTrack.move(colLeft, 70, lambda: colRight.color() != Color.BLACK, target = 600, minSpeed = 25)
  base.hold()
  
  if time == 1:
    depositBatteryFront()
    if extraCol == Color.GREEN:
      depositBatteryBack()
      GyroTurn.turn(-89)
    else:
      # single motor turn to avoid hitting wall of battery area
      base.reset()
      GyroStraightDeg.move(-60, -80)
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
  base.reset()
  GyroStraight.move(85, lambda: leftMotor.angle() < 250)   
  LineTrack.move(colRight, 80, lambda: colLeft.color() != Color.BLACK, side = -1, target = 900)  
  base.hold()
  base.reset()
  GyroStraightDeg.move(85, 320)
  base.hold()
  GyroTurn.turn(-89)
  base.run_time(-100, 500)
  gyro.reset_angle(0)  
  scanHouseEV3(Houses[0], ev3Col)
  PID_SingleMotorTurn(base, gyro, -179, 0.06, 1)

def returnHouse1():
  global surplus
  if surplus == Color.BLUE:
    GyroTurn.turn(-89)   
  else:
    GyroTurn.turn(-89)      
    
  LineTrack.move(colRight, 85, lambda: colLeft.color() != Color.BLACK, side = -1)
  LineTrack.move(colRight, 85, lambda: colLeft.color() != Color.WHITE, side = -1)
  base.reset()
  LineTrack.move(colRight, 85, lambda: leftMotor.angle() < 1250, side = -1, target = 1100)
  base.hold()
  base.reset() 
  GyroStraightDeg.move(85, 150)
  base.hold()

  depositHouse(Houses[0], 1, 1)
  
  # return to house 2 intersection
  LineTrack.move(colRight, 85, lambda: colLeft.color() != Color.BLACK, side = -1)
  LineTrack.move(colRight, 85, lambda: colLeft.color() != Color.WHITE, side = -1)  

def checkHouse2():
  # scan house 2
  if surplus == Color.BLUE and Color.GREEN not in Houses[0] and len(Houses[0]) != 1:
    # if house 1 has nothing to be deposited, go to house 2 directly from blue surplus area
    PID_SingleMotorTurn(base, gyro, 89, 0, 1)    
  
  else:
    if numSurplus == 4:
      # only raise claw if it hasnt been raised at house 1
      frontClaw.run_target(-30, -300, wait = False)
    LineTrack.move(colRight, 70, lambda: colLeft.color() != Color.BLACK, side = -1)
    base.reset()

    LineTrack.move(colRight, 45, lambda: leftMotor.angle() < 350, side = -1, target = 350)
    base.hold()
    gyro.reset_angle(0)
    wait(100)
    PID_AngleOffSet(base, gyro, 76)
    
  scanHouseEV3(Houses[1], ev3Col)  
  base.reset()
  if surplus in Houses[1] and (numSurplus == 2 or len(Houses[1]) == 1):
    frontClaw.run_target(50, 300, wait = False)
  GyroStraightDeg.move(80, 120)
  base.hold()
  
  # deposit at house 2 
  if Color.GREEN in Houses[1] or len(Houses[1]) == 1 or (surplus in Houses[1] and surplus != Color.YELLOW) :
    depositHouse(Houses[1], 1, 2)
  else:
    PID_SingleMotorTurn(base, gyro, -180, 0.1, 1)
       
def checkHouse3():
  # move to house 3 
  base.reset()
  LineTrack.move(colLeft, 75, lambda: colRight.color() != Color.BLACK, target = 600)
  base.reset()
  LineTrack.move(colLeft, 50, lambda: leftMotor.angle() < 140, target = 140)
  base.hold()
  GyroTurn.turn(89)
  base.reset()
  LineTrack.move(colRight, 85, lambda: leftMotor.angle() < 800, target = 800, accel = True)
  base.hold()  
  GyroTurn.turn(-89)
  base.reset()
  GyroStraightDeg.move(-40, -50)
  base.hold()
  
  # scan house 3
  PID_LineSquare(base, direction = -1)
  gyro.reset_angle(0)
  base.reset()
  GyroStraightDeg.move(-85, -300)
  base.hold()
  scanHouseEV3(Houses[2], ev3Col)
  base.reset()
  GyroStraightDeg.move(85, 140)
  base.hold()
  
  # deposit at house 3
  if Color.GREEN in Houses[2] or len(Houses[2]) == 1:
    depositHouse(Houses[2], 1, 3)
  else:
    GyroTurn.turn(-89)
  GyroTurn.maxSpeed = 100
  
def returnBase():
  if Color.BLUE or Color.YELLOW in Houses[0]:
    base.reset()
    LineTrack.move(colLeft, 80, lambda: leftMotor.angle() < 900, target = 900)
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
    LineTrack.move(colLeft, 80, lambda: colRight.color() != Color.WHITE)
    LineTrack.move(colLeft, 80, lambda: colRight.color() != Color.BLACK, target = 1200)
    base.hold()
    base.reset()
    GyroStraightDeg.move(80, 100)
    base.hold()
    GyroTurn.turn(-92)
    
  base.run_time(-100, 1500)  

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
    wait(1000)
    #PID_AngleOffSet(base, gyro, 25)
    
  # collect green energy
  wait(10000)
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
      GyroStraight.move(40, lambda: leftMotor.angle() < 80)
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
      GyroStraight.move(-40, lambda: leftMotor.angle() > -80)
      base.hold()
      PID_SingleMotorTurn(base, gyro, -89, 0, 1)
    elif extraCol == Color.BLUE:
      GyroTurn.turn(89)
    base.reset()      
    LineTrack.move(colRight, 45, lambda: leftMotor.angle() < 700, side = -1)
    base.hold()
 
    depositHouse(Houses[1], 2, 2)
    LineTrack.move(colLeft, 60, lambda: colRight.color() != Color.BLACK)
    LineTrack.move(colLeft, 70, lambda: colRight.color() != Color.WHITE)
    
    
  elif Color.BLUE or Color.YELLOW in Houses[0]:
    if extraCol == Color.YELLOW:
      GyroStraight.move(-40, lambda: leftMotor.angle() > -80)
      base.hold()
      PID_SingleMotorTurn(base, gyro, 89, 1, 0)
    elif extraCol == Color.BLUE:
      GyroTurn.turn(-89)
      
  LineTrack.move(colLeft, 70, lambda: colRight.color() != Color.BLACK)
  LineTrack.move(colLeft, 70, lambda: colRight.color() != Color.WHITE)
  # deposit last energy and return to base
  returnBase()
  
# GyroTurn.maxSpeed = 40
# checkHouse3()


#while True: print(colLeft.reflection(), colRight.reflection())
backClaw.run_time(100, 1000)
collectBlue()
wait(1000)
#debug_LineSquare()

# frontClaw.dc(dir=-1)
# backClaw.dc()
# wait(1200)
# ev3.speaker.beep()
# frontClaw.hold()
# backClaw.hold()
# main()