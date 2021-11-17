from helper import *
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, ColorSensor, GyroSensor)
from pybricks.nxtdevices import ColorSensor as nxtColorSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.iodevices import Ev3devSensor

class PID(object):
  def __init__(self, 
               kp: float,
               ki: float, 
               kd: float):
    self.kp = kp
    self.ki = ki
    self.kd = kd
    self.integral = 0
    self.lastError = 0
    self.correction = 0
    self.stopwatch = StopWatch()
    
  def resetIntegral(self):
    self.integral = 0
    
  def update(self, 
             error: float, 
             kp: float = None,
             ki: float = None, 
             kd: float = None):
    if kp is None:
      kp = self.kp
    if ki is None:
      ki = self.ki
    if kd is None:
      kd = self.kd
    self.integral = self.integral * 0.5 + error
    self.correction = kp * error + ki * self.integral + kd * (error - self.lastError)
    self.lastError = error
    
   
class PID_LineTrack(PID):
  def __init__(self, 
               base: Base,
               kp: float, 
               ki: float, 
               kd: float, 
               threshold: int):
    super().__init__(kp, ki, kd)
    self.base = base
    self.threshold = threshold    
    
  def move(self, 
           sensor: ColorSensor,
           maxSpeed: float, 
           condition, 
           threshold: int = None, 
           kp: float = None, 
           ki: float = None, 
           kd: float = None, 
           side = 1, 
           target = None, 
           minSpeed = 35,
           accel = False, 
           deccel = True):
    # update control constants if given

    if threshold is None:
      threshold = self.threshold
    speed = maxSpeed
    if target is not None:
      rate = 2 * maxSpeed / (target * 0.04)
    else:
      rate = 1
    if accel:
      speed = maxSpeed /abs(maxSpeed) * minSpeed
    slowingDown = False
    while condition():
      
        
      error = threshold - sensor.reflection()
      
      self.update(error, kp, ki, kd)
      if accel and target is None:
        if speed < maxSpeed:
            speed = speed + rate
        if speed > maxSpeed:
          speed = maxSpeed
      
      if target is not None: # decceleration
        angle = self.base.rightMotor.angle()
        if deccel and abs(abs(angle) - abs(target)) <= 100 * maxSpeed / 40:
          slowingDown = True
          if abs(speed) > minSpeed:
            speed = speed - rate 
          if speed < minSpeed:
            speed = minSpeed
        elif accel and not slowingDown:
          if speed < maxSpeed:
            speed = speed + rate
          if speed > maxSpeed:
            speed = maxSpeed
      #print(speed + side * self.correction, speed - side * self.correction)
      self.base.run(max(min(speed + side * self.correction, 100), 0), max(min(speed - side * self.correction, 100),0) )   
      
          

class PID_GyroStraight(PID):
  def __init__(self, 
               base: Base, 
               kp: float,
               ki: float,
               kd: float,
               gyro: GyroSensor):
    super().__init__(kp, ki, kd)
    self.base = base
    self.gyro = gyro
    
  def move(self, 
           speed: float, 
           condition,
           kp: float = None, 
           ki: float = None, 
           kd: float = None,
           target = 0, 
           maxSpeed = 100,
           minSpeed = 0):

    while condition():

      error = self.gyro.angle() - target
      self.update(error, kp, ki, kd)
      if abs(self.correction) > maxSpeed and self.correction != 0:
        self.correction = maxSpeed * self.correction/abs(self.correction)
      if abs(self.correction) < minSpeed and self.correction != 0:
        self.correction = minSpeed * self.correction/abs(self.correction)
      
      self.base.run(speed - self.correction, speed + self.correction)
      
class PID_GyroStraightDegrees(PID):
  def __init__(self, 
               base: Base, 
               kp: float,
               ki: float,
               kd: float,
               gyro: GyroSensor):
    super().__init__(kp, ki, kd)
    self.base = base
    self.gyro = gyro
    
  def move(self, 
           maxSpeed: float, 
           target, 
           kp: float = None, 
           ki: float = None, 
           kd: float = None,
           minSpeed = 30, 
           accel = False,
           deccel = True, condition = lambda: True):
    angle = self.base.rightMotor.angle()
    rate =  min(abs(2 * maxSpeed / (target * 0.04)), 10)
   
    polarity = maxSpeed /abs(maxSpeed)
    if accel:
      speed = polarity * minSpeed
    else:
      speed = maxSpeed
    
      
    while (target < 0 and angle > target) or (target >= 0 and angle < target) and condition():

      error = self.gyro.angle() 
      self.update(error, kp, ki, kd)      
      angle = self.base.rightMotor.angle()

      if abs(abs(angle) - abs(target)) <= 100 * abs(maxSpeed) / 40 and deccel :
        if abs(speed) > minSpeed:
          speed = (abs(speed) - rate) * polarity
        if abs(speed) < (minSpeed):
          speed = minSpeed * polarity
      elif accel:
        # otherwise accelerate up to speed from minSpeed
        if speed < maxSpeed:
          speed = (abs(speed) + rate) * polarity 
        if speed > maxSpeed:
          speed = maxSpeed
        
      self.base.run(speed - self.correction, speed + self.correction)
  
class PID_GyroTurn(PID_GyroStraight):  
  def __init__(self,
                base: Base, 
                kp: float, 
                ki: float, 
                kd: float, 
                gyro: GyroSensor,
                maxSpeed = 100):
      super().__init__(base, kp, ki, kd, gyro)
      self.maxSpeed = maxSpeed
      
  def turn(self, angle, kp = None, ki = None, kd = None):
    self.base.reset()
    self.gyro.reset_angle(0)
    self.resetIntegral()
    
    self.move(0, lambda: self.gyro.angle() != angle, kp = kp, ki = ki, kd = kd, target = angle, maxSpeed = self.maxSpeed, minSpeed = 5)
    self.base.hold()
    
    self.gyro.reset_angle(0)
    
      
def PID_SingleMotorTurn(base, gyro, angle, leftM, rightM, kp = 0.9, ki = 0, kd = 1.2, minSpeed = 15, reset = True):
  pid = PID(kp, ki, kd)
  while gyro.angle() != angle:
    error = (gyro.angle() - angle)
    pid.update(error, kp, ki, kd)
    if abs(pid.correction) < minSpeed:
      base.run(-minSpeed * (pid.correction / abs(pid.correction)) * leftM, minSpeed * (pid.correction / abs(pid.correction)) * rightM)
    else:
      base.run(-pid.correction * leftM, pid.correction * rightM)
  base.hold()
  if reset:
    gyro.reset_angle(0)
    wait(10)

def PID_AngleOffSet(base, gyro, angle):
  if angle > 0:
    PID_SingleMotorTurn(base, gyro, angle, 1, 0, reset = False, kp = 1.1)
    wait(10)
    PID_SingleMotorTurn(base, gyro, 0, 0, 1)
  else:
    PID_SingleMotorTurn(base, gyro, angle, 0, 1,reset = False, kp = 1.1)
    wait(10)
    PID_SingleMotorTurn(base, gyro, 0, 1, 0)
    

def PID_LineSquare(base, direction = 1, leeway = 1): # direction = 1 for forward, direction = -1 for backwar
  kp = 0.15
  ki = 0.01
  kd = 3
  leftThresh = 34
  rightThresh = 42
  leftPID = PID(kp, ki, kd)
  rightPID = PID(kp, ki, kd)
  while True:
    
    leftVal = base.colLeft.reflection()
    rightVal = base.colRight.reflection()

    leftError = leftVal - leftThresh
    rightError = rightVal - rightThresh
    if abs(leftError) <= leeway and abs(rightError) <= leeway:
      break
    leftPID.update(leftError, kp, ki, kd)
    rightPID.update(rightError, kp, ki, kd)
    outLeft = direction * leftPID.correction
    outRight = direction * rightPID.correction
    #print(leftVal, rightVal, outLeft, outRight)
    base.run(outLeft, outRight)
    
  base.hold()
  