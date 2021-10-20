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
    self.integral += error * 0.5
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
           speed: float, 
           condition, 
           threshold: int = None, 
           kp: float = None, 
           ki: float = None, 
           kd: float = None, 
           side = 1, 
           reset = True):
    # update control constants if given
    if threshold is None:
      threshold = self.threshold
    while condition():
      error = threshold - sensor.reflection()
      self.update(error, kp, ki, kd)
      self.base.run(speed + side * self.correction, speed - side * self.correction)        
    
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
           minSpeed = 0,
           reset = True):
    while condition():
      error = self.gyro.angle() - target
      self.update(error, kp, ki, kd)
      if abs(self.correction) > maxSpeed:
        self.correction = maxSpeed * self.correction/abs(self.correction)
      if abs(self.correction) < minSpeed:
        self.correction = minSpeed * self.correction/abs(self.correction)
      
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
    
    self.move(0, lambda: self.gyro.angle() != angle, kp, ki, kd, target = angle, maxSpeed = self.maxSpeed, minSpeed = 5)
    self.base.hold()
    self.gyro.reset_angle(0)
    
      
def PID_SingleMotorTurn(base, gyro, angle, leftM, rightM, kp = 1.1, ki = 0.00001, kd = 2, minSpeed = 20, reset = True):
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

def PID_AngleOffSet(base, gyro, angle):
  if angle > 0:
    PID_SingleMotorTurn(base, gyro, angle, 1, 0, reset = False)
    wait(50)
    PID_SingleMotorTurn(base, gyro, 0, 0, 1)
  else:
    PID_SingleMotorTurn(base, gyro, angle, 0, 1,reset = False)
    wait(50)
    PID_SingleMotorTurn(base, gyro, 0, 1, 0)
    

def PID_LineSquare(base, threshold = 40, direction = 1, leeway = 3): # direction = 1 for forward, direction = -1 for backwar
  kp = 0.13
  ki = 0.0002
  kd = 0.2
  leftPID = PID(kp, ki, kd)
  rightPID = PID(kp, ki, kd)
  while True:
    leftVal = base.colLeft.reflection()
    rightVal = base.colRight.reflection()
    leftError = base.colLeft.reflection() - threshold
    rightError = base.colRight.reflection() - threshold
    if abs(leftError) <= leeway and abs(rightError) <= leeway:
      break
    leftPID.update(leftError, kp, ki, kd)
    rightPID.update(rightError, kp, ki, kd)
    outLeft = direction * leftPID.correction
    outRight = direction * rightPID.correction
    base.run(outLeft, outRight)
    
  base.hold()
  