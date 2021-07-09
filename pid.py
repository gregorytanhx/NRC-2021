from helper import *

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
    
  def reset(self):
    self.integral = 0
    
  def update(self, 
             error: float, 
             kp: float,
             ki: float, 
             kd: float):
    self.integral += error
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
           threshold: int = None, 
           kp: int = None, 
           ki: int = None, 
           kd: int = None):
    # update control constants if given
    if threshold is None:
      threshold = self.threshold
    if kp is None:
      kp = self.kp
    if ki is None:
      ki = self.ki
    if kd is None:
      kd = self.kd
    self.update(threshold - sensor.reflection(), kp, ki, kd) 
    self.base.run(speed + self.correction, speed - self.correction)

class PID_Straight(PID):
  def __init__(self, 
               base: Base, 
               kp: float,
               ki: float,
               kd: float):
    super().__init__(kp, ki, kd)
    self.base = base
    
  def move(self, 
           speed: float, 
           kp: float = None, 
           ki: float = None, 
           kd: float = None):
    if kp is None:
      kp = self.kp
    if ki is None:
      ki = self.ki
    if kd is None:
      kd = self.kd
    error = self.base.rightMotor.angle() - self.base.leftMotor.angle() 
    self.update(error, kp, ki, kd)
    self.base.run(speed + self.correction, speed - self.correction)
    
def PID_Turn(base: Base, degrees: int, kp: float, ki: , kd):
  base.reset()
  left = PID(kp, ki, kd)
  right = PID(kp, ki, kd)
  while degrees - abs(base.leftMotor.angle()) > 10 or degrees - abs(base.rightMotor.angle()) > 10:
    left.update(degrees - base.leftMotor.angle(), kp, ki, kd)
    right.update(degrees - base.rightMotor.angle(), kp, ki, kd)
    leftOut = min(30, abs(left.correction))
    rightOut = min(30, abs(right.correction))
    base.run(leftOut, rightOut)
    
def PID_Distance(degrees: int,
                 speed: float, moveObj: PID_LineTrack, 
                 leeway: int = 5, 
                 kp: int = 1.5, 
                 ki: int = 0.01, 
                 kd: int = 1, 
                 minSpeed: int = 40):
  
  resetMotor(moveObj.leftMotor, moveObj.rightMotor)
  pid = PID(kp, ki, kd)
  while abs(moveObj.leftMotor.angle() - degrees) > leeway:
    # only activate PID slowdown when close to destination
    if (degrees - moveObj.leftMotor.angle()) < 50:    
      pid.update(degrees - moveObj.leftMotor.angle(), kp, ki, kd)
    
    moveObj.move(speed)