from helper import *

class PID(object):
  def __init__(self, kp, ki, kd):
    self.kp = kp
    self.ki = ki
    self.kd = kd
    self.integral = 0
    self.lastError = 0
    self.leftMotor = leftMotor
    self.rightMotor = rightMotor
    self.correction = 0
    
  def reset(self):
    self.integral = 0
    
  def update_pid(self, error, kp, ki, kd):
    self.integral += error
    self.correction = kp * error + ki * self.integral + kd * (error - self.lastError)
    self.lastError = error
    
   
class PID_LineTrack(PID):
  def __init__(self, leftMotor, rightMotor, kp, ki, kd):
    super().__init__(kp, ki, kd)
    self.leftMotor = leftMotor
    self.rightMotor = rightMotor
  def track(self, sensor, speed, threshold, kp = None, ki = None, kd = None):
    # update control constants if given
    if kp is None:
      kp = self.kp
    if ki is None:
      ki = self.ki
    if kd is None:
      kd = self.kd
    self.update_pid(threshold - sensor.reflection(), kp, ki, kd) 
    self.leftMotor.run(CorrectSpeed(speed + self.correction))
    self.rightMotor.run(CorrectSpeed(speed - self.correction))
    print(self.correction)
    print(CorrectSpeed(speed + self.correction))

class PID_Straight(PID):
  def __init__(self, leftMotor, rightMotor, kp, ki, kd):
    super().__init__(kp, ki, kd)
    self.leftMotor = leftMotor
    self.rightMotor = rightMotor
  def reset(self):
    self.leftMotor.reset_angle(0)
    self.rightMotor.reset_angle(0)
  def move(self, speed, kp = None, ki = None, kd = None):
    if kp is None:
      kp = self.kp
    if ki is None:
      ki = self.ki
    if kd is None:
      kd = self.kd
    error = self.rightMotor.angle() - self.leftMotor.angle() 
    self.update_pid(error, kp, ki, kd)
    self.leftMotor.run(CorrectSpeed(speed + self.correction))
    self.rightMotor.run(CorrectSpeed(speed - self.correction))
    
def PID_Turn(leftMotor, rightMotor, degrees, kp, ki, kd):
  leftMotor.reset_angle(0)
  rightMotor.reset_angle(0)
  left = PID(kp, ki, kd)
  right = PID(kp, ki, kd)
  while degrees - abs(leftMotor.angle()) > 10 or degrees - abs(rightMotor.angle()) > 10:
    left.update_pid(degrees - leftMotor.angle(), kp, ki, kd)
    right.update_pid(degrees - rightMotor.angle(), kp, ki, kd)
    leftOut = min(30, abs(left.correction))
    rightOut = min(30, abs(right.correction))
    leftMotor.run(CorrectSpeed(leftOut))
    rightMotor.run(-CorrectSpeed(rightMotor))
    
  
  
  
class PID_Turn(PID):
  def __init__(self):
    super().__init__()
  def turn(self, kp = None, ki = None, kd = None):
    if kp is None:
      kp = self.kp
    if ki is None:
      ki = self.ki
    if kd is None:
      kd = self.kd
    while degrees - self.leftMotor.angle() > 10:
      error = degrees - leftMotor.angle()
      correction = self.update(error, kp, ki, kd)
      self.leftMotor.run(CorrectSpeed(self.correction))
      self.rightMotor.run(CorrectSpeed(-self.correction))
    self.leftMotor.hold()
    self.rightMotor.hold()