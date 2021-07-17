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
           threshold: int = None, 
           kp: float = None, 
           ki: float = None, 
           kd: float = None, side = 1):
    # update control constants if given
    if threshold is None:
      threshold = self.threshold

    self.update(threshold - sensor.reflection(), kp, ki, kd) 
    self.base.run(speed + side * self.correction, speed - side * self.correction)

class PID_EncoderStraight(PID):
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

    error = self.base.rightMotor.angle() - self.base.leftMotor.angle() 
    self.update(error, kp, ki, kd)
    self.base.run(speed + self.correction, speed - self.correction)
    
def PID_EncoderTurn(base: Base, degrees: int, kp: float, ki: float, kd: float):
  base.reset()
  left = PID(kp, ki, kd)
  right = PID(kp, ki, kd)
  while degrees - abs(base.leftMotor.angle()) > 10 or degrees - abs(base.rightMotor.angle()) > 10:
    print('reading:',base.leftMotor.angle(), base.rightMotor.angle())
    left.update(degrees - base.leftMotor.angle(), kp, ki, kd)
    right.update(degrees - base.rightMotor.angle(), kp, ki, kd)
    leftOut = max(30, abs(left.correction))
    rightOut = max(30, abs(right.correction))
    base.run(leftOut, -rightOut)
    print(leftOut, -rightOut)


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
           kp: float = None, 
           ki: float = None, 
           kd: float = None,
           error = None):
    if error is None:
      error = self.gyro.angle()
    self.update(error, kp, ki, kd)
    self.base.run(speed - self.correction, speed + self.correction)
  
class PID_GyroTurn(PID_GyroStraight):  
  def __init__(self,
                base: Base, 
                kp: float, 
                ki: float, 
                kd: float, 
                gyro: GyroSensor):
      super().__init__(base, kp, ki, kd, gyro)
  
  def turn(self, angle, kp = None, ki = None, kd = None):
    self.base.reset()
    self.gyro.reset_angle(0)
    self.resetIntegral()

    while self.gyro.angle() != angle:
      self.move(0, kp, ki, kd, error = self.gyro.angle()-angle)
    self.base.stop()
      
def PID_SingleMotorTurn(motor, gyro, angle, kp = 1.8, ki = 0, kd = 2, minSpeed = 15, direction = 1):
  pid = PID(kp, ki, kd)
  while gyro.angle() != angle:
    error = (gyro.angle() - angle) * direction
    pid.update(error, kp, ki, kd)
    if abs(pid.correction) < minSpeed:
      motor.run(CorrectSpeed(minSpeed * (pid.correction / abs(pid.correction))))
    else:
      motor.run(CorrectSpeed(pid.correction))
  motor.hold()

def PID_AngleOffSet(base, gyro, angle):
  if angle > 0:
    PID_SingleMotorTurn(base.leftMotor, gyro, angle, direction = -1)
    PID_SingleMotorTurn(base.rightMotor, gyro, 0)
  else:
    PID_SingleMotorTurn(base.rightMotor, gyro, angle)
    PID_SingleMotorTurn(base.leftMotor, gyro, 0, direction = -1)
  


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
    
def PID_LineSquare(base, threshold = 50, kp = 0.2, ki = 0.0005, kd = 0.6, direction = 1, leeway = 3): # direction = 1 for forward, direction = -1 for backwar
  leftPID = PID(kp, ki, kd)
  rightPID = PID(kp, ki, kd)
  stopwatch = StopWatch()
  start = stopwatch.time()
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
    # print('Sensors: ', leftVal, rightVal)
    # print('Speed: ', outLeft, outRight)
    base.run(outLeft, outRight)
    if stopwatch.time() - start > 2000:
      break
  base.stop()
  