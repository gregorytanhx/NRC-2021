def CorrectSpeed(x):
  return (x/100) * 1400

def LineSquare(leftMotor, rightMotor, leftSensor, rightSensor, color, speed):
  run1 = True
  run2 = True
  while run1 or run2:
    if leftSensor.color() == color:
      leftMotor.hold()
      run1 = False
    else:
      leftMotor.run(CorrectSpeed(speed))
      
    if rightSensor.color() == color:
      rightMotor.hold()
      run2 = False
    else:
      rightMotor(CorrectSpeed(speed))
  
def resetMotor(leftMotor, rightMotor):
  leftMotor.reset_angle(0)
  rightMotor.reset_angle(0)

def stop(leftMotor, rightMotor):
  leftMotor.hold()
  rightMotor.hold()