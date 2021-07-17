
def checkSurplus(ev3ColSensor, GyroStraight, degrees)
surplus = False
if ev3ColSensor.reflection() > 20:
  base.run_target(-30, degrees)
  
  return True
else:
  return False

def collectSurplus(ev3ColSensor,GyroStraight, degrees):
  GyroStraight.gyro.reset_angle(0)
  while ev3ColSensor.reflection() > 1:   
    GyroStraight.move(-40)
  while ev3ColSensor.reflection() < 20:   
    GyroStraight.move(-40)
  GyroStraight.base.stop()
  # while ev3ColSensor.reflection() > 1:   
  #   GyroStraight.move(-30)
  # base.stop()
  GyroTurn.turn(90)
  gyro.reset_angle(0)

  while colRight.reflection() > 15:
    GyroStraight.move(-40)
  base.hold()
  
  while colRight.reflection() < 80:
    GyroStraight.move(40)
  base.hold()
  
  PID_LineSquare(base, direction = -1)
  
  frontClaw.run_time(CorrectSpeed(100), 1200)
  base.reset()
  base.run_target(50, 400)
  frontClaw.run_target(CorrectSpeed(-30), -100)