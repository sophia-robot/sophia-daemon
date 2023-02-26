import humanoidRobotCatWalk as hrcw
import time

robot = hrcw.HumanoidRobotCatWalk()

the_id = 0x15

print("open = ", end='')
print(robot.open())

print("baud = ", end='')
print(robot.setBaud())

robot.torque(the_id, robot.DISABLE)

while True:
  pos = robot.getPos(the_id)
  print(pos)
  time.sleep(0.02) 
