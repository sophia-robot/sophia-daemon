import lofaroDynamixel2 as ld2
import time

robot = ld2.LofaroDynamixel2(baud=3000000)

the_id = 0x15

print("open = ", end='')
print(robot.open())

print("baud = ", end='')
print(robot.setBaud())

robot.torque(the_id, robot.DISABLE)


for i in range(1,253):
  ret = robot.ping(i)
  print(ret)
  time.sleep(0.1) 
