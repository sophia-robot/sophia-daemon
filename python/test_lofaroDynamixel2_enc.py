import lofaroDynamixel2 as ld2
import time

robot = ld2.LofaroDynamixel2()

the_id = 0x1b

print("open = ", end='')
print(robot.open())

print("baud = ", end='')
print(robot.setBaud(baud=3000000))

robot.torque(the_id, robot.DISABLE)

while True:
  pos, err = robot.getPos(the_id)
  print(type(pos))
  pos_enc, err = robot.getPosEnc(the_id)
  pos_convt = robot.rad2enc(pos)
  print((pos, pos_enc, pos_convt))
  time.sleep(0.02) 
