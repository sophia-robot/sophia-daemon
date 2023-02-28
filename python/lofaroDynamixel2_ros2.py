import lofaroDynamixel2 as ld2
import time

import rclpy
from sensor_msgs.msg import JointState

robot = None
node  = None
pub   = None
sub   = None

ENUM_NAME    = 0
ENUM_REF     = 1
ENUM_STATE   = 2
ENUM_ENABLED = 3

#        name   pos_id  enc_id  enabled
IDs = { ("rhy", 0x1c,   0x10,   False ),
        ("rhr", 0x1d,   0x12,   False  ),
        ("rhp", 0x1e,   0x14,   False  ),
        ("rkn", 0x1f,   0x16,   False  ),
        ("ray", 0x18,   0x18,   False  ),
        ("rar", 0x19,   0x19,   False  ),
        ("rap", 0x1a,   0x1a,   False  ),
        ("rtp", 0x1b,   0x1b,   True  ), 
        ("lhy", 0x3c,   0x30,   False ),
        ("lhr", 0x3d,   0x32,   False ),
        ("lhp", 0x3e,   0x34,   False ),
        ("lkn", 0x3f,   0x36,   False ),
        ("lay", 0x38,   0x38,   False ),
        ("lar", 0x39,   0x39,   False ),
        ("lap", 0x3a,   0x3a,   False ),
        ("ltp", 0x3b,   0x3b,   False ) 
      }

def callback(msg):
  m = msg
  for i in range(len(msg.name)):
#    try:
      for p in IDs:
        pos = m.position[i]

        if p[ENUM_ENABLED]:
          name    = p[ENUM_NAME]
          the_id  = p[ENUM_REF]
          enabled = p[ENUM_ENABLED]
          name2 = m.name[i]
          if (name2 == name):       
            err    = robot.stagePos(the_id, pos)
#    except:
#      pass
  pass

def init():
  global robot, node, pub, sub

  robot = ld2.LofaroDynamixel2(baud=3000000, port='/dev/ttyUSB0')
  robot.open()
  robot.setBaud()
  rclpy.init()
  node = rclpy.create_node('lofaro_dynamixel2_ros2_node')
  pub  = node.create_publisher(JointState, '/state',1)
  sub  = node.create_subscription(JointState,'/ref', callback, 10)

def torqueEnable():
  for p in IDs:
    name    = p[ENUM_NAME]
    the_id  = p[ENUM_REF]
    enabled = p[ENUM_ENABLED]
    if enabled:
      err = robot.torque(the_id, robot.ENABLE)
      print("Torque Enable status for ", end='')
      print(hex(the_id), end=' = ')
      print(err)

t0 = time.time()
T_des = 0.02
def sleep():
  global t0, node
  t1 = time.time()
  dt = t1 - t0
  if dt < 0.0:
    rclpy.spin_once(node,timeout_sec=0)
    dt = T_des
  while dt < T_des:
    rclpy.spin_once(node,timeout_sec=0)
    time.sleep(0.002)
    t1 = time.time()
    dt = t1 - t0
  t0 = t1 
  pass

def putPos():
  robot.putPos()
  pass

def getPos():
  state = JointState()
  for p in IDs:
    if p[ENUM_ENABLED]:
      name   = p[ENUM_NAME]
      the_id = p[ENUM_STATE]
      pos, err    = robot.getPos(the_id)
      if err == robot.OK:
        state.name.append(name)
        state.position.append(pos)

  pub.publish(state) 
  pass

def loop():
  i = 0
  while True:
    putPos()
    getPos()
    sleep()
    print(".",end='')
    i = i+1
    if i > 100:
      print()
      i = 0

init()
torqueEnable()
loop()
#robot.torque(the_id, robot.DISABLE)


#while True:
#  pos = robot.getPos(the_id)
#  print(pos)
#  time.sleep(0.02) 
