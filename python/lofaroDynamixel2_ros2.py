import lofaroDynamixel2 as ld2
import time

import rclpy
from sensor_msgs.msg import JointState

robot = None
node  = None
pub   = None
sub   = None

ENUM_NAME    = 0
ENUM_POS     = 1
ENUM_ENC     = 2
ENUM_ENABLED = 3

#        name   pos_id  enc_id  enabled
IDs = { ("rhy", 0x1c,   0x10,   False ),
        ("rhr", 0x1d,   0x12,   True  ),
        ("rhp", 0x1e,   0x14,   True  ),
        ("rkn", 0x1f,   0x16,   True  ),
        ("ray", 0x18,   0x18,   True  ),
        ("rar", 0x19,   0x19,   True  ),
        ("rap", 0x1a,   0x1a,   True  ),
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
  pass

def init():
  global robot, node, pub, sub

  robot = ld2.LofaroDynamixel2(baud=3000000, port='/dev/ttyUSB0')
  robot.open()
  robot.setBaud()
  rclpy.init()
  node = rclpy.create_node('lofaro_dynamixel2_ros2_node')
  pub  = node.create_publisher(JointState, '/state',1)
  sub  = node.create_subscription(JointState,'/ref', callback, 1)

def torqueEnable():
  print(robot)

t0 = time.time()
T_des = 0.5
def sleep():
  global t0
  t1 = time.time()
  tSleep = T_des - (t1-t0)
  if tSleep < 0.0:
    tSleep = 0.0
    print('.', end='')
  time.sleep(tSleep)
  t0 = time.time() 
  pass

def setPos():
  pass

def getPos():
  state = JointState()
  for p in IDs:
#    print(p)
    if p[ENUM_ENABLED]:
      name   = p[ENUM_NAME]
      the_id = p[ENUM_ENC]
      tt0 = time.time()
      pos, err    = robot.getPos(the_id)
      tt1 = time.time()
      print(name, end=' ')
      print(tt1-tt0)
      if err == robot.OK:
        state.name.append(name)
        state.position.append(pos)
#    pos = robot.getPos(p{ENUM_ENC})
#    state.name +=  p{ENUM_NAME}

  pub.publish(state) 
  pass

def loop():
  while True:
    setPos()
    getPos()
    sleep()

init()
torqueEnable()
loop()
#robot.torque(the_id, robot.DISABLE)


#while True:
#  pos = robot.getPos(the_id)
#  print(pos)
#  time.sleep(0.02) 
