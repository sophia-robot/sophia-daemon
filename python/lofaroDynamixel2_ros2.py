import lofaroDynamixel2 as ld2
import time

import rclpy
from sensor_msgs.msg import JointState

robot = None
node  = None
pub   = None
sub   = None

ENUM_NAME          = 0
ENUM_REF           = 1
ENUM_STATE         = 2
ENUM_ENABLED       = 3
ENUM_POS_MAX       = 4
ENUM_POS_MIN       = 5
ENUM_MOT_INDEX     = 6

the_id_i = -1
def getNextI():
  global the_id_i
  the_id_i += 1
  return the_id_i

#        name   pos_id  enc_id  enabled  max_pos (rad)   min_pos (rad)  filter_id
IDs = { ("rhy", 0x1c,   0x10,   False,   1.0,            -1.0,          getNextI()),
        ("rhr", 0x1d,   0x12,   False,   0.2,            -0.2,          getNextI()),
        ("rhp", 0x1e,   0x14,   False,   0.7,            -1.3,          getNextI()),
        ("rkn", 0x1f,   0x16,   False,   1.2,             0.0,          getNextI()),
        ("ray", 0x18,   0x18,   False,   1.0,            -1.0,          getNextI()),
        ("rar", 0x19,   0x19,   False,   1.0,            -1.0,          getNextI()),
        ("rap", 0x1a,   0x1a,   False,   0.4,            -1.0,          getNextI()),
        ("rtp", 0x1b,   0x1b,   True ,   1.0,            -1.0,          getNextI()), 
        ("lhy", 0x3c,   0x30,   False,   1.0,            -1.0,          getNextI()),
        ("lhr", 0x3d,   0x32,   False,   0.2,            -0.2,          getNextI()),
        ("lhp", 0x3e,   0x34,   False,   0.7,            -1.3,          getNextI()),
        ("lkn", 0x3f,   0x36,   False,   1.2,             0.0,          getNextI()),
        ("lay", 0x38,   0x38,   False,   1.0,            -1.0,          getNextI()),
        ("lar", 0x39,   0x39,   False,   1.0,            -1.0,          getNextI()),
        ("lap", 0x3a,   0x3a,   False,   0.4,            -1.0,          getNextI()),
        ("ltp", 0x3b,   0x3b,   False,   1.0,            -1.0,          getNextI()) 
      }

FILTER_REF_0    = None
FILTER_REF_1    = None
FILTER_L        = None
FILTER_MOT_NUM  = len(IDs)
STATE_POS       = None

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

def init(L=None):
  global robot, node, pub, sub, FILTER_REF_0, FILTER_REF_1, FILTER_L
  
  if (L==None):
    L = 100
  if (L < 0):
    L = 100
  
  FILTER_L = L

  FILTER_REF_0 = [0.0]  
  FILTER_REF_1 = [0.0]
  STATE_POS    = [0.0]
  for i in range(1, FILTER_MOT_NUM):
    FILTER_REF_0.append(0.0)
    FILTER_REF_1.append(0.0)
    STATE_POS.append(0.0)

  robot = ld2.LofaroDynamixel2(baud=3000000, port='/dev/ttyUSB0')
  robot.open()
  robot.setBaud()
  rclpy.init()
  node = rclpy.create_node('lofaro_dynamixel2_ros2_node')
  pub  = node.create_publisher(JointState, '/state',1)
  sub  = node.create_subscription(JointState,'/ref', callback, 10)

  for i in range(10):
    init_filter_ref()
    time.sleep(0.05)

def init_filter_ref():
  global FILTER_REF_0, FILTER_REF_1, STATE_POS
  for p in IDs:
    if p[ENUM_ENABLED]:
      name        = p[ENUM_NAME]
      the_id      = p[ENUM_STATE]
      mot_index   = p[ENUM_MOT_INDEX]
      print(mot_index)
      pos, err    = robot.getPos(the_id)
      print(pos)
      if err == robot.OK:
          STATE_POS[mot_index] = pos
          FILTER_REF_0[mot_index] = pos
          FILTER_REF_1[mot_index] = pos


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
  global STATE_POS
  state = JointState()
  for p in IDs:
    if p[ENUM_ENABLED]:
      name        = p[ENUM_NAME]
      the_id      = p[ENUM_STATE]
      mot_index   = P[ENUM_MOT_INDEX]
      pos, err    = robot.getPos(the_id)
      if err == robot.OK:
        state.name.append(name)
        state.position.append(pos)
        STATE_POS[mot_index] = pos

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
