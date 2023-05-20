import lofaroDynamixel2 as ld2
import time

import rclpy
from sensor_msgs.msg import JointState

robot = None
node  = None
pub   = None
sub   = None
tor   = None

ENUM_NAME          = 0
ENUM_TORQUE_EN_0   = 1
ENUM_TORQUE_EN_1   = 2
ENUM_REF           = 3
ENUM_STATE         = 4
ENUM_ENABLED       = 5
ENUM_POS_MAX       = 6
ENUM_POS_MIN       = 7
ENUM_MOT_INDEX     = 8

the_id_i = -1
def getNextI():
  global the_id_i
  the_id_i += 1
  return the_id_i

ENABLE_RIGHT = True
ENABLE_LEFT  = True

#        name   torque_en_0  torque_en_1  pos_id  enc_id  enabled         max_pos (rad)   min_pos (rad)  filter_id
IDs = { ("lhy", 0x30,        0x31,        0x3c,   0x30,   ENABLE_LEFT ,   1.0,            -1.0,          getNextI()),
        ("lhr", 0x32,        0x33,        0x3d,   0x32,   ENABLE_LEFT ,   0.2,            -0.2,          getNextI()),
        ("lhp", 0x34,        0x35,        0x3e,   0x34,   ENABLE_LEFT ,   0.7,            -1.3,          getNextI()),
        ("lkp", 0x36,        0x37,        0x3f,   0x36,   ENABLE_LEFT ,   1.2,             0.0,          getNextI()),
        ("lay", 0x38,        0x38,        0x38,   0x38,   ENABLE_LEFT ,   1.0,            -1.0,          getNextI()),
        ("lar", 0x39,        0x39,        0x39,   0x39,   ENABLE_LEFT ,   1.0,            -1.0,          getNextI()),
        ("lap", 0x3a,        0x3a,        0x3a,   0x3a,   ENABLE_LEFT ,   0.4,            -1.0,          getNextI()),
        ("ltp", 0x3b,        0x3b,        0x3b,   0x3b,   ENABLE_LEFT ,   1.0,            -1.0,          getNextI()), 
        ("rhy", 0x10,        0x11,        0x1c,   0x10,   ENABLE_RIGHT ,   1.0,            -1.0,          getNextI()),
        ("rhr", 0x12,        0x13,        0x1d,   0x12,   ENABLE_RIGHT ,   0.2,            -0.2,          getNextI()),
        ("rhp", 0x14,        0x15,        0x1e,   0x14,   ENABLE_RIGHT ,   0.7,            -1.3,          getNextI()),
        ("rkp", 0x16,        0x17,        0x1f,   0x16,   ENABLE_RIGHT ,   1.2,             0.0,          getNextI()),
        ("ray", 0x18,        0x18,        0x18,   0x18,   ENABLE_RIGHT ,   1.0,            -1.0,          getNextI()),
        ("rar", 0x19,        0x19,        0x19,   0x19,   ENABLE_RIGHT ,   1.0,            -1.0,          getNextI()),
        ("rap", 0x1a,        0x1a,        0x1a,   0x1a,   ENABLE_RIGHT ,   0.4,            -1.0,          getNextI()),
        ("rtp", 0x1b,        0x1b,        0x1b,   0x1b,   ENABLE_RIGHT ,   1.0,            -1.0,          getNextI()) 
      }

FILTER_REF_0    = None
FILTER_REF_1    = None
FILTER_REF_GOAL = None
FILTER_L        = None
FILTER_MOT_NUM  = len(IDs)
STATE_POS       = None
STATE_TORQUE    = None

FILTER_L_DEFAULT = 50

def callback(msg):
  global FILTER_REF_GOAL
  m = msg
  for i in range(len(msg.name)):
#    try:
      for p in IDs:
        pos = m.position[i]

        if p[ENUM_ENABLED]:
          name    = p[ENUM_NAME]
          the_id  = p[ENUM_REF]
          enabled = p[ENUM_ENABLED]
          mot_index   = p[ENUM_MOT_INDEX]
          name2 = m.name[i]
          if (name2 == name):
            FILTER_REF_GOAL[mot_index] = pos
#            print(name, end=' ')       
#            err    = robot.stagePos(the_id, pos)
#    except:
#      pass
  pass

def doFilterRef(mode=None):
  global FILTER_REF_0
  for p in IDs:
    the_id      = p[ENUM_REF]
    mot_index   = p[ENUM_MOT_INDEX]
    if p[ENUM_ENABLED]:
      r = (FILTER_REF_0[mot_index] * (FILTER_L - 1.0) + FILTER_REF_GOAL[mot_index]) / (FILTER_L * 1.0)
      FILTER_REF_0[mot_index] = r
      err = stagePos(the_id, r)  
      #err = robot.stagePos(the_id, r)  

def stagePos(the_id, r):
    err = 0
    for p in IDs:
      enabled = p[ENUM_ENABLED]

      if the_id == p[ENUM_REF]:
        if enabled:
          if r > p[ENUM_POS_MAX]:
            r = p[ENUM_POS_MAX]
          if r < p[ENUM_POS_MIN]:
            r = p[ENUM_POS_MIN]
          err += robot.stagePos(the_id, r)  
    return err

def init(L=None):
  global robot, node, pub, sub, tor, FILTER_REF_0, FILTER_REF_1, FILTER_L, STATE_POS, FILTER_REF_GOAL, STATE_TORQUE
  
  if (L==None):
    L = FILTER_L_DEFAULT
  elif (L < 0):
    L = FILTER_L_DEFAULT
  
  FILTER_L = L

  FILTER_REF_0    = [0.0]  
  FILTER_REF_1    = [0.0]
  FILTER_REF_GOAL = [0.0]
  STATE_POS       = [0.0]
  STATE_TORQUE    = [0.0]
  for i in range(1, FILTER_MOT_NUM):
    FILTER_REF_0.append(0.0)
    FILTER_REF_1.append(0.0)
    FILTER_REF_GOAL.append(0.0)
    STATE_POS.append(0.0)
    STATE_TORQUE.append(0.0)


  robot = ld2.LofaroDynamixel2(baud=3000000, port='/dev/ttyUSB0')
  robot.open()
  robot.setBaud()
  rclpy.init()
  node = rclpy.create_node('lofaro_dynamixel2_ros2_node')
  pub  = node.create_publisher(JointState, '/state',1)
  tor  = node.create_publisher(JointState, '/torque',1)
  sub  = node.create_subscription(JointState,'/ref', callback, 10)

  print('Init ......')
  for i in range(10):
    init_filter_ref()
    time.sleep(0.05)

  for i in range(len(STATE_POS)):
    FILTER_REF_0[i]    = STATE_POS[i]
    FILTER_REF_1[i]    = STATE_POS[i]
    FILTER_REF_GOAL[i] = STATE_POS[i]

def init_filter_ref():
  global FILTER_REF_0, FILTER_REF_1, STATE_POS
  for p in IDs:
    if p[ENUM_ENABLED]:
      name        = p[ENUM_NAME]
      the_id      = p[ENUM_STATE]
      mot_index   = p[ENUM_MOT_INDEX]
      pos, err    = robot.getPos(the_id)
      if err == robot.OK:
          STATE_POS[mot_index] = pos
          FILTER_REF_0[mot_index] = pos
          FILTER_REF_1[mot_index] = pos


def torqueEnable():
  for p in IDs:
    name      = p[ENUM_NAME]
    the_id_0  = p[ENUM_TORQUE_EN_0]
    the_id_1  = p[ENUM_TORQUE_EN_1]
    enabled   = p[ENUM_ENABLED]
    if enabled:
      err = robot.torque(the_id_0, robot.ENABLE)
      print("Torque Enable status for ", end='')
      print(hex(the_id_0), end=' = ')
      print(err)
      if the_id_1 != the_id_0:
        err = robot.torque(the_id_1, robot.ENABLE)
        print("Torque Enable status for ", end='')
        print(hex(the_id_1), end=' = ')
        print(err)

t0 = time.time()
T_des = 0.03
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
      mot_index   = p[ENUM_MOT_INDEX]
      pos, err    = robot.getPos(the_id)
      if err == robot.OK:
        state.name.append(name)
        state.position.append(pos)
        STATE_POS[mot_index] = pos

  pub.publish(state) 
  pass

def getTorque():
  global STATE_TORQUE
  state = JointState()
  for p in IDs:
    if p[ENUM_ENABLED]:
      name        = p[ENUM_NAME]
      the_id      = p[ENUM_STATE]
      mot_index   = p[ENUM_MOT_INDEX]
      torque, err    = robot.getTorque(the_id)
      if err == robot.OK:
        state.name.append(name)
        state.effort.append(torque)
        STATE_TORQUE[mot_index] = torque

  tor.publish(state) 
  pass

def loop():
  i = 0
  while True:
    doFilterRef()
    putPos()
    getPos()
    getTorque()
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
