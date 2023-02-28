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
  for m in msg:
    for p in IDs:
      pos = m.position

      if p[ENUM_ENABLED]:
        name    = p[ENUM_NAME]
        the_id  = p[ENUM_REF]
        enabled = p[ENUM_ENABLED]
        if (m.name == name):          
          print(1)
  pass

def init():
  global node, pub, sub

  rclpy.init()
  node = rclpy.create_node('test_lofaro_dynamixel2_ros2_node')
  pub  = node.create_publisher(JointState, '/ref',1)
  sub  = node.create_subscription(JointState,'/state', callback, 1)

t0 = time.time()
T_des = 1.0
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

def loop():
  global pub
  j = JointState()
  j.name.append("rtp")
  j.position.append(0.1)
  while True:
    j.position[0] = -j.position[0]
    pub.publish(j)
    sleep()

init()
loop()
