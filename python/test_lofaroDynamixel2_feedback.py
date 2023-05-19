import time

import rclpy
from sensor_msgs.msg import JointState

node = None
pub  = None
sub  = None

state = JointState()
state_rhp = 0.0
state_rap = 0.0
state_rkp = 0.0
state_rtp = 0.0

def state_cb(msg):
  global state
  global state_rhp
  global state_rap
  global state_rkp
  global state_rtp

  state = msg
  i_rhp = msg.name.index("rhp")
  i_rkp = msg.name.index("rkp")
  i_rap = msg.name.index("rap")
  i_rtp = msg.name.index("rtp")

  state_rhp = msg.position[i_rhp]
  state_rkp = msg.position[i_rkp]
  state_rap = msg.position[i_rap]
  state_rtp = msg.position[i_rtp]

def init():
  global node, pub, sub

  rclpy.init()
  node = rclpy.create_node('test_lofaro_dynamixel2_ros2_node')
  pub  = node.create_publisher(JointState, '/ref',1)
  sub  = node.create_subscription(JointState, '/state', state_cb, 1)
  sub


def loop():
  global pub
  global state
  global node

  j_zero = JointState()
  j_zero.name.append("rhp")
  j_zero.position.append(-0.0)
  j_zero.name.append("rap")
  j_zero.position.append(-0.0)
  j_zero.name.append("rkp")
  j_zero.position.append(0.0)
  j_zero.name.append("rtp")
  j_zero.position.append(0.0)

  T = 0.05
#  pub.publish(j_zero)
#  exit()
  while True:
    j_val = JointState()
    j_val.name.append('rhp')
#    j_val.name.append('rap')
#    j_val.name.append('rkp')
#    j_val.name.append('rtp')
    j_val.position.append(state_rhp)
#    j_val.position.append(state_rap)
#    j_val.position.append(state_rkp)
#    j_val.position.append(state_rtp)
    print(j_val)
    pub.publish(j_val)
    rclpy.spin_once(node)
    time.sleep(T)

init()
loop()
