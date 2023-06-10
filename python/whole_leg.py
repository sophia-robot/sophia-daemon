import time

import rclpy
from sensor_msgs.msg import JointState

node = None
pub  = None
sub  = None

def init():
  global node, pub, sub

  rclpy.init()
  node = rclpy.create_node('test_lofaro_dynamixel2_ros2_node')
  pub  = node.create_publisher(JointState, '/ref',1)

def loop():
  global pub
  bend = -1.1
  j_bend = JointState()
  j_bend.name.append("rhp")
  j_bend.position.append(bend)
  j_bend.name.append("rap")
  j_bend.position.append(-0.5)
  j_bend.name.append("rkp")
  j_bend.position.append(-2.0*bend)
  j_bend.name.append("rtp")
  j_bend.position.append(bend)

  j_zero = JointState()
  j_zero.name.append("rhp")
  j_zero.position.append(-0.0)
  j_zero.name.append("rap")
  j_zero.position.append(-0.0)
  j_zero.name.append("rkp")
  j_zero.position.append(0.0)
  j_zero.name.append("rtp")
  j_zero.position.append(0.0)

  T = 5.0
  while True:
    pub.publish(j_bend)
    time.sleep(T)
    pub.publish(j_zero)
    time.sleep(T)

init()
loop()
