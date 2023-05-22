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
  pub  = node.create_publisher(JointState, '/ref/pos',1)

def loop():
  global pub
  bend = 0.0
  j_bend = JointState()
  j_bend.name.append("rhy")
  j_bend.position.append(bend)
  j_bend.name.append("rhr")
  j_bend.position.append(bend)
  j_bend.name.append("rhp")
  j_bend.position.append(bend)
  j_bend.name.append("rkp")
  j_bend.position.append(bend)
  j_bend.name.append("ray")
  j_bend.position.append(bend)
  j_bend.name.append("rar")
  j_bend.position.append(bend)
  j_bend.name.append("rap")
  j_bend.position.append(bend)
  j_bend.name.append("rtp")
  j_bend.position.append(bend)

  j_bend.name.append("lhy")
  j_bend.position.append(bend)
  j_bend.name.append("lhr")
  j_bend.position.append(bend)
  j_bend.name.append("lhp")
  j_bend.position.append(bend)
  j_bend.name.append("lkp")
  j_bend.position.append(bend)
  j_bend.name.append("lay")
  j_bend.position.append(bend)
  j_bend.name.append("lar")
  j_bend.position.append(bend)
  j_bend.name.append("lap")
  j_bend.position.append(bend)
  j_bend.name.append("ltp")
  j_bend.position.append(bend)

  for i in range(5):
    pub.publish(j_bend)
    time.sleep(0.1)

init()
loop()
