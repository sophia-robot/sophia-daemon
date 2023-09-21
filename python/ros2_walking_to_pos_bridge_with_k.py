#!/usr/bin/env python

class HansonWalkingBridge:
  from std_msgs.msg import Float64
  try:
    import rospy 
    from std_msgs.msg import String 
    from std_msgs.msg import Float64
    from hr_msgs.msg import TargetPosture 
  except:
    String        = None
    TargetPosture = None
    rospy         = None
  import socket

  import sophia_h as sh
  sophia = sh.Sophia()

  try:
    import rclpy
    from sensor_msgs.msg import JointState
  except:
    rclpy = None
    JointState = None

  def __init__(self, mode=None):
    pass

  sub = None
  pub = None
  def ros2(self):
    self.rclpy.init()
    node = self.rclpy.create_node('walking_ros2_k')
    self.sub           = node.create_subscription(self.JointState, self.sophia.ROS_CHAN_WALKING, self.cb_walking, 10)

    self.walking_k = 1.0
    self.sub_walking_k = node.create_subscription(self.Float64, self.sophia.ROS_CHAN_WALKING_K, self.cb_walking_k, 10)

    self.pub  = node.create_publisher(self.JointState, self.sophia.ROS_CHAN_REF_POS,1)
    print("ROS2 Node Running")
    while True:
      self.rclpy.spin_once(node)

  def cb_walking(self, msg):
    try:
      for i in range(len(msg.position)):
        extra_k = 1.0
        name = msg.name[i]
        if (name == 'rhp') | (name == 'lhp'):
          if msg.position[i] < 0.0:
            extra_k = 2.5
        elif (name == 'rap') | (name == 'lap'):
          if msg.position[i] < 0.0:
            extra_k = 3.0
#        elif (name == 'rkp') | (name == 'lkp'):
#          if msg.position[i] > 0.0:
#            extra_k = 3.0
        msg.position[i] = msg.position[i] * self.walking_k * extra_k
      self.pub.publish(msg)
    except:
      print("err")

  def cb_walking_k(self, msg):
    self.walking_k = msg.data


import sys


if __name__ == '__main__':
  args = sys.argv[1:]
  hw = None

  hw = HansonWalkingBridge()

  hw.ros2()
