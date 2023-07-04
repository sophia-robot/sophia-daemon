#!/usr/bin/env python

class HansonWalkingBridge:
  try:
    import rospy 
    from std_msgs.msg import String 
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
    node = self.rclpy.create_node('walking_ros2')
    self.sub = node.create_subscription(self.JointState, self.sophia.ROS_CHAN_WALKING, self.cb_walking, 10)

    self.pub  = node.create_publisher(self.JointState, self.sophia.ROS_CHAN_REF_POS,1)
    print("ROS2 Node Running")
    while True:
      self.rclpy.spin_once(node)

  def cb_walking(self, msg):
    try:
      self.pub.publish(msg)
    except:
      print("err")



import sys


if __name__ == '__main__':
  args = sys.argv[1:]
  hw = None

  hw = HansonWalkingBridge()

  hw.ros2()
