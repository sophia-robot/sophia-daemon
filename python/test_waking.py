class TestWalking:
  import time
  import rclpy
  from sensor_msgs.msg import JointState
  import sophia_h as sh
  sophia = sh.Sophia()

  def __init__(self):
    self.rclpy.init()
    self.node = self.rclpy.create_node('test_walking_node')
    self.pub  = self.node.create_publisher(   self.JointState, self.sophia.ROS_CHAN_REF_POS,1)
    self.sub  = self.node.create_subscription(self.JointState, self.sophia.ROS_CHAN_WALKING, self.cb_walking, 1)

  def cb_walking(self, msg):
    k = 0.8
    for i in range(len(msg.name)):
      d = 1.0
      n = msg.name[i]
      if   n == 'lkp':
        d = -d
      elif n == 'lhp':
        d = -d
      elif n == 'ltp':
        d = -d
      elif n == 'lap':
        d = -d
      elif n == 'rkp':
        d = -d
      elif n == 'rhp':
        d = -d
      elif n == 'rtp':
        d = -d
      elif n == 'rap':
        d = -d
      msg.position[i] = msg.position[i]*k*d
    self.pub.publish(msg)

  def loop(self):
    self.rclpy.spin(self.node)

tw = TestWalking()
tw.loop()
