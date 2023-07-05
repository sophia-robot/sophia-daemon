class TestSophiaWalkingDaemon:
  import sophia_walking_h as swh
  import numpy as np
  import rclpy
  from geometry_msgs.msg import Twist
  import sophia_h as sh
  def __init__(self):
    self.sophia = self.sh.Sophia()
    self.turn_max = 10.0
    self.walking = self.swh.SophiaWalking()
    try:
      self.rclpy.init()
    except:
      pass
    self.node = self.rclpy.create_node('sophia_turn_ctrl')
    self.pub  = self.node.create_publisher(self.Twist, self.sophia.ROS_CHAN_TURN_CTRL, 1)
    pass

  def TestSophiaWalkingDaemon(self):
    pass

  def rad2deg(self, val):
    return val * 180.0 / self.np.pi

  def deg2rad(self, val):
    return val * self.np.pi / 180.0

  def turn(self, val):
    print("Turning = ", val)
    msg = self.Twist()
    msg.angular.z = val
    self.pub.publish(msg)

  def exit(self):
    self.node.destroy_node()
    self.rclpy.shutdown()

import time
if __name__ == '__main__':
  swd = TestSophiaWalkingDaemon()
  swd.turn(0.0)
  time.sleep(0.1)
  swd.turn(0.0)
  time.sleep(5.0)
  swd.turn(0.0)
  time.sleep(5.0)
  swd.exit()



