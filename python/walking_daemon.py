class SophiaWalkingDaemon:
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
    self.sub  = self.node.create_subscription(self.Twist, self.sophia.ROS_CHAN_TURN_CTRL, self.cb_turn, 1)
    pass

  def SophiaWalkingDaemon(self):
    pass

  def rad2deg(self, val):
    return val * 180.0 / self.np.pi

  def deg2rad(self, val):
    return val * self.np.pi / 180.0

  def cb_turn(self, msg):
    turn = msg.angular.z
    if turn >  self.turn_max:
      turn =  self.turn_max
    if turn < -self.turn_max:
      turn = -self.turn_max
    print("Turn = ",turn)
    while True:
      ret = self.walking.walk(d_turn=turn)
      if ret == self.walking.DONE:
        break
    pass

  def loop(self):
    print("Turning Daemon Started")
    while self.rclpy.ok():
      self.rclpy.spin_once(self.node)
    self.node.destroy_node()
    self.rclpy.shutdown()


if __name__ == '__main__':
  swd = SophiaWalkingDaemon()
  swd.loop()



