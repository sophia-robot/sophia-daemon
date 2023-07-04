class SophiaWalkingCtrl:
  import sophia_walking_h as swh
  import sophia_h as sh
  import rclpy
  from sensor_msgs.msg import JointState
  from geometry_msgs.msg import Twist


  TURN_MAX = 40.0
  DO_BLENDER = False

  pub_ref     = None
  sub_walking = None
  node        = None

  def SophiaWalkingCtrl(self):
    pass

  def __init__(self):
    self.sw      = self.swh.SophiaWalking()
    self.DONE    = self.sw.DONE
    self.RUNNING = self.sw.RUNNING
    self.Ok      = self.sw.OK
    self.FAIL    = self.sw.FAIL
    self.sophia  = sh.Sophia()
    self.init_ros()
    pass

  def turn(self, speed=0.002, val=0.0):
    do_ret = self.FAIL


    if val > self.TURN_MAX:
      val = self.TURN_MAX
    elif val < -self.TURN_MAX:
      val = -self.TURN_MAX

    while True:
      ret = self.sw.walk(T_walking=speed, d_turn=val, step_l = 0.0)
      if ret == self.sw.DONE:
        do_ret = self.OK
        break
    return do_ret

  def walk(self, speed=0.002, num_steps=1):
    do_ret = self.FAIL

    while True:
      ret = self.sw.walk(T_walking=speed, d_turn=0.0, step_l = 0.0, num_steps=num_steps)
      if ret == self.sw.DONE:
        do_ret = self.OK
        break
    return do_ret

  def init_ros(self):
    self.rclpy.init(args=None)
    self.node        = self.rclpy.create_node("Sophia_Walking_Ctrl")
    self.pub_ref     = self.node.create_publisher(self.JointState, self.sophia.ROS_CHAN_REF_POS, 1)
    self.sub_walking = self.node.create_subscription(self.JointState, self.sophia.ROS_CHAN_WALKING, self.cb_blender, 1)

  def cb_blender(self, msg):
    lmsg = len(msg.names)
    if lmsg == 1:
      # do cmd thing

    if DO_BLENDER == False:
      return
    else:
      self.pub_ref(msg)

