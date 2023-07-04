class SophiaWalkingCtrl:
  import sophia_walking_h as swh
  import sophia_h as sh
  import rclpy
  from sensor_msgs.msg import JointState
  from geometry_msgs.msg import Twist
  import time

  TURN_MAX     = 25.0
  DO_BLENDER   = False
  MAX_STEP_NUM = 5
  state_name   = []
  state_pos    = []
  ref_0        = []

  pub_ref     = None
  sub_walking = None
  node        = None
  T = 0.01
  def SophiaWalkingCtrl(self):
    pass

  def __init__(self):
    self.sw      = self.swh.SophiaWalking()
    self.DONE    = self.sw.DONE
    self.RUNNING = self.sw.RUNNING
    self.OK      = self.sw.OK
    self.FAIL    = self.sw.FAIL
    self.sophia  = self.sh.Sophia()
    self.ros_init()
    self.t0      = self.time.time()
    self.state_init()
    pass

  def turn(self, speed=0.002, val=0.0):
    do_ret = self.FAIL


    if val > self.TURN_MAX:
      val = self.TURN_MAX
    elif val < -self.TURN_MAX:
      val = -self.TURN_MAX

    while True:
      ret = self.sw.walk( T_walking=speed, d_turn=val, step_l = 0.0)
      if ret == self.sw.DONE:
        do_ret = self.OK
        break
    return do_ret

  def walk(self, speed=0.002, num_steps=1):
    do_ret = self.FAIL
    if num_steps > self.MAX_STEP_NUM:
      num_steps = self.MAX_STEP_NUM
    while True:
      ret = self.sw.walk(T_walking=speed, d_turn=0.0, step_l = 0.0, num_steps=num_steps)
      if ret == self.sw.DONE:
        do_ret = self.OK
        break
    return do_ret

  def ros_init(self):
    try:
      self.rclpy.init(args=None)
    except:
      pass
    self.node        = self.rclpy.create_node("Sophia_Walking_Ctrl")
    self.pub_ref     = self.node.create_publisher(self.JointState, self.sophia.ROS_CHAN_REF_POS, 1)
    self.sub_walking = self.node.create_subscription(self.JointState, self.sophia.ROS_CHAN_WALKING, self.cb_blender, 10)
    self.sub_state   = self.node.create_subscription(self.JointState, self.sophia.ROS_CHAN_STATE_POS, self.cb_state, 10)

  def state_init(self):
    for p in self.sophia.IDs:
      name = p[self.sophia.ENUM_NAME]
      self.state_name.append(name)
      self.state_pos.append(0.0)
      self.ref_0.append(0.0)

  def cb_state(self, msg):
    for n in msg.name:
      try:
        i = self.state_name.index(n)
        self.state_pos[i] = msg.position[i]
      except:
        pass

  def cb_blender(self, msg):
    lmsg = len(msg.name)
    if lmsg == 1:
      if msg.name[0] == 'turn':
        val = msg.position[0]
        speed = msg.velocity[0]
        print('speed from ros = ', speed)
        if speed <= 0.0:
          speed = 0.002
        if speed > 0.01:
          speed = 0.01
        print('speed from ros = ', speed)
        self.turn(val=val, speed=speed)
      elif msg.name[0] == 'step':
        val = msg.position[0]
        val_i = int(val)
        speed = msg.velocity[0]
        print('speed from ros = ', speed)
        if speed <= 0.0:
          speed = 0.002
        if speed > 0.01:
          speed = 0.01
        print('speed from ros = ', speed)
        self.walk(num_steps=val_i, speed=speed)
      elif msg.name[0] == 'blender':
        if msg.position[0] > 0:
          self.DO_BLENDER = True
        else:
          self.DO_BLENDER = False

    if self.DO_BLENDER == False:
      return
    else:
      self.pub_ref(msg)

  def sleep(self, TT=None):
    if TT == None:
      TT = self.T
    if TT < 0.0:
      TT = self.T
    t1 = self.time.time()
    dt = t1 - self.t0
    if dt < 0.0:
      self.rclpy.spin_once(self.node,timeout_sec=0)
      dt = TT
    while dt < TT:
      self.rclpy.spin_once(self.node,timeout_sec=0)
      self.time.sleep(0.002)
      t1 = self.time.time()
      dt = t1 - self.t0
    self.t0 = t1
    pass
  def update(self):
      self.rclpy.spin_once(self.node,timeout_sec=self.T)

  L = 200.0
  def zero(self, zero_ref=0.0):
    ctrl = self.JointState()
    
    for i in range(len(self.state_name)):
      r0 = self.state_pos[i]
      r = ( zero_ref + r0*(self.L - 1.0) ) / self.L
      ctrl.name.append(self.state_name[i])
      ctrl.position.append(r)

    self.pub_ref.publish(ctrl)
    


swc = SophiaWalkingCtrl()

while True:
  # updates callbacks and sleeps for swc.T sec
  #swc.sleep()
  swc.update()

  # goto zero
#  swc.zero()

  



