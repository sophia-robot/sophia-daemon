



class SophiaControlExample:
  import rclpy
  from geometry_msgs.msg import Twist
  from sensor_msgs.msg import JointState
  import time
  import sophia_h as sh
  import sophia_walking_h as swh

  node   = None
  pub    = None
  sub    = None
  sophia = None

  T = 0.01
  t0 = None
  def SophiaControlExample(self):
    return 

  def __init__(self):
    self.init()

  def init(self):
    self.t0 = self.time.time()
    self.sophia = self.sh.Sophia()
    self.walking = self.swh.SophiaWalking()
    try:
      self.rclpy.init()
    except:
      pass
    self.node = self.rclpy.create_node('sophia_fidget_node')
    self.pub  = self.node.create_publisher(self.JointState, self.sophia.ROS_CHAN_REF_POS,1)
    return 

  def sleep(self, sleep_time=0):
    tick = self.time.time()
    self.rclpy.spin_once(self.node,timeout_sec=sleep_time)
    tock = self.time.time()
    dt = tock - tick
    while dt < sleep_time:
      tock = self.time.time()
      dt = tock-tick
      do_sleep = sleep_time - dt
      if do_sleep < 0.0:
        do_sleep = 0.0
      self.rclpy.spin_once(self.node,timeout_sec=do_sleep)
      tock = self.time.time()
      dt = tock - tick


  def cb_acc(self, msg):
    print(msg)


  import numpy as np
  def deg2rad(self, val):
    return val * self.np.pi / 180.0
 
  import random

  def fidgetSway(self):
    t_end = self.random.random() * 4.0
    t_sleep = 0.01 * self.random.random()
    amp = 10.0 * self.random.random()
    PAUSE_FLAG = True
    for p in self.np.arange(0.0, 1.0, t_sleep):
      roll = amp * self.np.sin( p * self.np.pi)
      roll = self.deg2rad(roll)
      msg = self.JointState()
      msg.name.append('rhr')
      msg.position.append(roll)
      msg.name.append('lhr')
      msg.position.append(roll)
      msg.name.append('lar')
      msg.position.append(roll)
      msg.name.append('rar')
      msg.position.append(roll)
      self.pub.publish(msg)
      if (p > 0.5) & (PAUSE_FLAG == True):
        PAUSE_FLAG = False
        self.sleep(self.random.random()*4.0)
      self.sleep(t_sleep)



  def fidgetToe(self):
    bend_val_up   = -25.0
    bend_val_down = 0.0
    bend_val = bend_val_down
    i_exit = 0

    foot = self.random.random() * 2.1
    tap_time = self.random.random() * 0.75
    while True:
      if (i_exit % 2)==0:
        bend_val = bend_val_up
      else:
        bend_val = bend_val_down

      bend = self.deg2rad(bend_val)

      j_bend = self.JointState()
      if foot < 1.0:
        j_bend.name.append("rtp")
        j_bend.position.append(bend)
      elif foot < 2.0:
        j_bend.name.append("ltp")
        j_bend.position.append(bend)
      else:
        j_bend.name.append("rtp")
        j_bend.position.append(bend)
        j_bend.name.append("ltp")
        j_bend.position.append(bend)

      self.pub.publish(j_bend)
      self.sleep(tap_time)
      
      i_exit += 1
      if i_exit == 6:
        break
      

 

  def fidgetYaw(self):
    t_end = self.random.random() * 4.0
    t_sleep = 0.01 * self.random.random()
    amp = 10.0 * self.random.random()
    PAUSE_FLAG = True
    for p in self.np.arange(0.0, 1.0, t_sleep):
      roll = amp * self.np.sin( p * self.np.pi)
      roll = self.deg2rad(roll)
      msg = self.JointState()
      msg.name.append('rhy')
      msg.position.append(roll)
      msg.name.append('lhy')
      msg.position.append(roll)
      msg.name.append('lay')
      msg.position.append(roll)
      msg.name.append('ray')
      msg.position.append(roll)
      self.pub.publish(msg)
      if (p > 0.5) & (PAUSE_FLAG == True):
        PAUSE_FLAG = False
        self.sleep(self.random.random()*4.0)
      self.sleep(t_sleep)
 
  def fidgetStep(self): 
    T_walking = 0.005 * self.random.random() + 0.002
    amp_knee = 15.0 * self.random.random()
    d_turn = 0.002 * self.random.random() - 0.001
    while True:
      ret = self.walking.walk(amp_knee=amp_knee,d_turn=d_turn, T_walking=T_walking)
      if ret == self.walking.DONE:
        break
    #self.sleep(1.0)
    self.sleep(self.random.random() * 0.001)

  def loop(self):
    r = self.random.random() * 4.0
    if   r <= 1.0:
      print("Fidget Toe")
      self.fidgetToe()
    elif r <= 2.0:
      print("Fidget Step")
      self.fidgetStep()
    elif r <= 3.0:
      print("Fidget Sway")
#      self.fidgetSway()
    elif r <= 4.0:
      print("Fidget Yaw")
      self.fidgetYaw()

sce = SophiaControlExample()
while True:
  sce.loop()
