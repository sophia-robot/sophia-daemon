



class SophiaWalking:
  import rclpy
  from geometry_msgs.msg import Twist
  from sensor_msgs.msg import JointState
  import time
  import sophia_h as sh
  import numpy as np
  import random
  import sophia_filter

  node   = None
  pub    = None
  sub    = None
  sophia = None

  T = 0.01
  t0 = None
  OK = False
  FAIL = True
  RUNNING = 2
  DONE    = 3
  def SophiaWalking(self):
    return 

  def __init__(self):
    self.sf = self.sophia_filter.SophiaFilter()
    self.init()

  def init(self):
    self.pitch0 = 0.0
    self.t0 = self.time.time()
    self.sophia = self.sh.Sophia()
    self.rclpy.init()
    self.node = self.rclpy.create_node('sophia_example_control_node')
    self.pub  = self.node.create_publisher(self.JointState, self.sophia.ROS_CHAN_REF_POS,10)
    self.sub  = self.node.create_subscription(self.Twist, self.sophia.ROS_CHAN_STATE_ACC,  self.cb_acc, 10)
    self.tor  = self.node.create_subscription(self.JointState, self.sophia.ROS_CHAN_STATE_TORQUE,  self.cb_tor, 10)
    return 

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

  tor_right = 0.0
  tor_left  = 0.0
  def cb_tor(self, msg):
    try:
      i_right = msg.name.index('rtp')
#      print("i right = ", i_right)
      self.tor_right = msg.effort[i_right]
    except:
      pass
    try:
      i_left  = msg.name.index('ltp')
#      print("i left = ", i_left)
      self.tor_left  = msg.effort[i_left ]
    except:
      pass

  roll  = 0.0
  pitch = 0.0
  def cb_acc(self, msg):
    # note, pitch and roll are in deg
    self.roll  = msg.angular.x
    self.pitch = msg.angular.y
   
  def deg2rad(self, val):
    return val / 180.0 * self.np.pi 

  def rad2deg(self, val):
    return val / self.np.pi * 180.0

  def getLegCycleLegsTogether(self, p, amp = 30.0, amp_ankkle=5.0, amp_fwd=-5.0):
    # P is the percentage of the cycle from 0.0 to 1.0 that the leg is in
    fwd = amp_fwd * self.np.sin(p*self.np.pi)

    ap = -amp * self.np.sin(p*self.np.pi) - fwd
    hp = ap + fwd
    kp = -2.0*ap
    ar = self.LEG*amp_ankkle * self.np.sin(p*self.np.pi)
    hr = ar

    return hp, kp, ap, hr, ar

  def getLegCycle(self, p, amp_pitch = 40.0, amp_ankkle=5.0, amp_fwd=-5.0):
    # P is the percentage of the cycle from 0.0 to 1.0 that the leg is in
    fwd = amp_fwd * self.np.sin(p*self.np.pi)

    ap = -amp_pitch * self.np.sin(p*self.np.pi) - fwd
    hp = ap + fwd
    kp = -2.0*ap
    ar = -self.LEG*amp_ankkle * self.np.sin(p*self.np.pi)
    hr = ar

    return hp, kp, ap, hr, ar



  def getLegCycleYaw(self, p, d_turn=0.0):

    hyl = 0.0
    hyr = 0.0
    ayl = 0.0
    ayr = 0.0
    
    d  = 0.13
    s0 = 0.18
    e0 = s0 + d
    e1 = 1.0 - s0
    s1 = e1 - d

    # phase 2
    y = d_turn

    if p < s0:
      y = 0.0
    elif p > e1:
      y = 0.0

    # phase 3
    elif (p > s1) & (p <=e1):
        dp = e1 - s1
        pp = p - s1
        pp = 0.5 - pp / dp / 2.0
        print(pp)
        y  = d_turn * self.np.sin(pp*self.np.pi)
    # phase 1
    elif (p > s0) & (p <=e0):
        dp = e0 - s0
        pp = p - s0
        pp = pp / dp / 2.0
        print(pp)
        y = d_turn * self.np.sin(pp*self.np.pi)
    
    if d_turn <0.0:
      hyl = -y
      ayl = -y
      hyr =  y
      ayr =  y
    else:
      hyl =  y
      ayl =  y
      hyr = -y
      ayr = -y

    return hyl, ayl, hyr, ayr


  def foodIk(self, foot='left', x=0.0, z=0.0):
    if foot != 'left':
      if foot != 'right':
        return self.FAIL



  p_turn    = 0.0
  p_turn_i  = 0
  p_walking = 0.0
  LEG = 1.0
  LEG_LEFT  = 1
  LEG_RIGHT = 2
  DO_EXIT = 0
  def walk(self, hip_pitch_offset=10.0, T_walking=0.003, d_turn=0.0, step_l=0.0, num_steps=1, amp_knee=30.0):
    d_turn = -d_turn
    #hp, kp, ap, hr, ar = self.getLegCycle(self.p_walking, amp_pitch = 10.0)
    hp, kp, ap, hr, ar = self.getLegCycle(self.p_walking, amp_pitch=amp_knee)


    k_roll = 0.5

    hpr = self.deg2rad(hp)
    kpr = self.deg2rad(kp)
    apr = self.deg2rad(ap)
    hrr = self.deg2rad(hr) * k_roll
    arr = self.deg2rad(ar) * k_roll

    hpoff = self.deg2rad(hip_pitch_offset)

    ctrl = self.JointState()
    the_leg = 0
    if d_turn >= 0.0:
      if self.LEG > 0:
        the_leg_down = self.LEG_LEFT
      else:
        the_leg_down = self.LEG_RIGHT
    else:
      if self.LEG > 0:
        the_leg_down = self.LEG_RIGHT
      else:
        the_leg_down = self.LEG_LEFT

#    if d_turn < 0.0:
#      the_leg_down = self.LEG_LEFT

    hyl, ayl, hyr, ayr = self.getLegCycleYaw(self.p_turn, d_turn=d_turn)
    hylr = self.deg2rad(hyl)
    aylr = self.deg2rad(ayl)
    hyrr = self.deg2rad(hyr)
    ayrr = self.deg2rad(ayr)

    step_d = -step_l * self.np.sin(self.p_walking * 2.0 * self.np.pi)
    step_r = self.deg2rad(step_d)

    if the_leg_down == self.LEG_LEFT:
      ctrl.name.append("rhp")
      ctrl.position.append(hpr + hpoff + step_r)
      ctrl.name.append("rkp")
      ctrl.position.append(kpr - 2.0 * step_r)
      ctrl.name.append("rap")
      ctrl.position.append(apr + step_r)
      ctrl.name.append("rar")
      ctrl.position.append(arr)
      ctrl.name.append("rhr")
      ctrl.position.append(hrr)

      ctrl.name.append("rhy")
      ctrl.position.append(hyrr)
      ctrl.name.append("ray")
      ctrl.position.append(ayrr)

      ctrl.name.append("lhp")
      ctrl.position.append(0.0 + hpoff)
      ctrl.name.append("lkp")
      ctrl.position.append(0.0)
      ctrl.name.append("lap")
      ctrl.position.append(0.0)
      ctrl.name.append("lar")
      ctrl.position.append(arr)
      ctrl.name.append("lhr")
      ctrl.position.append(hrr)

      ctrl.name.append("lhy")
      ctrl.position.append(hylr)
      ctrl.name.append("lay")
      ctrl.position.append(aylr)

    else:
      ctrl.name.append("lhp")
      ctrl.position.append(hpr + hpoff + step_r)
      ctrl.name.append("lkp")
      ctrl.position.append(kpr - 2.0 * step_r)
      ctrl.name.append("lap")
      ctrl.position.append(apr + step_r)
      ctrl.name.append("lar")
      ctrl.position.append(arr)
      ctrl.name.append("lhr")
      ctrl.position.append(hrr)

      ctrl.name.append("lhy")
      ctrl.position.append(hylr)
      ctrl.name.append("lay")
      ctrl.position.append(aylr)

      ctrl.name.append("rhp")
      ctrl.position.append(0.0 + hpoff)
      ctrl.name.append("rkp")
      ctrl.position.append(0.0)
      ctrl.name.append("rap")
      ctrl.position.append(0.0)
      ctrl.name.append("rar")
      ctrl.position.append(arr)
      ctrl.name.append("rhr")
      ctrl.position.append(hrr)

      ctrl.name.append("rhy")
#      ctrl.position.append(0.0)
      ctrl.position.append(hyrr)
      ctrl.name.append("ray")
#      ctrl.position.append(0.0)
      ctrl.position.append(ayrr)



    #print(kpr)
    self.pub.publish(ctrl)
    self.p_walking += T_walking
    self.p_turn    += T_walking / 2.0
    if self.p_walking > 1.0:
      self.p_walking = 0.0
      self.LEG = -self.LEG
      self.p_turn_i += 1
      if self.p_turn_i > 1:
        self.p_turn = 0.0
        self.p_turn_i = 0

        self.DO_EXIT += 1
        ns = num_steps - 1
        if ns < 0:
          ns = 0
        if self.DO_EXIT > ns:
          return self.DONE
          ##exit()
    self.sleep()
    return self.RUNNING
    

  def legUpTrajectory(self, p, amp_pitch = 40.0, amp_ankkle=5.0, amp_fwd=-5.0):
    # P is the percentage of the cycle from 0.0 to 1.0 that the leg is in
    if p < 0.0:
      p = 0.0
    if p > 1.0:
      p = 1.0

    fwd = amp_fwd * self.np.sin(p*self.np.pi)

    ap = -amp_pitch * self.np.sin(p*self.np.pi) - fwd
    hp = ap + fwd
    kp = -2.0*ap
    ar = -self.LEG*amp_ankkle * self.np.sin(p*self.np.pi)
    hr = ar

    return hp, kp, ap, hr, ar

  def legBend(self, leg='right', d='up'):
    # right leg up
    ret = self.FAIL
    s = 0.0
    e = 0.5
    if d == 'up':
      s = 0.0
      e = 0.5
    elif d == 'down':
      s = 0.5
      e = 0.0
    else:
      return ret

    for p in self.np.arange(s,e,0.005):
      hp, kp, ap, hr, ar = self.legUpTrajectory(p)
      hpr = self.deg2rad(hp)
      kpr = self.deg2rad(kp)
      apr = self.deg2rad(ap)
      hrr = self.deg2rad(hr)
      arr = self.deg2rad(ar)
      if leg == 'right':
        ctrl = self.JointState()
        ctrl.name.append("rhp")
        ctrl.position.append(hpr)
        ctrl.name.append("rkp")
        ctrl.position.append(kpr)
        ctrl.name.append("rap")
        ctrl.position.append(apr)
        ctrl.name.append("lhr")
        ctrl.position.append(hrr)
        ctrl.name.append("lar")
        ctrl.position.append(arr)
        self.pub.publish(ctrl)
        ret = self.OK
      elif leg == 'left':
        ctrl = self.JointState()
        ctrl.name.append("lhp")
        ctrl.position.append(hpr)
        ctrl.name.append("lkp")
        ctrl.position.append(kpr)
        ctrl.name.append("lap")
        ctrl.position.append(apr)
        ctrl.name.append("rhr")
        ctrl.position.append(hrr)
        ctrl.name.append("rar")
        ctrl.position.append(arr)
        self.pub.publish(ctrl)
        ret = self.OK
      else:
        return ret
      self.sleep()
    return ret
    
 
  def halfSin(self, p, amp_pitch = 40.0, amp_ankkle=5.0, amp_fwd=-5.0):
    # P is the percentage of the cycle from 0.0 to 1.0 that the leg is in
    if p < 0.0:
      p = 0.0
    if p > 1.0:
      p = 1.0

    fwd = amp_fwd * self.np.sin(p*self.np.pi)

    return fwd

  def legYaw(self, amp=30.0, leg=None, d=None):
    ret = self.FAIL
    if leg == None:
      return ret
    if d == None:
      return ret

    if leg != 'left':
      if leg != 'right':
        return ret
    if d != 'turn':
      if d != 'return':
        return ret
 
    s = 0.0
    e = 1.0
    if d == 'turn':
      s = 0.0
      e = 1.0
    elif d == 'return':
      s = 1.0
      e = 0.0
    else:
      return ret

    for p in self.np.arange(s,e,0.005):
      v = self.halfSin(p)
      vr = self.deg2rad(v)
      if leg == 'right':
        ctrl = self.JointState()
        ctrl.name.append("rhy")
        ctrl.position.append(vr)
        self.pub.publish(ctrl)
        ret = self.OK
      elif leg == 'left':
        ctrl = self.JointState()
        ctrl.name.append("lhy")
        ctrl.position.append(vr)
        self.pub.publish(ctrl)
        ret = self.OK
      else:
        return ret
      self.sleep()
    return ret


  def turnLeft(self, amp=30.0):
    print("right leg up")
    sce.legBend( leg='right', d='up'     )
    sce.sleep(TT=1.0)
    
    print("left leg yaw")
    sce.legYaw( amp=amp,  leg='left',  d='turn'   )
    sce.sleep(TT=1.0)

    print("right leg down")
    sce.legBend( leg='right', d='down'   )
    sce.sleep(TT=1.0)

    print("left leg up")
    sce.legBend( leg='left',  d='up'     )
    sce.sleep(TT=1.0)

    print("left leg un-yaw")
    sce.legYaw( amp=amp,  leg='left', d='return'  )
    sce.sleep(TT=1.0)

    print("left leg down")
    sce.legBend( leg='left', d='down'    )
    sce.sleep(TT=1.0)
    
    exit()
    #sce.sleep()
     
