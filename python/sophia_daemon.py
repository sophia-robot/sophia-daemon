class SophiaDaemon:
  import lofaroDynamixel2 as ld2
  import time

  import rclpy
  from sensor_msgs.msg import JointState
  from std_msgs.msg import Float64
  import math

  import sophia_h as sh


  def __init__(self, port='/dev/ttyUSB0', L=3, baud=3000000, T=0.018):
    self.sophia   = self.sh.Sophia()
    self.IDs_orig = self.sophia.IDs
    self.IDs      = None
    self.OK       = self.sophia.OK
    self.FAIL     = self.sophia.FAIL

    self.t0 = None

    self.YES = 0
    self.NO  = 1

    self.T_des = T

    self.FILTER_L_DEFAULT = 3
    self.FILTER_MOT_NUM  = 0 #len(self.IDs)

    if (L==None):
      L = self.FILTER_L_DEFAULT
    elif (L < 0):
      L = self.FILTER_L_DEFAULT
  
    self.FILTER_L = L

    self.FILTER_REF_0    = [0.0]  
    self.FILTER_REF_1    = [0.0]
    self.FILTER_REF_GOAL = [0.0]
    self.STATE_POS       = [0.0]
    self.STATE_TORQUE    = [0.0]
    for i in range(1, self.FILTER_MOT_NUM):
      self.FILTER_REF_0.append(0.0)
      self.FILTER_REF_1.append(0.0)
      self.FILTER_REF_GOAL.append(0.0)
      self.STATE_POS.append(0.0)
      self.STATE_TORQUE.append(0.0)


    self.robot = self.ld2.LofaroDynamixel2(baud=baud, port=port)
    self.robot.open()
    self.robot.setBaud()
    self.rclpy.init()
    node_name   = 'sophia_daemon_'+port
    self.node   = self.rclpy.create_node(node_name)
    self.pub    = self.node.create_publisher(self.JointState, self.sophia.ROS_CHAN_STATE_POS,1)
    self.heart  = self.node.create_publisher(self.Float64, self.sophia.ROS_CHAN_HEART,1)
    self.tor    = node.create_publisher(self.JointState, self.sophia.ROS_CHAN_STATE_TORQUE,1)
    self.sub    = node.create_subscription(self.JointState,self.sophia.ROS_CHAN_REF_POS, self.callback, 10)

    print('Init ......')
    for i in range(10):
      self.init_filter_ref()
      self.time.sleep(0.05)

    for i in range(len(self.STATE_POS)):
      self.FILTER_REF_0[i]    = self.STATE_POS[i]
      self.FILTER_REF_1[i]    = sell.STATE_POS[i]
      self.FILTER_REF_GOAL[i] = self.STATE_POS[i]




  def callback(self, msg):
    m = msg
    for i in range(len(msg.name)):
#      try:
        for p in self.IDs:
          pos = m.position[i]

          if p[self.ENUM_ENABLED]:
            name        = p[self.ENUM_NAME]
            the_id      = p[self.ENUM_REF]
            enabled     = p[self.ENUM_ENABLED]
            mot_index   = p[self.ENUM_MOT_INDEX]
            name2       = m.name[i]
            if (name2 == name):
              self.FILTER_REF_GOAL[mot_index] = pos
#            print(name, end=' ')       
#            err    = robot.stagePos(the_id, pos)
#    except:
#      pass
    pass

  def doFilterRef(self, mode=None):
    err = self.FAIL
    for p in self.IDs:
      the_id      = p[self.ENUM_REF]
      mot_index   = p[self.ENUM_MOT_INDEX]
      if p[self.ENUM_ENABLED]:
        r = (self.FILTER_REF_0[mot_index] * (self.FILTER_L - 1.0) + self.FILTER_REF_GOAL[mot_index]) / (self.FILTER_L * 1.0)
        self.FILTER_REF_0[mot_index] = r
        err = self.stagePos(the_id, r)  
    return err

  def deg2rad(self, val):
    return val / 180.0 * math.pi

  def stagePos(self, the_id, r):
    err = self.FAIL
    for p in self.IDs:
      enabled = p[self.ENUM_ENABLED]

      if the_id == p[self.ENUM_REF]:
        if enabled:
          r = r + self.deg2rad(p[self.ENUM_POS_OFFSET])
          if r > p[self.ENUM_POS_MAX]:
            r = p[self.ENUM_POS_MAX]
          if r < p[self.ENUM_POS_MIN]:
            r = p[self.ENUM_POS_MIN]
          err = self.robot.stagePos(the_id, r) | err 
    return err

  def init_filter_ref(self):
    for p in self.IDs:
      if p[self.ENUM_ENABLED]:
        name        = p[self.ENUM_NAME]
        the_id      = p[self.ENUM_STATE]
        mot_index   = p[self.ENUM_MOT_INDEX]
        pos, err    = self.robot.getPos(the_id)
        if err == self.robot.OK:
            self.STATE_POS[mot_index] = pos
            self.FILTER_REF_0[mot_index] = pos
            self.FILTER_REF_1[mot_index] = pos


  def torqueEnable(self):
    for p in self.IDs:
      name      = p[self.ENUM_NAME]
      the_id_0  = p[self.ENUM_TORQUE_EN_0]
      the_id_1  = p[self.ENUM_TORQUE_EN_1]
      enabled   = p[self.ENUM_ENABLED]
      if enabled:
        err = self.robot.torque(the_id_0, self.robot.ENABLE)
        print("Torque Enable status for ", end='')
        print(hex(the_id_0), end=' = ')
        print(err)
        if the_id_1 != the_id_0:
          err = self.robot.torque(the_id_1, self.robot.ENABLE)
          print("Torque Enable status for ", end='')
          print(hex(the_id_1), end=' = ')
          print(err)

  def sleep(self):
    if self.t0 == None
      self.t0 = self.time.time()
    t1 = self.time.time()
    dt = t1 - self.t0
    if dt < 0.0:
      self.rclpy.spin_once(self.node,timeout_sec=0)
      dt = self.T_des
    while dt < self.T_des:
      self.rclpy.spin_once(self.node,timeout_sec=0)
      self.time.sleep(0.002)
      t1 = self.time.time()
      dt = t1 - t0
    self.t0 = t1 
    pass

  def putPos(self):
    self.robot.putPos()
    pass

  def getPos(self, the_mot=None):
    state = self.JointState()
    if the_mot == None:
      for p in self.IDs:
        if p[self.ENUM_ENABLED]:
          name        = p[self.ENUM_NAME]
          the_id      = p[self.ENUM_STATE]
          mot_index   = p[self.ENUM_MOT_INDEX]
          pos, err    = self.robot.getPos(the_id)
          if err == self.robot.OK:
            self.STATE_POS[mot_index] = pos
    elif (the_mot >= 0) & (the_mot < len(self.IDs)):
      p = list(self.IDs)[the_mot]
      if p[self.ENUM_ENABLED]:
        name        = p[self.ENUM_NAME]
        the_id      = p[self.ENUM_STATE]
        mot_index   = p[self.ENUM_MOT_INDEX]
        pos, err    = self.robot.getPos(the_id)
        if err == self.robot.OK:
          self.STATE_POS[mot_index] = pos

  def postPos(self):
    state = self.JointState()
    for p in self.IDs:
      mot_index   = p[self.ENUM_MOT_INDEX]
      name        = p[self.ENUM_NAME]
      pos         = self.STATE_POS[mot_index]
      self.state.name.append(name)
      self.state.position.append(pos)
    self.pub.publish(state) 
    pass

  def getTorque(self, the_mot=None):
    state = self.JointState()
    if the_mot == None:
      for p in self.IDs:
        if p[self.ENUM_ENABLED]:
          name        = p[self.ENUM_NAME]
          the_id      = p[self.ENUM_STATE]
          mot_index   = p[self.ENUM_MOT_INDEX]
          torque, err = self.robot.getTorque(the_id)
          if err == robot.OK:
            self.STATE_TORQUE[mot_index] = torque
    elif (the_mot >= 0) & (the_mot < len(self.IDs)):
      p = list(self.IDs)[the_mot]
      if p[self.ENUM_ENABLED]:
        name        = p[self.ENUM_NAME]
        the_id      = p[self.ENUM_STATE]
        mot_index   = p[self.ENUM_MOT_INDEX]
        torque, err = self.robot.getTorque(the_id)
        if err == self.robot.OK:
          self.STATE_TORQUE[mot_index] = torque

  def postTorque(self):
    state = self.JointState()
    for p in IDs:
      mot_index   = p[self.ENUM_MOT_INDEX]
      name        = p[self.ENUM_NAME]
      torque      = self.STATE_TORQUE[mot_index]
      self.state.name.append(name)
      self.state.position.append(torque)

    self.tor.publish(state) 
    pass

  def heartBeat(self):
    t = self.Float64()
    t.data = self.time.time()
    self.heart.publish(t)

  def loop(self, SPLIT=4):
    STATE_SPLIT_I = SPLIT
    i = 0
    while True:
      self.doFilterRef()
      self.putPos()
      for j in range(i, len(self.IDs), STATE_SPLIT_I):
        self.getPos(j)
        self.getTorque(j)
      self.postPos()
      self.postTorque()
      self.heartBeat()
      self.sleep()
      i = i+1
      if i > STATE_SPLIT_I:
        i = 0


  def isMotActive(self, mot_id, num_check=10):
    ok_i = 0
    for i in range(num_check):
      m_id, got_ping = self.robot.ping(mot_id)
      if got_ping == self.YES:
        ok_i += 1

    ret = self.FAIL
    if ok_i > 0:
      ret = self.OK

    return ret
    

  def getActiveMot(self):
    ret = self.FAIL
    if self.IDs == None:
      self.IDs = []
      for p in self.IDs_orig:
        if p[self.ENUM_ENABLED]:
          the_id      = p[self.ENUM_STATE]
          en = self.isMotActive(the_id)
          if en == self.OK:
            self.IDs.append(p)
    return ret

  def setup(self):
    self.getActiveMot()
    self.FILTER_MOT_NUM  = len(self.IDs)


  def run(self, SPLIT=None):
    self.setup()
    self.torqueEnable()
    self.loop(SPLIT=SPLIT)
