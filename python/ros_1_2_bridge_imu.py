#!/usr/bin/env python

# try this: https://github.com/ros2/ros1_bridge

from ctypes import Structure,c_uint16,c_double,c_ubyte,c_uint32,c_int16


class VECTOR_3(Structure):
  _pack_ = 1
  _fields_ = [('x', c_double),
              ('y', c_double),
              ('z', c_double)]

class WALKING_IMU(Structure):
  _pack_ = 1
  _fields_ = [('linear', VECTOR_3),
              ('angular', VECTOR_3)]

class WALKIN_ROS2_TO_ROS2(Structure):
  _pack_ = 1
  _fields_ = [('lhy', c_double),
              ('lhr', c_double),
              ('lhp', c_double),
              ('lkp', c_double),
              ('lay', c_double),
              ('lar', c_double),
              ('lap', c_double),
              ('ltp', c_double),
              ('rtp', c_double),
              ('rhy', c_double),
              ('rhr', c_double),
              ('rhp', c_double),
              ('rkp', c_double),
              ('ray', c_double),
              ('rar', c_double),
              ('rap', c_double),
              ('rtp', c_double)]


class HansonWalking:
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
    from geometry_msgs.msg import Twist
  except:
    rclpy = None
    JointState = None
  

  mode = None

  def __init__(self, mode=None):
    self.OK   = False
    self.FAIL = True
    self.topic_ros1 = '/hr/animation/walking/joints'
    self.topic_ros2 = self.sophia.ROS_CHAN_WALKING
    self.walking = WALKIN_ROS2_TO_ROS2()
    self.imu     = WALKING_IMU()
    self.PORT     = 44455
    self.PORT_IMU = 44456
    self.IP_READ  = '0.0.0.0'
    self.IP   = '127.0.0.1'
    if   mode == 'client':
      print("ini Client")
      self.ini_client()
      self.mode = mode
    elif mode == 'server':
      print("ini server")
      self.ini_server()
      self.mode = mode
    elif mode == 'ros1-imu':
      print("ini ros1 imu")
      self.ini_ros1_imu()
      self.mode = mode
    elif mode == 'ros2-imu':
      print("ini ros1 imu")
      self.ini_ros2_imu()
      self.mode = mode
    elif mode == 'ros2-imu-real':
      print("ini ros1 imu real")
      self.ini_ros2_imu()
      self.mode = mode

  def ini_ros1_imu(self):
    self.sock        = self.socket.socket(self.socket.AF_INET,    # Internet
                                          self.socket.SOCK_DGRAM) # UDP
    self.sock_server = self.sock.bind((self.IP_READ, self.PORT_IMU))
    #self.set_timeout()

  def ini_ros2_imu(self):
    self.sock        = self.socket.socket(self.socket.AF_INET,    # Internet
                                          self.socket.SOCK_DGRAM) # UDP

  def ini_server(self):
    self.sock        = self.socket.socket(self.socket.AF_INET,    # Internet
                                          self.socket.SOCK_DGRAM) # UDP
    self.sock_server = self.sock.bind((self.IP_READ, self.PORT))
    #self.set_timeout()

  def ini_client(self):
    self.sock        = self.socket.socket(self.socket.AF_INET,    # Internet
                                          self.socket.SOCK_DGRAM) # UDP

  def set_timeout(self, t=0.001):
    self.sock.settimeout(t)

  i_dan = 0
  def cb_walking(self, msg):
    names  = msg.names
    values = msg.values
    self.i_dan += 1
    k_knee = 1.5
    for i in range(len(names)):
      name = names[i]
      val  = values[i]
      if name   == 'LeftLegYaw':
        self.walking.lhy = val 
      elif name == 'LeftLegRoll':
        self.walking.lhr = val 
      elif name == 'LeftLegPitch':
        self.walking.lhp = -val 
      elif name == 'LeftKneePitch':
        self.walking.lkp = -val * k_knee
      elif name == 'leftFootYaw':
        self.walking.lay = val 
      elif name == 'LeftFootRoll':
        self.walking.lar = val 
      elif name == 'LeftFootPitch':
        self.walking.lap = -val 
      elif name == 'LeftToePitch':
        self.walking.ltp = -val 
      elif name == 'RightLegYaw':
        self.walking.rhy = val 
      elif name == 'RightLegRoll':
        self.walking.rhr = val 
      elif name == 'RightLegPitch':
        self.walking.rhp = -val 
      elif name == 'RightKneePitch':
        self.walking.rkp = -val * k_knee
      elif name == 'RightFootYaw':
        self.walking.ray = val 
      elif name == 'RightFootRoll':
        self.walking.rar = val 
      elif name == 'RightFootPitch':
        self.walking.rap = -val 
      elif name == 'RightToePitch':
        self.walking.rtp = -val 
   
    buff = bytes(self.walking)
    self.sock.sendto(buff, (self.IP, self.PORT))
    print(self.i_dan)
  
  i_dan = 0
  def read(self):
    ret = self.FAIL
    try:
      if self.mode == 'ros1-imu':
        buff = self.sock.recv(1024) # buffer size is 1024 bytes 
        data = WALKING_IMU.from_buffer_copy(buff)
        self.imu = data
        ret = self.OK
      else:
        buff = self.sock.recv(1024) # buffer size is 1024 bytes 
        data = WALKIN_ROS2_TO_ROS2.from_buffer_copy(buff)
        self.walking = data
        ret = self.OK
    except:
      ret = self.FAIL
      #print(self.i_dan)
      ret = self.OK
    return ret
#    return ret



  itmp = 0
  ENUM_LHY = itmp
  itmp += 1
  ENUM_LHR = itmp
  itmp += 1
  ENUM_LHP = itmp
  itmp += 1
  ENUM_LKP = itmp
  itmp += 1
  ENUM_LAY = itmp
  itmp += 1
  ENUM_LAR = itmp
  itmp += 1
  ENUM_LAP = itmp
  itmp += 1
  ENUM_LTP = itmp
  itmp += 1
  ENUM_RHY = itmp
  itmp += 1
  ENUM_RHR = itmp
  itmp += 1
  ENUM_RHP = itmp
  itmp += 1
  ENUM_RKP = itmp
  itmp += 1
  ENUM_RAY = itmp
  itmp += 1
  ENUM_RAR = itmp
  itmp += 1
  ENUM_RAP = itmp
  itmp += 1
  ENUM_RTP = itmp
  itmp += 1
  ENUM_MOT_NUM = itmp
  ref_buff = [0.0] * ENUM_MOT_NUM

  L_filt = 20

  def doFilter(self, mot, val):
    ret = 0.0
    err = self.FAIL

    if mot < 0:
      return ret, err
    if mot >= self.ENUM_MOT_NUM:
      return ret, err

    self.ref_buff[mot] = (val + self.ref_buff[mot]*(self.L_filt - 1.0)) / self.L_filt
    err = self.OK
    return self.ref_buff[mot], err

  '''
  ENUM_LHY = 0
  ENUM_LHR = 1
  ENUM_LHP = 2
  ENUM_LKP = 3
  ENUM_LAY
  ENUM_
  ENUM_
  ENUM_
  ENUM_
  ENUM_
  ENUM_
  ENUM_
  ENUM_
  '''
 

  def ros1_imu(self):
    from geometry_msgs.msg import Twist
    pub = self.rospy.Publisher('/sophia/body/imu', Twist, queue_size=1)
    self.rospy.init_node('body_imu_node', anonymous=True)
    while not self.rospy.is_shutdown():
      ret = self.read()
      if ret == self.OK:
        msg = Twist()
        msg.linear.x  = self.imu.linear.x 
        msg.linear.y  = self.imu.linear.y 
        msg.linear.z  = self.imu.linear.z
        msg.angular.x = self.imu.angular.x 
        msg.angular.y = self.imu.angular.y 
        msg.angular.z = self.imu.angular.z 
        pub.publish(msg) 

  def ros2(self):
    self.rclpy.init()
    node = self.rclpy.create_node('walking_ros2')
    pub      = node.create_publisher(self.JointState, self.sophia.ROS_CHAN_WALKING,1)
    pub_ref  = node.create_publisher(self.JointState, self.sophia.ROS_CHAN_REF_POS,1)
    print("ROS2 Node Running")
    while True:
      ret = self.read()
      #print('.',end='',flush=True)
      if ret == self.OK:
        msg = self.JointState()
        msg.name.append('lhy')
        msg.position.append(self.doFilter(self.ENUM_LHY  ,self.walking.lhy)[0])
        msg.name.append('lhr')
        msg.position.append(self.doFilter(self.ENUM_LHR  ,self.walking.lhr)[0])
        msg.name.append('lhp')
        msg.position.append(self.doFilter(self.ENUM_LHP  ,self.walking.lhp)[0])
        msg.name.append('lkp')
        msg.position.append(self.doFilter(self.ENUM_LKP  ,self.walking.lkp)[0])
        msg.name.append('lay')
        msg.position.append(self.doFilter(self.ENUM_LAY  ,self.walking.lay)[0])
        msg.name.append('lar')
        msg.position.append(self.doFilter(self.ENUM_LAR  ,self.walking.lar)[0])
        msg.name.append('lap')
        msg.position.append(self.doFilter(self.ENUM_LAP  ,self.walking.lap)[0])
        msg.name.append('ltp')
        msg.position.append(self.doFilter(self.ENUM_LTP  ,self.walking.ltp)[0])
        msg.name.append('rhy')
        msg.position.append(self.doFilter(self.ENUM_RHY  ,self.walking.rhy)[0])
        msg.name.append('rhr')
        msg.position.append(self.doFilter(self.ENUM_RHR  ,self.walking.rhr)[0])
        msg.name.append('rhp')
        msg.position.append(self.doFilter(self.ENUM_RHP  ,self.walking.rhp)[0])
        msg.name.append('rkp')
        msg.position.append(self.doFilter(self.ENUM_RKP  ,self.walking.rkp)[0])
        msg.name.append('ray')
        msg.position.append(self.doFilter(self.ENUM_RAY  ,self.walking.ray)[0])
        msg.name.append('rar')
        msg.position.append(self.doFilter(self.ENUM_RAR  ,self.walking.rar)[0])
        msg.name.append('rap')
        msg.position.append(self.doFilter(self.ENUM_RAP  ,self.walking.rap)[0])
        msg.name.append('rtp')
        msg.position.append(self.doFilter(self.ENUM_RTP  ,self.walking.rtp)[0])
        pub.publish(msg)
        pub_ref.publish(msg)
        print('.',flush=True)

  def ros1(self):
    node = self.rospy.init_node('walking_ros1', anonymous=True)
    sub  = self.rospy.Subscriber(self.topic_ros1, self.TargetPosture, self.cb_walking)
    # spin() simply keeps python from exiting until this node is stopped
    self.rospy.spin()

  def ros2_imu(self):
    self.rclpy.init(args=None)
    node = self.rclpy.create_node('ROS2_IMU_Node')
    sub  = node.create_subscription(self.JointState, self.sophia.ROS_CHAN_REF_POS, self.cb_ros2_imu, 1)
    dan_i = 0
    while self.rclpy.ok():
      dan_i += 1
      print(dan_i)
      self.rclpy.spin_once(node) 

  def ros2_imu_real(self):
    from geometry_msgs.msg import Twist
    print("real imu state")
    self.rclpy.init(args=None)
    node = self.rclpy.create_node('ROS2_IMU_Node_Real')
    sub  = node.create_subscription(Twist, self.sophia.ROS_CHAN_STATE_ACC, self.cb_ros2_imu_real, 1)
    dan_i = 0
    while self.rclpy.ok():
      dan_i += 1
      print(dan_i)
      self.rclpy.spin_once(node) 

  import numpy as np

  def deg2rad(self, val):
    return self.np.pi * val / 180.0
 

  L_imu_real = 20.0 
  lx0 = 0.0
  ly0 = 0.0
  lz0 = 0.0
  ax0 = 0.0
  ay0 = 0.0
  az0 = 0.0
  k_imu_y = 1.3
  def cb_ros2_imu_real(self, msg):
    #dandan
    lx = self.deg2rad(msg.linear.x)
    ly = self.deg2rad(msg.linear.y - 9.0) * self.k_imu_y
    lz = self.deg2rad(msg.linear.z)
    ax = self.deg2rad(msg.angular.x)
    ay = self.deg2rad(msg.angular.y)
    az = self.deg2rad(msg.angular.z)

    self.lx0 = (lx + self.lx0*(self.L_imu_real - 1.0)) / self.L_imu_real
    self.ly0 = (ly + self.ly0*(self.L_imu_real - 1.0)) / self.L_imu_real
    self.lz0 = (lz + self.lz0*(self.L_imu_real - 1.0)) / self.L_imu_real
    self.ax0 = (ax + self.ax0*(self.L_imu_real - 1.0)) / self.L_imu_real
    self.ay0 = (ay + self.ay0*(self.L_imu_real - 1.0)) / self.L_imu_real
    self.az0 = (az + self.az0*(self.L_imu_real - 1.0)) / self.L_imu_real

    self.imu.linear.x  = self.lx0
    self.imu.linear.y  = self.ly0
    self.imu.linear.z  = self.lz0
    self.imu.angular.x = self.ax0
    self.imu.angular.y = self.ay0
    self.imu.angular.z = self.az0
    print(self.imu.angular.y)
    buff = bytes(self.imu)
    self.sock.sendto(buff, (self.IP, self.PORT_IMU))
    pass

  def cb_ros2_imu(self, msg):
    roll1  = msg.position[msg.name.index('rhr')]
    roll2  = msg.position[msg.name.index('lhr')]

    roll = roll1
    if abs(roll1) < abs(roll2):
      roll = roll2

    pitch1 = msg.position[msg.name.index('rhp')]
    pitch2 = msg.position[msg.name.index('lhp')]
    
    pitch = pitch1
    if abs(pitch1) < abs(pitch2):
      pitch = pitch2

    yaw1 = 0.0
    yaw2 = 0.0

    try:
      yaw1   = msg.position[msg.name.index('rhy')]
      yaw2   = msg.position[msg.name.index('lhy')]
    except:
      pass
    yaw = yaw1
    if abs(yaw1) < abs(yaw2):
      yaw = yaw2
     
    self.imu.angular.x = roll
    self.imu.angular.y = pitch
    self.imu.angular.z = yaw

    buff = bytes(self.imu)
    self.sock.sendto(buff, (self.IP, self.PORT_IMU))
    pass

  def run(self):
    if   self.mode == 'client':
      self.ros1()
    elif self.mode == 'server':
      self.ros2()
    elif self.mode == 'ros1-imu':
      self.ros1_imu()
    elif self.mode == 'ros2-imu':
      self.ros2_imu()
    elif self.mode == 'ros2-imu-real':
      print('running mode : ', self.mode)
      self.ros2_imu_real()

import sys


if __name__ == '__main__':
  args = sys.argv[1:]
  hw = None

  if len(args) > 0:
    d0 = args[0]
  d0 = 'ros2-imu'
#  d0 = 'ros2-imu-real'
#  d0 = 'ros2'
#  d0 = 'ros1-imu'
  if True:
#  if len(args) > 0:
#    d0 = args[0]
#    d0 = 'ros1-imu'
    print(d0)
    if   d0 == 'server':
      hw = HansonWalking(mode='server')
    elif d0 == 'client':
      hw = HansonWalking(mode='client')
    elif d0 == 'ros1':
      hw = HansonWalking(mode='client')
    elif d0 == 'ros2':
      hw = HansonWalking(mode='server')
    elif d0 == 'ros1-imu':
      hw = HansonWalking(mode='ros1-imu')
    elif d0 == 'ros2-imu':
      hw = HansonWalking(mode='ros2-imu')
    elif d0 == 'ros2-imu-real':
      print('setting ',d0)
      hw = HansonWalking(mode='ros2-imu-real')
    else:
      hw = HansonWalking()
  else:
    hw = HansonWalking()

  hw.run()
