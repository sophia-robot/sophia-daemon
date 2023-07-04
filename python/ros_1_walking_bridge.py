#!/usr/bin/env python

# try this: https://github.com/ros2/ros1_bridge

from ctypes import Structure,c_uint16,c_double,c_ubyte,c_uint32,c_int16

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
              ('rtp', c_double),
              ('turn', c_double),
              ('step', c_double),
              ('blender', c_double)]


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
    self.PORT = 44455
    self.IP   = '127.0.0.1'
    if   mode == 'client':
      print("ini Client")
      self.ini_client()
      self.mode = mode
    elif mode == 'server':
      print("ini server")
      self.ini_server()
      self.mode = mode

  def ini_server(self):
    self.sock        = self.socket.socket(self.socket.AF_INET,    # Internet
                                          self.socket.SOCK_DGRAM) # UDP
    self.sock_server = self.sock.bind((self.IP, self.PORT))
    #self.set_timeout()

  def ini_client(self):
    self.sock        = self.socket.socket(self.socket.AF_INET,    # Internet
                                          self.socket.SOCK_DGRAM) # UDP

  def set_timeout(self, t=0.001):
    self.sock.settimeout(t)

  def cb_walking(self, msg):
    names  = msg.names
    values = msg.values
    for i in range(len(names)):
      name = names[i]
      val  = values[i]
      if name   == 'LeftLegYaw':
        self.walking.lhy = val 
      elif name == 'LeftLegRoll':
        self.walking.lhr = val 
      elif name == 'LeftLegPitch':
        self.walking.lhp = val 
      elif name == 'LeftKneePitch':
        self.walking.lkp = val 
      elif name == 'leftFootYaw':
        self.walking.lay = val 
      elif name == 'LeftFootRoll':
        self.walking.lar = val 
      elif name == 'LeftFootPitch':
        self.walking.lap = val 
      elif name == 'LeftToePitch':
        self.walking.ltp = val 
      elif name == 'RightLegYaw':
        self.walking.rhy = val 
      elif name == 'RightLegRoll':
        self.walking.rhr = val 
      elif name == 'RightLegPitch':
        self.walking.rhp = val 
      elif name == 'RightKneePitch':
        self.walking.rkp = val 
      elif name == 'RightFootYaw':
        self.walking.ray = val 
      elif name == 'RightFootRoll':
        self.walking.rar = val 
      elif name == 'RightFootPitch':
        self.walking.rap = val 
      elif name == 'RightToePitch':
        self.walking.rtp = val 
      elif name == 'blender':
        self.walking.blender = val
      elif name == 'turn':
        self.walking.turn = val
      elif name == 'step':
        self.walking.step = val
   
    buff = bytes(self.walking)
    self.sock.sendto(buff, (self.IP, self.PORT))
  
  def read(self):
    ret = self.FAIL
    try:
      buff = self.sock.recv(1024) # buffer size is 1024 bytes 
      data = WALKIN_ROS2_TO_ROS2.from_buffer_copy(buff)
      self.walking = data
      ret = self.OK
    except:
      ret = self.FAIL
    return ret
 
  def ros2(self):
    self.rclpy.init()
    node = self.rclpy.create_node('walking_ros2')
    pub  = node.create_publisher(self.JointState, self.sophia.ROS_CHAN_WALKING,1)

    while True:
      ret = self.read()
      if ret == self.OK:
        msg = self.JointState()
        msg.name.append('lhy')
        msg.position.append(self.walking.lhy)
        msg.name.append('lhr')
        msg.position.append(self.walking.lhr)
        msg.name.append('lhp')
        msg.position.append(self.walking.lhp)
        msg.name.append('lkp')
        msg.position.append(self.walking.lkp)
        msg.name.append('lay')
        msg.position.append(self.walking.lay)
        msg.name.append('lar')
        msg.position.append(self.walking.lar)
        msg.name.append('lap')
        msg.position.append(self.walking.lap)
        msg.name.append('ltp')
        msg.position.append(self.walking.ltp)
        msg.name.append('rhy')
        msg.position.append(self.walking.rhy)
        msg.name.append('rhr')
        msg.position.append(self.walking.rhr)
        msg.name.append('rhp')
        msg.position.append(self.walking.rhp)
        msg.name.append('rkp')
        msg.position.append(self.walking.rkp)
        msg.name.append('ray')
        msg.position.append(self.walking.ray)
        msg.name.append('rar')
        msg.position.append(self.walking.rar)
        msg.name.append('rap')
        msg.position.append(self.walking.rap)
        msg.name.append('rtp')
        msg.position.append(self.walking.rtp)
        msg.name.append('step')
        msg.position.append(self.walking.step)
        msg.name.append('turn')
        msg.position.append(self.walking.turn)
        msg.name.append('blender')
        msg.position.append(self.walking.blender)
        pub.publish(msg)

  def ros1(self):
    node = self.rospy.init_node('walking_ros1', anonymous=True)
    sub  = self.rospy.Subscriber(self.topic_ros1, self.TargetPosture, self.cb_walking)
    # spin() simply keeps python from exiting until this node is stopped
    self.rospy.spin()

  def run(self):
    if   self.mode == 'client':
      self.ros1()
    elif self.mode == 'server':
      self.ros2()

import sys


if __name__ == '__main__':
  args = sys.argv[1:]
  hw = None
  if len(args) > 0:
    d0 = args[0]
    print(d0)
    if   d0 == 'server':
      hw = HansonWalking(mode='server')
    elif d0 == 'client':
      hw = HansonWalking(mode='client')
    else:
      hw = HansonWalking()
  else:
    hw = HansonWalking()


  hw.run()
