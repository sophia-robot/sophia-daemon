class SophiaTurnUDP:
  import time
  try: 
    import sophia_walking_h as swh
  except:
    pass

  try: 
    import numpy as np
  except:
    pass

  try: 
    import rclpy
  except:
    pass

  try: 
    from geometry_msgs.msg import Twist
  except:
    pass

  try: 
    import sophia_h as sh
  except:
    pass

  try:
    import socket
  except:
    pass

  def __init__(self):
    self.UDP_PORT = 8008 

  def SophiaTurnUDP(self):
    pass

  def init_udp(self):
    self.UDP_IP = "10.0.0.204"
    self.sock = self.socket.socket(self.socket.AF_INET, # Internet
                                   self.socket.SOCK_DGRAM) # UDP
    pass

  def init_ros2(self):
    self.sophia = self.sh.Sophia()
    self.turn_max = 10.0
    self.walking = self.swh.SophiaWalking()
    self.UDP_IP = '0.0.0.0'

    self.sock = self.socket.socket(self.socket.AF_INET, # Internet
                                     self.socket.SOCK_DGRAM) # UDP
    self.sock.bind((self.UDP_IP, self.UDP_PORT))

    try:
      self.rclpy.init()
    except:
      pass
    try:
      self.node = self.rclpy.create_node('udp_turn_sophia_turn_ctrl')
      self.pub  = self.node.create_publisher(self.Twist, self.sophia.ROS_CHAN_TURN_CTRL, 1)
    except:
      print("ros publisher error")
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
    try:
      self.node.destroy_node()
      self.rclpy.shutdown()
    except:
      pass

  def decode(self, val):
    string_out = bytes(val).decode('utf-8')
    return string_out

  def ros2(self):
    while True:
      print("Start Reading")
      data, addr = self.sock.recvfrom(1024)
      ds = data.split()
      dsl = len(ds)
      print(ds[0], ds[1])
      print(str(ds[0]), str(ds[1]))
      print(self.decode(ds[0]), self.decode(ds[1]))
#      try:
      if dsl > 1:
          d0 = self.decode(ds[0])
          d1 = self.decode(ds[1])
          if d0 == 'turn':
            val = float(d1)
            for i in range(2):
              self.turn(val)
              self.time.sleep(0.001)
#      except:
#        print("bad message")

  def send(self, msg=None):
    if msg == None:
      return
    self.sock.sendto(msg.encode('utf-8'), (self.UDP_IP, self.UDP_PORT))
    return


import sys
import time
if __name__ == '__main__':
  swd = SophiaTurnUDP()
  mode = 'ros2'
  if mode == 'ros2':
    print('init ros2')
    swd.init_ros2()
    print('done init ros2')
    print('starting ros2 udp loop')
    swd.ros2()
    print('ending ros2 udp loop')
  if mode == 'udp':
    swd.init_udp()
    swd.send()

  swd.exit()

