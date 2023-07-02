import time
import sophia_h as sh
import numpy as np

import rclpy
from geometry_msgs.msg import Twist

import serial
ser = serial.Serial('/dev/ttyACM0', 115200)


node = None
pub  = None
sub  = None
pub_gyro = None

sophia = sh.Sophia()

def openSerial():
  global ser
  ser.close()
  time.sleep(0.1)
  ser.open()

def init():
  global node, pub, sub, pub_gyro
  openSerial()
  rclpy.init()
  node      = rclpy.create_node('Sophia_Acc_Node')
  pub       = node.create_publisher(Twist, sophia.ROS_CHAN_STATE_ACC,1)
  pub_gyro  = node.create_publisher(Twist, sophia.ROS_CHAN_STATE_GYRO,1)

def loop():
  global pub
  global node
  global pub_gyro
  msg      = Twist()
  msg_gyro = Twist()

  while rclpy.ok():
#   try:
    data = ser.readline()
    print(data, flush=True)
    spl = data.split()
    f0 = float(spl[0])
    f1 = float(spl[1])
    f2 = float(spl[2])
    g0 = float(spl[3])
    g1 = float(spl[4])
    g2 = float(spl[5])

    ass = 9.81 / 16000.0
    xp = f0 * ass
    yp = f1 * ass
    zp = f2 * ass
    x = -zp
    y = -yp
    z = -xp

    gs = 1.0
    gxp = g0 * gs
    gyp = g1 * gs
    gzp = g2 * gs

    gx = -gzp
    gy = -gyp
    gz = -gxp


    roll   = 180.0 * np.arctan2(y,z) / np.pi +90.0
    pitch  = -( 180.0 * np.arctan2(y, x) / np.pi  + 90.0 )
    #pitch = -180.0 * np.arctan2(x, np.sqrt(y*y + z*z)) / np.pi 
    #roll  = 180.0 * np.arctan2(y, np.sqrt(x*x + z*z)) / np.pi 
    msg.linear.x = x
    msg.linear.y = y
    msg.linear.z = z
    msg.angular.x = roll
    msg.angular.y = pitch
    msg_gyro.angular.x = gx
    msg_gyro.angular.y = gy
    msg_gyro.angular.z = gz
    pub.publish(msg)
    pub_gyro.publish(msg_gyro)
    time.sleep(0.01)
#   except:
#    print('err')
#    pass
  node.destroy_node()
  rclpy.shutdown()


init()
loop()
