import time
import sophia_h as sh

import rclpy
from geometry_msgs.msg import Twist

import serial
ser = serial.Serial('/dev/ttyACM0', 115200)


node = None
pub  = None
sub  = None

sophia = sh.Sophia()

def openSerial():
  global ser
  ser.close()
  time.sleep(0.1)
  ser.open()

def init():
  global node, pub, sub
  openSerial()
  rclpy.init()
  node = rclpy.create_node('Sophia_Acc_Node')
  pub  = node.create_publisher(Twist, sophia.ROS_CHAN_STATE_ACC,1)

def loop():
  global pub
  global node
  msg = Twist()

  while rclpy.ok():
    data = ser.readline()
    spl = data.split()
    f0 = float(spl[0])
    f1 = float(spl[1])
    f2 = float(spl[2])
    z = f1
    x = f0
    y = -f2 
    msg.linear.x = x
    msg.linear.y = y
    msg.linear.z = z
    pub.publish(msg)
    time.sleep(0.01)
  node.destroy_node()
  rclpy.shutdown()


init()
loop()
