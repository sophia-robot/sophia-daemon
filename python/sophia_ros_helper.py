
class SophiaRos:
  # VISIR headers
  import sophia_h as sh
  # ROS2 Headers
  import rclpy
  from std_msgs.msg import String
  from std_msgs.msg import Float64
  from geometry_msgs.msg import Twist
  from sensor_msgs.msg import JointState
  import math

  sophia = sh.Sophia()
  pub_joint_ref_pos      = None
  sub_joint_ref_pos      = None
  sub_joint_state_pos    = None
  sub_joint_state_torque = None
  node = None
  def SophiaRos(self, args=None):
    self.init(args)

  def __init__(self, args=None):
    print("Start Setup")
    self.rclpy.init(args=args)
    self.node = self.rclpy.create_node('Sophia_ROS_Helper')
    self.ini_state(self.node) 
    self.ini_ref(self.node)
#    self.sophia = sh.Sophia()
    print("Node Setup")

  def ini_ref(self, node):
    self.pub_joint_ref_pos = node.create_publisher(self.JointState, self.sophia.ROS_CHAN_REF_POS, 1)
    self.pub_walking_k     = node.create_publisher(self.Float64,    self.sophia.ROS_CHAN_WALKING_K, 1)
    return 

  def deg2rad(self, val):
    return val / 180.0 * self.math.pi

  def rad2deg(self, val):
    return val / self.math.pi * 180.0

  def cb_joint_ref_pos(self, msg):
    pass 

  def cb_joint_state_pos(self, msg):
    pass 

  def cb_joint_state_torque(self, msg):
    pass


  def getWalkingK(self):
    return self.walking_k

  def setWalkingK(self, k):
    msg = self.Float64()
    msg.data = k
    self.pub_walking_k.publish(msg)

  def cb_walking_k(self, msg):
    self.walking_k = msg.data

  def sendRefPos(self, ref_val):
    self.pub_joint_ref_pos.publish(ref_val)

  def ini_state(self, node=None):
    if node == None:
      return
    self.sub_joint_ref_pos      = node.create_subscription(self.JointState, self.sophia.ROS_CHAN_REF_POS,      self.cb_joint_ref_pos,      10)
    self.sub_joint_state_pos    = node.create_subscription(self.JointState, self.sophia.ROS_CHAN_STATE_POS,    self.cb_joint_state_pos,    10)
    self.sub_joint_state_torque = node.create_subscription(self.JointState, self.sophia.ROS_CHAN_STATE_TORQUE, self.cb_joint_state_torque, 10)

    self.waking_k = 1.0
    self.sub_walking_k = node.create_subscription(self.Float64, self.sophia.ROS_CHAN_WALKING_K, self.cb_walking_k, 10)
