class TestWalking:
  import time
  import rclpy
  from sensor_msgs.msg import JointState
  import sophia_h as sh
  sophia = sh.Sophia()

  def __init__(self):
    self.rclpy.init()
    self.node = self.rclpy.create_node('test_walking_node')
    self.pub  = self.node.create_publisher(   self.JointState, self.sophia.ROS_CHAN_WALKING,1)


  def turn(self,n=1.0):      
    msg = self.JointState()
    msg.name.append('turn')
    msg.position.append(n)
    self.pub.publish(msg)

  def step(self,n=1):      
    msg = self.JointState()
    msg.name.append('step')
    msg.position.append(n)
    self.pub.publish(msg)

  def loop(self):
    self.rclpy.spin(self.node)

tw = TestWalking()
tw.time.sleep(5.0)
for i in range(3):
  tw.turn(-10.0)
  tw.time.sleep(0.1)

tw.time.sleep(10.0)
for i in range(3):
  tw.step(1)
  tw.time.sleep(0.1)

tw.time.sleep(5.0)
