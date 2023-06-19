#!/usr/bin/env python

class HansonWalking:
  import rospy
  from std_msgs.msg import String

  def __init__(self):
    self.topic = '/hr/animations/walking/joint'

  def cb_walking(self, msg):
    self.rospy.loginfo(self.rospy.get_caller_id() + "I heard %s", msg.data)
    
  def run():

    self.rospy.init_node('hanson_walking_listener', anonymous=True)

    self.rospy.Subscriber(self.topic, String, cb_walking)

    # spin() simply keeps python from exiting until this node is stopped
    self.rospy.spin()


hw = HansonWalking()

if __name__ == '__main__':
    hw.run()
