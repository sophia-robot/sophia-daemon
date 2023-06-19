#!/usr/bin/env python

# try this: https://github.com/ros2/ros1_bridge

class HansonWalking:
  import rospy
  from std_msgs.msg import String

  joints = [ 'LeftLegYaw',
             'LeftLegRoll',
             'LeftLegPitch',
             'LeftKneePitch',
             'leftFootYaw',
             'LeftFootRoll',
             'LeftFootPitch',
             'LeftToePitch',
             'RightLegYaw',
             'RightLegRoll',
             'RightLegPitch',
             'RightKneePitch',
             'RightFootYaw',
             'RightFootRoll',
             'RightFootPitch',
             'RightToePitch']

  def __init__(self):
    self.topic = '/hr/animations/walking/joint'

  def cb_walking(self, msg):
    self.rospy.loginfo(self.rospy.get_caller_id() + "I heard %s", msg.data)
    
  def run():

    node = self.rospy.init_node('hanson_walking_listener', anonymous=True)

    sub  = self.rospy.Subscriber(self.topic, String, cb_walking)

    # spin() simply keeps python from exiting until this node is stopped
    self.rospy.spin()


hw = HansonWalking()

if __name__ == '__main__':
    hw.run()
