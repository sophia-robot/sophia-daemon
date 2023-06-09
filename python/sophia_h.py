class Sophia:
  ROS_CHAN_REF_POS='/ref/pos'
  ROS_CHAN_STATE_POS='/state/pos'
  ROS_CHAN_STATE_TORQUE='/state/torque'

  init_name = "none"
  sophia_name = "none"

  def Sophia(self):
    self.sophia_name = "sophia_name"
    print("made the class")

  def __init__(self):
    print("in init")
    self.init_name = "init_name"

  def printThis(self):
    print("this")
    print(self.init_name)
    print(self.sophia_name)

