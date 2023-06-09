class Sophia:
  ROS_CHAN_REF_POS='/ref/pos'
  ROS_CHAN_STATE_POS='/state/pos'
  ROS_CHAN_STATE_TORQUE='/state/torque'
  ROS_CHAN_STATE_ACC='/state/acc'
  init_name = "none"
  sophia_name = "none"

  NAME = 0

  ENABLE_RIGHT = True
  ENABLE_LEFT  = True



  #        name   torque_en_0  torque_en_1  pos_id  enc_id  enabled         max_pos (rad)   min_pos (rad)  filter_id  offset (deg)
  IDs = { ("lhy", 0x30,        0x31,        0x3c,   0x30,   ENABLE_LEFT ,   1.0,            -1.0,                  0,  -19.4     ),
          ("lhr", 0x32,        0x33,        0x3d,   0x32,   ENABLE_LEFT ,   0.2,            -0.2,                  1,    0.0     ),
          ("lhp", 0x34,        0x35,        0x3e,   0x34,   ENABLE_LEFT ,   0.7,            -1.3,                  2,    0.0     ),
          ("lkp", 0x36,        0x37,        0x3f,   0x36,   ENABLE_LEFT ,   1.2,             0.0,                  3,    0.0     ),
          ("lay", 0x38,        0x38,        0x38,   0x38,   ENABLE_LEFT ,   1.0,            -1.0,                  4,    0.0     ),
          ("lar", 0x39,        0x39,        0x39,   0x39,   ENABLE_LEFT ,   1.0,            -1.0,                  5,    0.0     ),
          ("lap", 0x3a,        0x3a,        0x3a,   0x3a,   ENABLE_LEFT ,   0.4,            -1.0,                  6,    0.0     ),
          ("ltp", 0x3b,        0x3b,        0x3b,   0x3b,   ENABLE_LEFT ,   1.0,            -1.0,                  7,    0.0     ),
          ("rhy", 0x10,        0x11,        0x1c,   0x10,   ENABLE_RIGHT,   1.0,            -1.0,                  8,  -13.4     ),
          ("rhr", 0x12,        0x13,        0x1d,   0x12,   ENABLE_RIGHT,   0.2,            -0.2,                  9,    0.0     ),
          ("rhp", 0x14,        0x15,        0x1e,   0x14,   ENABLE_RIGHT,   0.7,            -1.3,                 10,    0.0     ),
          ("rkp", 0x16,        0x17,        0x1f,   0x16,   ENABLE_RIGHT,   1.2,             0.0,                 11,    0.0     ),
          ("ray", 0x18,        0x18,        0x18,   0x18,   ENABLE_RIGHT,   1.0,            -1.0,                 12,    0.0     ),
          ("rar", 0x19,        0x19,        0x19,   0x19,   ENABLE_RIGHT,   1.0,            -1.0,                 13,    0.0     ),
          ("rap", 0x1a,        0x1a,        0x1a,   0x1a,   ENABLE_RIGHT,   0.4,            -1.0,                 14,    0.0     ),
          ("rtp", 0x1b,        0x1b,        0x1b,   0x1b,   ENABLE_RIGHT,   1.0,            -1.0,                 15,    0.0     )
        }




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

