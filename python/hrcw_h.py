from dynamixel_sdk import *                    # Uses Dynamixel SDK library

class HumanoidRobotCatWalk:
  OK   = 0
  FAIL = 1

  ENABLE  = 0
  DISABLE = 1

  def __init__(self, baud=None, port=None):
    self.ADDR_TORQUE_ENABLE          = 64
    self.ADDR_GOAL_POSITION          = 116
    self.LEN_GOAL_POSITION           = 4         # Data Byte Length
    self.ADDR_PRESENT_POSITION       = 132


    if baud == None:
      self.BAUDRATE                    = 1000000
    else:
      self.BAUDRATE                    = baud


    self.PROTOCOL_VERSION            = 2.0


    if port == None:
      self.DEVICENAME                  = '/dev/ttyUSB0'
    else:
      self.DEVICENAME                  = port


    self.TORQUE_ENABLE               = 1                 # Value for enabling the torque
    self.TORQUE_DISABLE              = 0                 # Value for disabling the torque

    self.portHandler    = PortHandler(self.DEVICENAME)
    self.packetHandler  = PacketHandler(self.PROTOCOL_VERSION)
    self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, self.ADDR_GOAL_POSITION, self.LEN_GOAL_POSITION)
    self.groupSyncRead  = GroupSyncRead(self.portHandler, self.packetHandler, self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION)

    return

  def getDxlError(self, dxl_comm_result=None, dxl_error=None):
    if dxl_comm_result == None:
      return FAIL
    elif dxl_error == None:
      return FAIL
    elif dxl_comm_result != COMM_SUCCESS:
      return FAIL
    elif dxl_error != 0:
      return FAIL
    else:
      return OK


  def open(self):
    # Opens the port
    if self.portHandler.openPort():
      return OK
    else:
      return FAIL

  def setBaud(self):
    # sets the baud
    if self.portHandler.setBaudRate(self.BAUDRATE):
      return OK
    else:
      return FAIL
    
  def torque(self, the_id=None, do_enable=None):
    if   the_id == None:
      return FAIL
    elif the_id < 0:
      return FAIL
    elif the_id > 253:
      return FAIL
    elif do_enable == None:
      return FAIL
    elif do_enable == ENABLE:
      dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, the_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
      return self.getDxlError(dxl_comm_result, dxl_error)
    elif do_enable == DISABLE
      dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, the_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
      return self.getDxlError(dxl_comm_result, dxl_error)
    return FAIL

  def stagePos(self, the_id=None, the_pos=None):

    if the_id == None:
      return FAIL
    elif the_pos == None:
      return FAIL

    dxl_goal_position = the_pos
    # Allocate goal position value into byte array
    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(the_pos)), 
                           DXL_HIBYTE(DXL_LOWORD(the_pos)), 
                           DXL_LOBYTE(DXL_HIWORD(the_pos)), 
                           DXL_HIBYTE(DXL_HIWORD(the_pos))]

    # Add Dynamixel goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite.addParam(the_id, param_goal_position)
    if dxl_addparam_result != True:
      return FAIL
    else:
      return OK

  def resetStagePos(self):
    self.groupSyncWrite.clearParam()
    return OK


  def putPos(self):
    # Syncwrite goal position
    dxl_comm_result, dexl_error = self.groupSyncWrite.txPacket()
    return self.getDxlError(dxl_comm_result, dxl_error)
        

