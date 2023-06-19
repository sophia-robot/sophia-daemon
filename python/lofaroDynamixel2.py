from dynamixel_sdk import *                    # Uses Dynamixel SDK library
import os
import numpy as np

class LofaroDynamixel2:
  OK      = None
  FAIL    = None
  ENABLE  = None
  DISABLE = None


  def __init__(self, baud=None, port=None):
    global OK, FAIL, ENABLE, DISABLE
    self.OK   = 0
    self.FAIL = 1

    self.ENABLE  = 0
    self.DISABLE = 1

    OK      = self.OK
    FAIL    = self.FAIL
    ENABLE  = self.ENABLE
    DISABLE = self.DISABLE
  
    self.PI                          = 3.141592653589793238
    self.ENC_REZ                     = 4096
    self.ADDR_TORQUE_ENABLE          = 64
    self.ADDR_GOAL_POSITION          = 116
    self.LEN_GOAL_POSITION           = 4         # Data Byte Length
    self.ADDR_PRESENT_POSITION       = 132
    self.LEN_PRESENT_POSITION        = 4
    self.ADDR_PRESENT_TORQUE         = 132
    self.LEN_PRESENT_TORQUE          = 2

    self.TORQUE_RAW_TO_AMP           = 0.00269
    self.TORQUE_OFFSET_INT           = 2048
    self.TORQUE_INT_MIN              = -2048
    self.TORQUE_INT_MAX              =  2048

    if baud == None:
      self.BAUDRATE                    = 1000000
    else:
      self.BAUDRATE                    = baud


    self.PROTOCOL_VERSION            = 2.0


    if port == None:
      self.DEVICENAME                  = '/dev/ttyUSB0'
    else:
      self.DEVICENAME                  = port

    ss = 'setserial '+self.DEVICENAME+' low_latency'
    os.system(ss)

    self.TORQUE_ENABLE               = 1                 # Value for enabling the torque
    self.TORQUE_DISABLE              = 0                 # Value for disabling the torque

    self.portHandler    = PortHandler(self.DEVICENAME)
    self.packetHandler  = PacketHandler(self.PROTOCOL_VERSION)
    self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, self.ADDR_GOAL_POSITION, self.LEN_GOAL_POSITION)
    self.groupSyncRead  = GroupSyncRead(self.portHandler, self.packetHandler, self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION)

    return

  def getDxlError(self, dxl_comm_result=None, dxl_error=None):
    if dxl_comm_result == None:
      return self.FAIL
    elif dxl_error == None:
      return self.FAIL
    elif dxl_comm_result != COMM_SUCCESS:
      return self.FAIL
    elif dxl_error != 0:
      return self.FAIL
    else:
      return self.OK


  def open(self):
    # Opens the port
    if self.portHandler.openPort():
      return self.OK
    else:
      return self.FAIL

  def setBaud(self, baud=None):
    if baud == None:
      baud = self.BAUDRATE
    # sets the baud
    if self.portHandler.setBaudRate(baud):
      self.portHandler.setPacketTimeout(1)
      self.portHandler.setPacketTimeoutMillis(1)
      self.BAUDRATE = baud
      return self.OK
    else:
      return self.FAIL
    
  def torque(self, the_id=None, do_enable=None):
    if   the_id == None:
      return self.FAIL
    elif the_id < 0:
      return self.FAIL
    elif the_id > 253:
      return self.FAIL
    elif do_enable == None:
      return self.FAIL
    elif do_enable == ENABLE:
      dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, the_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
      return self.getDxlError(dxl_comm_result, dxl_error)
    elif do_enable == DISABLE:
      dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, the_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
      return self.getDxlError(dxl_comm_result, dxl_error)
    return self.FAIL

  def stagePos(self, the_id=None, the_pos=None):
    if the_pos == None:
      return self.FAIL
    
    enc = self.rad2enc(the_pos)
    return self.stagePosEnc(the_id, enc)

  def stagePosDeg(self, the_id=None, the_pos=None):
    if the_pos == None:
      return self.FAIL
    
    enc = self.deg2enc(the_pos)
    return self.stagePosEnc(the_id, enc)
    

  def stagePosEnc(self, the_id=None, the_pos=None):
    if the_id == None:
      return self.FAIL
    elif the_pos == None:
      return self.FAIL

    the_pos = int(the_pos)
    dxl_goal_position = the_pos

    # Allocate goal position value into byte array
    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(the_pos)), 
                           DXL_HIBYTE(DXL_LOWORD(the_pos)), 
                           DXL_LOBYTE(DXL_HIWORD(the_pos)), 
                           DXL_HIBYTE(DXL_HIWORD(the_pos))]

    # Add Dynamixel goal position value to the Syncwrite parameter storage
    dxl_addparam_result = self.groupSyncWrite.addParam(the_id, param_goal_position)
    if dxl_addparam_result != True:
      return self.FAIL
    else:
      return self.OK

  def resetStagePos(self):
    self.groupSyncWrite.clearParam()
    return self.OK


  def putPos(self):
    # Syncwrite goal position
    dxl_comm_result = self.groupSyncWrite.txPacket()
    dxl_error       = 0
    self.resetStagePos()
    return self.getDxlError(dxl_comm_result, dxl_error)
        
  def getPos(self, the_id):
    # Read present position
    enc, err = self.getPosEnc(the_id)
    rad = self.enc2rad(enc)
    return (rad, err)

  def getPosDeg(self, the_id):
    # Read present position
    enc, err = self.getPosEnc(the_id)
    deg = self.enc2deg(enc)
    return (deg, err)

  def getInt16(self, val):
    
    val16 = val  - self.TORQUE_OFFSET_INT
    if val16 > self.TORQUE_INT_MAX:
      val16 = self.TORQUE_INT_MAX
    elif val16 < self.TORQUE_INT_MIN:
      val16 = self.TORQUE_INT_MIN 
    return val16

  def getTorque(self, the_id):
    torqueRaw, err = self.getTorqueRaw(the_id)
    if err == self.OK:
      torqueRaw = self.getInt16(torqueRaw)
      torque = torqueRaw * self.TORQUE_RAW_TO_AMP
      return (torque, err)
    return (torqueRaw, err)

  def getTorqueRaw(self, the_id):
    # Read present position
    dxl_present_torque, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, the_id, self.ADDR_PRESENT_TORQUE)
    return (dxl_present_torque, self.getDxlError(dxl_comm_result, dxl_error))


  def getPosEnc(self, the_id):
    # Read present position
    dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, the_id, self.ADDR_PRESENT_POSITION)
    return (dxl_present_position, self.getDxlError(dxl_comm_result, dxl_error))

  def enc2rad(self, enc):
    res = 1.0/self.ENC_REZ 
    rad = enc * res * 2.0 * self.PI - self.PI
    return rad    

  def enc2deg(self, enc):
    rad = self.enc2rad(enc)
    deg = self.rad2deg(rad)
    return deg

  def rad2deg(self, rad):
    deg = rad * 180.0 / self.PI
    return deg

  def deg2rad(self, deg):
    rad = deg * self.PI / 180.0
    return rad

  def rad2enc(self, rad):
    enc = rad / (2.0 * self.PI) * self.ENC_REZ + self.ENC_REZ/2.0
    enc = int(enc)
    if enc < 0:
      enc = 0
    elif enc > (self.ENC_REZ-1):
      enc = self.ENC_REZ - 1
    return enc

  def deg2enc(self, deg):
    rad = self.deg2rad(deg)
    return self.rad2enc(rad)

  def ping(self, the_id):
    DXL_ID = the_id
    dxl_model_number, dxl_comm_result, dxl_error = self.packetHandler.ping(self.portHandler, DXL_ID)
    return DXL_ID, self.getDxlError(dxl_comm_result, dxl_error)
