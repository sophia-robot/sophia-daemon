class SophiaFilter:
  def __init__(self):
    itmp = 0
    self.ENUM_LHY = itmp
    itmp += 1
    self.ENUM_LHR = itmp
    itmp += 1
    self.ENUM_LHP = itmp
    itmp += 1
    self.ENUM_LKP = itmp
    itmp += 1
    self.ENUM_LAY = itmp
    itmp += 1
    self.ENUM_LAR = itmp
    itmp += 1
    self.ENUM_LAP = itmp
    itmp += 1
    self.ENUM_LTP = itmp
    itmp += 1
    self.ENUM_RHY = itmp
    itmp += 1
    self.ENUM_RHR = itmp
    itmp += 1
    self.ENUM_RHP = itmp
    itmp += 1
    self.ENUM_RKP = itmp
    itmp += 1
    self.ENUM_RAY = itmp
    itmp += 1
    self.ENUM_RAR = itmp
    itmp += 1
    self.ENUM_RAP = itmp
    itmp += 1
    self.ENUM_RTP = itmp
    itmp += 1
    self.ENUM_MOT_NUM = itmp
    ref_buff = [0.0] * self.ENUM_MOT_NUM

    self.L_filt = 20

  def SophiaFilter(self):
    pass

  def doFilter(self, mot, val):
    ret = 0.0
    err = self.FAIL

    if mot < 0:
      return ret, err
    if mot >= self.ENUM_MOT_NUM:
      return ret, err

    self.ref_buff[mot] = (val + self.ref_buff[mot]*(self.L_filt - 1.0)) / self.L_filt
    err = self.OK
    return self.ref_buff[mot], err

