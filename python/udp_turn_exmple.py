import udp_turn_h as uth
import time
swd = uth.SophiaTurnUDP()

swd.send(msg='turn -10.0')
time.sleep(15.0)
swd.send(msg='turn 10.0')
time.sleep(5.0)
