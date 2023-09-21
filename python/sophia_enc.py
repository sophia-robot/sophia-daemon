import sophia_h as sh
import sophia_ros_helper as srh
from sensor_msgs.msg import JointState

from serial.tools import list_ports
from pyvesc import VESC
import time

sr = srh.SophiaRos()

enc_val = JointState()

def find_vesc_ports(manID=0x0483, prodID=0x5740):
    """Find VESC ports based on manufacturer and product IDs."""
    matched_ports = []
    for port in list_ports.comports():
        if port.pid == prodID and port.vid == manID:
            matched_ports.append(port.device)
    return matched_ports

def read_vesc_servo(vesc):
    """Read servo position from VESC."""    
    # Request servo position
    response = vesc.get_rpm()
    # Return servo position
    return response

if __name__ == '__main__':
    matched_ports = find_vesc_ports()
    vecs = [VESC(port) for port in matched_ports]
    while True:
        time.sleep(0.1)
        rpm = []
        try:
            for v in vecs:
                servo_position = read_vesc_servo(v)
                rpm.append(servo_position)
        except Exception:
            continue
        print(rpm)

