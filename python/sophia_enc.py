import socket
import sophia_h as sh
import sophia_ros_helper as srh
from sensor_msgs.msg import JointState

from serial.tools import list_ports
from pyvesc import VESC
import time


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
    sr = srh.SophiaRos()
    matched_ports = find_vesc_ports()
    vecs = [VESC(port) for port in matched_ports]

    UDP_IP = "10.0.0.10"
    UDP_PORT = 5505
    MESSAGE = "42.0"

    the_max = 5000.0

    print("UDP target IP:", UDP_IP)
    print("UDP target port:", UDP_PORT)
    print("message:", MESSAGE)

    sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

    while True:
        time.sleep(0.2)
        rpm = []
        try:
            for v in vecs:
                servo_position = read_vesc_servo(v)
                rpm.append(servo_position)
            k = (rpm[0] + rpm[1]) / 2.0
            k = k / the_max
            sr.setWalkingK(k)
            sock.sendto(str(k).encode('utf-8'), (UDP_IP, UDP_PORT))
            print(k, end=' ')
        except Exception:
            continue
        print(rpm)




