import socket
import rospy
from geometry_msgs.msg import Twist

UDP_IP = "192.168.0.100"
UDP_PORT = 1234
SPEED = ''
ka = 2500
kr = 572.96

print("UDP target IP: %s" % UDP_IP)
print("UDP target port: %s" % UDP_PORT)

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP


def sendSpeed(data):
    global SPEED, UDP_IP, UDP_PORT, ka
    x = str(int(ka*data.linear.x))
    y = str(int(ka*data.linear.y))
    yaw = str(int(data.angular.z * kr))
    SPEED = ('ix' + x + 'y' + y + 'r' + yaw)
    print(SPEED)
    SPEED = bytes(SPEED, "utf-8")
    print('Sending speed to motors.')
    sock.sendto(SPEED, (UDP_IP, UDP_PORT))

def speedSub():

    rospy.init_node('speedSub', anonymous=True)

    rospy.Subscriber("cmd_vel", Twist, sendSpeed)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    speedSub()
