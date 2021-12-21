import rospy
from nav_msgs.msg import Odometry
import azmutils
import math

class Test():
    def __init__(self):
        rospy.init_node('test_node')
        self.sub = rospy.Subscriber('/odom', Odometry, self.odom)
        self.odom = Odometry()
    
    def odom(self, msg):
        self.odom = msg.pose.pose.orientation
        _a = azmutils.euler_from_quaternion(self.odom)[2]

        #print(_a * 180 / math.pi)
        t = [self.odom.x, self.odom.y, self.odom.z, self.odom.w]
        t2 = [round(i, 2) for i in t]
        print(t2)


t = Test()
rospy.spin()