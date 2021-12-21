#!/usr/bin/python3.8

print("Starting stopextra.py")

import rospy
from std_msgs.msg import String
import os
import azmutils
from pathlib import Path
from geometry_msgs.msg import Twist

class StopExtra():
    '''
    this is an extra safeguard to make sure we can stop the robot when the other script might be busy
    '''
    def __init__(self, voice_file):
        # Base node inits
        rospy.loginfo("Initiating stop_extra_node")
        rospy.init_node('stop_extra_node')

        self.rate = rospy.Rate(10) # 10hz
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        # cmd_vel publisher
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.vel_msg = Twist()

        # stop pub
        self.stop_pub = rospy.Publisher('/stop_code', String, queue_size=1)

        _last_mod = os.stat(voice_file).st_mtime_ns
        while True:
            if _last_mod < os.stat(voice_file).st_mtime_ns:
                _last_mod = os.stat(voice_file).st_mtime_ns
                with open(voice_file, 'r') as f:
                    data = azmutils.str_to_obj(f.read())
                    # find the correct function from data["intent"] and send it the data
                    if data["intent"] == "stop_movement": 
                        self.stop_movement("")
                        
            self.rate.sleep() # remember this only sleeps if the simulation is running, otherwise time doesnt pass for rospy
        
    
    # TODO add error checking if msg doesnt match topic type
    # TODO DEBUG maybe make an ID to link attempt to success?
    def publish_once(self, topic, msg, content="message"):
        """
        Will keep retrying to publish the message if the publisher has no connections
        Returns true for msg sent success and false for failure
        Args:
            topic (rospy publisher object): topic object to publish to
            msg (rospy message object): message object to publish 
            content (String): very short description of message for debug purposes
        """
        attempts = 8
        time_multiplier = 1.5
        sleep = 0.2
        while not self.ctrl_c and attempts:
            connections = topic.get_num_connections()
            if not attempts:
                rospy.logwarn("No subscribers on {}, {} wasn't sent.".format(topic, content))
                return 0
            if connections > 0:
                topic.publish(msg)
                rospy.loginfo("{} published to {}".format(content, topic.name))
                return 1
            else:
                rospy.loginfo("Attempting to publish {} to {} but there are no subscribers on this topic, sleeping.".format(content, topic.name))
                rospy.sleep(sleep)
                attempts -= 1
                sleep *= time_multiplier

    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.stop_movement("")
        self.ctrl_c = True
   
    def stop_movement(self, input):
        stop_msg = String()
        stop_msg.data = "stop"

        self.vel_msg.linear.x = 0
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.z = 0
        self.publish_once(self.vel_pub, self.vel_msg, content="turtle stop message")
        self.publish_once(self.stop_pub, stop_msg, content="node stop message")


if __name__ == '__main__':
    print("Executing stopextra.py as main")
    print("Creating StopExtra obj")
    voice_file = Path(__file__).parent.as_posix() + "/../extra/o.json"
    stop_extra_obj = StopExtra(voice_file)