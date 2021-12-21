#!/usr/bin/python3.8

print("Starting voicecontrol.py")

import rospy
import os
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from nav_msgs.msg import Odometry
import azmutils
from pathlib import Path
import math
from tf import transformations as transf
import numpy as np

# these are used for the turn left/right command and are repeated again at half value
TURN_SPEED = 0.05
TURN_THRESHOLD = 0.025
VOICE_FILE = Path(__file__).parent.as_posix() + "/../extra/o.json"

class VoiceController():
    def __init__(self, voice_file):
        # Base node inits
        rospy.loginfo("Initiating voice_control_node")
        rospy.init_node('voice_control_node')

        self.ctrl_c = False
        self.rate = rospy.Rate(10) # 10hz
        rospy.on_shutdown(self.shutdownhook)

        # cmd_vel publisher
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.vel_msg = Twist()

        # odom readers
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)
        self.last_odom = Odometry()
        
        # stop sub
        self.stop_sub = rospy.Subscriber('/stop_code', String, self.stop_movement)

        self.last_mod = os.stat(voice_file).st_mtime_ns
        while True:
            if self.last_mod < os.stat(voice_file).st_mtime_ns:
                self.last_mod = os.stat(voice_file).st_mtime_ns
                with open(voice_file) as f:
                    data = azmutils.str_to_obj(f.read())
                    # find the correct function from data["intent"] and send it the data
                    if data["intent"] == "unknown_intent": continue
                    try:
                        getattr(self, data["intent"])(data["fulfillment_text"])
                    except AttributeError as e:
                        rospy.logwarn("Input intent not recognised.")
                    except Exception as e:
                        rospy.logfatal(f"Fatal exception occurred: {e}")
                        self.shutdownhook()
                        break
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
   
    def odom_cb(self, msg):
        self.last_odom = msg

    def set_speed(self, input):
        # assumes input is a string of format "setting speed to dddd"
        speed = float(input[17:])/10
        speed = min(speed, 2)
        self.vel_msg.linear.x = speed
        self.publish_once(self.vel_pub, self.vel_msg, content="new velocity")

    def stop_movement(self, input):
        self.stop = True
        self.vel_msg.linear.x = 0
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.z = 0
        self.publish_once(self.vel_pub, self.vel_msg, content="Stop message")

    def set_angular(self, input):
        # assumes input is a string of format "setting angular to dddd"
        speed = float(input[19:])/10
        speed = min(speed, 1)
        self.vel_msg.angular.z = speed
        self.publish_once(self.vel_pub, self.vel_msg, content="new velocity")

    def turn_left(self, input):
        # assumes input is a string of format "turning xxx degrees right"
        angle = -float(input[8:-13])
        self.turn(angle, -1)

    def turn_right(self, input):
        # assumes input is a string of format "turning xxx degrees left"
        angle = float(input[8:-14])
        self.turn(angle, 1)

    def turn(self, input, dir, threshold=TURN_THRESHOLD, rot_speed=TURN_SPEED):
        # dir positive for right and neg for left
        rospy.loginfo("Starting to turn")
        # save last speed and stop moving
        last_speed = self.vel_msg.linear.x
        self.stop_movement("")

        # figure out goal rotation
        _t = self.last_odom.pose.pose.orientation
        last_ori = np.array([_t.x, _t.y, _t.z, _t.w])
        wanted_ori = transf.quaternion_from_euler(0, 0, -(input * math.pi)/180)
        goal_ori = transf.quaternion_multiply(wanted_ori, last_ori)
        
        # start rotating
        self.vel_msg.angular.z = -dir * rot_speed * 10
        _f = self.publish_once(self.vel_pub, self.vel_msg, content="rotation")
        if not _f:
            rospy.logwarn("Failed rotate, aborting")
            return 0
        
        # find relative rotation 
        # last_ori_inverse = np.array([last_ori[0], last_ori[1], last_ori[2], -last_ori[3]])
        # print(f"last :{last_ori}")
        # print(f"last inv: {last_ori_inverse}")
        # relative_ori = transf.quaternion_multiply(goal_ori, last_ori_inverse)
        # print(f"rel ori: {relative_ori}")

        # until we're close enough to the right direction, recheck
        self.stop = False
        while not azmutils.are_quaternions_close(goal_ori, last_ori, threshold) and not self.stop:
            # _t2 = self.last_odom.pose.pose.orientation
            # _latest_ori_i = np.array([_t2.x, _t2.y, _t2.z, -_t2.w])
            # relative_ori = transf.quaternion_multiply(goal_ori, _latest_ori_i)
            _t = self.last_odom.pose.pose.orientation
            last_ori = np.array([_t.x, _t.y, _t.z, _t.w])
        
        # second pass
        # self.vel_msg.angular.z = -dir * rot_speed * 10 / 2
        # self.publish_once(self.vel_pub, self.vel_msg, content="rotation pass 2")
        # while not azmutils.are_quaternions_close(goal_ori, last_ori, threshold/2):
        #     _t = self.last_odom.pose.pose.orientation
        #     last_ori = np.array([_t.x, _t.y, _t.z, _t.w])
        
        self.stop_movement("")
        self.set_speed(f"Setting speed to {last_speed*10}")
        rospy.loginfo(f"Finished rotating")
        return 1

    def do_dance(self, input):
        dance_number = int(input[12:])
        dance_file = Path(__file__).parent.as_posix() + f"/../dances/{dance_number}.dance"
        
        with open(dance_file) as f:
            dance_moves = f.readlines()

        for move in dance_moves:
            if not self.stop: 
                ... # Do the move
            else:
                break
            

if __name__ == '__main__':
    print("Executing VoiceControl.py as main")
    print("Creating VoiceController obj")
    voice_controller_obj = VoiceController(VOICE_FILE)