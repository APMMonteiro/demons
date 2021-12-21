from math import sqrt
from math import pi
import json
import tf
from geometry_msgs.msg import Quaternion
import numpy as np

def dynamic_euclid_dist(a, b):
    o = 0
    for i in range(len(a)):
        o += (a[i]-b[i])**2
    return sqrt(o)

def quaternion_from_euler(roll, pitch, yaw):
    q = tf.transformations.quaternion_from_euler(roll, pitch, yaw, 'rxyz')
    return Quaternion(q[0], q[1], q[2], q[3])

def euler_from_quaternion(q):
    q = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w], 'rxyz')
    return (q[0], q[1], q[2])

def are_quaternions_close(q1, q2, threshold=0.05):
    # checks if the two quaternions represent close to the same rotation
    # returns True if theyre close enough
    pos_check = True
    neg_check = True
    for e1, e2 in zip(q1, q2):
        if np.abs(e1 - e2) > threshold:
            pos_check = False
    for e1, e2 in zip(q1, -q2):
        if np.abs(e1 - e2) > threshold:
            neg_check = False
    return neg_check or pos_check

def str_to_obj(string):
    try:
        return json.loads(string)
    except ValueError as e:
        raise ValueError("ValueError occured when loading JSON string: {}, the input was: {}".format(e, string))
    
def obj_to_str(obj):
    return json.dumps(obj)