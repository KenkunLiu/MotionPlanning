#! /usr/bin/env python

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *

import math

pub_ = None
regions_ = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
}
state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
}

# when active_ is True, start this node
active_ = False


# determine when to start this node
def wall_follower_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res


# laser scan callback function
def clbk_laser(msg):
    global regions_
    regions_ = {
        # scan the right front region by laser
        'fright': min(msg.ranges[270:330]),
        # scan the front region by laser
        'front':  min(msg.ranges[0:30]+msg.ranges[330:360]),
        # scan the left front region by laser
        'fleft':  min(msg.ranges[30:90]),
    }

    take_action()


# state changing function
def change_state(state):
    global state_, state_dict_
    if state is not state_:
        print('Wall follower - [%s] - %s' % (state, state_dict_[state]))
        state_ = state


# change the following state according to the information from laser
def take_action():
    global regions_
    regions = regions_

    d = 0.3  # the threshold to determine whether there is a obstacle in a certain direction

    if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 1 - nothing'
        change_state(0)
    elif regions['front'] < d < regions['fleft'] and regions['fright'] > d:
        state_description = 'case 2 - front'
        change_state(1)
    elif regions['front'] > d > regions['fright'] and regions['fleft'] > d:
        state_description = 'case 3 - fright'
        change_state(2)
    elif regions['front'] > d > regions['fleft'] and regions['fright'] > d:
        state_description = 'case 4 - fleft'
        change_state(0)
    elif regions['front'] < d < regions['fleft'] and regions['fright'] < d:
        state_description = 'case 5 - front and fright'
        change_state(1)
    elif regions['front'] < d < regions['fright'] and regions['fleft'] < d:
        state_description = 'case 6 - front and fleft'
        change_state(1)
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 7 - front and fleft and fright'
        change_state(1)
    elif regions['front'] > d > regions['fleft'] and regions['fright'] < d:
        state_description = 'case 8 - fleft and fright'
        change_state(0)
    else:
        rospy.loginfo(regions)
    print(state_description)


# the action to take in the first state
def find_wall():
    msg = Twist()
    msg.linear.x = 0.1
    msg.angular.z = -0.3
    return msg


# the action to take in the second state
def turn_left():
    msg = Twist()
    msg.angular.z = 0.3
    return msg


# the action to take in the third state
def follow_the_wall():
    global regions_

    msg = Twist()
    msg.linear.x = 0.2
    return msg


def main():
    global pub_, active_

    rospy.init_node('wall_follower')
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    srv = rospy.Service('wall_follower_switch', SetBool, wall_follower_switch)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not active_:
            rate.sleep()
            continue

        msg = Twist()
        if state_ == 0:
            msg = find_wall()
        elif state_ == 1:
            msg = turn_left()
        elif state_ == 2:
            msg = follow_the_wall()
            pass
        else:
            rospy.logerr('Unknown state!')

        pub_.publish(msg)

        rate.sleep()


if __name__ == '__main__':
    main()
