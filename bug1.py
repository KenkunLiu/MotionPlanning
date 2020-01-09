#! /usr/bin/env python

# import ros stuff
import rospy
# import ros message
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
# import ros service
from std_srvs.srv import *

import math

srv_client_go_to_point_ = None
srv_client_wall_follower_ = None
yaw_ = 0
yaw_error_allowed_ = 5 * (math.pi / 180)  # 5 degrees
position_ = Point()
desired_position_ = Point()
#desired_position_.x = rospy.get_param('des_pos_x')
#desired_position_.y = rospy.get_param('des_pos_y')
desired_position_.x = -1
desired_position_.y = 4
desired_position_.z = 0
regions_ = None
state_desc_ = ['Go to point', 'circumnavigate obstacle', 'go to closest point']
state_ = 0
circumnavigate_starting_point_ = Point()
circumnavigate_closest_point_ = Point()  # the closest point to the goal from obstacle
count_state_time_ = 0  # seconds the robot is in a state
count_loop_ = 0


# 0 - go to point
# 1 - circumnavigate
# 2 - go to closest point

# callback function of odometer
def clbk_odom(msg):
    global position_, yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


# laser scan callback function
def clbk_laser(msg):
    global regions_
    regions_ = {
        # scan the left region by laser
        'left': min(msg.ranges[75:105]),
        # scan the right front region by laser
        'fright': min(msg.ranges[270:330]),
        # scan the front region by laser
        'front': min(msg.ranges[0:30] + msg.ranges[330:360]),
        # scan the left front region by laser
        'fleft': min(msg.ranges[30:90]),
        # scan the right region by laser
        'right': min(msg.ranges[255:285])
    }


# state changing function
def change_state(state):
    global state_, state_desc_
    global srv_client_wall_follower_, srv_client_go_to_point_
    global count_state_time_
    count_state_time_ = 0
    state_ = state
    log = "state changed: %s" % state_desc_[state]
    rospy.loginfo(log)
    # go to the goal straightly
    if state_ == 0:
        resp = srv_client_go_to_point_(True)
        resp = srv_client_wall_follower_(False)
    # follow the wall
    if state_ == 1:
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(True)
    # follow the wall
    if state_ == 2:
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(True)


# calculate the distance between two points
def calc_dist_points(point1, point2):
    dist = math.sqrt((point1.y - point2.y) ** 2 + (point1.x - point2.x) ** 2)
    return dist


def main():
    global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_
    global srv_client_go_to_point_, srv_client_wall_follower_
    global circumnavigate_closest_point_, circumnavigate_starting_point_
    global count_loop_, count_state_time_

    rospy.init_node('bug1')

    sub_laser = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)

    rospy.wait_for_service('/go_to_point_switch')
    rospy.wait_for_service('/wall_follower_switch')

    srv_client_go_to_point_ = rospy.ServiceProxy('/go_to_point_switch', SetBool)
    srv_client_wall_follower_ = rospy.ServiceProxy('/wall_follower_switch', SetBool)

    # initialize going to the point
    change_state(0)

    rate_hz = 20
    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        if regions_ is None:
            continue

        if state_ == 0:
            if 0.1 < regions_['front'] < 0.3:
                circumnavigate_closest_point_ = position_
                circumnavigate_starting_point_ = position_
                change_state(1)

        elif state_ == 1:
            # if current position is closer to the goal than the previous closest_position, assign current position to closest_point
            if calc_dist_points(position_, desired_position_) < calc_dist_points(circumnavigate_closest_point_,
                                                                                 desired_position_):
                circumnavigate_closest_point_ = position_

            # compare only after 5 seconds - need some time to get out of starting_point
            # if robot reaches (is close to) starting point
            if count_state_time_ > 5 and \
                    calc_dist_points(position_, circumnavigate_starting_point_) < 0.2:
                change_state(2)

        elif state_ == 2:
            # if robot reaches (is close to) closest point
            if calc_dist_points(position_, circumnavigate_closest_point_) < 0.3:
                change_state(0)

        count_loop_ = count_loop_ + 1
        # 20 loops = 1 second
        if count_loop_ == 20:
            count_state_time_ = count_state_time_ + 1
            count_loop_ = 0

        rate.sleep()


if __name__ == "__main__":
    main()
