#! /usr/bin/env python

# import ros stuff
import rospy
# import ros message
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
# import ros service
from std_srvs.srv import *

import math

srv_client_go_to_point_ = None
srv_client_wall_follower_ = None
yaw_ = 0
yaw_error_allowed_ = 5 * (math.pi / 180)  # 5 degrees
position_ = Point()
initial_position_ = Point()
initial_position_.x = 0
initial_position_.y = 0
initial_position_.z = 0
desired_position_ = Point()
#desired_position_.x = rospy.get_param('des_pos_x')
#desired_position_.y = rospy.get_param('des_pos_y')
desired_position_.x = -1
desired_position_.y = 4
desired_position_.z = 0
regions_ = None
state_desc_ = ['Go to point', 'wall following']
state_ = 0
count_state_time_ = 0  # seconds the robot is in a state
count_loop_ = 0

# 0 - go to point
# 1 - wall following


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


# calculate the distance between two points
def calc_dist_points(point1, point2):
    dist = math.sqrt((point1.y - point2.y) ** 2 + (point1.x - point2.x) ** 2)
    return dist


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


# calculate the distance of the robot to the line connecting the starting point and goal point
def distance_to_line(p0):
    # p0 is the current position
    # p1 and p2 points define the line
    global initial_position_, desired_position_
    p1 = initial_position_
    p2 = desired_position_
    # distance = |A*x + B*y + C|/sqrt(A^2 + B^2)
    # A = p2.y - p1.y, B = - (p2.x - p1.x), C = p2.x*p1.y - p2.y*p1.x
    up_eq = math.fabs((p2.y - p1.y) * p0.x - (p2.x - p1.x) * p0.y + (p2.x * p1.y) - (p2.y * p1.x))
    lo_eq = math.sqrt(pow(p2.y - p1.y, 2) + pow(p2.x - p1.x, 2))
    distance = up_eq / lo_eq

    return distance


def main():
    global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_
    global srv_client_go_to_point_, srv_client_wall_follower_
    global count_state_time_, count_loop_

    # record the distance between desired position and current position in the last the robot leaved the obstacle
    last_distance = calc_dist_points(initial_position_, desired_position_)

    rospy.init_node('bug2')

    sub_laser = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)

    rospy.wait_for_service('/go_to_point_switch')
    rospy.wait_for_service('/wall_follower_switch')
    rospy.wait_for_service('/gazebo/set_model_state')

    srv_client_go_to_point_ = rospy.ServiceProxy('/go_to_point_switch', SetBool)
    srv_client_wall_follower_ = rospy.ServiceProxy('/wall_follower_switch', SetBool)

    # initialize going to the point
    change_state(0)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if regions_ == None:
            continue

        distance_position_to_line = distance_to_line(position_)

        if state_ == 0:
            # When the robot is going to hit the wall, change state to follow the wall
            if 0.1 < regions_['front'] < 0.3:
                change_state(1)

        elif state_ == 1:
            # when the robot go through the line connecting the initial point and goal point
            # and there is no obstacle in the front and
            # the distance between the robot and the goal point is smaller than last time,
            # go straightly ahead to the goal point
            if count_state_time_ > 5 and regions_['front'] > 0.3 and \
               distance_position_to_line < 0.1 and calc_dist_points(position_, desired_position_) < last_distance:
                last_distance = calc_dist_points(position_, desired_position_)
                change_state(0)

        count_loop_ = count_loop_ + 1
        if count_loop_ == 20:
            count_state_time_ = count_state_time_ + 1
            count_loop_ = 0

        rospy.loginfo("distance to line: [%.2f], position: [%.2f, %.2f]", distance_to_line(position_), position_.x, position_.y)
        rate.sleep()


if __name__ == "__main__":
    main()
