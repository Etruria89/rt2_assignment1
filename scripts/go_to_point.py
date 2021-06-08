## @package rt2_assignment1
#	\file go_to_point.py
#	\brief This file contain the description of the movement of the holonomic robot
#	\author Federico Danzi	
#	\date 08/06/2021
#	
#	\details
#
#	\Publisher:
#	    \cmd_vel
#
#	\Subscriber:
#	    \odom
#
#	\Action Server: <BR>
#	    \go_to_point
#
#	This node define the movememnt of the holonomic robot


#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
#from rt2_assignment1.srv import Position
import math
import actionlib
import actionlib.msg
import rt2_assignment1.msg #this package

# robot state variables
position_ = Point()
yaw_ = 0
position_ = 0
state_ = 0
# publisher
pub_ = None

# action_server
act_s = None

# parameters for control
yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
dist_precision_ = 0.1
kp_a = -3.0 
kp_d = 0.2
ub_a = 0.6
lb_a = -0.5
ub_d = 0.6


##
# Service callback of the odometry subscriber
#
#
#    Retrieve (x,y,z and theta) from the Odom message.
#    Args:
#      msg (Odometry): odometry message.
# 

def clbk_odom(msg):

    global position_
    global yaw_

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


##
#  Function to specify the state_ value
#
#   
#   Update the current global state
#    Args: state (int):  new state
#
# 

def change_state(state):

    global state_
    state_ = state
    print ('State changed to [%s]' % state_)

##
#    Function to normalize an angle
#
#    Args: angle (float):  angle to be normalized
#    
# 

def normalize_angle(angle):

    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

##
#    Orient the robot in a desired way
#
#    Args:  des_yaw (float):  desired yaw
#      next_state (int): next state to set
#   

def fix_yaw(des_yaw, next_state):

    err_yaw = normalize_angle(des_yaw - yaw_)
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(next_state)


##
#   Move straignt to the target
#    Set the linear and angular speed
#    depending on the distance to the 
#    goal pose.
#
#    Args:
#      des_pos (Point):  desired (x, y) position
#    

def go_straight_ahead(des_pos):

    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
                        pow(des_pos.x - position_.x, 2))
    err_yaw = normalize_angle(desired_yaw - yaw_)
    rospy.loginfo(err_yaw)

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.3+0.5*err_pos
        if twist_msg.linear.x > ub_d:
            twist_msg.linear.x = ub_d

        twist_msg.angular.z = kp_a*err_yaw
        pub_.publish(twist_msg)
    else: # state change conditions
        #print ('Position error: [%s]' % err_pos)
        change_state(2)

    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(0)

##
#    Turns the robot to reach
#    the final desired yaw
#    Args:
#      des_yaw (Point):  desired final yaw
#    


def fix_final_yaw(des_yaw):
    err_yaw = normalize_angle(des_yaw - yaw_)
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(3)
 
##
#    Stop the robot
#
#    Args: 
#      None
#    Set the robot velocities to 0.
#  
        
def done():
    """
    
    
    """
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub_.publish(twist_msg)
 
 
##
#    Set the appropriate behaviour depending
#    on the current robot state, in orderd
#    to reach the goal.
#    The state machine keeps running until
#    the goal is reached or the action is
#    preempted (the goal gets cancelled).
#
#    Args:
#      goal (PoseActionGoal): (x,y,theta) goal pose
#     
    
def go_to_point(goal):

    global act_s
    
    desired_position = Point()
    # get the desired position from the goal received
    desired_position.x = goal.x 
    desired_position.y = goal.y
    des_yaw = goal.theta

    rate = rospy.Rate(20)
    success = True
    change_state(0)

    feedback = rt2_assignment1.msg.Control2_1Feedback()
    result = rt2_assignment1.msg.Control2_1Result()
    
    mission_complete = False
    
    while not rospy.is_shutdown() and not mission_complete:
        if act_s.is_preempt_requested():
            feedback.status = 'Goal was preempted'
            act_s.set_preempted() # if we received the cancel we interrupt
            success = False
            done()
            change_state(-1)    
            mission_complete = True
        else:
            if state_ == 0:
                feedback.status = "Aligning with goal"                
                desired_yaw = math.atan2(desired_position.y - position_.y, desired_position.x - position_.x)
                fix_yaw(desired_yaw, 1)
            elif state_ == 1:
                feedback.status = "Aligned with goal"
                go_straight_ahead(desired_position)
            elif state_ == 2:
                feedback.status = "Goal [x, y] position reached"
                fix_yaw( des_yaw, 3)
            elif state_ == 3:
                feedback.status = "Goal pose reached!"
                done
                mission_complete = True
            else:
                rospy.logerr('Unknown state!')
                
        rate.sleep()
    if success:
        rospy.loginfo('TYPE RESULT'+str(type(result)))
        result.reached = success
        rospy.loginfo('Mission Complete!')
        act_s.set_succeeded(result)

##
#    Main function to manage 
#    the robot behaviour
#
#    Including :
#	- the initialization of the "go_to_point"  node
#	- the publisher for the "\cmd_vel" topic
#	- the subscriber to the "\odom" topic
#	- the action server "\go_to_point"
#

def main():

    global pub_, act_s
    rospy.init_node('go_to_point')
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    act_s = actionlib.SimpleActionServer(
        '/go_to_point', rt2_assignment1.msg.Control2_1Action, go_to_point, auto_start=False) 
    act_s.start()
    
    rospy.spin()

if __name__ == '__main__':
    main()
