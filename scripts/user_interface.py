""" 
..module: user_interface
    :platform:Unix
    :synopsis: This file contain the user interface service server
    :moduleauthor: Federico Danzi

Client : <BR>
    user_interface

This node define the command line user interface for the control of the robot
"""

import rospy
import time
from rt2_assignment1.srv import Command

"""   
Definition of the command line
user interface to set the *start*
*stop* varaible to control the robot by relying on the `rospy <http://wiki.ros.org/rospy/>`_ 
module.

The service is passed to the service ``user_interface``, 
advertised by the :mod:`state_machine`
"""  

def main():

    rospy.init_node('user_interface')
    ui_client = rospy.ServiceProxy('/user_interface', Command)
    time.sleep(10)
    rate = rospy.Rate(20)
    x = int(input("\nPress 1 to start the robot "))
    while not rospy.is_shutdown():
        if (x == 1):
            ui_client("start")
            x = int(input("\nPress 0 to stop the robot "))
        else:
            print("The robot stopped.")
            ui_client("stop")
            x = int(input("\nPress 1 to start the robot "))
            
if __name__ == '__main__':
    main()
