/**
 * \file state_machine.cpp
 * \brief This files does a lot of things
 * \author Federico Danzi
 * \date 06/06/2021
 * 
*/

#include "ros/ros.h"
#include "rt2_assignment1/Command.h"
#include "rt2_assignment1/RandomPosition.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <rt2_assignment1/Control2_1Action.h>

bool start = false;
bool stop = false;

int state = 0;
/*
    0 -> static
    1 -> start
    2 -> end_reached
    -1 -> interrupt goal
*/


	  /****************************************//**
	  * Service callback setting the start/stop
	  * robot state
	  *
	  *   Service call header (unused).
	  * \param req (const std::shared_ptr<Command::Request>):
	  *   Service request, containing the command (string).
	  * \param res (const std::shared_ptr<Command::Response>):
	  *   Service response, the value of the 'start' state (bool).
	  *
	  ********************************************/ 

bool user_interface(rt2_assignment1::Command::Request &req, rt2_assignment1::Command::Response &res){
    if (req.command == "start"){
    	state = 1;
    }
    else {
    	state = -1;
    }
    return true;
}

/****************************************//**
* Callback launched at the end of the action
*
* This function is launched once the goal of
* of the action is reached.
*
* \param goal_state (const actionlib::SimpleClientGoalState&):
*   The goal state reached.
* \param result (const rt2_assignment1::PoseResultConstPtr&):
*   The result of the action.
*
********************************************/ 

void done(const actionlib::SimpleClientGoalState& goal_state,
                const rt2_assignment1::Control2_1ResultConstPtr& result){    
    state = 2;
}

int main(int argc, char **argv)
{
   /* Action client definition */	
   ros::init(argc, argv, "state_machine");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
   ros::ServiceClient client_rp = n.serviceClient<rt2_assignment1::RandomPosition>("/position_server");
   actionlib::SimpleActionClient<rt2_assignment1::Control2_1Action> ac("/go_to_point", true);
   
   rt2_assignment1::RandomPosition rp;
   rp.request.x_max = 5.0;
   rp.request.x_min = -5.0;
   rp.request.y_max = 5.0;
   rp.request.y_min = -5.0;
   rt2_assignment1::Control2_1Goal goal;
     //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(10.0))){ // timeout can be set also for the wait for server
        ROS_INFO("Waiting for the go_to_point action server to come up");
    }
   
   while(ros::ok()){
       	ros::spinOnce();
       	switch (state){
           	case 1:
		/*  Go to the goal  */
                client_rp.call(rp);
			/*  Goal set  */
                goal.x = rp.response.x;
            	goal.y = rp.response.y;
               	goal.theta = rp.response.theta;
               	std::cout << "\nGoing to the position: x= " << goal.x << " y= " << goal.y << " theta = " << goal.theta << std::endl;
			/*  Send goal information and monitor the   */
               	ac.sendGoal(goal, &done);
               	state = 0;
               	break;
            case -1:
		/*  Stop the action cancelling the Goal  */
                ac.cancelGoal();
                state = 0;
                break;
                
            case 2:
                
	            actionlib::SimpleClientGoalState goal_state = ac.getState();
                if(goal_state == actionlib::SimpleClientGoalState::SUCCEEDED){
			/*  goal reached state */
                    ROS_INFO("Pose goal reached");
                    state = 1;
                }
                else if(goal_state == actionlib::SimpleClientGoalState::PREEMPTED){
			/*  goal cancelled state */
                    ROS_INFO("Goal canceled");
                    state = 0;
                }
                
                ROS_INFO("STATE: %d", state);
        }
   }
   return 0;
}
