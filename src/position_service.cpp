#include "ros/ros.h"
#include "rt2_assignment1/RandomPosition.h"

      /****************************************//**
      * Generate a bounded random number
      *
      * \param M (double):
      *   Lower bound.
      * \param N (double):
      *   Upper bound.
      *
      * \return randMToN (double):
      *   random number, between M and N.
      *
      ********************************************/
double randMToN(double M, double N)
{     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }

      /****************************************//**
      * Service callback to define the robot target pose
      * 			  *
      * \param request_header (const std::shared_ptr<rmw_request_id_t>):
      *   Service call header (unused).
      * \param req (const std::shared_ptr<RandomPosition::Request>):
      *   Service request, containing the x and y ranges.
      * \param res (const std::shared_ptr<RandomPosition::Response>):
      *   Service response, containing the robot pose.
      *
      ********************************************/
bool myrandom (rt2_assignment1::RandomPosition::Request &req, rt2_assignment1::RandomPosition::Response &res){
    res.x = randMToN(req.x_min, req.x_max);
    res.y = randMToN(req.y_min, req.y_max);
    res.theta = randMToN(-3.14, 3.14);
    return true;
}


int main(int argc, char **argv)
{
   ros::init(argc, argv, "random_position_server");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/position_server", myrandom);
   ros::spin();

   return 0;
}
