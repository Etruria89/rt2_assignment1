#include <inttypes.h>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "rt2_assignment1/srv/random_position.hpp"

using RandomPosition = rt2_assignment1::srv::RandomPosition;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace rt2_assignment1
{
/****************************************//**
* Server to generate a random position
*
* This server provides the  '/position_server'
* service via the node random_position_server.
********************************************/
	
	class RPS : public rclcpp::Node
	{
		public:
		
			RPS(const rclcpp::NodeOptions & options) : Node("random_position_server", options)
			{ 
				// create a new service to provide the random position when requested
				service_ = this->create_service<RandomPosition>("/position_server", std::bind(&RPS::my_random, this, _1, _2, _3));
			}
	
		private:
			  /****************************************//**
			  * Generate a random number
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
			{     
				return M + (rand() / ( RAND_MAX / (N-M) ) ) ; 
			}

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
			void my_random (
				const std::shared_ptr<rmw_request_id_t> request_header,
				const std::shared_ptr<RandomPosition::Request> req, 
				const std::shared_ptr<RandomPosition::Response> res)
			{		
				(void) request_header;
				res->x = randMToN(req->x_min, req->x_max);
				res->y = randMToN(req->y_min, req->y_max);
				res->theta = randMToN(-3.14, 3.14);
			}
	
		rclcpp::Service<RandomPosition>::SharedPtr service_; 

	};
	
}

RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1::RPS)
