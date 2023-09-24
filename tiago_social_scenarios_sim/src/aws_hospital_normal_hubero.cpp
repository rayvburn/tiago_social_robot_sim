#include <ros/ros.h>
#include <string>
#include <iostream>
#include <thread>
#include <chrono>

#include <hubero_common/defines.h>
#include <hubero_ros/task_request_ros_api.h>

#include "tiago_social_scenarios_sim/utils.h"

using namespace hubero;
using namespace tiago_social_scenarios_sim;

// poses config
const std::string TF_FRAME_REF = "world";
const Vector3 A1_NURSE_DESK_POS(-1.95, +6.55, 0.0);
const double A1_NURSE_DESK_YAW = +0.5-1.5;
const Vector3 A2_NURSE_DESK_POS(-4.00, +4.00, 0.0);
const double A2_NURSE_DESK_YAW = -0.4;
// both Y would be at 19.95 if map of the floor was complete
const Vector3 A1_EXIT_POS(-5.20, +13.50, 0.0);
const double A1_EXIT_YAW = +1.5708; // orientation in the exit: +3.1415;
const Vector3 A2_EXIT_POS(+5.20, +13.50, 0.0);
const double A2_EXIT_YAW = +1.5708; // orientation in the exit: +0.0000;

int main(int argc, char** argv) {
	// node initialization
	ros::init(argc, argv, "aws_hospital_scenario_hubero_node");
	ros::NodeHandle nh;

	// check if argument(s) were provided
	long int launch_delay = 0;
	if ( argc >= 2 ) {
		launch_delay = std::stoi(argv[1]);
	}

	// create interfaces to request tasks from actors
	hubero::TaskRequestRosApi actor1("actor1");
	hubero::TaskRequestRosApi actor2("actor2");

	// abort any pending actions
	actor1.stopMovingToGoal();
	actor2.stopMovingToGoal();
	waitRefreshingRos();

	// wait
	ROS_INFO("TaskRequestRos APIs fired up, scenario execution will start in %lu seconds", launch_delay);
	waitForRosTime(ros::Duration(launch_delay));

	// =================== 1st stage ========================================
	ROS_INFO("[SCENARIO] Firing up the execution! Actors will approach the nurse desk");
	actor1.moveToGoal(A1_NURSE_DESK_POS, A1_NURSE_DESK_YAW, TF_FRAME_REF);
	actor2.moveToGoal(A2_NURSE_DESK_POS, A2_NURSE_DESK_YAW, TF_FRAME_REF);
	waitRefreshingRos();

	std::thread a1_move_to_goal_handler(moveToGoalActorHandler, std::ref(actor1));
	std::thread a2_move_to_goal_handler(moveToGoalActorHandler, std::ref(actor2));

	// check until one of the actors finishes his task
	while (!a1_move_to_goal_handler.joinable() && !a2_move_to_goal_handler.joinable()) {
		waitRefreshingRos();
	}

	ROS_INFO("[SCENARIO] 1st stage completed!");

	// =================== 2nd stage ========================================
	// one of the actors approached his goal
	if (a1_move_to_goal_handler.joinable()) {
		a1_move_to_goal_handler.join();
		ROS_INFO("actor1 approached the goal, he is going to start talking to a nurse");
		// talk, maintaining recently approached pose
		actor1.talk(A1_NURSE_DESK_POS, A1_NURSE_DESK_YAW, TF_FRAME_REF);
		// wait for actor2 to arrive
		waitRefreshingRos(std::bind(&hubero::TaskRequestRosApi::getMoveToGoalState, actor2), TASK_FEEDBACK_ACTIVE);
		a2_move_to_goal_handler.join();
		ROS_INFO("actor2 also approached the goal, he is going to start talking to a nurse");
		// start talking
		actor2.talk(A2_NURSE_DESK_POS, A2_NURSE_DESK_YAW, TF_FRAME_REF);
		waitRefreshingRos();
	} else if (a2_move_to_goal_handler.joinable()) {
		a2_move_to_goal_handler.join();
		ROS_INFO("actor2 approached the goal, he is going to start talking to a nurse");
		actor2.talk(A2_NURSE_DESK_POS, A2_NURSE_DESK_YAW, TF_FRAME_REF);
		// wait for actor1 to arrive
		waitRefreshingRos(std::bind(&hubero::TaskRequestRosApi::getMoveToGoalState, actor1), TASK_FEEDBACK_ACTIVE);
		a1_move_to_goal_handler.join();
		ROS_INFO("actor1 also approached the goal, he is going to start talking to a nurse");
		// start talking
		actor1.talk(A1_NURSE_DESK_POS, A1_NURSE_DESK_YAW, TF_FRAME_REF);
		waitRefreshingRos();
	} else {
		throw std::runtime_error("Thread should've been finished but neither of 2 is joinable");
	}

	ROS_INFO("[SCENARIO] 2nd stage completed!");

	// =================== 3rd stage ========================================
	// actor that approached later talks for a few seconds
	waitForRosTime(ros::Duration(5.0));
	waitRefreshingRos();

	// stop talking
	actor1.stopTalking();
	actor2.stopTalking();
	waitRefreshingRos();

	// wait a second and process pending callbacks
	waitForRosTime(ros::Duration(1.0));
	waitRefreshingRos();

	ROS_INFO("Both actors will start moving to the exit");
	actor1.moveToGoal(A1_EXIT_POS, A1_EXIT_YAW, TF_FRAME_REF);
	actor2.moveToGoal(A2_EXIT_POS, A2_EXIT_YAW, TF_FRAME_REF);
	waitRefreshingRos();

	// renew threads
	a1_move_to_goal_handler = std::thread(moveToGoalActorHandler, std::ref(actor1));
	a2_move_to_goal_handler = std::thread(moveToGoalActorHandler, std::ref(actor2));

	a1_move_to_goal_handler.join();
	a2_move_to_goal_handler.join();

	ROS_INFO("[SCENARIO] 3rd stage completed!");

	// ==================== finish ==========================================

	ROS_INFO("Scenario operation finished!");
	return 0;
}
