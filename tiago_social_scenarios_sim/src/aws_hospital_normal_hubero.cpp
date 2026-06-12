#include <ros/ros.h>
#include <string>
#include <iostream>
#include <thread>
#include <chrono>

#include <hubero_common/defines.h>
#include <hubero_ros/task_request_ros_api.h>

using namespace hubero;

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

const auto TASK_TIMEOUT = ros::Duration(100.0);

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

	// wait
	ROS_INFO("TaskRequestRos APIs fired up, scenario execution will start in %lu seconds", launch_delay);
	hubero::TaskRequestRosApi::waitRosTime(launch_delay);

	// =================== 1st stage ========================================

	// preparation
	std::function<TaskFeedbackType()> actor1_move_feedback_checker = [&actor1]() -> hubero::TaskFeedbackType {
		return actor1.getMoveToGoalState();
	};
	std::function<TaskFeedbackType()> actor2_move_feedback_checker = [&actor2]() -> hubero::TaskFeedbackType {
		return actor2.getMoveToGoalState();
	};

	ROS_INFO("[SCENARIO] Firing up the execution! Actors will approach the nurse desk");

	actor1.moveToGoal(A1_NURSE_DESK_POS, A1_NURSE_DESK_YAW, TF_FRAME_REF);
	actor2.moveToGoal(A2_NURSE_DESK_POS, A2_NURSE_DESK_YAW, TF_FRAME_REF);

	actor1.startThreadedExecution(std::cref(actor1_move_feedback_checker), "moveToGoal", TASK_TIMEOUT);
	actor2.startThreadedExecution(std::cref(actor2_move_feedback_checker), "moveToGoal", TASK_TIMEOUT);

	// check until one of the actors finishes his task
	ROS_INFO("Waiting until one of the actors will reach its goal pose");
	while (actor1.isThreadExecuting() && actor2.isThreadExecuting()) {
		hubero::TaskRequestRosApi::wait();
	}

	ROS_INFO("[SCENARIO] 1st stage completed!");

	// =================== 2nd stage ========================================
	// one of the actors approached his goal
	if (!actor1.isThreadExecuting()) {
		actor1.join();
		ROS_INFO("actor1 approached the goal, he is going to start talking to a nurse");
		// talk, maintaining recently approached pose
		actor1.talk(A1_NURSE_DESK_POS, A1_NURSE_DESK_YAW, TF_FRAME_REF);
		// wait for actor2 to arrive
		actor2.join();
		ROS_INFO("actor2 also approached the goal, he is going to start talking to a nurse");
		// start talking
		actor2.talk(A2_NURSE_DESK_POS, A2_NURSE_DESK_YAW, TF_FRAME_REF);
	} else if (!actor2.isThreadExecuting()) {
		actor2.join();
		ROS_INFO("actor2 approached the goal, he is going to start talking to a nurse");
		actor2.talk(A2_NURSE_DESK_POS, A2_NURSE_DESK_YAW, TF_FRAME_REF);
		// wait for actor1 to arrive
		actor1.join();
		ROS_INFO("actor1 also approached the goal, he is going to start talking to a nurse");
		// start talking
		actor1.talk(A1_NURSE_DESK_POS, A1_NURSE_DESK_YAW, TF_FRAME_REF);
	} else {
		throw std::runtime_error("Any thread should've been finished but neither of 2 started is executing");
	}

	ROS_INFO("[SCENARIO] 2nd stage completed!");

	// =================== 3rd stage ========================================
	// actor that approached later talks for a few seconds
	hubero::TaskRequestRosApi::waitRosTime(5.0);

	// stop talking
	actor1.stopTalking();
	actor2.stopTalking();

	// wait a second and process pending callbacks
	hubero::TaskRequestRosApi::waitRosTime(1.0);

	ROS_INFO("Both actors will start moving to the exit");
	actor1.moveToGoal(A1_EXIT_POS, A1_EXIT_YAW, TF_FRAME_REF);
	actor2.moveToGoal(A2_EXIT_POS, A2_EXIT_YAW, TF_FRAME_REF);

	// renew threads
	actor1.startThreadedExecution(std::cref(actor1_move_feedback_checker), "moveToGoal", TASK_TIMEOUT);
	actor2.startThreadedExecution(std::cref(actor2_move_feedback_checker), "moveToGoal", TASK_TIMEOUT);

	actor1.join();
	actor2.join();

	ROS_INFO("[SCENARIO] 3rd stage completed!");

	// ==================== finish ==========================================

	ROS_INFO("Scenario operation finished!");
	return 0;
}
