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
// waypoints
const Vector3 Ax_WP1_POS(2.0, 4.75, 0.0);
const double Ax_WP1_YAW = 0.5 - IGN_PI_2;
const Vector3 Ax_WP2_POS(2.0, -0.5, 0.0);
const double Ax_WP2_YAW = -0.5 - IGN_PI_2;
// finish poses
const Vector3 A1_FINISH_POS(-0.10, -4.05, 0.0);
const double A1_FINISH_YAW = 1.41 - IGN_PI_2;
const Vector3 A2_FINISH_POS(0.9, -4.1, 0.0);
const double A2_FINISH_YAW = -2.00 - IGN_PI_2;

void actorWaypointFollowingThread(
	hubero::TaskRequestRosApi& actor,
	const Vector3& pos_finish,
	const double& yaw_finish
) {
	actor.moveToGoal(Ax_WP1_POS, Ax_WP1_YAW, TF_FRAME_REF);
	waitRefreshingRos();
	std::thread actor_move_to_goal_handler(moveToGoalActorHandler, std::cref(actor));
	while (!actor_move_to_goal_handler.joinable()) {
		waitRefreshingRos();
	}
	actor_move_to_goal_handler.join();

	actor.moveToGoal(Ax_WP2_POS, Ax_WP2_YAW, TF_FRAME_REF);
	waitRefreshingRos();
	actor_move_to_goal_handler = std::thread(moveToGoalActorHandler, std::cref(actor));
	while (!actor_move_to_goal_handler.joinable()) {
		waitRefreshingRos();
	}
	actor_move_to_goal_handler.join();

	actor.moveToGoal(pos_finish, yaw_finish, TF_FRAME_REF);
	waitRefreshingRos();
	actor_move_to_goal_handler = std::thread(moveToGoalActorHandler, std::cref(actor));
	while (!actor_move_to_goal_handler.joinable()) {
		waitRefreshingRos();
	}
	actor_move_to_goal_handler.join();
}

int main(int argc, char** argv) {
	// node initialization
	ros::init(argc, argv, "012_scenario_hubero_node");
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

	ROS_INFO("[SCENARIO] Firing up the execution! Actors will approach the goal poses");

	// moving through waypoints
	std::thread a2_move_to_goal_handler(actorWaypointFollowingThread, std::ref(actor2), A2_FINISH_POS, A2_FINISH_YAW);
	/*
	 * It is crucial to tune the duration so the second actor don't think that his first waypoint
	 * is not reachable (occupied by the second actor)
	 */
	waitForRosTime(ros::Duration(15.0));
	std::thread a1_move_to_goal_handler(actorWaypointFollowingThread, std::ref(actor1), A1_FINISH_POS, A1_FINISH_YAW);

	// check until both actors finish tasks
	while (!a1_move_to_goal_handler.joinable() || !a2_move_to_goal_handler.joinable()) {
		waitRefreshingRos();
	}

	a1_move_to_goal_handler.join();
	a2_move_to_goal_handler.join();

	ROS_INFO("Scenario operation finished!");
	return 0;
}
