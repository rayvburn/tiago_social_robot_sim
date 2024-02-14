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

const auto TASK_TIMEOUT = ros::Duration(60.0);

void actorWaypointFollowingThread(
	hubero::TaskRequestRosApi& actor,
	const Vector3& pos_finish,
	const double& yaw_finish
) {
	std::function<TaskFeedbackType()> feedback_checker = [&actor]() -> hubero::TaskFeedbackType {
		return actor.getMoveToGoalState();
	};

	actor.moveToGoal(Ax_WP1_POS, Ax_WP1_YAW, TF_FRAME_REF);
	actor.startThreadedExecution(feedback_checker, "moveToGoal", TASK_TIMEOUT);
	actor.join();

	actor.moveToGoal(Ax_WP2_POS, Ax_WP2_YAW, TF_FRAME_REF);
	actor.startThreadedExecution(feedback_checker, "moveToGoal", TASK_TIMEOUT);
	actor.join();

	actor.moveToGoal(pos_finish, yaw_finish, TF_FRAME_REF);
	actor.startThreadedExecution(feedback_checker, "moveToGoal", TASK_TIMEOUT);
	actor.join();
}

void actorFollowingThread(hubero::TaskRequestRosApi& actor, const std::string& obj_name) {
	std::function<TaskFeedbackType()> feedback_checker = [&actor]() -> hubero::TaskFeedbackType {
		return actor.getFollowObjectState();
	};

	actor.followObject(obj_name);
	actor.startThreadedExecution(feedback_checker, "FollowObject");
	actor.join();
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

	// wait
	ROS_INFO("TaskRequestRos APIs fired up, scenario execution will start in %lu seconds", launch_delay);
	hubero::TaskRequestRosApi::waitRosTime(launch_delay);

	ROS_INFO("[SCENARIO] Firing up the execution! Actors will approach the goal poses");

	// moving through waypoints
	std::thread a2_move_to_goal_handler(actorWaypointFollowingThread, std::ref(actor2), A2_FINISH_POS, A2_FINISH_YAW);
	std::thread a1_follow_handler(actorFollowingThread, std::ref(actor1), "actor2");

	ROS_INFO("[SCENARIO] Waiting until the Actor2 approaches the goal pose");
	a2_move_to_goal_handler.join();

	// let's stop following - move to goal instead
	ROS_INFO("[SCENARIO] Actor1 will stop following the Actor2");
	actor1.stopFollowingObject();
	a1_follow_handler.join();

	// move to the goal pose
	ROS_INFO("[SCENARIO] Actor1 will move to the finish pose");
	std::function<TaskFeedbackType()> a1_feedback_checker = [&actor1]() -> hubero::TaskFeedbackType {
		return actor1.getMoveToGoalState();
	};
	actor1.moveToGoal(A1_FINISH_POS, A1_FINISH_YAW, TF_FRAME_REF);
	actor1.startThreadedExecution(std::cref(a1_feedback_checker), "moveToGoal", TASK_TIMEOUT);

	actor1.join();
	actor2.join();

	ROS_INFO("Scenario operation finished!");
	return 0;
}
