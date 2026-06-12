#include <ros/ros.h>
#include <string>
#include <iostream>
#include <thread>
#include <chrono>

#include <hubero_common/defines.h>
#include <hubero_ros/task_request_ros_api.h>
#include <hubero_ros/task_helpers.h>

using namespace hubero;

// poses config
const std::string TF_FRAME_REF = "world";

// distance between actor and a robot that will trigger the actor's movement
const auto ROBOT_ACTOR_DISTANCE_THRESHOLD_DEFAULT = 1.0;

const auto TASK_TIMEOUT = ros::Duration(60.0);

int main(int argc, char** argv) {
	// node initialization
	ros::init(argc, argv, "012_scenario_hubero_node");
	ros::NodeHandle nh;

	// check if argument(s) were provided
	long int launch_delay = 0;
	if ( argc >= 2 ) {
		launch_delay = std::stoi(argv[1]);
	}

	double robot_actor1_distance_threshold = ROBOT_ACTOR_DISTANCE_THRESHOLD_DEFAULT;
	if (argc >= 3) {
		robot_actor1_distance_threshold = std::stod(argv[2]);
	}
	double robot_actor2_distance_threshold = ROBOT_ACTOR_DISTANCE_THRESHOLD_DEFAULT;
	if (argc >= 4) {
		robot_actor2_distance_threshold = std::stod(argv[3]);
	}
	ROS_INFO(
		"actor1's movement will start once the robot violates the %f m distance threshold between them",
		robot_actor1_distance_threshold
	);
	ROS_INFO(
		"actor2's movement will start once the robot violates the %f m distance threshold between them",
		robot_actor2_distance_threshold
	);

	// create interfaces to request tasks from actors
	hubero::TaskRequestRosApi actor1("actor1");
	hubero::TaskRequestRosApi actor2("actor2");

	ROS_INFO("TaskRequestRos APIs fired up, scenario execution will start in %lu seconds", launch_delay);
	hubero::TaskRequestRosApi::waitRosTime(launch_delay);

	// prep execution
	std::function<TaskFeedbackType()> a1_feedback_checker = [&actor1]() -> hubero::TaskFeedbackType {
		return actor1.getMoveToGoalState();
	};
	std::function<void()> fun_dist_tracker_robot_a1 = [&actor1, &a1_feedback_checker]() {
		ROS_INFO("[SCENARIO] 'actor1' is going to intersect the reference path of the robot");
		actor1.moveToGoal(Vector3(+0.7, -1.4, 0.0), -2.0 - IGN_PI_2, TF_FRAME_REF);
		actor1.startThreadedExecution(std::cref(a1_feedback_checker), "moveToGoal", TASK_TIMEOUT);
	};
	// subscribe robot's movement feedback to trigger the start of movement
	auto dist_tracker_robot_a1 = hubero::DistanceTracker(
		std::cref(actor1),
		"/mobile_base_controller/odom",
		std::cref(robot_actor1_distance_threshold),
		std::ref(fun_dist_tracker_robot_a1)
	);

	// prep execution
	std::function<TaskFeedbackType()> a2_feedback_checker = [&actor2]() -> hubero::TaskFeedbackType {
		return actor2.getMoveToGoalState();
	};
	std::function<void()> fun_dist_tracker_robot_a2 = [&actor2, &a2_feedback_checker]() {
		ROS_INFO("[SCENARIO] 'actor2' is going to intersect the reference path of the robot");
		actor2.moveToGoal(Vector3(+2.85, +2.45, 0.0), +1.57 - IGN_PI_2, TF_FRAME_REF);
		actor2.startThreadedExecution(std::cref(a2_feedback_checker), "moveToGoal", TASK_TIMEOUT);
	};
	// subscribe robot's movement feedback to trigger the start of movement
	auto dist_tracker_robot_a2 = hubero::DistanceTracker(
		std::cref(actor2),
		"/mobile_base_controller/odom",
		std::cref(robot_actor2_distance_threshold),
		std::ref(fun_dist_tracker_robot_a2)
	);

	ROS_INFO("[SCENARIO] Firing up the execution! Waiting until the robot gets closer to the 'actor1' and `actor2`");

	// order of `joins` matters as the `wp_follow_handler` may not be properly constructed when accessed before
	// a `dist_tracker`
	dist_tracker_robot_a1.getThread().join();
	dist_tracker_robot_a2.getThread().join();
	actor1.join();
	actor2.join();

	ROS_INFO("Scenario operation finished!");
	return 0;
}
