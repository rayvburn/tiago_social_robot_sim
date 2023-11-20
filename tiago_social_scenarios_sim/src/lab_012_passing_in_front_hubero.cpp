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
const auto ROBOT_ACTOR_DISTANCE_THRESHOLD_DEFAULT = 5.5;

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
	double robot_actor_distance_threshold = ROBOT_ACTOR_DISTANCE_THRESHOLD_DEFAULT;
	if (argc >= 3) {
		robot_actor_distance_threshold = std::stod(argv[2]);
	}
	ROS_INFO(
		"Actor's movement will start once the robot will violate the %f m distance threshold between them",
		robot_actor_distance_threshold
	);

	// create interfaces to request tasks from actors
	hubero::TaskRequestRosApi actor1("actor1");
	hubero::TaskRequestRosApi actor2("actor2");

	// variable that will trigger the movement of actor2
	std::atomic<bool> robot_close_enough_a2;
	robot_close_enough_a2 = false;
	std::function<void()> fun_dist_tracker_robot_a2 = [&robot_close_enough_a2]() {
		robot_close_enough_a2 = true;
	};
	auto dist_tracker_robot_a2 = hubero::DistanceTracker(
		std::cref(actor2),
		"/mobile_base_controller/odom",
		std::cref(robot_actor_distance_threshold),
		std::ref(fun_dist_tracker_robot_a2)
	);

	// wait
	ROS_INFO("TaskRequestRos APIs fired up, scenario execution will start in %lu seconds", launch_delay);
	hubero::TaskRequestRosApi::waitRosTime(launch_delay);

	ROS_INFO("[SCENARIO] Firing up the execution! The Actor1 will approach his goal pose");

	// moving through waypoints
	std::thread a1_wp_follow_handler = actor1.moveThroughWaypoints(
		std::vector<std::pair<Vector3, double>>{
			{{+2.70, +3.20, 0.00}, -2.90 - IGN_PI_2}, // near the pillar in the workshop area (on the right)
			{{-1.60, +4.50, 0.00}, -1.57 - IGN_PI_2}  // workshop area
		},
		TF_FRAME_REF
	);
	ROS_INFO("[SCENARIO] Waiting to make the Actor2 move");

	// flag is updated asynchronously
	while (!robot_close_enough_a2) {
		hubero::TaskRequestRosApi::waitRosTime(0.5);
	}
	ROS_INFO("[SCENARIO] Firing up the execution! The Actor2 will approach his goal pose");

	// moving through waypoints
	std::thread a2_wp_follow_handler = actor2.moveThroughWaypoints(
		std::vector<std::pair<Vector3, double>>{
			{{+2.20, +3.50, 0.00}, -2.90 - IGN_PI_2}, // near the pillar in the workshop area
			{{-0.20, +4.50, 0.00}, -1.57 - IGN_PI_2}  // workshop area
		},
		TF_FRAME_REF
	);

	ROS_INFO("[SCENARIO] Waiting until both actors approach their goal poses");

	a1_wp_follow_handler.join();
	a2_wp_follow_handler.join();
	dist_tracker_robot_a2.getThread().join();

	ROS_INFO("Scenario operation finished!");
	return 0;
}
