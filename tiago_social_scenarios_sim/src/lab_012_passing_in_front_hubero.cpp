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
	double robot_actor1_distance_threshold = ROBOT_ACTOR_DISTANCE_THRESHOLD_DEFAULT;
	double robot_actor2_distance_threshold = ROBOT_ACTOR_DISTANCE_THRESHOLD_DEFAULT;
	if (argc >= 3) {
		robot_actor1_distance_threshold = std::stod(argv[2]);
	}
	if (argc >= 4) {
		robot_actor2_distance_threshold = std::stod(argv[3]);
	}
	ROS_INFO(
		"Actor1 movement will start once the robot violates the %f m distance threshold between them",
		robot_actor1_distance_threshold
	);
	ROS_INFO(
		"Actor2 movement will start once the robot violates the %f m distance threshold between them",
		robot_actor2_distance_threshold
	);

	// create interfaces to request tasks from actors
	hubero::TaskRequestRosApi actor1("actor1");
	hubero::TaskRequestRosApi actor2("actor2");

	// wait
	ROS_INFO("TaskRequestRos APIs fired up, scenario execution will start in %lu seconds", launch_delay);
	hubero::TaskRequestRosApi::waitRosTime(launch_delay);

	// a thread handling the movement of actor1
	std::thread a1_wp_follow_handler;
	std::function<void()> fun_dist_tracker_robot_a1 = [&actor1, &a1_wp_follow_handler]() {
		ROS_INFO("[SCENARIO] Firing up the execution! The Actor1 will approach his goal pose");
		a1_wp_follow_handler = actor1.moveThroughWaypoints(
			std::vector<std::pair<Vector3, double>>{
				{{+2.80, +3.20, 0.00}, -2.90 - IGN_PI_2}, // near the pillar in the workshop area (on the right)
				{{-1.60, +4.50, 0.00}, -1.57 - IGN_PI_2}  // workshop area
			},
			TF_FRAME_REF
		);
	};
	auto dist_tracker_robot_a1 = hubero::DistanceTracker(
		std::cref(actor1),
		"/mobile_base_controller/odom",
		std::cref(robot_actor1_distance_threshold),
		std::ref(fun_dist_tracker_robot_a1)
	);

	// a thread handling the movement of actor2
	std::thread a2_wp_follow_handler;
	std::function<void()> fun_dist_tracker_robot_a2 = [&actor2, &a2_wp_follow_handler]() {
		ROS_INFO("[SCENARIO] Firing up the execution! The Actor2 will approach his goal pose");
		a2_wp_follow_handler = actor2.moveThroughWaypoints(
			std::vector<std::pair<Vector3, double>>{
				{{+2.20, +3.50, 0.00}, -2.90 - IGN_PI_2}, // near the pillar in the workshop area
				{{-0.20, +4.50, 0.00}, -1.57 - IGN_PI_2}  // workshop area
			},
			TF_FRAME_REF
		);
	};
	auto dist_tracker_robot_a2 = hubero::DistanceTracker(
		std::cref(actor2),
		"/mobile_base_controller/odom",
		std::cref(robot_actor2_distance_threshold),
		std::ref(fun_dist_tracker_robot_a2)
	);

	ROS_INFO("[SCENARIO] Waiting until both actors approach their goal poses");
	// order of `joins` matters as the `wp_follow_handler` may not be properly constructed when accessed before
	// a `dist_tracker`
	dist_tracker_robot_a1.getThread().join();
	dist_tracker_robot_a2.getThread().join();
	a1_wp_follow_handler.join();
	a2_wp_follow_handler.join();

	ROS_INFO("Scenario operation finished!");
	return 0;
}
