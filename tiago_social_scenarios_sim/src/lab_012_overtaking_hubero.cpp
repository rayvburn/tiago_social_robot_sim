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
		"Actor1 movement will start once the robot will violate the %f m distance threshold between them",
		robot_actor1_distance_threshold
	);
	ROS_INFO(
		"Actor2 movement will start once the robot will violate the %f m distance threshold between them",
		robot_actor2_distance_threshold
	);

	// create interfaces to request tasks from actors
	hubero::TaskRequestRosApi actor1("actor1");
	hubero::TaskRequestRosApi actor2("actor2");

	// wait
	ROS_INFO("TaskRequestRos APIs fired up, scenario execution will start in %lu seconds", launch_delay);
	hubero::TaskRequestRosApi::waitRosTime(launch_delay);

	// a thread handling the movement of actor1 in the first stage of the scenario
	std::thread a1_wp_follow_handler;
	std::function<void()> fun_dist_tracker_robot_a1 = [&actor1, &a1_wp_follow_handler]() {
		ROS_INFO("[SCENARIO] Firing up the execution! The Actor1 will start the first stage of the scenario");
		a1_wp_follow_handler = actor1.moveThroughWaypoints(
			std::vector<std::pair<Vector3, double>>{
				{{+2.10, -1.10, 0.00}, +2.35 - IGN_PI_2} // near the fridge
			},
			TF_FRAME_REF
		);
		ROS_INFO("[SCENARIO] Waiting until Actor1 finishes his movement");
	};
	auto dist_tracker_robot_a1 = hubero::DistanceTracker(
		std::cref(actor1),
		"/mobile_base_controller/odom",
		std::cref(robot_actor1_distance_threshold),
		std::ref(fun_dist_tracker_robot_a1)
	);

	// atomic variable that will trigger the movement of actor2
	std::atomic<bool> robot_close_enough_a2;
	robot_close_enough_a2.store(false);
	std::function<void()> fun_dist_tracker_robot_a2 = [&robot_close_enough_a2]() {
		robot_close_enough_a2.store(true);
	};
	// subscribe robot's movement feedback to trigger the start of movement
	auto dist_tracker_robot_a2 = hubero::DistanceTracker(
		std::cref(actor2),
		"/mobile_base_controller/odom",
		std::cref(robot_actor2_distance_threshold),
		std::ref(fun_dist_tracker_robot_a2)
	);

	ROS_INFO("[SCENARIO] Waiting until Actor1 task starts...");
	while (!actor1.isThreadExecuting()) {
		hubero::TaskRequestRosApi::waitRosTime(0.1);
	}

	ROS_INFO("[SCENARIO] The task of Actor1 has started");
	// wait until the actor1 finishes movement (approaching somewhere near the actor2)
	if (a1_wp_follow_handler.joinable()) {
		a1_wp_follow_handler.join();
	}

	// synchronization - wait until the robot approaches actor2 close enough
	while (!robot_close_enough_a2.load()) {
		hubero::TaskRequestRosApi::waitRosTime(0.1);
	}
	ROS_INFO("[SCENARIO] Starting the 2nd stage of the scenario");

	// both actors start moving so the robot has to manoeuver between them to overtake
	ROS_INFO("[SCENARIO] Actor1 continues moving through waypoints");
	a1_wp_follow_handler = actor1.moveThroughWaypoints(
		std::vector<std::pair<Vector3, double>>{
			{{+1.85, +2.00, 0.00}, +3.14 - IGN_PI_2}, // midway
			{{+1.60, +4.00, 0.00}, -2.50 - IGN_PI_2}  // near the workshop
		},
		TF_FRAME_REF
	);

	ROS_INFO("[SCENARIO] Actor2 starts moving through waypoints");
	std::thread a2_wp_follow_handler = actor2.moveThroughWaypoints(
		std::vector<std::pair<Vector3, double>>{
			{{+2.65, +5.00, 0.00}, +3.14 - IGN_PI_2}  // near the workshop
		},
		TF_FRAME_REF
	);

	ROS_INFO("[SCENARIO] Waiting until both actors approach their goal poses");
	dist_tracker_robot_a1.getThread().join();
	dist_tracker_robot_a2.getThread().join();
	a1_wp_follow_handler.join();
	a2_wp_follow_handler.join();

	ROS_INFO("Scenario operation finished!");
	return 0;
}
