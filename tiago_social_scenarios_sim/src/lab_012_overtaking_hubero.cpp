#include <ros/ros.h>
#include <string>
#include <iostream>
#include <thread>
#include <chrono>

#include <hubero_common/defines.h>
#include <hubero_ros/task_request_ros_api.h>

#include <move_base_msgs/MoveBaseActionFeedback.h>

using namespace hubero;

// poses config
const std::string TF_FRAME_REF = "world";

// distance between actor and a robot that will trigger the actor's movement
const auto ROBOT_ACTOR_DISTANCE_THRESHOLD_DEFAULT = 1.0;

const auto TASK_TIMEOUT = ros::Duration(60.0);

void robotMoveFeedbackCb(
	const move_base_msgs::MoveBaseActionFeedback::ConstPtr& feedback,
	const hubero::TaskRequestRosApi& actor,
	const double& robot_actor_distance_threshold,
	std::atomic<bool>& robot_close_enough
) {
	if (robot_close_enough) {
		// nothing more to do
		return;
	}

	// obtain robot pose
	double x_robot = feedback->feedback.base_position.pose.position.x;
	double y_robot = feedback->feedback.base_position.pose.position.y;
	// retrieve actor pose in the robot frame
	auto robot_frame = feedback->feedback.base_position.header.frame_id;
	double x_actor = actor.getPose(robot_frame).Pos().X();
	double y_actor = actor.getPose(robot_frame).Pos().Y();
	// compute distance between objects
	double dist = std::sqrt(
		std::pow(x_robot - x_actor, 2)
		+ std::pow(y_robot - y_actor, 2)
	);
	// if distance is small enough, set the flag
	if (dist < robot_actor_distance_threshold) {
		robot_close_enough = true;
	}
}

void actorWaypointFollowingThread(
	hubero::TaskRequestRosApi& actor,
	const std::vector<std::pair<Vector3, double>>& waypoints
) {
	std::function<TaskFeedbackType()> feedback_checker = [&actor]() -> hubero::TaskFeedbackType {
		return actor.getMoveToGoalState();
	};

	size_t waypoint_num = 1;
	for (const auto& wp: waypoints) {
		auto pos = wp.first;
		auto yaw = wp.second;
		ROS_INFO("Requesting the %lu/%lu waypoint", waypoint_num, waypoints.size());

		actor.moveToGoal(pos, yaw, TF_FRAME_REF);
		actor.startThreadedExecution(feedback_checker, "moveToGoal", TASK_TIMEOUT);
		// waiting for the execution to finish
		actor.join();

		waypoint_num++;
	}
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
	// subscribe robot's movement feedback to trigger the start of movement
	ros::Subscriber mb_sub = nh.subscribe<move_base_msgs::MoveBaseActionFeedback>(
		"/move_base/feedback",
		5,
		std::bind(
			robotMoveFeedbackCb,
			std::placeholders::_1,
			std::cref(actor2),
			std::cref(robot_actor_distance_threshold),
			std::ref(robot_close_enough_a2)
		)
	);

	// wait
	ROS_INFO("TaskRequestRos APIs fired up, scenario execution will start in %lu seconds", launch_delay);
	hubero::TaskRequestRosApi::waitRosTime(launch_delay);

	// moving through waypoints
	ROS_INFO("[SCENARIO] Firing up the execution! The Actor1 will approach his goal pose");
	std::thread a1_wp_follow_handler(
		actorWaypointFollowingThread,
		std::ref(actor1),
		std::vector<std::pair<Vector3, double>>{
			{{+2.85, -1.50, 0.00}, +3.00 - IGN_PI_2} // locker midway, on the left of the passage
		}
	);

	ROS_INFO("[SCENARIO] Waiting before making the Actor2 move");
	while (!robot_close_enough_a2) {
		hubero::TaskRequestRosApi::waitRosTime(0.5);
	}

	if (a1_wp_follow_handler.joinable()) {
		a1_wp_follow_handler.join();
	}
	a1_wp_follow_handler = std::thread(
		actorWaypointFollowingThread,
		std::ref(actor1),
		std::vector<std::pair<Vector3, double>>{
			{{+2.75, +3.20, 0.00}, +3.14 - IGN_PI_2}, // midway
			{{-1.55, +4.80, 0.00}, -2.84 - IGN_PI_2}  // near the desk in the workshop corner
		}
	);

	ROS_INFO("[SCENARIO] The Actor2 will approach his goal pose");
	std::thread a2_wp_follow_handler(
		actorWaypointFollowingThread,
		std::ref(actor2),
		std::vector<std::pair<Vector3, double>>{
			{{+1.85, +2.00, 0.00}, +3.14 - IGN_PI_2}, // midway
			{{+2.35, +5.00, 0.00}, +3.14 - IGN_PI_2}  // near the workshop
		}
	);

	ROS_INFO("[SCENARIO] Waiting until both actors approach their goal poses");

	a1_wp_follow_handler.join();
	a2_wp_follow_handler.join();

	ROS_INFO("Scenario operation finished!");
	return 0;
}
