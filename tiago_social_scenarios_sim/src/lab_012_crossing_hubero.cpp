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

	// create interfaces to request tasks from actors
	hubero::TaskRequestRosApi actor1("actor1");
	hubero::TaskRequestRosApi actor2("actor2");

	// variables that will trigger the movement of actors
	std::atomic<bool> robot_close_enough_a1;
	std::atomic<bool> robot_close_enough_a2;
	robot_close_enough_a1 = false;
	robot_close_enough_a2 = false;

	// subscribe robot's movement feedback to trigger the start of movement
	ros::Subscriber mb_sub_a1 = nh.subscribe<move_base_msgs::MoveBaseActionFeedback>(
		"/move_base/feedback",
		5,
		std::bind(
			robotMoveFeedbackCb,
			std::placeholders::_1,
			std::cref(actor1),
			std::cref(robot_actor1_distance_threshold),
			std::ref(robot_close_enough_a1)
		)
	);
	ros::Subscriber mb_sub_a2 = nh.subscribe<move_base_msgs::MoveBaseActionFeedback>(
		"/move_base/feedback",
		5,
		std::bind(
			robotMoveFeedbackCb,
			std::placeholders::_1,
			std::cref(actor2),
			std::cref(robot_actor2_distance_threshold),
			std::ref(robot_close_enough_a2)
		)
	);

	// wait
	ROS_INFO(
		"Actor1's movement will start once the robot violates the %f m distance threshold between them",
		robot_actor1_distance_threshold
	);
	ROS_INFO(
		"Actor2's movement will start once the robot violates the %f m distance threshold between them",
		robot_actor2_distance_threshold
	);
	ROS_INFO("TaskRequestRos APIs fired up, scenario execution will start in %lu seconds", launch_delay);
	hubero::TaskRequestRosApi::waitRosTime(launch_delay);

	// prep execution
	std::function<TaskFeedbackType()> a1_feedback_checker = [&actor1]() -> hubero::TaskFeedbackType {
		return actor1.getMoveToGoalState();
	};
	std::function<TaskFeedbackType()> a2_feedback_checker = [&actor2]() -> hubero::TaskFeedbackType {
		return actor2.getMoveToGoalState();
	};

	ROS_INFO("[SCENARIO] Firing up the execution! Waiting until the robot gets closer to the 'actor1'");

	while (!robot_close_enough_a1) {
		hubero::TaskRequestRosApi::waitRosTime(0.1);
	}
	ROS_INFO("[SCENARIO] 'actor1' is going to intersect the reference path of the robot");
	actor1.moveToGoal(Vector3(+0.7, -1.4, 0.0), -2.0 - IGN_PI_2, TF_FRAME_REF);
	actor1.startThreadedExecution(std::cref(a1_feedback_checker), "moveToGoal", TASK_TIMEOUT);

	while (!robot_close_enough_a2) {
		hubero::TaskRequestRosApi::waitRosTime(0.1);
	}
	ROS_INFO("[SCENARIO] 'actor2' is going to intersect the reference path of the robot");
	actor2.moveToGoal(Vector3(+2.75, +2.65, 0.0), +1.57 - IGN_PI_2, TF_FRAME_REF);
	actor2.startThreadedExecution(std::cref(a2_feedback_checker), "moveToGoal", TASK_TIMEOUT);

	actor1.join();
	actor2.join();

	ROS_INFO("Scenario operation finished!");
	return 0;
}
