#include <tiago_social_scenarios_sim/utils.h>

#include <chrono>
#include <thread>
#include <stdexcept>

using namespace hubero;

namespace tiago_social_scenarios_sim {

void waitRefreshingRos() {
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	ros::spinOnce();
}

void waitRefreshingRos(std::function<TaskFeedbackType(void)> fun_check, TaskFeedbackType value_to_keep_waiting) {
	while (fun_check() == value_to_keep_waiting) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		if (!ros::ok()) {
			throw std::runtime_error("Node stopped working!");
		}
		ros::spinOnce();
	}
}

void waitForRosTime(ros::Duration duration) {
	auto time_start_delay = ros::Time::now();
	auto duration_delay = ros::Duration(0);
	while (duration_delay <= duration) {
		duration_delay = ros::Time::now() - time_start_delay;
		waitRefreshingRos();
	}
}

void moveToGoalActorHandler(const hubero::TaskRequestRosApi& actor) {
	auto TIMEOUT = ros::Duration(100.0);
	auto timeout_start = ros::Time::now();

	// we will be waiting for actor tasks finishes, so we must also know if the task request was processed
	while (actor.getMoveToGoalState() != TASK_FEEDBACK_ACTIVE) {
		if (!ros::ok()) {
			throw std::runtime_error("Node stopped working!");
		}
		if ((ros::Time::now() - timeout_start) >= TIMEOUT) {
			throw std::runtime_error("Timeout of moveToGoalActorHandler (preparation) has elapsed!");
		}
		ROS_INFO(
			"Waiting for %s task to become active. Current state %d...",
			actor.getName().c_str(),
			actor.getMoveToGoalState()
		);
		waitRefreshingRos();
	}
	ROS_INFO(
		"%s properly activated `moveToGoal` task (state %d), starting the execution...",
		actor.getName().c_str(),
		actor.getMoveToGoalState()
	);

	while (actor.getMoveToGoalState() == TASK_FEEDBACK_ACTIVE) {
		if (!ros::ok()) {
			throw std::runtime_error("Node stopped working!");
		}
		if ((ros::Time::now() - timeout_start) >= TIMEOUT) {
			throw std::runtime_error("Timeout of moveToGoalActorHandler (execution) has elapsed!");
		}
		waitRefreshingRos();
	}

	ROS_INFO("%s finished `moveToGoal` task ", actor.getName().c_str());
}

} // tiago_social_scenarios_sim
