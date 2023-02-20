#pragma once

#include <ros/ros.h>

#include <hubero_common/defines.h>
#include <hubero_ros/task_request_ros_api.h>

#include <functional>

namespace tiago_social_scenarios_sim {

/// Allows to process any ROS callbacks
void waitRefreshingRos();

void waitRefreshingRos(std::function<hubero::TaskFeedbackType(void)> fun_check, hubero::TaskFeedbackType value_to_keep_waiting);

void waitForRosTime(ros::Duration duration);

void moveToGoalActorHandler(const hubero::TaskRequestRosApi& actor);

} // tiago_social_scenarios_sim
