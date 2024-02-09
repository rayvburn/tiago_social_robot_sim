#!/usr/bin/env bash
#
# Run the container with necessary volumes mounted
#
# Sources of the social navigation workspace must be passed to the container from local storage,
# as some repositories are limited to a private use ATM
#
# Remember to manually adjust the path where the benchmark logs are saved (see README)
#

set -e

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd $SCRIPT_DIR

echo ""
echo "Check the README for useful hints!"
echo ""

local_social_nav_src="${1:-../..}"
logs_source_dir="${2:-$HOME/srpb_logs}"
logs_target_dir="${3:-$HOME/srpb_logs_automation}"

local_social_nav_src=$(realpath $local_social_nav_src)
logs_source_dir=$(realpath $logs_source_dir)
logs_target_dir=$(realpath $logs_target_dir)

mkdir -p $logs_source_dir
mkdir -p $logs_target_dir

ws_src_volume="$local_social_nav_src"
# temporarily store compilation artifacts to avoid rebuilding each time the container is ran
ws_build_volume="/tmp/ws_social_nav/build"
ws_devel_volume="/tmp/ws_social_nav/devel"
ws_logs_volume="/tmp/ws_social_nav/logs"

mkdir -p $ws_build_volume
mkdir -p $ws_devel_volume
mkdir -p $ws_logs_volume

echo "Mounting local sources from '$local_social_nav_src'"
echo "Directory for storing logs is '$logs_source_dir' whereas grouped logs will be at '$logs_target_dir'"

docker run --rm -it  \
	-v "$local_social_nav_src":"/ws_ros/ws_social_nav/src" \
	-v "$ws_build_volume":"/ws_ros/ws_social_nav/build" \
	-v "$ws_devel_volume":"/ws_ros/ws_social_nav/devel" \
	-v "$ws_logs_volume":"/ws_ros/ws_social_nav/logs" \
	-v "$logs_source_dir":"/ws_ros/srpb/logs_launcher" \
	-v "$logs_target_dir":"/ws_ros/srpb/logs_grouped" \
	tiago_social_robot_sim
