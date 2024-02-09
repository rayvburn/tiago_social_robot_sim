#!/usr/bin/env bash

# Ref: https://github.com/osrf/rocker?tab=readme-ov-file#generic-gazebo

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd $SCRIPT_DIR

set -e

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

# try with 'sudo' if does not work, ref: https://github.com/osrf/rocker
rocker \
	--nvidia \
	--x11 \
	--ssh \
	--volume \
		"$ws_src_volume":"/ws_ros/ws_social_nav/src" \
		"$ws_build_volume":"/ws_ros/ws_social_nav/build" \
		"$ws_devel_volume":"/ws_ros/ws_social_nav/devel" \
		"$ws_logs_volume":"/ws_ros/ws_social_nav/logs" \
		"$logs_source_dir":"/ws_ros/srpb/logs_launcher" \
		"$logs_target_dir":"/ws_ros/srpb/logs_grouped" \
	-- \
	tiago_social_robot_sim:latest \
	"bash"

exit 0
