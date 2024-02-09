#!/usr/bin/env bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd $SCRIPT_DIR

# Move up to the directory with .repos/.rosinstalls
cd ..

docker build \
	-t tiago_social_robot_sim:latest \
	-f docker/Dockerfile \
	.
