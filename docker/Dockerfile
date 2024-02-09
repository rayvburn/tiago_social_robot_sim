ARG ROS_DISTRO="melodic"
FROM ros:$ROS_DISTRO-robot-bionic

# libffi-dev -> ModuleNotFoundError: No module named '_ctypes'
# https://github.com/pypa/packaging-problems/issues/573 (when installing gym)
RUN apt-get update && apt-get install -y \
	make \
	build-essential \
	software-properties-common \
	libssl-dev \
	zlib1g-dev \
	libbz2-dev \
	libreadline-dev \
	libsqlite3-dev \
	wget \
	curl \
	git \
	llvm \
	libncursesw5-dev \
	xz-utils \
	tk-dev \
	libxml2-dev \
	libxmlsec1-dev \
	libffi-dev \
	liblzma-dev

RUN apt-get update && apt-get install -y \
	virtualenv \
	python-pip \
	python3-pip \
	python3-dev

RUN pip install --user catkin-tools virtualenv
RUN pip3 install --user virtualenv

# CMake
RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null
RUN apt-add-repository "deb https://apt.kitware.com/ubuntu/ bionic main"
RUN apt update && apt-get install -y cmake

# make ROS packages available via apt
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt update && apt-get install -y \
	ros-$ROS_DISTRO-roslint \
	ros-$ROS_DISTRO-move-base-msgs \
	ros-$ROS_DISTRO-move-base \
	ros-$ROS_DISTRO-people-msgs \
	ros-$ROS_DISTRO-laser-filters \
	ros-$ROS_DISTRO-navigation-layers \
	ros-$ROS_DISTRO-diagnostic-updater \
	ros-$ROS_DISTRO-eband-local-planner \
	ros-$ROS_DISTRO-laser-geometry \
	ros-$ROS_DISTRO-pcl-conversions \
	ros-$ROS_DISTRO-pcl-ros \
	ros-$ROS_DISTRO-ira-laser-tools \
	ros-$ROS_DISTRO-sophus \
	ros-$ROS_DISTRO-pointcloud-to-laserscan \
	ros-$ROS_DISTRO-ddynamic-reconfigure \
	ros-$ROS_DISTRO-map-server \
	ros-$ROS_DISTRO-image-proc \
	ros-$ROS_DISTRO-costmap-converter \
	ros-$ROS_DISTRO-image-common \
	ros-$ROS_DISTRO-four-wheel-steering-msgs \
	ros-$ROS_DISTRO-urdf-geometry-parser \
	ros-$ROS_DISTRO-moveit-ros-planning-interface \
	ros-$ROS_DISTRO-joy \
	ros-$ROS_DISTRO-mbf-costmap-core \
	ros-$ROS_DISTRO-mbf-msgs \
	ros-$ROS_DISTRO-libg2o \
	ros-$ROS_DISTRO-twist-mux \
	ros-$ROS_DISTRO-moveit-planners-ompl \
	ros-$ROS_DISTRO-moveit-simple-controller-manager

RUN apt update && apt-get install -y \
	libnetpbm10-dev \
	libsuitesparse-dev \
	libeigen3-dev \
	libsvm-dev \
	libsdl-dev \
	libsdl2-dev \
	libsdl-image1.2-dev \
	libignition-common-dev \
	libfmt-dev \
	libmove-base-msgs-dev \
	libceres-dev \
	python-tk \
	python-scipy \
	python3-pkg-resources \
	python3-catkin-tools \
	python3-vcstool

# Simulation and GUI stuff
RUN apt update && apt-get install -y \
	ros-$ROS_DISTRO-gazebo-ros \
	ros-$ROS_DISTRO-gazebo-ros-pkgs \
	ros-$ROS_DISTRO-gazebo-ros-control \
	ros-$ROS_DISTRO-rviz \
	ros-$ROS_DISTRO-rqt \
	ros-$ROS_DISTRO-rqt-common-plugins

# database of models for Gazebo (to avoid downloading in launch time)
RUN git clone https://github.com/osrf/gazebo_models /gazebo_models
ENV GAZEBO_MODEL_PATH="/gazebo_models:${GAZEBO_MODEL_PATH}"

# SRPB scripts dependencies
RUN apt update && apt-get install -y \
	python3-openpyxl \
	python3-matplotlib \
	python3-numpy

# Install fuzzylite library required by the HUMAP planner;
# NOTE: the build script is only available along the older, 6.0 version
# Ref: https://forums.docker.com/t/best-practices-for-git-clone-make-etc-via-dockerfile-run/79152/4
RUN git clone https://github.com/fuzzylite/fuzzylite.git -b fuzzylite-6.x && \
	cd fuzzylite/fuzzylite && \
	./build.sh
# See the README for additional instructions

## Preparing the workspace structure
RUN mkdir -p /ws_ros
WORKDIR /ws_ros
# and directories for logs
RUN mkdir -p srpb/logs_launcher
RUN mkdir -p srpb/logs_grouped

# 1st workspace - TIAGo robot software packages
RUN mkdir -p ws_tiago/src/
COPY tiago_public-melodic.repos /tmp/tiago_public-melodic.repos
# 'exit' is a workaround for detached HEADs (returns non-zero code)
RUN cd /ws_ros/ws_tiago/src; vcs import < /tmp/tiago_public-melodic.repos
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd /ws_ros/ws_tiago; catkin build'

# 2nd workspace - human perception packages
RUN mkdir -p ws_perception/src
RUN git clone https://github.com/rayvburn/spencer_people_tracking.git -b melodic-devel ws_perception/src/spencer_people_tracking
# source underlying workspace and build overlay
RUN /bin/bash -c '. /ws_ros/ws_tiago/devel/setup.bash; cd /ws_ros/ws_perception; catkin build'

# 3rd workspace - Social navigation packages (prepare structure as it is currently passed as external volume)
RUN mkdir -p ws_social_nav/src
# source underlying workspace and build overlay
RUN /bin/bash -c '. /ws_ros/ws_perception/devel/setup.bash; cd /ws_ros/ws_social_nav; catkin config -DIGN_MATH_VER=4; catkin build'
RUN /bin/bash -c '. /ws_ros/ws_social_nav/devel/setup.bash'

# create a basic script to (re)build the overlay workspace
# FIXME: some steps could be omitted once the `ws_social_nav` is cloned in the Dockerfile
WORKDIR /
# NOTE: remember to source Gazebo paths
RUN /bin/bash -c \
	'echo "source /usr/share/gazebo/setup.sh; source /ws_ros/ws_perception/devel/setup.bash; cd /ws_ros/ws_social_nav; catkin build; source devel/setup.bash" > prepare_social_nav_ws.sh; chmod +x prepare_social_nav_ws.sh'

RUN /bin/bash -c \
	'echo "source /usr/share/gazebo/setup.sh; export GAZEBO_MODEL_PATH=/gazebo_models:$GAZEBO_MODEL_PATH ; source /ws_ros/ws_social_nav/devel/setup.bash" > prepare_for_launch.sh; chmod +x prepare_for_launch.sh'

WORKDIR /ws_ros/ws_social_nav
