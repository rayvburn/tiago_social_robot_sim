# Container setup

Please note that despite the following instructions lead to a properly running simulation inside a container, this is not usable for benchmarking due to strange problems that were not investigated further. For example:

- `gpu_ray` sensor type is not recognized (despite `--nvidia`),
- HuBeRo actors are not spawned correctly (possibly due to the above issue),
- navigation sometimes got stuck in random moments.

## Prerequisites

Sources of the "social navigation" workspace must be passed to the container from local storage, as some repositories are limited to a private use ATM. Also, the following extra steps are required:

* change the directory that will store the logs. Go to `tiago_social_robot/tiago_social_navigation/launch/move_base.launch`, and change to the following:
  ```sh
  <arg name="benchmark_log_file" default="/ws_ros/srpb/logs_launcher/log_$(arg global_planner)_$(arg local_planner)_$(arg costmap_contexts).txt" unless="$(arg multiple)"/>
  <arg name="benchmark_log_file" default="/ws_ros/srpb/logs_launcher/log_$(arg robot_namespace)_$(arg global_planner)_$(arg local_planner)_$(arg costmap_contexts).txt" if="$(arg multiple)"/>
  ```

* change the target directory of `fuzzylite` library (until more elegant way of handling this is implemented) at `humap_local_planner/CMakeLists.txt`:
  ```sh
  find_library(fuzzylite_LIBRARY fuzzylite /fuzzylite/fuzzylite/release/bin)
  include_directories(/fuzzylite/fuzzylite)
  ```

## Building Docker image

* Build the image with workspace packages dependencies:
  ```sh
  ./docker_build.sh
  ```

* Provide the sources to locally stored social navigation packages workspace, mount `devel`, `build`, `logs` directories to `tmp`:
  ```sh
  ./docker_run.sh
  ```

## Running the image with Rocker

To run with the [graphical environment](https://github.com/osrf/rocker?tab=readme-ov-file#generic-gazebo), install `rocker` with:

```sh
sudo apt-get install python3-rocker
```

Next, run the `rocker`:
```sh
./rocker_run.sh
```


Then, inside the container, manually build the overlay workspace (underlay workspaces are built in the `Dockerfile`). Type:

```sh
/prepare_social_nav_ws.sh
source /prepare_for_launch.sh
```
