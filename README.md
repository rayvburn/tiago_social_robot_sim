# tiago_social_robot_sim

Packages for social navigation experiments based on a TIAGo robot. This repository stores contents that are required for simulated setup.

## Run

Typical usage is, e.g.:

```sh
roslaunch tiago_social_experiments_sim aws_hospital.launch navigation_benchmark:=false local_planner:=hateb
```

### Advanced usage

Advanced usage can involve the following.
Run HuBeRo example with sourced TIAGo packages (to spawn the robot in simulation):

Start Gazebo simulation:

```sh
cd ~/ros_workspace/ws_social_navigation
source /usr/share/gazebo/setup.sh && source ../ws_tiago/devel/setup.bash && source devel/setup.bash
roslaunch hubero_bringup_gazebo_ros example.launch world:=living_room rviz:=false
```

Spawn TIAGo robot:

```sh
cd ~/ros_workspace/ws_social_navigation
source ../ws_tiago/devel/setup.bash && source devel/setup.bash
roslaunch tiago_social_bringup_sim gazebo_spawn.launch
```

Run navigation modules for TIAGo (without robot simulation):

```sh
cd ~/ros_workspace/ws_social_navigation
source ../ws_tiago/devel/setup.bash && source devel/setup.bash
```

- localization
  - if `map_server` is already running
    ```sh
    roslaunch tiago_social_navigation navigation_base.launch state:=localization map:=none
    ```
  - if a new map should be loaded (map path is just an example)
    ```sh
    roslaunch tiago_social_navigation navigation_base.launch state:=localization map:=$(rospack find hubero_bringup_gazebo_ros)/maps/living_room.yaml
    ```

- mapping (SLAM)
  ```sh
  roslaunch tiago_social_navigation navigation_base.launch state:=mapping map:=none
  ```

Alternatively, navigation modules can be run using this command (which also opens a special `rviz` config provided by `PAL`):

```sh
roslaunch tiago_social_navigation navigation_tiago.launch map:=none
```

Custom `rviz` config:

```sh
rviz -d $(rospack find tiago_social_navigation)/rviz/tiago_navigation.rviz
```

NOTE: `tiago_social_navigation/navigation_tiago.launch` can only be used for localization.
