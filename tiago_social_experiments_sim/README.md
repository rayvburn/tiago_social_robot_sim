# tiago_social_experiments_sim

Package that runs TiAGO robot system to perform specific experiments.

Once workspace has been build, launch exemplary experiment (`012` scenario) with (Ubuntu 18 & ROS Melodic):

```bash
cd ~/ros_workspace/ws_social_navigation
source /usr/share/gazebo-9/setup.sh && source ../ws_tiago/devel/setup.bash && source devel/setup.bash
roslaunch tiago_social_experiments_sim 012.launch
```

One may want to launch `012` world without robot - it can be done with:

```bash
roslaunch gazebo_ros empty_world.launch world_name:=$(rospack find tiago_sim_integration)/worlds/lab_012_v2_actor.world
```

## `aws_hospital` scenario

[AWS Hospital world](https://github.com/aws-robotics/aws-robomaker-hospital-world) is a pretty big one and there are some issues launching it properly with a simulated TIAGo robot. Trying to launch TIAGo with a controller set that is appropriate to the TIAGo robot specification, e.g., Iron. Uncommenting the launch section pointed below in the `tiago_simulation/tiago_gazebo/launch/simulation_tiago_bringup.launch` file helps to properly run the Gazebo client (`gzclient`):

```xml
<include file="$(find tiago_controller_configuration_gazebo)/launch/tiago_controllers.launch">
    <arg name="arm" value="$(arg arm)"/>
    <arg name="end_effector" value="$(arg end_effector)"/>
    <arg name="ft_sensor" value="$(arg ft_sensor)"/>
    <arg name="laser_model" value="$(arg laser_model)"/>
    <arg name="camera_model" value="$(arg camera_model)"/>
    <arg name="use_moveit_camera" value="$(arg use_moveit_camera)"/>
    <arg name="robot_namespace" value="$(arg robot_namespace)"/>
    <arg name="base_type_suffix"  value="$(arg base_type_suffix)"/>
</include>
```

Therefore, a lightweight version of the world was prepared and is used in simulation setup.

## Evaluation

To evaluate the local planner, use metrics evaluation packages: [`srpb`](https://github.com/rayvburn/srpb) and [`srpb_move_base`](https://github.com/rayvburn/srpb_move_base).
