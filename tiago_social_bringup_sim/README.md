# tiago_social_bringup_sim

A package that runs the TIAGo robot system to execute certain tasks/scenarios in a simulation.

## Run exemplary world

Launch the whole system with:

```bash
roslaunch tiago_social_bringup_sim gazebo_sim_and_nav.launch
```

Or run robot simulation and navigation in separate tabs:

```bash
roslaunch tiago_social_bringup_sim gazebo_sim.launch
roslaunch tiago_social_bringup_sim gazebo_nav.launch
```

## Spawn TIAGo in a custom world

```bash
roslaunch tiago_social_bringup_sim gazebo_spawn.launch
```
