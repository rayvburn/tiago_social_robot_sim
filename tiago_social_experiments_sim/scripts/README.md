# Scripts

## Launch

The script for evaluating different local trajectory planners can be launched with the following:

```sh
$(rospack find tiago_social_experiments_sim)/scripts/sim_experiments_launcher.sh
```

## `transform_pose_map`

Allows transforming poses for the `map_ideal` case easily. Most poses with an `map_ideal` suffix (in the `config` directory) were created using this script.

## Survey experiments

### Individual

Following configurations were tested:

* first:

  ```sh
  ./run_aws_hospital_survey.sh SCENARIO=survey_individual LOCAL_PLANNER=trajectory COSTMAP_CONTEXTS=traditional
  ```

* second:

  ```sh
  ./run_aws_hospital_survey.sh SCENARIO=survey_individual LOCAL_PLANNER=hateb COSTMAP_CONTEXTS=social
  ```

* third:

  ```sh
  ./run_aws_hospital_survey.sh SCENARIO=survey_individual LOCAL_PLANNER=humap COSTMAP_CONTEXTS=social_extended
  ```
