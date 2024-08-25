#!/usr/bin/env bash

set -e

# Shell's named args parser, ref: https://unix.stackexchange.com/a/353639
for ARGUMENT in "$@"
do
   KEY=$(echo $ARGUMENT | cut -f1 -d=)

   KEY_LENGTH=${#KEY}
   VALUE="${ARGUMENT:$KEY_LENGTH+1}"

   export "$KEY"="$VALUE"
done

readonly TARGET_DIR_DEFAULT="~/metrics_survey"
readonly CAMERA_IMG_DIR_DEFAULT="/tmp/hospital"
readonly SCENARIO_DEFAULT="survey_individual"
readonly LOCAL_PLANNER_DEFAULT="humap"
readonly GLOBAL_PLANNER_DEFAULT="global_planner_contexts"
readonly COSTMAP_CONTEXTS_DEFAULT="social_extended"

TARGET_DIR="${TARGET_DIR:-$TARGET_DIR_DEFAULT}"
CAMERA_IMG_DIR="${CAMERA_IMG_DIR:-$CAMERA_IMG_DIR_DEFAULT}"
SCENARIO="${SCENARIO:-$SCENARIO_DEFAULT}"
LOCAL_PLANNER="${LOCAL_PLANNER:-$LOCAL_PLANNER_DEFAULT}"
GLOBAL_PLANNER="${GLOBAL_PLANNER:-$GLOBAL_PLANNER_DEFAULT}"
COSTMAP_CONTEXTS="${COSTMAP_CONTEXTS:-$COSTMAP_CONTEXTS_DEFAULT}"
TRIAL_NAME="${TRIAL_NAME:-${COSTMAP_CONTEXTS}-${GLOBAL_PLANNER}-${LOCAL_PLANNER}}"

echo "SCENARIO          $SCENARIO"
echo "LOCAL_PLANNER     $LOCAL_PLANNER"
echo "GLOBAL_PLANNER    $GLOBAL_PLANNER"
echo "COSTMAP_CONTEXTS  $COSTMAP_CONTEXTS"
echo "TRIAL_NAME        $TRIAL_NAME"
echo "TARGET_DIR        $TARGET_DIR"

# Check the value of the variable using if-elif-else
if [[ "${SCENARIO}" == "survey_individual" || "${SCENARIO}" == "survey_group" ]]; then
    # differentiating delays between the scenarios is no longer necessary
    export "PUBLISH_GOAL_DELAY"="50"
else
    echo "Scenario type not recognized."
    exit 0
fi

# desired value of the output directory
output_dir="${TARGET_DIR}/${SCENARIO}/${TRIAL_NAME}"
# Check if the directory exists, and if it does, find a new name
counter=1
while [[ -d "$output_dir" ]]; do
    # Increment the directory name with a zero-padded counter (e.g., 02, 03)
    printf -v output_dir "%s%02d" "${TARGET_DIR}/${SCENARIO}/${TRIAL_NAME}" "$counter"
    ((counter++))
done
mkdir -p $output_dir
echo "Output resources will be saved in '${output_dir}'"

#
# Parameters:
# * 'lockstep'
#   * Lockstep parameter is crucial to collect camera images with the desired update rate, see
#   * http://classic.gazebosim.org/tutorials?tut=lockstep_physics_sensors&cat=sensors
# * 'timeout --foreground 260 <roslaunch ...>'
#   * this would be handy but terminates the whole script from further execution
#
# Launch the scenario
roslaunch tiago_social_experiments_sim aws_hospital.launch \
    scenario:=${SCENARIO} \
    local_planner:=${LOCAL_PLANNER} \
    global_planner:=${GLOBAL_PLANNER} \
    costmap_contexts:=${COSTMAP_CONTEXTS} \
    navigation_benchmark:=true \
    perception_launch:=ground_truth \
    publish_goal:=true \
    publish_goal_delay:=${PUBLISH_GOAL_DELAY} \
    run_plot:=false \
    run_reconfigure:=false \
    gazebo_extra_args:="--lockstep" \
    gazebo_recording:=true

# Copy and process SRPB logs
rosrun srpb_evaluation copy_logs.sh ${HOME}/srpb_logs_survey ${output_dir}
rosrun srpb_evaluation evaluate_from_dir.sh ${output_dir}

# Relocate newest Gazebo log
# Find the newest directory in the source directory
newest_log_dir=$(ls -td "${HOME}/.gazebo/log/"*/ | head -1)
mv "${newest_log_dir}" "${output_dir}"

# Check if directory exists
if [[ -d "${CAMERA_IMG_DIR}" ]]; then
    # Check if there are any files or directories
    if [[ ! $(ls -A "${CAMERA_IMG_DIR}") ]]; then
        echo "${CAMERA_IMG_DIR} directory is empty, cannot process with image to video conversion"
        exit 0
    fi
fi

# Relocate camera streams
mv ${CAMERA_IMG_DIR}/* "${output_dir}"

# To perform image to video conversion once again, put a desired "output dir" here
# and uncomment the previous part of the script
# output_dir=""

# Convert JPG to MP4
for dir in "${output_dir}"/*/; do
    # Check if it is a directory
    if [[ -d "$dir" ]]; then
         # Check if there are any .jpg files in the directory
        if compgen -G "${dir}/*.jpg" > /dev/null; then
            # Process each directory
            echo "Processing camera images directory: $dir"
            dir_path="$(realpath $dir)"
            dirname="$(basename $dir_path)"
            # Ref: http://classic.gazebosim.org/tutorials?tut=camera_save&cat=sensors
            # Expecting images captured at 60 FPS
            ffmpeg \
                -r 60 \
                -pattern_type glob -i "${dir_path}/*.jpg" \
                -c:v libx264 \
                -b 3000k \
                -qscale 0 \
                -y \
                ${output_dir}/${dirname}.mp4
        else
            echo "No .jpg files found in $dir. Skipping..."
        fi
    fi
done

echo ""
echo "Now, you are ready to collect metric values into Excel spreadsheet like so:"
echo ""
echo '    python3 $(rospack find srpb_evaluation)/scripts/create_excel_from_results.py' "${TARGET_DIR}/${SCENARIO}" "<space-separated local planner list>"
echo ""
