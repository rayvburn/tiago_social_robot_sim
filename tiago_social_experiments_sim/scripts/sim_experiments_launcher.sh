#!/usr/bin/env bash
#
# Script that launches the simulation scenario multiple times to obtain representative results of robot navigating
# with different motion planning algorithms.
#
# It uses SRPB tooling to process logged data for benchmarking different navigation algorithms.
#
# Remember to source ROS workspace before launching. Run it like a typical bash script:
#
#   cd $(rospack find tiago_social_experiments_sim)/scripts
#   ./sim_experiments_launcher.sh
#

# Ref: https://stackoverflow.com/a/246128
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

get_scenario_type() {
    scenario="$1"
    world="$2"
    # trim trailing .launch (if exists)
    world="${world%.launch}"

    scenario_type="${world}-${scenario}"
    # return string from the function
    echo "$scenario_type"
}

get_planners_type() {
    local_planner="$1"
    global_planner="$2"
    costmap_contexts="$3"

    planners_type="${costmap_contexts}-${global_planner}-${local_planner}"
    # return string from the function
    echo "$planners_type"
}

get_experiment_type() {
    local_planner="$1"
    global_planner="$2"
    costmap_contexts="$3"
    scenario="$4"
    world="$5"
    # trim trailing .launch (if exists)
    world="${world%.launch}"

    experiment_type="$(get_scenario_type $scenario $world)-$(get_planners_type $local_planner $global_planner $costmap_contexts)"
    # return string from the function
    echo "$experiment_type"
}

# Runs the run_sim_experiment.py script
run_sim_experiment() {
    # Runs script that uses python roslaunch API to manage the execution
    timeout="$1" # ROS Time timeout
    local_planner="$2"
    global_planner="$3"
    costmap_contexts="$4"
    scenario="$5"
    world="$6"

    # 5.0, because on average, the real time factor is about 0.20;
    # plus 1 minute for the launch, 1 minute for shutdown,
    # and 1 minute just in case
    systim_timeout=$(echo "5.0 * $timeout + 60 + 60 + 60" | bc)

    echo "Running experiment with:"
    echo "  ROS time timeout: $timeout sec (system timeout is $systim_timeout sec),"
    echo "  local planner:    $local_planner,"
    echo "  global planner:   $global_planner,"
    echo "  costmap contexts: $costmap_contexts,"
    echo "  scenario:         $scenario,"
    echo "  world launch:     $world"

    echo "Experiment type: $(get_experiment_type $local_planner $global_planner $costmap_contexts $scenario $world)"

    # Main syntax:
    # python run_sim_experiment.py <timeout> <local_planner> <global_planner> <scenario> <launch_name> <launch_rel_dir> <launch_pkg_dir>

    # Run with a timeout, ref: https://stackoverflow.com/a/4581821
    # Duplicate stdout and stderr to log file
    timeout $systim_timeout \
        python $SCRIPT_DIR/run_sim_experiment.py \
            $timeout \
            $local_planner \
            $global_planner \
            $costmap_contexts \
            $scenario \
            $world \
        2>&1 | tee output.log

    # make sure all nodes and simulator processess are killed
    SLEEP_LEN=5
    echo ""
    echo "Sleeping for $SLEEP_LEN seconds before killing ROS nodes again..."
    echo ""
    sleep $SLEEP_LEN
    # https://answers.ros.org/question/237862/rosnode-kill/?answer=311516#post-id-311516
    rosnode list | grep -v rosout | xargs rosnode kill
    # try again
    pkill -9 gzclient
    pkill -9 gzserver
    pkill -9 rviz
    # ultimate - definitely breaks communication between nodes
    pkill -9 rosmaster

    echo ""
    echo "Simulation experiment finished operation"
    echo ""
}

# Searches through a given directory $1, looking for files matching a given pattern $2. Attaches a counter at the end
# of the output string that allows to create a directory with a unique name.
compute_experiment_id() {
    target_dir="$1"
    pattern="$2"

    ### obtained with ChatGPT ###
    # Find the highest counter among existing directories
    counter=0
    for dir in "${target_dir}/${pattern}"*/; do
        if [[ -d "$dir" && "$dir" =~ ${pattern}([0-9]+) ]]; then
            # 10# forces decimals
            current_counter=10#${BASH_REMATCH[1]}
            if ((current_counter > counter)); then
                counter=$current_counter
            fi
        fi
    done

    # Increment the counter for the next directory
    ((counter++))

    # Compose a name for the new directory
    dirname="${pattern}$(printf "%02d" $counter)"
    ###

    echo "$dirname"
}

# Calls run_sim_experiment and runs SRPB evaluation on the newest logs from the given directory (supposedly, related
# to the latest simulation trial)
run_benchmark_experiment() {
    # decode args as in run_sim_experiment function
    timeout="$1" # ROS Time timeout
    local_planner="$2"
    global_planner="$3"
    costmap_contexts="$4"
    scenario="$5"
    world="$6"

    # run the simulation experiment
    run_sim_experiment $timeout $local_planner $global_planner $costmap_contexts $scenario $world

    # when the experiment finishes - do stuff related to the SRPB logs
    local logs_source_dir=$7
    local logs_target_dir=$8
    echo ""
    echo "Starting SRPB stuff with logs located at $logs_source_dir. Target logs directory is $logs_target_dir"
    echo ""

    # grouping logs - make directories and files descriptive and unique
    # let's put logs from a certain scenario into the same directory
    scenario_type=$(get_scenario_type $scenario $world)
    logs_target_dir_scenario=$logs_target_dir/$scenario_type
    # directory for the specific world-scenario combo
    mkdir -p $logs_target_dir_scenario

    planners_type=$(get_planners_type $local_planner $global_planner $costmap_contexts)
    experiment_id=$(compute_experiment_id $logs_target_dir_scenario $planners_type)
    echo "Scenario type is '$scenario_type'"
    echo "Planners type is '$planners_type'"
    echo "Experiment ID is '$experiment_id'"

    logs_result_dir=$logs_target_dir_scenario/$experiment_id
    # make sure that the directory exists before call to 'copy_logs'
    mkdir -p $logs_result_dir
    echo "Logs will be copied to '$logs_result_dir'"

    # copy relevant logs (related to the latest experiment) to a separate directory under the requested target dir.
    rosrun srpb_evaluation copy_logs.sh $logs_source_dir $logs_result_dir

    # evaluate logs and create the metric results file
    rosrun srpb_evaluation evaluate_from_dir.sh $logs_result_dir

    echo ""
    echo "Finished evaluating logs with SRPB at $logs_result_dir"
    echo ""

    # copy the newest log file
    newest_log=$(ls -t *.log | head -1)
    if [ -e "$newest_log" ]; then
        echo "Copying the newest experiment's file '$newest_log' to $logs_result_dir"
        cp $newest_log $logs_result_dir
        echo "Deleting the experiment's log file '$newest_log' from the current directory"
        rm $newest_log
    fi
}

# Runs run_benchmark_experiment function given number of times
# Arguments:
#   $1 ROS time timeout
#   $2 local_planner
#   $3 global_planner
#   $4 costmap_contexts
#   $5 scenario
#   $6 world launch
#   $7 logs_source_dir
#   $8 logs_target_dir
#   $9 number of trials
run_benchmark_experiment_multiple() {
    local iterations=$9
    local i
    for ((i = 1; i <= iterations; i++)); do
        echo "Starting benchmark experiment, trial $i"
        run_benchmark_experiment $1 $2 $3 $4 $5 $6 $7 $8
    done
}

### 'Main' ###
# Arrays with scenarios to evaluate
# NOTE: timeouts for the specific scenarios are expressed as ROS Time in seconds

# declare array variables
declare -a scenarios=("hall_passing_group" "passing_in_front" "overtaking" "crossing")
declare -a launches=( "012.launch"         "012.launch"       "012.launch" "012.launch")
declare -a timeouts=( "70"                 "90"               "80"         "90")

# compare the lengths of the arrays
scenarios_num=${#scenarios[@]}
launches_num=${#launches[@]}
timeouts_num=${#timeouts[@]}
if [ "$scenarios_num" -ne "$launches_num" ] || [ "$scenarios_num" -ne "$timeouts_num" ]; then
    echo "'scenarios', 'launches' and 'timeouts' arrays must have the same length, whereas: "
    echo "  'scenarios' array has $scenarios_num elements"
    echo "  'launches' array has $launches_num elements"
    echo "  'timeouts' array has $timeouts_num elements"
    echo "Aborting"
    exit 1
fi

# SRPB stuff
## must match the main directory given in the `log_filename` parameter to the `srpb_move_base` node
readonly LOGS_SOURCE_DIR=$HOME/srpb_logs
## defines where logs will be grouped in directories and where metric results will be saved
readonly LOGS_TARGET_DIR=$HOME/srpb_logs_automation

mkdir -p "$LOGS_SOURCE_DIR"
mkdir -p "$LOGS_TARGET_DIR"

# how many times each planner will be evaluated
readonly TRIALS_NUM=8

# default configuration of the global planner
gplanner_cfg_default="global_planner_contexts"
# default configuration of the costmap layers (related to local and global costmaps)
costmap_contexts_cfg_default="social_extended"
# 'srl_eband' has a dedicated 'socially_normative' costmap plugins configuration - the main novelty are cost functions placed in the global costmap
costmap_contexts_cfg_srl_eband="socially_normative"

echo ""
echo "**Starting the experiments**"
echo ""
# use for loop to read all values and indexes
for (( i=0; i<${scenarios_num}; i++ )); do
    echo "**New scenarios loop iteration**"
    scenario="${scenarios[$i]}"
    experiment_launch="${launches[$i]}"
    timeout="${timeouts[$i]}"
    echo "  index: '$i', scenario: '$scenario', launch: '$experiment_launch', timeout: '$timeout'"

    # run the scenario with subsequent local trajectory planners
    run_benchmark_experiment_multiple $timeout teb          $gplanner_cfg_default $costmap_contexts_cfg_default   $scenario $experiment_launch $LOGS_SOURCE_DIR $LOGS_TARGET_DIR $TRIALS_NUM
    run_benchmark_experiment_multiple $timeout eband        $gplanner_cfg_default $costmap_contexts_cfg_default   $scenario $experiment_launch $LOGS_SOURCE_DIR $LOGS_TARGET_DIR $TRIALS_NUM
    run_benchmark_experiment_multiple $timeout trajectory   $gplanner_cfg_default $costmap_contexts_cfg_default   $scenario $experiment_launch $LOGS_SOURCE_DIR $LOGS_TARGET_DIR $TRIALS_NUM
    run_benchmark_experiment_multiple $timeout dwa          $gplanner_cfg_default $costmap_contexts_cfg_default   $scenario $experiment_launch $LOGS_SOURCE_DIR $LOGS_TARGET_DIR $TRIALS_NUM
    run_benchmark_experiment_multiple $timeout cohan        $gplanner_cfg_default $costmap_contexts_cfg_default   $scenario $experiment_launch $LOGS_SOURCE_DIR $LOGS_TARGET_DIR $TRIALS_NUM
    run_benchmark_experiment_multiple $timeout hateb        $gplanner_cfg_default $costmap_contexts_cfg_default   $scenario $experiment_launch $LOGS_SOURCE_DIR $LOGS_TARGET_DIR $TRIALS_NUM
    run_benchmark_experiment_multiple $timeout humap        $gplanner_cfg_default $costmap_contexts_cfg_default   $scenario $experiment_launch $LOGS_SOURCE_DIR $LOGS_TARGET_DIR $TRIALS_NUM
    run_benchmark_experiment_multiple $timeout cadrl        $gplanner_cfg_default $costmap_contexts_cfg_default   $scenario $experiment_launch $LOGS_SOURCE_DIR $LOGS_TARGET_DIR $TRIALS_NUM
    run_benchmark_experiment_multiple $timeout sarl         $gplanner_cfg_default $costmap_contexts_cfg_default   $scenario $experiment_launch $LOGS_SOURCE_DIR $LOGS_TARGET_DIR $TRIALS_NUM
    run_benchmark_experiment_multiple $timeout sarl_star    $gplanner_cfg_default $costmap_contexts_cfg_default   $scenario $experiment_launch $LOGS_SOURCE_DIR $LOGS_TARGET_DIR $TRIALS_NUM
    run_benchmark_experiment_multiple $timeout drl          $gplanner_cfg_default $costmap_contexts_cfg_default   $scenario $experiment_launch $LOGS_SOURCE_DIR $LOGS_TARGET_DIR $TRIALS_NUM
    run_benchmark_experiment_multiple $timeout drl_vo       $gplanner_cfg_default $costmap_contexts_cfg_default   $scenario $experiment_launch $LOGS_SOURCE_DIR $LOGS_TARGET_DIR $TRIALS_NUM
    run_benchmark_experiment_multiple $timeout srl_eband    $gplanner_cfg_default $costmap_contexts_cfg_srl_eband $scenario $experiment_launch $LOGS_SOURCE_DIR $LOGS_TARGET_DIR $TRIALS_NUM
done


echo ""
echo "**Finished conducting simulation experiments**"
echo ""

echo "**Preprocessing**"
echo "  Attempting to differentiate planner directories to avoid the wrong recognition by their names"
echo ""
for (( i=0; i<${scenarios_num}; i++ )); do
    scenario="${scenarios[$i]}"
    experiment_launch="${launches[$i]}"
    scenario_type=$(get_scenario_type $scenario $experiment_launch)
    if [ ! -d "${LOGS_TARGET_DIR}/${scenario_type}" ]; then
        echo "Attempted a cleanup of log directories for the '$scenario_type' scenario type but a directory '${LOGS_TARGET_DIR}/${scenario_type}' does not exist. Skipping"
        continue
    fi

    # extra dash to not mess up any other directories (e.g., drl and cadrl)
    $(rospack find srpb_evaluation)/scripts/rename_dirs_matching_pattern.sh ${LOGS_TARGET_DIR}/${scenario_type}/ -eband -srl_eband -eband_original
    $(rospack find srpb_evaluation)/scripts/rename_dirs_matching_pattern.sh ${LOGS_TARGET_DIR}/${scenario_type}/ -sarl -sarl_star -sarl_original
    $(rospack find srpb_evaluation)/scripts/rename_dirs_matching_pattern.sh ${LOGS_TARGET_DIR}/${scenario_type}/ -drl -drl_vo -drl_rgring
done

echo ""
echo "**Creating spreadsheets with the results**"
echo ""
for (( i=0; i<${scenarios_num}; i++ )); do
    scenario="${scenarios[$i]}"
    experiment_launch="${launches[$i]}"
    scenario_type=$(get_scenario_type $scenario $experiment_launch)

    if [ ! -d "${LOGS_TARGET_DIR}/${scenario_type}" ]; then
        echo "Attempted to create a SRPB results spreadsheet for the '$scenario_type' scenario type but a directory '${LOGS_TARGET_DIR}/${scenario_type}' does not exist. Skipping"
        continue
    fi

    # creates a spreadsheet from the logs that are inside directories matching the following patterns (planner names)
    python3 $(rospack find srpb_evaluation)/scripts/create_excel_from_results.py \
        ${LOGS_TARGET_DIR}/${scenario_type}/ \
        teb eband_original trajectory dwa cohan hateb humap cadrl sarl_original sarl_star drl_rgring drl_vo srl_eband
done

echo "**Finished**"
exit 0
