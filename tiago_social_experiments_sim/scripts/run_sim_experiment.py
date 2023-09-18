import roslaunch
import rospy
import subprocess
import sys
import os

from move_base_msgs.msg import MoveBaseActionResult
from actionlib_msgs.msg import GoalStatus


def mbCallback(data):

    rospy.loginfo(rospy.get_caller_id() + " move_base action status %s", data.status.status)
    # evaluate navigation state
    navigation_state = int(data.status.status)
    nav_succeeded = navigation_state == GoalStatus.SUCCEEDED
    nav_aborted = navigation_state == GoalStatus.ABORTED
    nav_rejected = navigation_state == GoalStatus.REJECTED

    if nav_succeeded or nav_aborted or nav_rejected:
        rospy.loginfo(rospy.get_caller_id() + " Shutting down the 'run_sim_experiment' script! move_base status: %s", data.status.status)
        rospy.sleep(1)
        parent.shutdown()


if __name__ == '__main__':
    argsnum = len(sys.argv)

    print('Printing script arguments:')
    for idx, arg in enumerate(sys.argv):
        print('\t(' + str(idx) + ') ' + str(arg))

    # defaults
    launch_pkg_dir = subprocess.check_output(['rospack', 'find', 'tiago_social_experiments_sim']).lstrip().rstrip()
    launch_rel_dir = 'launch'
    launch_name = 'aws_hospital.launch'
    scenario = 'normal'
    global_planner = 'global_planner'
    costmap_contexts = 'social_extended'
    local_planner = 'teb'
    timeout = 50.0 # expressed as ROS Time

    if argsnum >= 2:
        timeout = float(sys.argv[1])
    if argsnum >= 3:
        local_planner = str(sys.argv[2]).lstrip().rstrip()
    if argsnum >= 4:
        global_planner = str(sys.argv[3]).lstrip().rstrip()
    if argsnum >= 5:
        costmap_contexts = str(sys.argv[4]).lstrip().rstrip()
    if argsnum >= 6:
        scenario = str(sys.argv[5]).lstrip().rstrip()
    if argsnum >= 7:
        launch_name = str(sys.argv[6]).lstrip().rstrip()
    if argsnum >= 8:
        launch_rel_dir = str(sys.argv[7]).lstrip().rstrip()
    if argsnum >= 9:
        launch_pkg = str(sys.argv[8]).lstrip().rstrip()

    print('Running the simulation experiment launcher with:')
    print('\t timeout:          ' + str(timeout))
    print('\t local_planner:    ' + str(local_planner))
    print('\t global_planner:   ' + str(global_planner))
    print('\t costmap_contexts: ' + str(costmap_contexts))
    print('\t scenario:         ' + str(scenario))
    print('\t launch_name:      ' + str(launch_name))
    print('\t launch_rel_dir:   ' + str(launch_rel_dir))
    print('\t launch_pkg_dir:   ' + str(launch_pkg_dir))

    # create a launch path
    launch_path = os.path.join(launch_pkg_dir, launch_rel_dir, launch_name)

    # roslaunch API usage instructions can be found at: http://wiki.ros.org/roslaunch/API%20Usage
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    cli_args = [
        str(launch_path),
        'local_planner:=' + str(local_planner),
        'global_planner:=' + str(global_planner),
        'costmap_contexts:=' + str(costmap_contexts),
        'scenario:=' + str(scenario),
        'navigation_benchmark:=true',
        'perception_launch:=true',
        'publish_goal:=true',
        'run_reconfigure:=false'
    ]
    print("CLI args of the launcher are: " + str(cli_args[1:]))
    roslaunch_args = cli_args[1:]
    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

    # TODO: experimental timeouts to make shutdown more robust
    parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file, sigint_timeout=30, sigterm_timeout=15)
    parent.start()
    rospy.loginfo("started")

    # launch node that monitors the navigation progress
    rospy.init_node('sim_experiment_runner', anonymous=True)
    rospy.Subscriber("/move_base/result", MoveBaseActionResult, mbCallback)

    # sleep for 'timeout' if shutdown did not occur previously
    rospy.sleep(timeout)
    parent.shutdown()
