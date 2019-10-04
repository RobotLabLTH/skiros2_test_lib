#  SkiROS2 test package

Test package for [skiros2](https://github.com/RVMI/skiros2).

## Run

Single components testing:

```rosrun skiros2_test_lib skiros_doc_test```

To try some examples, checkout one of the following:

### Follow pose example

```roslaunch skiros2_test_lib follow_pose_example.launch```

Run "follow_pose" skill from skiros GUI

### Producer/Consumer example

```roslaunch skiros2_test_lib simple_params_example.launch```

Run "trajectory_coordinator" skill from skiros GUI

### Task planner example

```roslaunch skiros2_test_lib planning_example.launch```

Run "task_plan" skill from skiros GUI with Goal in PDDL format: (skiros:contain skiros:LargeBox-80 skiros:Starter-145)

### Turtlesim example

```roslaunch skiros2_test_lib turtlesim_example.launch```

Run "turtle_spawn_and_follow" skill from skiros GUI
