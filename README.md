# Safety Benchmark with System Modes 

This repository contains an example of how to use system modes to control the security of a system. The system comprises four lifecycle nodes: `nav2, manipulator, camera, and guard`.

There are two modes of operation:
`__DEFAULT__`: The system works normally, and all nodes are in `__DEFAULT__` mode.
`__COMPROMISED__`: The system has been compromised. All nodes are in `__COMPROMISED__` mode, and the `camera` node is down.

Guard is a node in charge of determining if the system is compromised. If this node detects this situation, it becomes in `__COMPROMISED__` mode, triggering the change of the rest of the nodes to `__COMPROMISED__` state.

File describing the system is in [benchmark_modes.yaml](benchmark_modes.yaml)

## Build

Just clone in your workspace and install dependencies with rosdep:

```
$ cd {workspace}
$ git clone https://github.com/DMARCE-PROJECT/system_modes_benchmark.git
$ rosdep install --from-paths src --ignore-src -r -y
$ colcon build --symlink-install
```

## How to run

1. (recommended) In terminal 1 launch the `mode_monitor`

```
ros2 launch system_modes mode_monitor.launch.py modelfile:=install/system_modes_benchmark/share/system_modes_benchmark/benchmark_modes.yaml
```

2. In terminal 2 launch teh system

```
ros2 launch system_modes_benchmark benchmark_system_started.launch.py
```

3. In terminal 3 Change the modes and params to see how the system evolves:
   1. `ros2 service call /guard/change_mode system_modes_msgs/srv/ChangeMode "{mode_name: 'COMPROMISED'}"`
   2. `ros2 service call /guard/change_mode system_modes_msgs/srv/ChangeMode "{mode_name: '__DEFAULT__'}"`
   3. `ros2 param set /guard compromised_system true`
   4. ...
   

## references

* https://micro.ros.org/docs/concepts/client_library/lifecycle_and_system_modes/
* https://github.com/micro-ROS/system_modes
* https://github.com/micro-ROS/system_modes/tree/master/system_modes_examples