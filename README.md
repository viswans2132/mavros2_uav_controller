# PX4 Offboard control using MAVROS and ROS2. 
> Tested with Humble and PX$ version: 1.15.0.80

## Testing with PX4_sitl.
````
# Run PX4 SITL environment with X500 UAV in Gazebo Ignition.
make px4_sitl gz_x500
````

````
# Run mavros with localhost to connect to the simulated UAV.
ros2 launch mavros px4.launch fcu_url:="udp://:14540@192.168.1.36:14557"
# or 
ros2 launch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
````
For some reason, the first IP works faster on my system.

### NOTE: Make sure to enable the quaternion in the _<mavros_dir>/launch/px4_config.xml_.

````
# Run the controller.
python3 scripts/mavros_pid_saa.py
````

## Provide set points using the topic _/new_pose_
````
ros2 topic pub -r1 /new_pose --qos-reliability best_effort --qos-durability volatile --qos-history keep_last geometry_msgs/msg/PoseStamped "{pose: {position: {x: 2, y: 0, z: 3}}}"
````


### Known Issues:
- Mavros is slow to load the parameters. It is a known issue with mavros because it creates separate nodes for each of the plugins. You may try restarting the environment or your computer. There is no fix currently.
