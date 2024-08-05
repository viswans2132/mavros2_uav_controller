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

#### NOTE: Make sure to enable the quaternion in the _<mavros_dir>/launch/px4_config.xml_ before starting the Mavros.

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
- /mavros/setpoint_attitude/attitude does not appear. Please set quaternions to true in _<mavros_dir>/launch/px4_config.xml_ file and restart _mavros_. Otherwise, you may send body velocity commands using _/mavros/setpoint_attitude/cmd_vel_ topic.
- _ERROR [flight_mode_manager] Matching flight task was not able to run, Nav state: 2, Task: 1_ Probably a Connection Loss. Make sure you're using the right topics (quaternion or angular velocities) for controlling the UAV. If there is still a connection loss, you may change **COM_OF_LOSS_T** parameter to 10 seconds. This is a failsafe parameter that enables the UAV to disarm or land when the flight controller does not receive a command for the specified time (default: 1 second).
- _Arming Denied: Manual Control Lost_ This is a failsafe to deny arming while RC is not available. In case of Simulation, please reset the parameter **COM_RCL_EXCEPT** to 4 for overriding the manual input during _OFFBOARD_ mode. **Caution: NEVER DO THIS WHILE USING ACTUAL HARDWARE. DANGEROUS!!!**
