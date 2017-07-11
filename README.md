# roomba_521

This is a repository which contains the driver packages and some functionalities like controlling the a turtlebot1. In particular this repository is an adapted version and in particular for the vacuum cleaner iRobot Roomba 521(Model version). This driver version is a slightly adapted version of the iRobot  [Create 1 and 2](http://www.irobot.com/About-iRobot/STEM/Create-2.aspx). This package also uses the C++ library [libcreate](https://github.com/AutonomyLab/libcreate) of the Create 1 and Create 2.

# Installation:

1. first install the turtlebot1 as described in [turtlebot1 Doku.](https://github.com/CesMak/turtlebot1)

2. Install the Roomba package:

```
turtle install roomba_521
turtle update_make
```

3. Give rights to the USB:

```
sudo usermod -a -G dialout $USER
```

4. Logout and login for permission to take effect

5. Connect your pc via usb with the open interface of the roomba

6. Start the roomba (click the power button) make sure the power light glows green (if not charge your roomba first)

7. Start the node:

```
roslaunch r_driver r_521.launch
```

8. Your Launch Output should look like the following (if the controller is disabled)
 ![](https://github.com/CesMak/roomba_521/blob/master/doc/launch_output.png    )

# Topics:
 ![](https://github.com/CesMak/roomba_521/blob/master/doc/topics.png)

# PD-Controller:
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/pNLwUF0T-Nc/0.jpg)](https://www.youtube.com/watch?v=pNLwUF0T-Nc)

# Possible Commands:

```
rostopic pub mySong std_msgs/Bool true

rostopic pub /sound std_msgs/UInt8MultiArray '{data:[70,102]}'

rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'

rostopic echo bumper

rostopic pub /dock_led std_msgs/Bool "data: false" // sets dock light to green

rostopic pub /power_led std_msgs/UInt8MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: '[12,100]'"   [works]

[...]

[if controller node is started in r_521.launch - you can drive to a determined position:]
rostopic pub /turtle1/PositionCommand geometry_msgs/Pose2D "x: 1.5 y: 0.0 theta: 10.0"
```

# Problems

## USB Connection should look like:
 ![](https://github.com/CesMak/roomba_521/blob/master/doc/usb_connection_output.png    )

`ls -la /dev/ttyUSB*` should give:
 crw-rw---- 1 root dialout 188, 0 Jul 10 16:52 /dev/ttyUSB0


## Errors:
<pre>
markus@markus:~/turtlebot1/src/turtlebot/roomba_521/r_driver/launch$ roslaunch r_521.launch
... logging to /home/markus/.ros/log/8bab9542-6174-11e7-bb0a-48d2244c225b/roslaunch-markus-26140.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://markus:40391/

SUMMARY
========

PARAMETERS
 * /r_driver/create_1: False
 * /r_driver/dev: /dev/ttyUSB0
 * /r_driver/loop_hz: 30
 * /r_driver/publish_tf: True
 * /rosdistro: kinetic
 * /rosversion: 1.12.7

NODES
  /
    r_driver (r_driver/r_driver)

auto-starting new master
process[master]: started with pid [26151]
ROS_MASTER_URI=http://localhost:11311

setting /run_id to 8bab9542-6174-11e7-bb0a-48d2244c225b
process[rosout-1]: started with pid [26164]
started core service [/rosout]
process[r_driver-2]: started with pid [26175]
[ INFO] [1499253885.046165984]: [CREATE] "CREATE_2" selected
terminate called after throwing an instance of 'boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::system::system_error> >'
  what():  open: Permission denied
[r_driver-2] process has died [pid 26175, exit code -6, cmd /home/markus/turtlebot1/devel/lib/r_driver/r_driver __name:=r_driver __log:=/home/markus/.ros/log/8bab9542-6174-11e7-bb0a-48d2244c225b/r_driver-2.log].
log file: /home/markus/.ros/log/8bab9542-6174-11e7-bb0a-48d2244c225b/r_driver-2*.log
</pre>

Solution:
```
$ sudo usermod -a -G dialout $USER
```
Log Out and log in for permission to take effect.


## Yellow Light is flashing fast when charging:
put battery in and out again.

# Some Messages:
![](https://github.com/CesMak/roomba_521/blob/master/doc/bumper.png)
![](https://github.com/CesMak/roomba_521/blob/master/doc/diagnostics.png)
![](https://github.com/CesMak/roomba_521/blob/master/doc/clean_button.png)

