[turtlebot1 Doku](https://github.com/CesMak/turtlebot1)

# roomba_521

This is a repository which contains the driver packages and some functionalities like controlling the a turtlebot1. In particular this repository is an adapted version and in particular for the vacuum cleaner iRobot Roomba 521(Model version). This driver version is a slightly adapted version of the iRobot  [Create 1 and 2](http://www.irobot.com/About-iRobot/STEM/Create-2.aspx). This package also uses the C++ library [libcreate](https://github.com/AutonomyLab/libcreate.aspx) of the Create 1 and Create 2.


## Topics:
 ![](https://github.com/CesMak/roomba_521/blob/master/doc/topics.png)
 

## Launch Output:
 ![](https://github.com/CesMak/roomba_521/blob/master/doc/launch_output.png    )


## USB Connection:
 ![](https://github.com/CesMak/roomba_521/blob/master/doc/usb_connection_output.png    )


## Problems

# Does not connect:
[under Ubuntu 16.04.2 - ROS KINETIC]
(this problem occured in low battery mode....)

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


# Yellow Light is flashing fast when charging:
put battery in and out again.
