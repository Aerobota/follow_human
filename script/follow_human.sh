#!/bin/bash
xterm -geometry 80x5+0+0   -e "/opt/ros/indigo/bin/roslaunch turtlebot_bringup minimal.launch" &
sleep 10s
xterm -geometry 80x5+0+130 -e "/opt/ros/indigo/bin/roslaunch turtlebot_bringup 3dsensor.launch" &
sleep 5s
xterm -geometry 80x5+0+260 -e "/opt/ros/indigo/bin/rqt -s kobuki_dashboard" &
sleep 5s
xterm -geometry 80x5+0+910 -e "~/catkin_ws/devel/lib/follow_human/follow_human" &
sleep 5s

