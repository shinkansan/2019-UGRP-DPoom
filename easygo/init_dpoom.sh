xterm -e "roscore" &
xterm -e "echo dpoom | sudo -S chmod 777 /dev/ttyACM0" &
xterm -e "sleep 3 && roslaunch turtlebot3_bringup turtlebot3_core.launch" &

