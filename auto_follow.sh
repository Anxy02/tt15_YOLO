#! /bin/bash
sudo chmod 777 /dev/ttyUSB0
gnome-terminal -- bash -c "cd /home/tt15/tt15_ws;source devel/setup.bash;roslaunch tt15_bringup tt15_bringup.launch; exec bash"
sleep 10
wait
gnome-terminal -- bash -c "cd /home/tt15/tt15_ws;source devel/setup.bash;roslaunch auto_follow auto_follow.launch"
sleep 10
wait
exit 0
