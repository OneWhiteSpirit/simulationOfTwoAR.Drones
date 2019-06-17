# Start in new Terminal roscore 
gnome-terminal -x bash -c 'roscore'
source ~/catkin_ws/devel/setup.bash
# Start in new Terminal after 6 second script of drones interaction 
gnome-terminal -x bash -c "source ~/catkin_ws/devel/setup.bash && sleep 6 && rosrun cvg_sim_gazebo drones_fly.py; exec bash"
# Sleep 3 seconds
sleep 3
# Start Gazebo simulator with environment
roslaunch cvg_sim_gazebo two_ardrone.launch
