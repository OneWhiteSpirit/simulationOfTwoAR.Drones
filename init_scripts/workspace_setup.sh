echo "Setting up workspace."

source /opt/ros/indigo/setup.bash
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
rosdep update
mkdir -p ~/catkin_ws/src

echo "Init catkin and make build"

cd ~/catkin_ws/src && catkin_init_workspace
cd ~/catkin_ws && catkin_make

echo "Download Tum simulator source code"

cd ~/catkin_ws/src && git clone https://github.com/OneWhiteSpirit/simulationOfTwoAR.Drones
cd ~/catkin_ws && catkin_make
cd ~/catkin_ws && catkin_make install
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo "export ROS_WORKSPACE='$HOME/catkin_ws'" >> ~/.bashrc

source ~/.bashrc