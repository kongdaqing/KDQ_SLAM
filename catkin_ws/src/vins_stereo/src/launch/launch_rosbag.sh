{
ps_out=`ps -ef | grep  roscore | grep -v 'grep'`
if [ ! -n "$ps_out" ];
then
roscore
else
echo "roscore already run!"
fi
}&
sleep 2s
{
  gnome-terminal --tab -e  'bash -c "cd ~/Workspace/KDQ_SLAM/catkin_ws;source devel/setup.bash;rosrun vins_stereo vins_node ./src/vins_stereo/src/config/car.yaml;exec bash"' --tab -e  'bash -c "cd ~/Workspace/SlamDatas/VINS;rosbag play car.bag"' --tab -e  "rosrun rviz rviz -d $1" 
}

