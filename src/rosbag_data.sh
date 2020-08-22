#!/bin/bash
path_src=`pwd`
cd ..
cd data
rm 'RAW_DATA_from_rosbag.txt'
path_data=`pwd`
echo $path_data
cd $path_src
#
echo "EKF2"
python ekf2.py &
echo "scan_mode_rosbag"
python scan_mode_rosbag.py 45 &
echo "rosbag play"
#rosbag play $path_data'/agrosavia_citrush.bag' &
rosbag play $path_data'/sebastian_1.bag' &
#
echo "waiting 50 secs"
sleep 50
echo "waiting 10 secs"
sleep 10
echo "T - 4"
sleep 6
rosnode kill -a
