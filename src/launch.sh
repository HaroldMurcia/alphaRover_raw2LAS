#!/bin/bash
path_LASTools="/home/hmurcia/Downloads/LAStools"
path_src=`pwd`
cd ..
cd data
rm reco_move_output.las
rm reco_move_output.txt
path_data=`pwd`
echo $path_data
cd $path_src
#python3 reco_move.py '/mnt/Harold/data/agrosavia marzo 3/alphaRover/31-12-69-07-03-26_guayaba.txt' 1000
#python3 reco_move.py $path_data'/31-12-69-07-03-30_guayaba.txt' 1000
#python3 reco_move.py '$path_src/31-12-69-07-09-15_citricos.txt' 10000
#python3 reco_move.py $path_data'/31-12-69-07-09-43_citricos.txt' 1000
python3 reco_move.py $path_data'/RAW_DATA_from_rosbag.txt' 'GPU'
#python3 reco_move.py $path_data'/rampa_down.txt' 'GPU'
echo "LASTools"
cd $path_LASTools'/bin'
wine txt2las -i $path_data'/reco_move_output.txt' -parse cinruaxyz -o $path_data'/reco_move_output.las'
#wine lasview -i $path_data'/reco_move_output.las'
#wine lasground_new -i $path_data'/reco_move_output.las' -o $path_data'/classified_terrain.las'
#wine lasview -i $path_data'/classified_terrain.las'
