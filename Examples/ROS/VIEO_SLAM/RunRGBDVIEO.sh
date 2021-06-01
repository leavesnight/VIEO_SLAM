#!/bin/bash
#OURFILE=~/dataset/Lab001easy
#OURFILE=~/dataset/Lab002medium
#OURFILE=~/dataset/Lab003difficult
#OURFILE=~/dataset/Lab004difficult
#OURFILE=~/dataset/Corridor001easy
#OURFILE=~/dataset/Corridor002medium
OURFILE=~/dataset/Corridor003difficult
#OURFILE=~/dataset/Corridor004difficult
#OURFILE=~/dataset/Farm002medium
#OURFILE=~/dataset/test_research_control
#EUROCFILE2=V203
cd ~/zzh/VIEO_SLAM/Examples/ROS/VIEO_SLAM

#MAP_REUSE
./RGBDpub ../../../Vocabulary/ORBvoc.bin ./kinect2_qhd.yaml $OURFILE $OURFILE/associate.txt $OURFILE/EncSensor.txt 2 0 $OURFILE/EncSensor.txt ./Map.bin
#./rgbd_tum ../../Vocabulary/ORBvoc.bin ./kinect2_qhd.yaml $OURFILE $OURFILE/associate.txt $OURFILE/odometrysensor.txt 2 0 $OURFILE/EncSensor.txt ./Map.bin
#./rgbd_tum ../../Vocabulary/ORBvoc.bin ./kinect2_qhd.yaml $OURFILE $OURFILE/associate.txt $OURFILE/IMUSensor.txt 9 0 $OURFILE/EncSensor.txt ./Map.bin

#VEO
#./rgbd_tum ../../Vocabulary/ORBvoc.bin ./kinect2_qhd.yaml $OURFILE $OURFILE/associate.txt $OURFILE/EncSensor.txt
#./rgbd_tum ../../Vocabulary/ORBvoc.bin ./kinect2_qhd.yaml $OURFILE $OURFILE/associate.txt $OURFILE/odometrysensor.txt

CAMTYPE=RGBD

#VIEO,VIO
#./rgbd_tum ../../Vocabulary/ORBvoc.bin ./kinect2_qhd.yaml $OURFILE $OURFILE/associate.txt $OURFILE/IMUSensor.txt 9 0 $OURFILE/EncSensor.txt
#./rgbd_tum ../../Vocabulary/ORBvoc.bin ./kinect2_qhd.yaml $OURFILE $OURFILE/associate.txt $OURFILE/IMUSensor.txt 9

