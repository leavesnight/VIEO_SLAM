#!/bin/bash
ourds_path=~/dataset/master_VIEO
#OURFILE=$ourds_path/Lab001easy
#OURFILE=$ourds_path/Lab002medium
#OURFILE=$ourds_path/VIEO_datasets_highfreq/Lab15_200Hz
#OURFILE=$ourds_path/Lab003difficult
#OURFILE=$ourds_path/VIEO_datasets_lowfreq/Lab7_15Hz
#OURFILE=$ourds_path/Lab004difficult
#OURFILE=$ourds_path/Corridor001easy
#OURFILE=$ourds_path/Corridor002medium
#OURFILE=$ourds_path/Corridor003difficult
OURFILE=$ourds_path/VIEO_datasets_highfreq/Corridor15_200Hz
#OURFILE=$ourds_path/Corridor004difficult
#OURFILE=$ourds_path/VIEO_datasets_lowfreq/Corridor7_15Hz
#OURFILE=$ourds_path/Farm002medium
#OURFILE=$ourds_path/test_research_control

CAMTYPE=RGBD
SUBFILE=""

curPath=$(dirname $(readlink -f "$0"))
curPath_Examples=$curPath/..
echo curPath_Examples_Run=$curPath_Examples
slam_path="$curPath_Examples/RGB-D"
transform_path="${HOME}/zzh/UsefulToolsForVIEORBSLAM2/get_estimatedtraj_cryst/build"
evaluate_path="${HOME}/zzh/rgbd_benchmark_tools/src/rgbd_benchmark_tools"

SLAMTYPE="VEO_MAPREUSE"

if [[ $1 != "" ]]; then
    SLAMTYPE=$1
fi
if [[ $2 != "" ]]; then
    OURFILE=$2
fi
echo "OURFILE=$OURFILE"

OFFSET=0.1
if [[ $3 != "" ]]; then
    OFFSET=$3
fi
echo "OFFSET=$OFFSET"

SUBFILE=$SLAMTYPE"_tmp"
if [[ $4 != "" ]]; then
    slam_path=`dirname $4``basename $4`
    echo slam_path
fi

cd $slam_path

keywords="MAPREUSE"
lenkws=${#keywords}
kwSLAM=( "VEO" "VEOLowFps" "VIEO" "VIO" "VO" )
lenkwSLAM=( ${#kwSLAM[0]} ${#kwSLAM[1]} ${#kwSLAM[2]} ${#kwSLAM[3]} ${#kwSLAM[4]} )
echo "keywords = ${SLAMTYPE:0-lenkws:lenkws}  ${SLAMTYPE:0:4} "
if [[ $keywords == ${SLAMTYPE:0-lenkws:lenkws} ]]; then
    echo $keywords
    if [[ ${kwSLAM[1]} == ${SLAMTYPE:0:${lenkwSLAM[1]}} ]]; then
    echo "VEOLowFps"
    ./rgbd_tum ../../Vocabulary/ORBvoc.bin ./kinect2_qhd.yaml $OURFILE $OURFILE/associate.txt $OURFILE/odometrysensor.txt 2 0 $OURFILE/EncSensor.txt ./Map.bin
    elif [[ ${kwSLAM[0]} == ${SLAMTYPE:0:${lenkwSLAM[0]}} ]]; then
    echo "VEO"
    ./rgbd_tum ../../Vocabulary/ORBvoc.bin ./kinect2_qhd.yaml $OURFILE $OURFILE/associate.txt $OURFILE/EncSensor.txt 2 0 $OURFILE/EncSensor.txt ./Map.bin
    elif [[ ${kwSLAM[2]} == ${SLAMTYPE:0:${lenkwSLAM[2]}} ]]; then
    echo "VIEO"
    ./rgbd_tum ../../Vocabulary/ORBvoc.bin ./kinect2_qhd.yaml $OURFILE $OURFILE/associate.txt $OURFILE/IMUSensor.txt 9 0 $OURFILE/EncSensor.txt ./Map.bin
    fi
else
    if [[ ${kwSLAM[1]} == ${SLAMTYPE:0:${lenkwSLAM[1]}} ]]; then
    echo "VEOLowFps"
    ./rgbd_tum ../../Vocabulary/ORBvoc.bin ./kinect2_qhd.yaml $OURFILE $OURFILE/associate.txt $OURFILE/odometrysensor.txt
    elif [[ ${kwSLAM[0]} == ${SLAMTYPE:0:${lenkwSLAM[0]}} ]]; then
    echo "VEO"
    ./rgbd_tum ../../Vocabulary/ORBvoc.bin ./kinect2_qhd.yaml $OURFILE $OURFILE/associate.txt $OURFILE/EncSensor.txt
    elif [[ ${kwSLAM[2]} == ${SLAMTYPE:0:${lenkwSLAM[2]}} ]]; then
    echo "VIEO"
    ./rgbd_tum ../../Vocabulary/ORBvoc.bin ./kinect2_qhd.yaml $OURFILE $OURFILE/associate.txt $OURFILE/IMUSensor.txt 9 0 $OURFILE/EncSensor.txt
    elif [[ ${kwSLAM[3]} == ${SLAMTYPE:0:${lenkwSLAM[3]}} ]]; then
    echo "VIO"
    ./rgbd_tum ../../Vocabulary/ORBvoc.bin ./kinect2_qhd.yaml $OURFILE $OURFILE/associate.txt $OURFILE/IMUSensor.txt 9#VIO
    elif [[ ${kwSLAM[4]} == ${SLAMTYPE:0:${lenkwSLAM[4]}} ]]; then
    echo "VO"
    ./rgbd_tum ../../Vocabulary/ORBvoc.bin ./kinect2_qhd.yaml $OURFILE $OURFILE/associate.txt
    fi
fi

mkdir $OURFILE/orbslam2/
mkdir $OURFILE/orbslam2/$SUBFILE
cp ./CameraTrajectory.txt ./CameraTrajectory_NO_FULLBA.txt KeyFrameTrajectory.txt KeyFrameTrajectoryIMU.txt $OURFILE/orbslam2/$SUBFILE/
cp ./Map.pcd $OURFILE/orbslam2/$SUBFILE/
cp ./CameraTrajectory.txt $OURFILE/orbslam2/

cd $transform_path

./get_estimate $OURFILE

cd $evaluate_path

python2 ./evaluate_ate.py $OURFILE/groundtruth.txt $OURFILE/orbslam2/CrystalTrajectory.txt --verbose --offset $OFFSET --plot PLOT.png > result_ate.txt

cp $OURFILE/orbslam2/CrystalTrajectory.txt ./result_ate.txt ./PLOT.png $OURFILE/orbslam2/$SUBFILE/

cat $OURFILE/orbslam2/$SUBFILE/result_ate.txt
xdg-open $OURFILE/orbslam2/$SUBFILE/PLOT.png


