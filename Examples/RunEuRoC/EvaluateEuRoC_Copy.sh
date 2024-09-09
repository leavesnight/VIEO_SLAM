#!/bin/bash
#EuRoCFolderRel=TUM_VI
#EUROCFILE=dataset-corridor1_512_16
#CAMTYPE=StereoVIO
#SUBFILE=StereoVIO
EuRoCFolderRel=EuRoC
EUROCFILE=MH05difficult
CAMTYPE=StereoVIO
SUBFILE=StereoVIO
if [[ $1 != "" ]]; then
  EuRoCFolderRel=$1
fi
if [[ $2 != "" ]]; then
  EUROCFILE=$2
fi
if [[ $3 != "" ]]; then
  CAMTYPE=$3
fi
if [[ $4 != "" ]]; then
  SUBFILE=$4
fi

CAMTYPEFOLDER=${CAMTYPE%VIO}
DstFolder=~/dataset/$EuRoCFolderRel/$EUROCFILE/vieo_slam
mkdir $DstFolder
DstFolder=${DstFolder}/$SUBFILE/
mkdir $DstFolder
cp -r ~/tmp/zzh $DstFolder
curPath=$(dirname $(readlink -f "$0"))
curPath_Examples=$curPath/..
echo curPath_Examples_Copy=$curPath_Examples
cp -r $curPath_Examples/$CAMTYPEFOLDER/CameraTrajectory.txt $curPath_Examples/$CAMTYPEFOLDER/CameraTrajectory_NO_FULLBA.txt \
  $curPath_Examples/$CAMTYPEFOLDER/KeyFrameTrajectory.txt $curPath_Examples/$CAMTYPEFOLDER/KeyFrameTrajectory_NO_FULLBA.txt \
  $curPath_Examples/$CAMTYPEFOLDER/CameraTrajectoryIMU.txt $curPath_Examples/$CAMTYPEFOLDER/CameraTrajectoryIMU_NO_FULLBA.txt \
  $curPath_Examples/$CAMTYPEFOLDER/KeyFrameTrajectoryIMU.txt $curPath_Examples/$CAMTYPEFOLDER/KeyFrameTrajectoryIMU_NO_FULLBA.txt $DstFolder
