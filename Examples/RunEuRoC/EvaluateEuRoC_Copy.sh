#!/bin/bash
#EuRoCFolderRel=TUM
#EUROCFILE=dataset-corridor1_512_16
#CAMTYPE=StereoVIO
#SUBFILE=StereoVIO

CAMTYPEFOLDER=${CAMTYPE%VIO}
DstFolder=~/dataset/$EuRoCFolderRel/$EUROCFILE/orbslam2
mkdir $DstFolder
DstFolder=${DstFolder}/$SUBFILE/
mkdir $DstFolder
cp -r ~/tmp/zzh $DstFolder
curPath=$(dirname $(readlink -f "$0"))
curPath_Examples=$curPath/..
echo curPath_Examples_Copy=$curPath_Examples
cp -r $curPath_Examples/$CAMTYPEFOLDER/CameraTrajectory.txt $curPath_Examples/$CAMTYPEFOLDER/KeyFrameTrajectory.txt $curPath_Examples/$CAMTYPEFOLDER/KeyFrameTrajectoryIMU.txt $curPath_Examples/$CAMTYPEFOLDER/KeyFrameTrajectoryIMU_NO_FULLBA.txt $DstFolder
