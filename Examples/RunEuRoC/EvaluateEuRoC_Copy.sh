#!/bin/bash
#EUROCFILE=MH02easy
#CAMTYPE=StereoVIO
#SUBFILE=StereoVIO
CAMTYPEFOLDER=${CAMTYPE%VIO}
mkdir ~/dataset/EuRoC/$EUROCFILE/orbslam2/
mkdir ~/dataset/EuRoC/$EUROCFILE/orbslam2/$SUBFILE/
cp -r ~/tmp/zzh ~/dataset/EuRoC/$EUROCFILE/orbslam2/$SUBFILE/
curPath=$(dirname $(readlink -f "$0"))
curPath_Examples=$curPath/..
echo curPath_Examples_Copy=$curPath_Examples
cp -r $curPath_Examples/$CAMTYPEFOLDER/CameraTrajectory.txt $curPath_Examples/$CAMTYPEFOLDER/KeyFrameTrajectory.txt $curPath_Examples/$CAMTYPEFOLDER/KeyFrameTrajectoryIMU.txt $curPath_Examples/$CAMTYPEFOLDER/KeyFrameTrajectoryIMU_NO_FULLBA.txt ~/dataset/EuRoC/$EUROCFILE/orbslam2/$SUBFILE/

