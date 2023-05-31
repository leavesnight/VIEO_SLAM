#!/bin/bash
CAMTYPE="StereoVIO"
EUROCFILE=V203difficult
EUROCFILE2=V203
if [[ $1 != "" ]]; then
  CAMTYPE=$1
fi
if [[ $2 != "" && $3 != "" ]]; then
  EUROCFILE=$2
  EUROCFILE2=$3
fi
EuRoCFolderRel=EuRoC
if [[ $4 != "" ]]; then
  EuRoCFolderRel=$4
fi
EuRoCFolder=./$EuRoCFolderRel
if [[ $CAMTYPE == "StereoVIO" ]]; then
  ConfigFile=$EuRoCFolder/${EuRoCFolderRel}_VIO_dist_fast.yaml
elif [[ $CAMTYPE == "MonocularVIO" ]]; then
  ConfigFile=$EuRoCFolder/${EuRoCFolderRel}_VIO.yaml
else
  ConfigFile=$EuRoCFolder/${EuRoCFolderRel}.yaml
fi
if [[ $5 != "" ]]; then
  ConfigFile=$EuRoCFolder/$5.yaml
fi
echo "ConfigFile="$ConfigFile
EuRoCLike_TimeStamps=$EuRoCFolder/${EuRoCFolderRel}_TimeStamps
echo "EuRoCLike_TimeStamps="$EuRoCLike_TimeStamps

echo "CAMTYPE="$CAMTYPE
echo "EUROCFILE="$EUROCFILE
echo "EUROCFILE2="$EUROCFILE2
curPath=$(dirname $(readlink -f "$0"))
curPath_Examples=$curPath/..
echo curPath_Examples_Run=$curPath_Examples
cd $curPath_Examples/${CAMTYPE%VIO}

if [[ $EuRoCFolderRel == EuRoC || $EuRoCFolderRel == TUM ]]; then
  DS_IMG=$EuRoCFolderRel/$EUROCFILE/mav0/cam0/data
  DS_IMG2=$EuRoCFolderRel/$EUROCFILE/mav0/cam1/data
  DS_IMU=$EuRoCFolderRel/$EUROCFILE/mav0/imu0/data.csv
else
  DS_IMG=""
  DS_IMG2=""
  DS_IMU=""
  ConfigFile=""
  echo "Unsupported Datasets!"
fi

if [[ $CAMTYPE == "StereoVIO" ]]; then
  ./stereo_euroc ../../Vocabulary/ORBvoc.bin "$ConfigFile" ~/dataset/$DS_IMG ~/dataset/$DS_IMG2 $EuRoCLike_TimeStamps/$EUROCFILE2.txt ~/dataset/$DS_IMU
elif [[ $CAMTYPE == "MonocularVIO" ]]; then
  ./mono_euroc ../../Vocabulary/ORBvoc.bin "$ConfigFile" ~/dataset/$DS_IMG $EuRoCLike_TimeStamps/$EUROCFILE2.txt ~/dataset/$DS_IMU
elif [[ $CAMTYPE == "Stereo" ]]; then
  ./stereo_euroc ../../Vocabulary/ORBvoc.bin "$ConfigFile" ~/dataset/$DS_IMG ~/dataset/$DS_IMG2 $EuRoCLike_TimeStamps/$EUROCFILE2.txt
elif [[ $CAMTYPE == "Monocular" ]]; then
  ./mono_euroc ../../Vocabulary/ORBvoc.bin "$ConfigFile" ~/dataset/$DS_IMG $EuRoCLike_TimeStamps/$EUROCFILE2.txt
fi
