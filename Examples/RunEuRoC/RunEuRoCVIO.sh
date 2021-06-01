#!/bin/bash
EUROCFILE=V203difficult
EUROCFILE2=V203
CAMTYPE="StereoVIO"
if [[ $1 != "" ]]; then
    CAMTYPE=$1
fi
if [[ $2 != "" && $3 != "" ]]; then
    EUROCFILE=$2
    EUROCFILE2=$3
fi
echo "CAMTYPE="$CAMTYPE
echo "EUROCFILE="$EUROCFILE
echo "EUROCFILE2="$EUROCFILE2
cd ~/zzh/VIEO_SLAM/Examples/${CAMTYPE%VIO}
if [[ $CAMTYPE == "StereoVIO" ]]; then
    ./stereo_euroc ../../Vocabulary/ORBvoc.bin ./EuRoC_VIO.yaml ~/dataset/EuRoC/$EUROCFILE/mav0/cam0/data ~/dataset/EuRoC/$EUROCFILE/mav0/cam1/data ./EuRoC_TimeStamps/$EUROCFILE2.txt ~/dataset/EuRoC/$EUROCFILE/mav0/imu0/data.csv
elif [[ $CAMTYPE == "MonocularVIO" ]]; then
    ./mono_euroc ../../Vocabulary/ORBvoc.bin ./EuRoC_VIO.yaml ~/dataset/EuRoC/$EUROCFILE/mav0/cam0/data ./EuRoC_TimeStamps/$EUROCFILE2.txt ~/dataset/EuRoC/$EUROCFILE/mav0/imu0/data.csv
elif [[ $CAMTYPE == "Stereo" ]]; then
    ./stereo_euroc ../../Vocabulary/ORBvoc.bin ./EuRoC_VIO.yaml ~/dataset/EuRoC/$EUROCFILE/mav0/cam0/data ~/dataset/EuRoC/$EUROCFILE/mav0/cam1/data ./EuRoC_TimeStamps/$EUROCFILE2.txt
elif [[ $CAMTYPE == "Monocular" ]]; then
    ./mono_euroc ../../Vocabulary/ORBvoc.bin ./EuRoC_VIO.yaml ~/dataset/EuRoC/$EUROCFILE/mav0/cam0/data ./EuRoC_TimeStamps/$EUROCFILE2.txt
fi

