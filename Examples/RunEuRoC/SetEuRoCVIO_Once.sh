#!/bin/bash
CAMTYPE="StereoVIO"
EUROCFILE=MH05difficult
EUROCFILE2=MH05
OFFSET=0.2
EuRoCFolderRel=EuRoC
if [[ $1 != "" ]]; then
  CAMTYPE=$1
fi
if [[ $2 != "" && $3 != "" ]]; then
  EUROCFILE=$2
  EUROCFILE2=$3
fi
if [[ $4 != "" ]]; then
  OFFSET=$4
fi
if [[ $5 != "" ]]; then
  EuRoCFolderRel=$5
fi
ConfigFileRel=""
if [[ $6 != "" ]]; then
  ConfigFileRel=$6
fi
MapSparseName=""
if [[ $7 != "" ]]; then
  MapSparseName=$7
fi
SUBFILE=$CAMTYPE
if [[ $8 != "" ]]; then
  SUBFILE=$8
fi

curPath=$(dirname $(readlink -f "$0"))
echo curPath_Once=$curPath

cd $curPath
bash ./RunEuRoCVIO.sh $CAMTYPE $EUROCFILE $EUROCFILE2 "$EuRoCFolderRel" "$ConfigFileRel" "$MapSparseName"
cd $curPath
bash ./EvaluateEuRoC_Copy.sh $EuRoCFolderRel $EUROCFILE $CAMTYPE $SUBFILE
cd $curPath
TrajectoryFileName=CameraTrajectoryIMU
bash ./EvaluateEuRoC_Evaluate.sh $EuRoCFolderRel $EUROCFILE $SUBFILE $OFFSET $TrajectoryFileName

cd $curPath
if [[ ${CAMTYPE:0:9} == "Monocular" ]]; then
  bash ./printResultATE.sh $EUROCFILE $SUBFILE GT $EuRoCFolderRel
else
  bash ./printResultATE.sh $EUROCFILE $SUBFILE "" $EuRoCFolderRel
fi

cd $curPath
