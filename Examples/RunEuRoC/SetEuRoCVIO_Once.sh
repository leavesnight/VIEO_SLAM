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

curPath=$(dirname $(readlink -f "$0"))
echo curPath_Once=$curPath

cd $curPath
source ./RunEuRoCVIO.sh $CAMTYPE $EUROCFILE $EUROCFILE2 $EuRoCFolderRel $ConfigFileRel
#cd $curPath
#SUBFILE=VIO/NoLoopMore_0.2
#source ./EvaluateEuRoC_Evaluate.sh
cd $curPath
SUBFILE=$CAMTYPE
source ./EvaluateEuRoC_Copy.sh
source ./EvaluateEuRoC_Evaluate.sh

cd $curPath
if [[ ${CAMTYPE:0:9} == "Monocular" ]]; then
  source ./printResultATE.sh $EUROCFILE $SUBFILE GT $EuRoCFolderRel
else
  source ./printResultATE.sh $EUROCFILE $SUBFILE "" $EuRoCFolderRel
fi

cd $curPath
