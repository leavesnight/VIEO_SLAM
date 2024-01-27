#!/bin/bash
#OFFSET=0.
#EuRoCFolderRel=TUM_VI
#EUROCFILE=dataset-corridor1_512_16
#SUBFILE=StereoVIO
EuRoCFolderRel=EuRoC
EUROCFILE=MH05difficult
SUBFILE=StereoVIO
OFFSET=0.2
if [[ $1 != "" ]]; then
  EuRoCFolderRel=$1
fi
if [[ $2 != "" ]]; then
  EUROCFILE=$2
fi
if [[ $3 != "" ]]; then
  SUBFILE=$3
fi
if [[ $4 != "" ]]; then
  OFFSET=$4
fi
TrajectoryFileName=KeyFrameTrajectoryIMU
if [[ $5 != "" ]]; then
  TrajectoryFileName=$5
fi

DstFolder=/media/sf_0Downloads/dataset/
#DstFolder=~/dataset/

DstGTFolder=${DstFolder}/$EuRoCFolderRel/$EUROCFILE
DstFolder=$DstGTFolder/vieo_slam/$SUBFILE/

curPath=$(dirname $(readlink -f "$0"))
echo curPath_Evaluate_Collect=$curPath
CollectFolder=${curPath}/$EuRoCFolderRel/$EUROCFILE/vieo_slam/$SUBFILE/
#cp -r $CollectFolder/* ${DstFolder}

Evaluator=~/zzh/rgbd_benchmark_tools/src/rgbd_benchmark_tools/evaluate_ate.py
EvaluatorS=~/zzh/rgbd_benchmark_tools/src/rgbd_benchmark_tools/evaluate_ate_scale.py
# to avoid parallal running empty txt problem, we cd to dst folder! so no cp file from evaluator folder to dst folder later
cd ${DstFolder}/
if [[ $EuRoCFolderRel == EuRoC ]]; then
  python2 $Evaluator ${DstGTFolder}/groundtruth_estimate0.txt ${DstFolder}/${TrajectoryFileName}.txt --verbose --offset 0. --plot PLOT_E >result_ate_Est.txt
  python2 $Evaluator ${DstGTFolder}/groundtruth_estimate0.txt ${DstFolder}/${TrajectoryFileName}_NO_FULLBA.txt --verbose --offset 0. --plot PLOT_N_E >result_ate_NO_Est.txt
  #cp ./result_ate_Est.txt ./PLOT_E.png ${DstFolder}
  #cp ./result_ate_NO_Est.txt ./PLOT_N_E.png ${DstFolder}

  python2 $EvaluatorS ${DstGTFolder}/groundtruth_estimate0.txt ${DstFolder}/${TrajectoryFileName}.txt --verbose --offset 0. --plot PLOT_GTest >result_ate_GTest.txt
  #cp ./result_ate_GTest.txt ./PLOT_GTest.png ${DstFolder}
fi
python2 $Evaluator ${DstGTFolder}/groundtruth.txt ${DstFolder}/${TrajectoryFileName}.txt --verbose --offset $OFFSET --plot PLOT >result_ate.txt
python2 $Evaluator ${DstGTFolder}/groundtruth.txt ${DstFolder}/${TrajectoryFileName}_NO_FULLBA.txt --verbose --offset $OFFSET --plot PLOT_N >result_ate_NO.txt
#cp ./result_ate.txt ./PLOT.png ${DstFolder}
#cp ./result_ate_NO.txt ./PLOT_N.png ${DstFolder}

python2 $EvaluatorS ${DstGTFolder}/groundtruth.txt ${DstFolder}/${TrajectoryFileName}.txt --verbose --offset $OFFSET --plot PLOT_GT >result_ate_GT.txt
#cp ./result_ate_GT.txt ./PLOT_GT.png ${DstFolder}

mkdir -p "$CollectFolder"
cp -r ${DstFolder}/* $CollectFolder
