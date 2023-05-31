#!/bin/bash
cd ~/zzh/rgbd_benchmark_tools/src/rgbd_benchmark_tools
#OFFSET=0.
#EuRoCFolderRel=TUM
#EUROCFILE=dataset-corridor1_512_16
#SUBFILE=StereoVIO

DstGTFolder=~/dataset/$EuRoCFolderRel/$EUROCFILE
DstFolder=$DstGTFolder/orbslam2/$SUBFILE/
if [[ $EuRoCFolderRel == EuRoC ]]; then
  python2 ./evaluate_ate.py ${DstGTFolder}/groundtruth_estimate0.txt ${DstFolder}/KeyFrameTrajectoryIMU.txt --verbose --offset 0. --plot PLOT_E >result_ate_Est.txt
  python2 ./evaluate_ate.py ${DstGTFolder}/groundtruth_estimate0.txt ${DstFolder}/KeyFrameTrajectoryIMU_NO_FULLBA.txt --verbose --offset 0. --plot PLOT_N_E >result_ate_NO_Est.txt
  cp ./result_ate_Est.txt ./PLOT_E.png ${DstFolder}
  cp ./result_ate_NO_Est.txt ./PLOT_N_E.png ${DstFolder}

  python2 ./evaluate_ate_scale.py ${DstGTFolder}/groundtruth_estimate0.txt ${DstFolder}/KeyFrameTrajectoryIMU.txt --verbose --offset 0. --plot PLOT_GTest >result_ate_GTest.txt
  cp ./result_ate_GTest.txt ./PLOT_GTest.png ${DstFolder}
fi
python2 ./evaluate_ate.py ${DstGTFolder}/groundtruth.txt ${DstFolder}/KeyFrameTrajectoryIMU.txt --verbose --offset $OFFSET --plot PLOT >result_ate.txt
python2 ./evaluate_ate.py ${DstGTFolder}/groundtruth.txt ${DstFolder}/KeyFrameTrajectoryIMU_NO_FULLBA.txt --verbose --offset $OFFSET --plot PLOT_N >result_ate_NO.txt
cp ./result_ate.txt ./PLOT.png ${DstFolder}
cp ./result_ate_NO.txt ./PLOT_N.png ${DstFolder}

python2 ./evaluate_ate_scale.py ${DstGTFolder}/groundtruth.txt ${DstFolder}/KeyFrameTrajectoryIMU.txt --verbose --offset $OFFSET --plot PLOT_GT >result_ate_GT.txt
cp ./result_ate_GT.txt ./PLOT_GT.png ${DstFolder}
