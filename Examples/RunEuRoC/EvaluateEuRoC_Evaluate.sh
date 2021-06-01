#!/bin/bash
cd ~/zzh/rgbd_benchmark_tools/src/rgbd_benchmark_tools
#EUROCFILE=MH02easy
#SUBFILE=VIO/NoLoopMore_0.2
#OFFSET=0.2

python2 ./evaluate_ate.py ~/dataset/EuRoC/$EUROCFILE/groundtruth_estimate0.txt ~/dataset/EuRoC/$EUROCFILE/orbslam2/$SUBFILE/KeyFrameTrajectoryIMU.txt --verbose --offset 0. --plot PLOT_E >result_ate_Est.txt 
python2 ./evaluate_ate.py ~/dataset/EuRoC/$EUROCFILE/groundtruth_estimate0.txt ~/dataset/EuRoC/$EUROCFILE/orbslam2/$SUBFILE/KeyFrameTrajectoryIMU_NO_FULLBA.txt --verbose --offset 0. --plot PLOT_N_E >result_ate_NO_Est.txt 
cp ./result_ate_Est.txt ./PLOT_E.png ~/dataset/EuRoC/$EUROCFILE/orbslam2/$SUBFILE/
cp ./result_ate_NO_Est.txt ./PLOT_N_E.png ~/dataset/EuRoC/$EUROCFILE/orbslam2/$SUBFILE/
python2 ./evaluate_ate.py ~/dataset/EuRoC/$EUROCFILE/groundtruth.txt ~/dataset/EuRoC/$EUROCFILE/orbslam2/$SUBFILE/KeyFrameTrajectoryIMU.txt --verbose --offset $OFFSET --plot PLOT >result_ate.txt 
python2 ./evaluate_ate.py ~/dataset/EuRoC/$EUROCFILE/groundtruth.txt ~/dataset/EuRoC/$EUROCFILE/orbslam2/$SUBFILE/KeyFrameTrajectoryIMU_NO_FULLBA.txt --verbose --offset $OFFSET --plot PLOT_N >result_ate_NO.txt 
cp ./result_ate.txt ./PLOT.png ~/dataset/EuRoC/$EUROCFILE/orbslam2/$SUBFILE/
cp ./result_ate_NO.txt ./PLOT_N.png ~/dataset/EuRoC/$EUROCFILE/orbslam2/$SUBFILE/

python2 ./evaluate_ate_scale.py ~/dataset/EuRoC/$EUROCFILE/groundtruth_estimate0.txt ~/dataset/EuRoC/$EUROCFILE/orbslam2/$SUBFILE/KeyFrameTrajectoryIMU.txt --verbose --offset 0. --plot PLOT_GTest >result_ate_GTest.txt 
python2 ./evaluate_ate_scale.py ~/dataset/EuRoC/$EUROCFILE/groundtruth.txt ~/dataset/EuRoC/$EUROCFILE/orbslam2/$SUBFILE/KeyFrameTrajectoryIMU.txt --verbose --offset $OFFSET --plot PLOT_GT >result_ate_GT.txt 
cp ./result_ate_GTest.txt ./PLOT_GTest.png ~/dataset/EuRoC/$EUROCFILE/orbslam2/$SUBFILE/
cp ./result_ate_GT.txt ./PLOT_GT.png ~/dataset/EuRoC/$EUROCFILE/orbslam2/$SUBFILE/

