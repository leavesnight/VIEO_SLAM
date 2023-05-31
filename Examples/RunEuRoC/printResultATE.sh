#!/bin/bash
EuRoCFolderRel=EuRoC
EUROCFILE="MH05difficult"
SUBFILE="StereoVIO"
if [[ $1 != "" ]]; then
  EUROCFILE=$1
fi
if [[ $2 != "" ]]; then
  SUBFILE=$2
fi
print_gt=0
if [[ $3 == "GT" ]]; then
  print_gt=1
fi
print_est=0
if [[ $3 == "Est" ]]; then
  print_est=1
fi
if [[ $4 != "" ]]; then
  EuRoCFolderRel=$4
fi

DstFolder=~/dataset/$EuRoCFolderRel/$EUROCFILE/orbslam2/$SUBFILE/
function printInfo() {
  echo "dataset_name=$EUROCFILE"
  echo "result_ate.txt="
  cat ${DstFolder}/result_ate.txt
  if [[ $print_gt != 0 ]]; then
    echo "result_ate_GT.txt="
    cat ${DstFolder}/result_ate_GT.txt
  fi
  if [[ $print_est != 0 ]]; then
    echo "result_ate_Est.txt="
    cat ${DstFolder}/result_ate_Est.txt
  fi
}

printInfo
