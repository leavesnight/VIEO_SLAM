#!/bin/bash
EUROCFILE="MH05difficult"
SUBFILE="StereoVIO"
EuRoCFolderRel=EuRoC
OutputFileNamePrint=""
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
if [[ $5 != "" ]]; then
  OutputFileNamePrint=$5
fi

#DstFolder=/media/sf_0Downloads/dataset/
DstFolder=~/dataset/
#DstFolder=./

DstFolder=${DstFolder}/$EuRoCFolderRel/$EUROCFILE/vieo_slam/$SUBFILE/
function printInfo() {
  echo "dataset_name=$EUROCFILE"
  echo "result_ate.txt="
  cat ${DstFolder}/result_ate.txt
#  cat ${DstFolder}/result_ate_NO.txt
  if [[ $print_gt != 0 ]]; then
    echo "result_ate_GT.txt="
    cat ${DstFolder}/result_ate_GT.txt
  fi
  if [[ $print_est != 0 ]]; then
    echo "result_ate_Est.txt="
    cat ${DstFolder}/result_ate_Est.txt
#    cat ${DstFolder}/result_ate_NO_Est.txt
  fi
}

if [[ $OutputFileNamePrint == "" ]]; then
  printInfo
else
  printInfo >> $OutputFileNamePrint
fi