#!/bin/bash
EUROCFILE="MH05difficult"
SUBFILE="StereoVIO"
if [[ $1 != ""  ]]; then
    EUROCFILE=$1
fi
if [[ $2 != ""  ]]; then
    SUBFILE=$2
fi
print_gt=0
if [[ $3 == "GT"  ]]; then
    print_gt=1
fi

function printInfo {
    echo "dataset_name=$EUROCFILE"
    echo "result_ate.txt="
    cat ~/dataset/EuRoC/$EUROCFILE/orbslam2/$SUBFILE/result_ate.txt
    if [[ $print_gt != 0 ]]; then
        echo "result_ate_GT.txt="
        cat ~/dataset/EuRoC/$EUROCFILE/orbslam2/$SUBFILE/result_ate_GT.txt	
    fi
}

printInfo

