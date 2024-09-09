#!/bin/bash
CAMTYPE="StereoVIO"
if [[ $1 != "" ]]; then
  CAMTYPE=$1
fi
BenchmarkName="EuRoC"
if [[ $2 != "" ]]; then
  BenchmarkName=$2
fi
OutputFileNameFiles=""
if [[ $3 != "" ]]; then
  OutputFileNameFiles=$3
fi
SUBFILE=$CAMTYPE
if [[ $4 != "" ]]; then
  SUBFILE=$4
fi

EuRoCFolderRel=$BenchmarkName
if [[ $BenchmarkName == "EuRoC" ]]; then
  ConfigFileRelBase=EuRoC_VIO_dist
elif [[ $BenchmarkName == "TUM_VI" ]]; then
  ConfigFileRelBase=TUM_VI_512_VIO
else
  echo "Unsupported Benchmark!"
  exit 1
fi
#if [[ $CAMTYPE == "StereoVIO" ]]; then
#  ConfigFileRelBase=${ConfigFileRelBase}_dist_fast
#fi
ConfigFileRel=${ConfigFileRelBase}

curPath=$(dirname $(readlink -f "$0"))
echo curPath_Files=$curPath
cd $curPath

SHELLNAME="./SetEuRoCVIO_Once.sh"
OFFSET=0.
if [[ $BenchmarkName == "EuRoC" ]]; then
  EUROCFILE=V101easy
  EUROCFILE2=V101
  bash $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel "" $SUBFILE # &
  EUROCFILE=V102medium
  EUROCFILE2=V102
  bash $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel "" $SUBFILE # &

  OFFSET=-0.2
  EUROCFILE=V201easy
  EUROCFILE2=V201
  bash $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel "" $SUBFILE # &
  EUROCFILE=V202medium
  EUROCFILE2=V202
  bash $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel "" $SUBFILE # &
  wait
  EUROCFILE=V203difficult
  EUROCFILE2=V203
  bash $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel "" $SUBFILE

  OFFSET=0.
  EUROCFILE=V103difficult
  EUROCFILE2=V103
  bash $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel "" $SUBFILE

  OFFSET=0.2
  EUROCFILE=MH04difficult
  EUROCFILE2=MH04
  bash $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel "" $SUBFILE
  EUROCFILE=MH01easy
  EUROCFILE2=MH01
  bash $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel "" $SUBFILE # &
  EUROCFILE=MH02easy
  EUROCFILE2=MH02
  bash $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel "" $SUBFILE # &
  EUROCFILE=MH03medium
  EUROCFILE2=MH03
  bash $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel "" $SUBFILE # &
  wait
  EUROCFILE=MH05difficult
  EUROCFILE2=MH05
  bash $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel "" $SUBFILE

elif [[ $BenchmarkName == "TUM_VI" ]]; then
  EUROCFILE=dataset-corridor1_512_16
  EUROCFILE2=dataset-corridor1_512
  bash $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel "" $SUBFILE
  EUROCFILE=dataset-corridor2_512_16
  EUROCFILE2=dataset-corridor2_512
  bash $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel "" $SUBFILE
  EUROCFILE=dataset-corridor3_512_16
  EUROCFILE2=dataset-corridor3_512
  bash $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel "" $SUBFILE
  EUROCFILE=dataset-corridor4_512_16
  EUROCFILE2=dataset-corridor4_512
  bash $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel "" $SUBFILE
  EUROCFILE=dataset-corridor5_512_16
  EUROCFILE2=dataset-corridor5_512
  bash $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel "" $SUBFILE

  EUROCFILE=dataset-magistrale1_512_16
  EUROCFILE2=dataset-magistrale1_512
  bash $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel "" $SUBFILE
  EUROCFILE=dataset-magistrale2_512_16
  EUROCFILE2=dataset-magistrale2_512
  bash $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel "" $SUBFILE
  EUROCFILE=dataset-magistrale3_512_16
  EUROCFILE2=dataset-magistrale3_512
  bash $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel "" $SUBFILE
  EUROCFILE=dataset-magistrale4_512_16
  EUROCFILE2=dataset-magistrale4_512
  bash $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel "" $SUBFILE
  EUROCFILE=dataset-magistrale5_512_16
  EUROCFILE2=dataset-magistrale5_512
  bash $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel "" $SUBFILE
  EUROCFILE=dataset-magistrale6_512_16
  EUROCFILE2=dataset-magistrale6_512
  bash $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel "" $SUBFILE

  keywords="dist_fast"
  lenkws=${#keywords}
  if [[ ${ConfigFileRelBase:0-lenkws:lenkws} != $keywords ]]; then
    ConfigFileRel=${ConfigFileRelBase}_outdoors
  fi
  EUROCFILE=dataset-outdoors1_512_16
  EUROCFILE2=dataset-outdoors1_512
  bash $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel "" $SUBFILE
  EUROCFILE=dataset-outdoors2_512_16
  EUROCFILE2=dataset-outdoors2_512
  bash $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel "" $SUBFILE
  EUROCFILE=dataset-outdoors3_512_16
  EUROCFILE2=dataset-outdoors3_512
  bash $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel "" $SUBFILE
  EUROCFILE=dataset-outdoors4_512_16
  EUROCFILE2=dataset-outdoors4_512
  bash $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel "" $SUBFILE
  EUROCFILE=dataset-outdoors5_512_16
  EUROCFILE2=dataset-outdoors5_512
  bash $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel "" $SUBFILE
  EUROCFILE=dataset-outdoors6_512_16
  EUROCFILE2=dataset-outdoors6_512
  bash $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel "" $SUBFILE
  EUROCFILE=dataset-outdoors7_512_16
  EUROCFILE2=dataset-outdoors7_512
  bash $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel "" $SUBFILE
  EUROCFILE=dataset-outdoors8_512_16
  EUROCFILE2=dataset-outdoors8_512
  bash $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel "" $SUBFILE

  ConfigFileRel=${ConfigFileRelBase}
  EUROCFILE=dataset-room1_512_16
  EUROCFILE2=dataset-room1_512
  bash $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel "" $SUBFILE
  EUROCFILE=dataset-room2_512_16
  EUROCFILE2=dataset-room2_512
  bash $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel "" $SUBFILE
  EUROCFILE=dataset-room3_512_16
  EUROCFILE2=dataset-room3_512
  bash $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel "" $SUBFILE
  EUROCFILE=dataset-room4_512_16
  EUROCFILE2=dataset-room4_512
  bash $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel "" $SUBFILE
  EUROCFILE=dataset-room5_512_16
  EUROCFILE2=dataset-room5_512
  bash $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel "" $SUBFILE
  EUROCFILE=dataset-room6_512_16
  EUROCFILE2=dataset-room6_512
  bash $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel "" $SUBFILE

  EUROCFILE=dataset-slides1_512_16
  EUROCFILE2=dataset-slides1_512
  bash $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel "" $SUBFILE
  EUROCFILE=dataset-slides2_512_16
  EUROCFILE2=dataset-slides2_512
  bash $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel "" $SUBFILE
  EUROCFILE=dataset-slides3_512_16
  EUROCFILE2=dataset-slides3_512
  bash $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel "" $SUBFILE
fi

#"" for shell param analysis will use IFS="\t\n" and next param will be advanced if use $this else use "$this"
printMore=""
if [[ $BenchmarkName == "EuRoC" && ${CAMTYPE:0:6} == "Stereo" ]]; then
  printMore="Est"
fi
if [[ $CAMTYPE == "Monocular" ]]; then
  printMore="GT"
fi

SHELLNAME="./printResultATE.sh"
if [[ $BenchmarkName == "EuRoC" ]]; then
  EUROCFILE=V101easy
  bash $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel $OutputFileNameFiles
  EUROCFILE=V102medium
  bash $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel $OutputFileNameFiles
  EUROCFILE=V103difficult
  bash $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel $OutputFileNameFiles
  EUROCFILE=V201easy
  bash $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel $OutputFileNameFiles
  EUROCFILE=V202medium
  bash $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel $OutputFileNameFiles
  EUROCFILE=V203difficult
  bash $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel $OutputFileNameFiles
  EUROCFILE=MH01easy
  bash $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel $OutputFileNameFiles
  EUROCFILE=MH02easy
  bash $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel $OutputFileNameFiles
  EUROCFILE=MH03medium
  bash $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel $OutputFileNameFiles
  EUROCFILE=MH04difficult
  bash $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel $OutputFileNameFiles
  EUROCFILE=MH05difficult
  bash $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel $OutputFileNameFiles
elif [[ $BenchmarkName == "TUM_VI" ]]; then
  EUROCFILE=dataset-corridor1_512_16
  bash $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel $OutputFileNameFiles
  EUROCFILE=dataset-corridor2_512_16
  bash $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel $OutputFileNameFiles
  EUROCFILE=dataset-corridor3_512_16
  bash $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel $OutputFileNameFiles
  EUROCFILE=dataset-corridor4_512_16
  bash $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel $OutputFileNameFiles
  EUROCFILE=dataset-corridor5_512_16
  bash $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel $OutputFileNameFiles
  EUROCFILE=dataset-magistrale1_512_16
  bash $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel $OutputFileNameFiles
  EUROCFILE=dataset-magistrale2_512_16
  bash $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel $OutputFileNameFiles
  EUROCFILE=dataset-magistrale3_512_16
  bash $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel $OutputFileNameFiles
  EUROCFILE=dataset-magistrale4_512_16
  bash $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel $OutputFileNameFiles
  EUROCFILE=dataset-magistrale5_512_16
  bash $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel $OutputFileNameFiles
  EUROCFILE=dataset-magistrale6_512_16
  bash $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel $OutputFileNameFiles
  EUROCFILE=dataset-outdoors1_512_16
  bash $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel $OutputFileNameFiles
  EUROCFILE=dataset-outdoors2_512_16
  bash $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel $OutputFileNameFiles
  EUROCFILE=dataset-outdoors3_512_16
  bash $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel $OutputFileNameFiles
  EUROCFILE=dataset-outdoors4_512_16
  bash $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel $OutputFileNameFiles
  EUROCFILE=dataset-outdoors5_512_16
  bash $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel $OutputFileNameFiles
  EUROCFILE=dataset-outdoors6_512_16
  bash $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel $OutputFileNameFiles
  EUROCFILE=dataset-outdoors7_512_16
  bash $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel $OutputFileNameFiles
  EUROCFILE=dataset-outdoors8_512_16
  bash $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel $OutputFileNameFiles
  EUROCFILE=dataset-room1_512_16
  bash $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel $OutputFileNameFiles
  EUROCFILE=dataset-room2_512_16
  bash $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel $OutputFileNameFiles
  EUROCFILE=dataset-room3_512_16
  bash $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel $OutputFileNameFiles
  EUROCFILE=dataset-room4_512_16
  bash $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel $OutputFileNameFiles
  EUROCFILE=dataset-room5_512_16
  bash $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel $OutputFileNameFiles
  EUROCFILE=dataset-room6_512_16
  bash $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel $OutputFileNameFiles
  EUROCFILE=dataset-slides1_512_16
  bash $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel $OutputFileNameFiles
  EUROCFILE=dataset-slides2_512_16
  bash $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel $OutputFileNameFiles
  EUROCFILE=dataset-slides3_512_16
  bash $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel $OutputFileNameFiles
fi
