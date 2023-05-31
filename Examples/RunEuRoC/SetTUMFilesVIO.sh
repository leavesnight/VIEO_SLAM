#!/bin/bash
CAMTYPE="StereoVIO"
if [[ $1 != "" ]]; then
  CAMTYPE=$1
fi
SHELLNAME="./SetEuRoCVIO_Once.sh"

curPath=$(dirname $(readlink -f "$0"))
echo curPath_Files=$curPath
cd $curPath

EuRoCFolderRel=TUM
ConfigFileRelBase=TUM_512_VIO
ConfigFileRel=${ConfigFileRelBase}

OFFSET=0.

EUROCFILE=dataset-corridor1_512_16
EUROCFILE2=dataset-corridor1_512
source $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel
EUROCFILE=dataset-corridor2_512_16
EUROCFILE2=dataset-corridor2_512
source $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel
EUROCFILE=dataset-corridor3_512_16
EUROCFILE2=dataset-corridor3_512
source $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel
EUROCFILE=dataset-corridor4_512_16
EUROCFILE2=dataset-corridor4_512
source $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel
EUROCFILE=dataset-corridor5_512_16
EUROCFILE2=dataset-corridor5_512
source $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel

EUROCFILE=dataset-magistrale1_512_16
EUROCFILE2=dataset-magistrale1_512
source $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel
EUROCFILE=dataset-magistrale2_512_16
EUROCFILE2=dataset-magistrale2_512
source $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel
EUROCFILE=dataset-magistrale3_512_16
EUROCFILE2=dataset-magistrale3_512
source $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel
EUROCFILE=dataset-magistrale4_512_16
EUROCFILE2=dataset-magistrale4_512
source $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel
EUROCFILE=dataset-magistrale5_512_16
EUROCFILE2=dataset-magistrale5_512
source $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel
EUROCFILE=dataset-magistrale6_512_16
EUROCFILE2=dataset-magistrale6_512
source $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel

keywords="dist_fast"
lenkws=${#keywords}
if [[ ${ConfigFileRelBase:0-lenkws:lenkws} != $keywords ]]; then
  ConfigFileRel=${ConfigFileRelBase}_outdoors
fi

EUROCFILE=dataset-outdoors1_512_16
EUROCFILE2=dataset-outdoors1_512
source $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel
EUROCFILE=dataset-outdoors2_512_16
EUROCFILE2=dataset-outdoors2_512
source $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel
EUROCFILE=dataset-outdoors3_512_16
EUROCFILE2=dataset-outdoors3_512
source $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel
EUROCFILE=dataset-outdoors4_512_16
EUROCFILE2=dataset-outdoors4_512
source $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel
EUROCFILE=dataset-outdoors5_512_16
EUROCFILE2=dataset-outdoors5_512
source $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel
EUROCFILE=dataset-outdoors6_512_16
EUROCFILE2=dataset-outdoors6_512
source $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel
EUROCFILE=dataset-outdoors7_512_16
EUROCFILE2=dataset-outdoors7_512
source $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel
EUROCFILE=dataset-outdoors8_512_16
EUROCFILE2=dataset-outdoors8_512
source $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel

ConfigFileRel=${ConfigFileRelBase}

EUROCFILE=dataset-room1_512_16
EUROCFILE2=dataset-room1_512
source $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel &
EUROCFILE=dataset-room2_512_16
EUROCFILE2=dataset-room2_512
source $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel &
wait
EUROCFILE=dataset-room3_512_16
EUROCFILE2=dataset-room3_512
source $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel &
EUROCFILE=dataset-room4_512_16
EUROCFILE2=dataset-room4_512
source $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel &
wait
EUROCFILE=dataset-room5_512_16
EUROCFILE2=dataset-room5_512
source $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel &
EUROCFILE=dataset-room6_512_16
EUROCFILE2=dataset-room6_512
source $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel &
wait

EUROCFILE=dataset-slides1_512_16
EUROCFILE2=dataset-slides1_512
source $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel
EUROCFILE=dataset-slides2_512_16
EUROCFILE2=dataset-slides2_512
source $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel
EUROCFILE=dataset-slides3_512_16
EUROCFILE2=dataset-slides3_512
source $SHELLNAME $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET $EuRoCFolderRel $ConfigFileRel

SUBFILE=$CAMTYPE
SHELLNAME="./printResultATE.sh"

#"" for shell param analysis will use IFS="\t\n" and next param will be advanced if use $this else use "$this"
printMore=""
if [[ $CAMTYPE == "Monocular" ]]; then
  printMore="GT"
fi

EUROCFILE=dataset-corridor1_512_16
source $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel
EUROCFILE=dataset-corridor2_512_16
source $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel
EUROCFILE=dataset-corridor3_512_16
source $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel
EUROCFILE=dataset-corridor4_512_16
source $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel
EUROCFILE=dataset-corridor5_512_16
source $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel
EUROCFILE=dataset-magistrale1_512_16
source $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel
EUROCFILE=dataset-magistrale2_512_16
source $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel
EUROCFILE=dataset-magistrale3_512_16
source $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel
EUROCFILE=dataset-magistrale4_512_16
source $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel
EUROCFILE=dataset-magistrale5_512_16
source $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel
EUROCFILE=dataset-magistrale6_512_16
source $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel
EUROCFILE=dataset-outdoors1_512_16
source $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel
EUROCFILE=dataset-outdoors2_512_16
source $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel
EUROCFILE=dataset-outdoors3_512_16
source $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel
EUROCFILE=dataset-outdoors4_512_16
source $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel
EUROCFILE=dataset-outdoors5_512_16
source $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel
EUROCFILE=dataset-outdoors6_512_16
source $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel
EUROCFILE=dataset-outdoors7_512_16
source $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel
EUROCFILE=dataset-outdoors8_512_16
source $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel
EUROCFILE=dataset-room1_512_16
source $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel
EUROCFILE=dataset-room2_512_16
source $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel
EUROCFILE=dataset-room3_512_16
source $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel
EUROCFILE=dataset-room4_512_16
source $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel
EUROCFILE=dataset-room5_512_16
source $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel
EUROCFILE=dataset-room6_512_16
source $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel
EUROCFILE=dataset-slides1_512_16
source $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel
EUROCFILE=dataset-slides2_512_16
source $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel
EUROCFILE=dataset-slides3_512_16
source $SHELLNAME $EUROCFILE $SUBFILE "$printMore" $EuRoCFolderRel

exit
