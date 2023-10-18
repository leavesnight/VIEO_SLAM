#!/bin/bash
CAMTYPE="StereoVIO"
if [[ $1 != "" ]]; then
  CAMTYPE=$1
fi
N_times=50
if [[ $2 != "" ]]; then
  N_times=$2
fi
EuRoCFolderRel="EuRoC"
if [[ $3 != "" ]]; then
  EuRoCFolderRel=$3
fi

OutputFileName="evaluate_ntimes_"$EuRoCFolderRel".txt"
echo "Evaluate Ntimes Start" >$OutputFileName
#OutputFileName=""

if [[ $EuRoCFolderRel == "EuRoC" ]]; then
  ShellNameFiles="./SetEuRoCFilesVIO.sh"
elif [[ $EuRoCFolderRel == "TUM" ]]; then
  ShellNameFiles="./SetTUMFilesVIO.sh"
fi
for ((i = 0; i < $N_times; ++i)); do
  echo "Evaluate Ntimes i="$i >>$OutputFileName
  source $ShellNameFiles $CAMTYPE $OutputFileName #
done

#SUBFILE=$CAMTYPE
#SHELLNAME="./printResultATE.sh"
#printMore=""
#if [[ ${CAMTYPE:0:6} == "Stereo" ]]; then
#  printMore="Est"
#fi
#if [[ $CAMTYPE == "Monocular" ]]; then
#  printMore="GT"
#fi
#EUROCFILE=V101easy
#source $SHELLNAME $EUROCFILE $SUBFILE $printMore $EuRoCFolderRel $OutputFileName
#EUROCFILE=V102medium
#source $SHELLNAME $EUROCFILE $SUBFILE $printMore $EuRoCFolderRel $OutputFileName
#EUROCFILE=V103difficult
#source $SHELLNAME $EUROCFILE $SUBFILE $printMore $EuRoCFolderRel $OutputFileName
#EUROCFILE=V201easy
#source $SHELLNAME $EUROCFILE $SUBFILE $printMore $EuRoCFolderRel $OutputFileName
#EUROCFILE=V202medium
#source $SHELLNAME $EUROCFILE $SUBFILE $printMore $EuRoCFolderRel $OutputFileName
#EUROCFILE=V203difficult
#source $SHELLNAME $EUROCFILE $SUBFILE $printMore $EuRoCFolderRel $OutputFileName
#EUROCFILE=MH01easy
#source $SHELLNAME $EUROCFILE $SUBFILE $printMore $EuRoCFolderRel $OutputFileName
#EUROCFILE=MH02easy
#source $SHELLNAME $EUROCFILE $SUBFILE $printMore $EuRoCFolderRel $OutputFileName
#EUROCFILE=MH03medium
#source $SHELLNAME $EUROCFILE $SUBFILE $printMore $EuRoCFolderRel $OutputFileName
#EUROCFILE=MH04difficult
#source $SHELLNAME $EUROCFILE $SUBFILE $printMore $EuRoCFolderRel $OutputFileName
#EUROCFILE=MH05difficult
#source $SHELLNAME $EUROCFILE $SUBFILE $printMore $EuRoCFolderRel $OutputFileName

# get only necessary info files
cat evaluate_ntimes_EuRoC.txt | grep -A11 "dataset_name=" | sed -E "N;N;N;N;N;N;N;N;N;N;N;N;s/(.*)\n--/\1/" | sed -E "N;N;N;N;N;N;N;N;N;N;N;s/dataset_name=(.*)\nresult(.*)rmse (.*) m\nabsolute_translational_error.mean(.*)rmse (.*) m/\1 \3 \5/" | awk -F' ' '{printf "%s %f %f\n",$1,$2,$3}' >evaluate_ntimes_EuRoC_result.txt
# print README.txt used evaluation results
cat evaluate_ntimes_EuRoC_result.txt | awk -F' ' '{printf "%s %f %f ",$1,$2,$3}' |
  awk -v N_times=$N_times -F' ' '
  {
   N=NF/3/N_times;
   printf "Total %u scenes in this dataset\n", N;
   #process each dataset total N=11 in EuRoC
   for (i=2;i<=N*3;i+=3) {
    num=0;num2=0;
    sum=0;sum2=0;
    {
     for (j=i;j<=NF;j+=N*3) {
      sum+=$j;
      sum2+=$(j+1);
      a[++num]=$j;
      a2[++num2]=$(j+1);
     }
    }
    #{for (itmp=1;itmp<=num;++itmp) {printf "%u:%f ",itmp,a[itmp]}}{printf "\n"}
    asort(a);
    #after asort, itmp start from 1! can not use (itmp in a) for it may be non-key-order
    #{for (itmp=1;itmp<=num;++itmp) {printf "%u:%f ",itmp,a[itmp]}}{print "\n"}
    asort(a2);
    {
     #after asort, itmp start from 1
     num_div2=int(num/2);
     if (num%2==0) {
      median=(a[num_div2]+a[num_div2+1])/2;
     } else {
      median=a[num_div2+1]
     }
     num2_div2=int(num2/2);
     if (num2%2==0) {
      median2=(a2[num2_div2]+a2[num2_div2+1])/2;
     } else {
      median2=a2[num2_div2+1]
     }
     printf "%-4s %-.6f/%-.6f|%-.6f/%-.6f --%-f %u\n",substr($(i-1),1,4),sum/num,median,sum2/num2,median2,$i,num
    }
   }
  }'

exit
