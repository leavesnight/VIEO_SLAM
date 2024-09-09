#!/bin/bash
CAMTYPE="StereoVIO"
if [[ $1 != "" ]]; then
  CAMTYPE=$1
fi
N_times=10
if [[ $2 != "" ]]; then
  N_times=$2
fi
BenchmarkName="EuRoC"
if [[ $3 != "" ]]; then
  BenchmarkName=$3
fi
EuRoCFolderRel=$BenchmarkName
OutputFileName=${EuRoCFolderRel}/"evaluate_ntimes_"$BenchmarkName".txt"
OutputFileNameResult=${EuRoCFolderRel}/"evaluate_ntimes_"$BenchmarkName"_result.txt"
mkdir $EuRoCFolderRel

ShellNameFiles="./SetEuRoCFilesVIO.sh"
echo "Evaluate Ntimes Start" >$OutputFileName
for ((i = 0; i < $N_times; ++i)); do
  echo "Evaluate Ntimes i="$i >>$OutputFileName
  bash $ShellNameFiles $CAMTYPE $BenchmarkName $OutputFileName $CAMTYPE$i
  if [[ $? == 1 ]]; then
    exit 1
  fi
done

# get only necessary info files; you may need to install gawk: "sudo apt-get install gawk"
cat $OutputFileName | grep -A11 "dataset_name=" | sed -E "N;N;N;N;N;N;N;N;N;N;N;N;s/(.*)\n--/\1/" | sed -E "N;N;N;N;N;N;N;N;N;N;N;s/dataset_name=(.*)\nresult(.*)rmse (.*) m\nabsolute_translational_error.mean(.*)rmse (.*) m/\1 \3 \5/" | awk -F' ' '{printf "%s %f %f\n",$1,$2,$3}' >$OutputFileNameResult
# print README.txt used evaluation results
cat $OutputFileNameResult | awk -F' ' '{printf "%s %f %f ",$1,$2,$3}' |
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
