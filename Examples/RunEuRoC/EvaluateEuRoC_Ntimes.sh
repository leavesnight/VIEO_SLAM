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
cat $OutputFileName | grep -A16 "dataset_name=" | sed "/--/d" | sed -E "N;N;N;N;N;N;N;N;N;N;N;N;N;N;N;N;s/dataset_name=(.*)\nresult.*rmse (.*) m\n.*mean.*max (.*) m\nresult.*rmse (.*) m\n.*mean.*max (.*) m/\1 \2 \4 \3 \5/" | awk -F' ' '{printf "%s %f %f %f %f\n",$1,$2,$3,$4,$5}' >$OutputFileNameResult
# print README.txt used evaluation results
cat $OutputFileNameResult | awk -F' ' '{printf "%s %f %f %f %f ",$1,$2,$3,$4,$5}' |
  awk -v N_times=$N_times -F' ' '
  {
   M=5;
   N=NF/M/N_times;
   printf "Total %u scenes in this dataset\n", N;
   #process each dataset total N=11 in EuRoC
   for (i=2;i<=N*M;i+=M) {
    num=0;num2=0;num3=0;num4=0;
    sum=0;sum2=0;sum3=0;sum4=0;
    {
     for (j=i;j<=NF;j+=N*M) {
      sum+=$j;sum2+=$(j+1);sum3+=$(j+2);sum4+=$(j+3);
      a[++num]=$j;a2[++num2]=$(j+1);a3[++num3]=$(j+2);a4[++num4]=$(j+3);
     }
    }
    #{for (itmp=1;itmp<=num;++itmp) {printf "%u:%f ",itmp,a[itmp]}}{printf "\n"}
    asort(a);
    #after asort, itmp start from 1! can not use (itmp in a) for it may be non-key-order
    #{for (itmp=1;itmp<=num;++itmp) {printf "%u:%f ",itmp,a[itmp]}}{print "\n"}
    asort(a2);
    asort(a3);
    asort(a4);
    {
     #after asort, itmp start from 1
     num_div2=int(num/2);num2_div2=int(num2/2);num3_div2=int(num3/2);num4_div2=int(num4/2);
     if (num%2==0) {
      median=(a[num_div2]+a[num_div2+1])/2;
     } else {
      median=a[num_div2+1]
     }
     if (num2%2==0) {
      median2=(a2[num2_div2]+a2[num2_div2+1])/2;
     } else {
      median2=a2[num2_div2+1]
     }
     if (num3%2==0) {
      median3=(a3[num3_div2]+a3[num3_div2+1])/2;
     } else {
      median3=a3[num3_div2+1]
     }
     if (num4%2==0) {
      median4=(a4[num4_div2]+a4[num4_div2+1])/2;
     } else {
      median4=a4[num4_div2+1]
     }
     printf "%-4s %-.6f/%-.6f|%-.6f/%-.6f max:%-.6f/%-.6f|%-.6f/%-.6f --%-f %u\n",substr($(i-1),1,4),sum/num,median,sum2/num2,median2,sum3/num3,median3,sum4/num4,median4,$i,num
    }
   }
  }'

exit
