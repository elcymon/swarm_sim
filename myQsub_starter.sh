# testing simulation on local machine
hpc=$1
paramLine=$2
experiment=$3

if (( hpc )); then
    submit=qsub
else
    submit=sh
fi

#qsub hpc_qsub.sh $hpc OneCluster $experiment $paramLine 1$paramLine

# qsub hpc_qsub.sh $hpc TwoClusters $experiment $paramLine 2$paramLine

#qsub hpc_qsub.sh $hpc FourClusters $experiment $paramLine 3$paramLine

#qsub hpc_qsub.sh $hpc HalfCluster $experiment $paramLine 4$paramLine

$submit hpc_qsub.sh $hpc Uniform $experiment $paramLine 5$paramLine

#qsub hpc_qsub.sh $hpc OneCluster100m $experiment $paramLine 11$paramLine

#qsub hpc_qsub.sh $hpc TwoClusters100m $experiment $paramLine 22$paramLine

#qsub hpc_qsub.sh $hpc FourClusters100m $experiment $paramLine 33$paramLine

#qsub hpc_qsub.sh $hpc HalfCluster100m $experiment $paramLine 44$paramLine

#qsub hpc_qsub.sh $hpc Uniform100m $experiment $paramLine 55$paramLine

