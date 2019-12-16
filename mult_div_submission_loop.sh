#Examples:
# ./mult_div_submission_loop.sh 1 1 900 Uniform100m
# ./mult_div_submission_loop.sh 1 1 900 OneCluster100m

hpc=$1
start=$2
stop=$3
world=$4

if (( hpc )); then
    for (( paramLine = start; paramLine <= stop; paramLine++)) do
        echo $paramLine # $(( $paramLine + $stop))
#        qsub hpc_qsub.sh $hpc $world N0-Q1 $paramLine

#        qsub hpc_qsub.sh $hpc $world N100-Q40  $paramLine
# $(( $paramLine + $stop ))

        qsub hpc_qsub.sh $hpc $world var-s2s-u2s $paramLine
    done
else
    experiment=test
    paramLine=$stop
    sh hpc_qsub.sh $hpc $world $experiment $paramLine
fi
