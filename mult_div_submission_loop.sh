#Examples:
# ./mult_div_submission_loop.sh 1 1 900 Uniform100m N0-Q100 params/params_search_n100_q40.csv
# ./mult_div_submission_loop.sh 1 1 900 OneCluster100m var-s2s-u2s params/params_var_s2s_u2s.csv

hpc=$1
start=$2
stop=$3
world=$4
experiment=$5
param_file=$6

if (( hpc )); then
    for (( paramLine = start; paramLine <= stop; paramLine++)) do
        echo $paramLine # $(( $paramLine + $stop))
        qsub hpc_qsub.sh $hpc $world $experiment $param_file $paramLine

#        qsub hpc_qsub.sh $hpc $world N100-Q40  $param_file $paramLine
# $(( $paramLine + $stop ))

#        qsub hpc_qsub.sh $hpc $world var-s2s-u2s $param_file $paramLine
    done
else
    # experiment=test
    paramLine=$stop
    sh hpc_qsub.sh $hpc $world $experiment $param_file $paramLine
fi
