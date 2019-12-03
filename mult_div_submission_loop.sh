hpc=$1
start=$2
stop=$3
experiment=$4

if (( hpc )); then
    for (( paramLine = start; paramLine <= stop; paramLine++)) do
        echo $paramLine
        #qsub hpc_qsub.sh $hpc Uniform $experiment $paramLine 5$paramLine
    done
else
    experiment=test
    paramLine=$stop
    sh hpc_qsub.sh $hpc Uniform $experiment $paramLine 5$paramLine
fi