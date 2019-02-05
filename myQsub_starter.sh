# testing simulation on local machine
hpc=$1

if((! hpc)); then
    lastPort=0
    skipRows=0
    ./hpc_qsub.sh $hpc OneCluster100m SR-Noise0-100pct-Q1-40-100m $skipRows $lastPort
else
    # # # SR and SA TESTS 100m World 2018-12-11
    # ##SR
    # taskSize=120
    # lastPort=0
    # skipRows=0
    # hpc=true
    # qsub hpc_qsub.sh $hpc Uniform100m SR-Noise0-100pct-Q1-40-100m $skipRows $lastPort

    # lastPort=$(($lastPort + $taskSize))
    # qsub hpc_qsub.sh $hpc HalfCluster100m SR-Noise0-100pct-Q1-40-100m $skipRows $lastPort

    # lastPort=$(($lastPort + $taskSize))
    # qsub hpc_qsub.sh $hpc OneCluster100m SR-Noise0-100pct-Q1-40-100m $skipRows $lastPort

    # lastPort=$(($lastPort + $taskSize))
    # qsub hpc_qsub.sh $hpc TwoClusters100m SR-Noise0-100pct-Q1-40-100m $skipRows $lastPort

    # lastPort=$(($lastPort + $taskSize))
    # qsub hpc_qsub.sh $hpc FourClusters100m SR-Noise0-100pct-Q1-40-100m $skipRows $lastPort

    # ##SA
    # lastPort=$(($lastPort + $taskSize))
    # skipRows=120
    # qsub hpc_qsub.sh $hpc Uniform100m SA-Noise0-100pct-Q1-40-100m $skipRows $lastPort

    # lastPort=$(($lastPort + $taskSize))
    # qsub hpc_qsub.sh $hpc HalfCluster100m SA-Noise0-100pct-Q1-40-100m $skipRows $lastPort

    # lastPort=$(($lastPort + $taskSize))
    # qsub hpc_qsub.sh $hpc OneCluster100m SA-Noise0-100pct-Q1-40-100m $skipRows $lastPort

    # lastPort=$(($lastPort + $taskSize))
    # qsub hpc_qsub.sh $hpc TwoClusters100m SA-Noise0-100pct-Q1-40-100m $skipRows $lastPort

    # lastPort=$(($lastPort + $taskSize))
    # qsub hpc_qsub.sh $hpc FourClusters100m SA-Noise0-100pct-Q1-40-100m $skipRows $lastPort
    # # # RepAtt TESTS 100m World 2018-12-07
    # taskSize=120
    # lastPort=0
    # qsub hpc_qsub.sh $hpc Uniform100m RepAtt-Noise0-100pct-Q1-40-100m 0 $lastPort

    # lastPort=$(($lastPort + $taskSize))
    # qsub hpc_qsub.sh $hpc HalfCluster100m RepAtt-Noise0-100pct-Q1-40-100m 0 $lastPort

    # lastPort=$(($lastPort + $taskSize))
    # qsub hpc_qsub.sh $hpc OneCluster100m RepAtt-Noise0-100pct-Q1-40-100m 0 $lastPort

    # lastPort=$(($lastPort + $taskSize))
    # qsub hpc_qsub.sh $hpc TwoClusters100m RepAtt-Noise0-100pct-Q1-40-100m 0 $lastPort

    # lastPort=$(($lastPort + $taskSize))
    # qsub hpc_qsub.sh $hpc FourClusters100m RepAtt-Noise0-100pct-Q1-40-100m 0 $lastPort

    # # REPATT VS RANDOM WALK 100M WORLD 2018-12-05
    # qsub hpc_qsub.sh $hpc TwoClusters100m RepAtt-vs-RW-100m 0 0

    # # RepAtt TESTS 2019-02-05
    taskSize=60
    lastPort=0
    qsub hpc_qsub.sh $hpc Uniform RepAtt-Noise0-100pct-Q1-40-xtlog 0 $lastPort

    # lastPort=$(($lastPort + $taskSize))
    # qsub hpc_qsub.sh $hpc HalfCluster RepAtt-Noise0-100pct-Q1-40-xtlog 0 $lastPort

    lastPort=$(($lastPort + $taskSize))
    qsub hpc_qsub.sh $hpc OneCluster RepAtt-Noise0-100pct-Q1-40-xtlog 0 $lastPort

    # lastPort=$(($lastPort + $taskSize))
    # qsub hpc_qsub.sh $hpc TwoClusters RepAtt-Noise0-100pct-Q1-40-xtlog 0 $lastPort

    # lastPort=$(($lastPort + $taskSize))
    # qsub hpc_qsub.sh $hpc FourClusters RepAtt-Noise0-100pct-Q1-40-xtlog 0 $lastPort


    # RW TESTS 2018-11-30
    # taskSize=150
    # lastPort=0
    # qsub hpc_qsub.sh $hpc Uniform RW-turnP-xtlog 0 $lastPort

    # lastPort=$(($lastPort + $taskSize))
    # qsub hpc_qsub.sh $hpc HalfCluster RW-turnP-xtlog 0 $lastPort

    # lastPort=$(($lastPort + $taskSize))
    # qsub hpc_qsub.sh $hpc OneCluster RW-turnP-xtlog 0 $lastPort

    # lastPort=$(($lastPort + $taskSize))
    # qsub hpc_qsub.sh $hpc TwoClusters RW-turnP-xtlog 0 $lastPort

    # lastPort=$(($lastPort + $taskSize))
    # qsub hpc_qsub.sh $hpc FourClusters RW-turnP-xtlog 0 $lastPort
fi