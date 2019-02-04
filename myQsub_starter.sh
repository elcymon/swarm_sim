# testing simulation on local machine
# if set hpc to 1 if using hpc, 0 if not
hpc=$1

if ((! hpc)); then
    skipRows=0
    resultFolder=SA-N0-M1-1000-D1-1000
    lastPort=0
    worldName=Uniform
    ./hpc_qsub.sh hpc $worldName $resultFolder $skipRows $lastPort

else

# investigating effect of using home signal to form boundary for swarm
    taskSize=150
    skipRows=0
    resultFolder=SA-N0-M1-1000-D1-1000

    lastPort=0
    worldName=Uniform
    qsub hpc_qsub.sh hpc $worldName $resultFolder $skipRows $lastPort

    lastPort=$(($lastPort + $taskSize))
    worldName=Uniform100m
    qsub hpc_qsub.sh hpc $worldName $resultFolder $skipRows $lastPort

    lastPort=$(($lastPort + $taskSize))
    worldName=OneCluster
    qsub hpc_qsub.sh hpc $worldName $resultFolder $skipRows $lastPort

    lastPort=$(($lastPort + $taskSize))
    worldName=OneCluster100m
    qsub hpc_qsub.sh hpc $worldName $resultFolder $skipRows $lastPort

#unbounded test only for homing signal as boundary
    taskSize=90
    skipRows=60
    lastPort=$(($lastPort + $taskSize))
    worldName=OneClusterUnbounded
    qsub hpc_qsub.sh hpc $worldName $resultFolder $skipRows $lastPort

    lastPort=$(($lastPort + $taskSize))
    worldName=UniformUnbounded
    qsub hpc_qsub.sh hpc $worldName $resultFolder $skipRows $lastPort

fi

# # # SR and SA TESTS 100m World 2018-12-11
# ##SR
# taskSize=120
# lastPort=0
# skipRows=0
# hpc=true
# qsub hpc_qsub.sh hpc Uniform100m SR-Noise0-100pct-Q1-40-100m $skipRows $lastPort

# lastPort=$(($lastPort + $taskSize))
# qsub hpc_qsub.sh hpc HalfCluster100m SR-Noise0-100pct-Q1-40-100m $skipRows $lastPort

# lastPort=$(($lastPort + $taskSize))
# qsub hpc_qsub.sh hpc OneCluster100m SR-Noise0-100pct-Q1-40-100m $skipRows $lastPort

# lastPort=$(($lastPort + $taskSize))
# qsub hpc_qsub.sh hpc TwoClusters100m SR-Noise0-100pct-Q1-40-100m $skipRows $lastPort

# lastPort=$(($lastPort + $taskSize))
# qsub hpc_qsub.sh hpc FourClusters100m SR-Noise0-100pct-Q1-40-100m $skipRows $lastPort

# ##SA
# lastPort=$(($lastPort + $taskSize))
# skipRows=120
# qsub hpc_qsub.sh hpc Uniform100m SA-Noise0-100pct-Q1-40-100m $skipRows $lastPort

# lastPort=$(($lastPort + $taskSize))
# qsub hpc_qsub.sh hpc HalfCluster100m SA-Noise0-100pct-Q1-40-100m $skipRows $lastPort

# lastPort=$(($lastPort + $taskSize))
# qsub hpc_qsub.sh hpc OneCluster100m SA-Noise0-100pct-Q1-40-100m $skipRows $lastPort

# lastPort=$(($lastPort + $taskSize))
# qsub hpc_qsub.sh hpc TwoClusters100m SA-Noise0-100pct-Q1-40-100m $skipRows $lastPort

# lastPort=$(($lastPort + $taskSize))
# qsub hpc_qsub.sh hpc FourClusters100m SA-Noise0-100pct-Q1-40-100m $skipRows $lastPort
# # # RepAtt TESTS 100m World 2018-12-07
# taskSize=120
# lastPort=0
# qsub hpc_qsub.sh hpc Uniform100m RepAtt-Noise0-100pct-Q1-40-100m 0 $lastPort

# lastPort=$(($lastPort + $taskSize))
# qsub hpc_qsub.sh hpc HalfCluster100m RepAtt-Noise0-100pct-Q1-40-100m 0 $lastPort

# lastPort=$(($lastPort + $taskSize))
# qsub hpc_qsub.sh hpc OneCluster100m RepAtt-Noise0-100pct-Q1-40-100m 0 $lastPort

# lastPort=$(($lastPort + $taskSize))
# qsub hpc_qsub.sh hpc TwoClusters100m RepAtt-Noise0-100pct-Q1-40-100m 0 $lastPort

# lastPort=$(($lastPort + $taskSize))
# qsub hpc_qsub.sh hpc FourClusters100m RepAtt-Noise0-100pct-Q1-40-100m 0 $lastPort

# # REPATT VS RANDOM WALK 100M WORLD 2018-12-05
# qsub hpc_qsub.sh hpc TwoClusters100m RepAtt-vs-RW-100m 0 0

# # RepAtt TESTS 2018-11-30
# taskSize=600
# lastPort=0
# qsub hpc_qsub.sh hpc Uniform RepAtt-Noise0-100pct-Q1-40-xtlog 0 $lastPort

# lastPort=$(($lastPort + $taskSize))
# qsub hpc_qsub.sh hpc HalfCluster RepAtt-Noise0-100pct-Q1-40-xtlog 0 $lastPort

# lastPort=$(($lastPort + $taskSize))
# qsub hpc_qsub.sh hpc OneCluster RepAtt-Noise0-100pct-Q1-40-xtlog 0 $lastPort

# lastPort=$(($lastPort + $taskSize))
# qsub hpc_qsub.sh hpc TwoClusters RepAtt-Noise0-100pct-Q1-40-xtlog 0 $lastPort

# lastPort=$(($lastPort + $taskSize))
# qsub hpc_qsub.sh hpc FourClusters RepAtt-Noise0-100pct-Q1-40-xtlog 0 $lastPort


# RW TESTS 2018-11-30
# taskSize=150
# lastPort=0
# qsub hpc_qsub.sh hpc Uniform RW-turnP-xtlog 0 $lastPort

# lastPort=$(($lastPort + $taskSize))
# qsub hpc_qsub.sh hpc HalfCluster RW-turnP-xtlog 0 $lastPort

# lastPort=$(($lastPort + $taskSize))
# qsub hpc_qsub.sh hpc OneCluster RW-turnP-xtlog 0 $lastPort

# lastPort=$(($lastPort + $taskSize))
# qsub hpc_qsub.sh hpc TwoClusters RW-turnP-xtlog 0 $lastPort

# lastPort=$(($lastPort + $taskSize))
# qsub hpc_qsub.sh hpc FourClusters RW-turnP-xtlog 0 $lastPort
