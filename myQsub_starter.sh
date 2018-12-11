# # SR and SA TESTS 100m World 2018-12-11
##SR
taskSize=120
lastPort=0
skipRows=0
qsub hpc_qsub.sh Uniform100m SR-Noise0-100pct-Q1-40-100m $skipRows $lastPort

lastPort=$(($lastPort + $taskSize))
qsub hpc_qsub.sh HalfCluster100m SR-Noise0-100pct-Q1-40-100m $skipRows $lastPort

lastPort=$(($lastPort + $taskSize))
qsub hpc_qsub.sh OneCluster100m SR-Noise0-100pct-Q1-40-100m $skipRows $lastPort

lastPort=$(($lastPort + $taskSize))
qsub hpc_qsub.sh TwoClusters100m SR-Noise0-100pct-Q1-40-100m $skipRows $lastPort

lastPort=$(($lastPort + $taskSize))
qsub hpc_qsub.sh FourClusters100m SR-Noise0-100pct-Q1-40-100m $skipRows $lastPort

##SA
lastPort=$(($lastPort + $taskSize))
skipRows=120
qsub hpc_qsub.sh Uniform100m SA-Noise0-100pct-Q1-40-100m $skipRows $lastPort

lastPort=$(($lastPort + $taskSize))
qsub hpc_qsub.sh HalfCluster100m SA-Noise0-100pct-Q1-40-100m $skipRows $lastPort

lastPort=$(($lastPort + $taskSize))
qsub hpc_qsub.sh OneCluster100m SA-Noise0-100pct-Q1-40-100m $skipRows $lastPort

lastPort=$(($lastPort + $taskSize))
qsub hpc_qsub.sh TwoClusters100m SA-Noise0-100pct-Q1-40-100m $skipRows $lastPort

lastPort=$(($lastPort + $taskSize))
qsub hpc_qsub.sh FourClusters100m SA-Noise0-100pct-Q1-40-100m $skipRows $lastPort
# # # RepAtt TESTS 100m World 2018-12-07
# taskSize=120
# lastPort=0
# qsub hpc_qsub.sh Uniform100m RepAtt-Noise0-100pct-Q1-40-100m 0 $lastPort

# lastPort=$(($lastPort + $taskSize))
# qsub hpc_qsub.sh HalfCluster100m RepAtt-Noise0-100pct-Q1-40-100m 0 $lastPort

# lastPort=$(($lastPort + $taskSize))
# qsub hpc_qsub.sh OneCluster100m RepAtt-Noise0-100pct-Q1-40-100m 0 $lastPort

# lastPort=$(($lastPort + $taskSize))
# qsub hpc_qsub.sh TwoClusters100m RepAtt-Noise0-100pct-Q1-40-100m 0 $lastPort

# lastPort=$(($lastPort + $taskSize))
# qsub hpc_qsub.sh FourClusters100m RepAtt-Noise0-100pct-Q1-40-100m 0 $lastPort

# # REPATT VS RANDOM WALK 100M WORLD 2018-12-05
# qsub hpc_qsub.sh TwoClusters100m RepAtt-vs-RW-100m 0 0

# # RepAtt TESTS 2018-11-30
# taskSize=600
# lastPort=0
# qsub hpc_qsub.sh Uniform RepAtt-Noise0-100pct-Q1-40-xtlog 0 $lastPort

# lastPort=$(($lastPort + $taskSize))
# qsub hpc_qsub.sh HalfCluster RepAtt-Noise0-100pct-Q1-40-xtlog 0 $lastPort

# lastPort=$(($lastPort + $taskSize))
# qsub hpc_qsub.sh OneCluster RepAtt-Noise0-100pct-Q1-40-xtlog 0 $lastPort

# lastPort=$(($lastPort + $taskSize))
# qsub hpc_qsub.sh TwoClusters RepAtt-Noise0-100pct-Q1-40-xtlog 0 $lastPort

# lastPort=$(($lastPort + $taskSize))
# qsub hpc_qsub.sh FourClusters RepAtt-Noise0-100pct-Q1-40-xtlog 0 $lastPort


# RW TESTS 2018-11-30
# taskSize=150
# lastPort=0
# qsub hpc_qsub.sh Uniform RW-turnP-xtlog 0 $lastPort

# lastPort=$(($lastPort + $taskSize))
# qsub hpc_qsub.sh HalfCluster RW-turnP-xtlog 0 $lastPort

# lastPort=$(($lastPort + $taskSize))
# qsub hpc_qsub.sh OneCluster RW-turnP-xtlog 0 $lastPort

# lastPort=$(($lastPort + $taskSize))
# qsub hpc_qsub.sh TwoClusters RW-turnP-xtlog 0 $lastPort

# lastPort=$(($lastPort + $taskSize))
# qsub hpc_qsub.sh FourClusters RW-turnP-xtlog 0 $lastPort
