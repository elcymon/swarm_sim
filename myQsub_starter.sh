# # REPATT VS RANDOM WALK SOUND ATTRACTION 2018-12-20
# qsub hpc_qsub.sh 50_by_50 RepAtt-vs-RW-attraction2sound 0 0
qsub hpc_qsub.sh rectangle RepAtt-vs-RW-attraction2sound 0 0

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
