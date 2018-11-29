taskSize=150
lastPort=0
qsub hpc_qsub.sh Uniform RW-turnP 0 $lastPort

lastPort=$(($lastPort + $taskSize))
qsub hpc_qsub.sh HalfCluster RW-turnP 0 $lastPort

lastPort=$(($lastPort + $taskSize))
qsub hpc_qsub.sh OneCluster RW-turnP 0 $lastPort

lastPort=$(($lastPort + $taskSize))
qsub hpc_qsub.sh TwoClusters RW-turnP 0 $lastPort

lastPort=$(($lastPort + $taskSize))
qsub hpc_qsub.sh FourClusters RW-turnP 0 $lastPort
