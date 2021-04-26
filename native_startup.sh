hpc=$1
param_line=$2
world_name=$3
experiment=$4
param_file=$5
port_shift=$6 #default is 0, but can take a value to prevent gzserver of two separate experiments from overlapping
swarmsize=$7

folder=.
JOB_ID=123
SGE_TASK_ID=1
gzmode=gazebo
QUEUE=mypc

port_number=$(( $SGE_TASK_ID + ( $port_shift + $param_line ) * 32 ))
echo world_name: $world_name, experiment: $experiment, param_line: $param_line, port_number: $port_number
mkdir -p $folder/results

python3 hpc_start_simulation2.py $world_name $experiment $param_file $param_line $SGE_TASK_ID $JOB_ID $port_number $gzmode $swarmsize
