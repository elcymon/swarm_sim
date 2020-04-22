#USAGE
#./hpc_qsub world_name experiment row_shift prev_ID_end
#WHERE
#world_name is Uniform, OneCluster, TwoClusters, FourClusters, or HalfCluster

# Use current working directory and current modules
#$ -cwd -V

#$ -e /nobackup/scsoo/logs/errors
#$ -o /nobackup/scsoo/logs/outputs

# Request a full node (24 cores and 128 GB or 768 GB on ARC3)
#$ -l nodes=0.1
# -l node_type=24core-128G

# To request for x cores on a single machine, with around y memory per core
# -pe smp x -l h_vmem=yG

#memory?
#$ -l h_vmem=1G

#no of cores
#$ -pe smp 3

# Request Wallclock time of hh:mm:ss
#$ -l h_rt=01:0:0

#Iterations
#$ -t 1-30

#Iterations in batch of
#$ -tc 30


#e-mail
#$ -m a
#$ -M scsoo@leeds.ac.uk

#Run the job
#You can add cd to program directory to be sure
# environment variable SGE_TASK_ID varies based on range in -t option
#load singularity
echo $@
hpc=$1 # true if working on hpc false otherwise
if (( hpc )); then
    module load singularity
    folder=$PWD
    gzmode=gzserver
else
    folder=.
    JOB_ID=123
    SGE_TASK_ID=1
    gzmode=gazebo
    QUEUE=mypc
fi

#execute simulation
local_loc=$folder/local

#set python script input arguments
#name of the world to simulate on
world_name=$2
#experiment is used to know which parameter you are investigating
experiment=$3
param_file=$4
#how many rows of parameters should be ignored use 0 if none
paramLine=$5
port_shift=$6 #to prevent overlap with another gzserver of a different experiment submission
swarmsize=$7

#there should be no repetition of server port or else they will overwrite each other. Adding 1 just to be safe
port_number=$(( $SGE_TASK_ID + ( $port_shift + $paramLine ) * 32 ))
echo world_name: $world_name, experiment: $experiment, paramLine: $paramLine, port_number: $port_number
mkdir -p $local_loc/$JOB_ID.$SGE_TASK_ID.$QUEUE $folder/results

singularity exec --bind $folder:$PWD,$local_loc:/local $folder/20190708-libgazebo7-xenial.simg python3 hpc_start_simulation2.py $world_name $experiment $param_file $paramLine $SGE_TASK_ID $JOB_ID $port_number $gzmode $swarmsize
