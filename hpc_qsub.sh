#USAGE
#./hpc_qsub world_name experiment row_shift prev_ID_end
#WHERE
#world_name is Uniform, OneCluster, TwoClusters, FourClusters, or HalfCluster

# Use current working directory and current modules
#$ -cwd -V

#$ -e /nobackup/scsoo/logs/errors
#$ -o /nobackup/scsoo/logs/outputs

# Request a full node (24 cores and 128 GB or 768 GB on ARC3)
# -l nodes=1
# -l node_type=24core-128G

# To request for x cores on a single machine, with around y memory per core
# -pe smp x -l h_vmem=yG

#memory?
#$ -l h_vmem=2G

#no of cores
#$ -pe smp 8

# Request Wallclock time of hh:mm:ss
#$ -l h_rt=2:0:0

#Iterations
#$ -t 1-480

#Iterations in batch of
##$ -tc 8


#e-mail
#$ -m abe
#$ -M scsoo@leeds.ac.uk

#Run the job
#You can add cd to program directory to be sure
# environment variable SGE_TASK_ID varies based on range in -t option
#load singularity
module load singularity
folder=/nobackup/scsoo

# folder=.
# JOB_ID=123
# SGE_TASK_ID=1

#execute simulation
local_loc=$folder/local

#set python script input arguments
#name of the world to simulate on
world_name=$1
#experiment is used to know which parameter you are investigating
experiment=$2
#how many rows of parameters should be ignored us 0 if none
row_shift=$3
#the previous SGE_TASK_ID maximum value
prev_ID_end=$4
line_number=$(($SGE_TASK_ID + $row_shift))
#there should be no repetition of server port or else they will overwrite each other. Adding 1 just to be safe
port_number=$(($SGE_TASK_ID + $prev_ID_end + 1))
echo world_name: $world_name, experiment: $experiment, row_shift: $row_shift, prev_ID_end: $prev_ID_end, line_number: $line_number, port_number: $port_number
mkdir -p $local_loc/$JOB_ID.$SGE_TASK_ID.24core-128G.q $folder/results

singularity exec --bind $folder/results:$PWD/results,$folder/results:$folder/swarm_sim*/results,$local_loc:/local $folder/gazebo-libgazebo7-xenial.simg python3 hpc_start_simulation2.py $world_name $experiment $line_number $port_number
