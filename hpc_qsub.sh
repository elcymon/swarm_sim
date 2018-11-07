
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
#$ -l h_rt=1:0:0

#Iterations
#$ -t 1-120

#Iterations in batch of
#$ -tc 8


#e-mail
#$ -m be
#$ -M scsoo@leeds.ac.uk

#Run the job
#You can add cd to program directory to be sure
# environment variable SGE_TASK_ID varies based on range in -t option
#load singularity
module load singularity

folder=/nobackup/scsoo
# folder=/tmp/nobackup/scsoo
# JOB_ID=123
# SGE_TASK_ID=11

#execute simulation
LOCAL_LOC=$folder/local

mkdir -p $LOCAL_LOC/$JOB_ID.$SGE_TASK_ID.24core-128G.q $folder/results

singularity exec --bind $folder/results:$PWD/results,$LOCAL_LOC:/local $folder/gazebo-libgazebo7-xenial.simg python3 hpc_start_simulation2.py Uniform-180FoV $SGE_TASK_ID $SGE_TASK_ID
