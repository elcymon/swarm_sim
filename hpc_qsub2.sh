
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
#$ -l h_vmem=12G

#no of cores
##$ -pe smp 8

# Request Wallclock time of hh:mm:ss
#$ -l h_rt=1:0:0

#Iterations
#$ -t 1-1

#Iterations in batch of
#$ -tc 1


#e-mail
#$ -m be
#$ -M scsoo@leeds.ac.uk

#Run the job
#You can add cd to program directory to be sure
# environment variable SGE_TASK_ID varies based on range in -t option
#load singularity
module load singularity

#execute simulation
singularity exec /nobackup/scsoo/gazebo-libgazebo7-xenial.simg gzserver --verbose $PWD/sources/w_swarm1/world_db/20180209_w_swarm1_circular_uniform_litter.world &
singularity exec -B /nobackup/scsoo/results:$PWD/results /nobackup/scsoo/gazebo-libgazebo7-xenial.simg ./world_governor Uniform-180FoV 1 &

#singularity exec -B /nobackup/scsoo/results:$PWD/results /nobackup/scsoo/gazebo-libgazebo7-xenial.simg python3 hpc_start_simulation2.py Uniform-180FoV $SGE_TASK_ID $SGE_TASK_ID
