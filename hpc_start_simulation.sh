export GAZEBO_MASTER_URI=http://localhost:$1

python3 hpc_start_simulation2.py $2 $3 & 

# #create singularity instance
# singularity instance.start ../gazebo-libgazebo7-xenial.simg sim$1

# #execute the container instance
# singularity run  instance://sim$1 hpc_start_simulation2.py $2 $3 &
