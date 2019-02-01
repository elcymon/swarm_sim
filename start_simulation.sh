#! /bin/sh

world_name=rectangle
experiment=RepAtt-vs-RW-attraction2sound
line_number=1
port_number=0

export GAZEBO_RESOURCE_PATH=$PWD/sources
export GAZEBO_PLUGIN_PATH=$PWD/compiled_plugins
export GAZEBO_MODEL_PATH=$PWD/sources
export LD_LIBRARY_PATH=$PWD/sources

# gzserver -u sources/w_swarm1/world_db/20180208_w_swarm1_circular_two_region_cluster.world

python3 hpc_start_simulation2.py $world_name $experiment $line_number $port_number