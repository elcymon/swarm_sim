#! /bin/sh

export GAZEBO_RESOURCE_PATH=$PWD/sources
export GAZEBO_PLUGIN_PATH=$PWD/compiled_plugins
export GAZEBO_MODEL_PATH=$PWD/sources
export LD_LIBRARY_PATH=$PWD/sources

gzserver sources/w_swarm1/world_db/20180208_w_swarm1_circular_two_region_cluster.world