#! /bin/sh

export GAZEBO_RESOURCE_PATH=$PWD/sources
export GAZEBO_PLUGIN_PATH=$PWD/compiled_plugins
export GAZEBO_MODEL_PATH=$PWD/sources
export LD_LIBRARY_PATH=$PWD/sources

gzserver sources/w_swarm1/world_db/w_swarm1_no_litter.world