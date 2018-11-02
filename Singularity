Bootstrap: docker

From: gazebo:libgazebo9

%post
	apt-get -y update
	apt-get install python3
	apt-get -y install cmake

%environment
	export LC_ALL=C.UTF-8
	export GAZEBO_RESOURCE_PATH=$PWD/sources
    export GAZEBO_PLUGIN_PATH=$PWD/compiled_plugins
    export GAZEBO_MODEL_PATH=$PWD/sources
    export LD_LIBRARY_PATH=$PWD/sources