Bootstrap: docker

From: gazebo:libgazebo7

%post
	apt-get -y update
	apt-get install python3
	apt-get -y install cmake gcc g++

	mkdir /local
	

%environment
	export LC_ALL=C.UTF-8
	export GAZEBO_MASTER_URI=http://localhost:11345
	export GAZEBO_MODEL_DATABASE_URI=http://models.gazebosim.org
	export GAZEBO_RESOURCE_PATH=$PWD/sources:/usr/share/gazebo-7:
    export GAZEBO_PLUGIN_PATH=$PWD/compiled_plugins:/usr/lib/x86_64-linux-gnu/gazebo-7/plugins:
    export GAZEBO_MODEL_PATH=$PWD/sources:/usr/share/gazebo-7/models:
    export LD_LIBRARY_PATH=$PWD/sources:/.singularity.d/libs:/usr/lib/x86_64-linux-gnu/gazebo-7/plugins
	export OGRE_RESOURCE_PATH=/usr/lib/x86_64-linux-gnu/OGRE-1.9.0
%runscript
	"$@"