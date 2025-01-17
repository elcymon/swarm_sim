## INTRODUCTION
This repository contains all files necessary for running the RepAtt and Random Walk swarm foraging algorithms. A detailed description of the algorithm and performance assessment have been published in the Conference for Automation Science and Engineering (CASE), 2020. The paper is available at: http://eprints.whiterose.ac.uk/163130/8/CASE_accepted_Version.pdf

## SETUP SIMULATION IN SINGULARITY CONTAINER
### DEPENDENCIES
* An Ubuntu16.04 (or more recent version) is recommended for setting up and running the simulation.
* *Singularity* was used to containerize the environment for building and visualising the simulation. Instruction for installing Singularity and its dependencies for Linux (including links to settingi it up on Mac and Windows) are found at this link: https://sylabs.io/guides/3.7/user-guide/quick_start.html
* *Git* version control system is needed to navigate different branches of the source code. More instructions on setting up Git for different platforms are at this link: https://git-scm.com/

### SETUP FILES
Once Git and Singularity are set up correctly, the next step is to download the source files from the Github repository and checkout/switch to the desired branch.

1. From the terminal, clone the swarm_sim repository using

    `git clone https://github.com/elcymon/swarm_sim.git`

1. Change directory to the just downloaded swarm_sim repository 

    `cd swarm_sim`
1. The repository contains a Singularity recipe file (similar to Dockerfile for docker), which will be used to create the simulating the swarm algorithm using Gazebo simulator and build environment. Build the Singularity container using: 

    `sudo singularity build 20190708-libgazebo7-xenial.simg Singularity` 

    This creates a `20190708-libgazebo7-xenial.simg` file which contains the Gazebo simulator, python3 and cmake build setup for compiling Gazebo simulation libraries.
1. The simulation start up command is: 
    `./mult_div_submission_loop.sh platform start_row end_row target_distribution experiment_name parameter_file gzserver_port_offset swarm_size`
    where:
    * `./mult_div_sumbission_loop.sh` is the shell script for starting up 1 or more Gazebo simulations.
    * `platform` can be either the local environment (`platform=0`) or HPC platform (`platform=1`). For HPC, the simulation startup requests will be sent to the Sun Grid qsheduler software.
    * `start_row` is an integer value that indicates the first row in the `parameter_file` to be used in setting up the simulation. Each row represents a simulation algorithm configuration to be used by the swarm robots.
    * `end_row` is an integer that indicates the last row in the `parameter_file` to be used in setting up the simulation. The `./mult_div_submission_loop.sh` uses each row of paramters starting from `start_row` to `end_row` for launching multiple simulations in parallel. Each simulation is independent of others.
    * `target_distribution` is the distribution of targets in the simulation world. The valid options are: Uniform, OneCluster, TwoClusters, FourClusters, HalfCluster, Uniform100m, OneCluster100m, TwoClusters100m, FourClusters100m, HalfCluster100m
    * `experiment_name` a string identifier for differentiating between different categories of experiments
    * `parameter_file` is the relative path to a csv file containing the set of parameters for a simulation. In the csv file, each column represents a parameter while each row represents a set of parameters for a simulation.
    * `gzserver_port_offset` is an integer used to assign a port number for a simulation. In Gazebo, each simulation in parallel must be assigned a unique port number.
    * `swarm_size` is an integer representing the number of foraging robots in the swarm. Valid options are 1, 9, 16, 25, 36, 49, 64, 81, 100

    A sample simulation start up is:
    `./mult_div_submission_loop.sh 0 1 1 Uniform var_qsize params/var_qsize.csv 0 9`

    (NB: You will need to zoom out to get a complete view of the simulation world. You can use the scroll on your mouse to do that. http://gazebosim.org/hotkeys)

    Sample simulation view in progress is shown in the figure below

    ![Sample simulation](sample_simulation_view.png "Gazebo simulation in progress")

1. To (re)build the project, run 
    
    `singularity exec 20190708-libgazebo7-xenial.simg ./build.sh`

## SETUP SIMULATION USING LOCAL INSTALLATION OF GAZEBO
### DEPENDENCIES
This project needs some dependencies to be installed for correct running of the simulation:
```
apt-get -y install python3 cmake lsb-release wget
```

### INSTALLATION STEPS

The official steps for setting up Gazebo on local machine is available at: http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install

The major difference between the steps outlined here is that this simulation uses Gazebo version 7 (not version 11 used in the official repo - as of 2021-04-27)

1. Setup your computer to accept software from packages.osrfoundation.org.
```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
```
You can check to see if the file was written correctly. For example, in Ubuntu Bionic (18.04), you can type:
```
cat /etc/apt/sources.list.d/gazebo-stable.list
```
And if everything is correct, you should see:
```
deb http://packages.osrfoundation.org/gazebo/ubuntu-stable bionic main
```


2. Setup keys
```
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```

3. Install Gazebo.
First update the debian database:
```
sudo apt-get update
```
Hint: make sure the apt-get update process ends without any errors, the console output ends in `Done` similar to below:
```
sudo apt-get update
...
Hit http://ppa.launchpad.net bionic/main Translation-en
Ign http://us.archive.ubuntu.com bionic/main Translation-en_US
Ign http://us.archive.ubuntu.com bionic/multiverse Translation-en_US
Ign http://us.archive.ubuntu.com bionic/restricted Translation-en_US
Ign http://us.archive.ubuntu.com bionic/universe Translation-en_US
Reading package lists... Done
```
Next install gazebo-7 by:
```
sudo apt-get install gazebo

# For developers that work on top of Gazebo, one extra package
sudo apt-get install libgazebo7-dev
```
If you see the error below:
```
sudo apt-get install gazebo11
Reading package lists... Done
Building dependency tree
Reading state information... Done
E: Unable to locate package gazebo11
```
It's possible the version of Gazebo you are looking for is not supported on the version of OS you are using. For example, installing gazebo11 on Ubuntu Xenial (16.04) will produce the error above. Hint: Take a look at "Project Status" section at http://gazebosim.org/#status, next to each version is the supported ubuntu versions and ROS versions.

4. Check your installation
```
gazebo
```

### STARTING THE SIMULATION
Run the `native_startup.sh` shell script to start the simulation:
```
./native_startup.sh platform param_line target_distribution experiment_name parameter_file gzserver_port_offset swarm_size
```
Command line arguments carry the same meaning as those in the steps for running the containerized version of the simulation.