## INTRODUCTION
This repository contains all files necessary for running the RepAtt and Random Walk swarm foraging algorithms.

## DEPENDENCIES
* Singularity - https://sylabs.io/guides/3.7/user-guide/quick_start.html
* git - https://git-scm.com/
* Linux (Ubuntu was used)

## SETUP FILES
1. Install git
1. Install Singularity  by following the quick start guide for Linux or alternate steps for other platforms
1. Clone the repository for the source files  `git clone https://github.com/elcymon/swarm_sim.git`
1. `cd swarm_sim`
1. `sudo singularity build 20190708-libgazebo7-xenial.simg Singularity` This will create a singularity image with for the Gazebo simulator and the required environment for building the project.
1. To run a sample simulation use: `./mult_div_submission_loop.sh 0 1 1 Uniform var_qsize params/var_qsize.csv 0 36`