/**
 * Adapted from mp_model_push_split
 * 
 * **/

//Header model plugin class declaration including all fields and methods.
#include "mp_model_push_split.hh"
//Swarm Algorithm
#include "load_onupdate_random_walk.cc"

//Handling sensor readings
#include "contact_sensor.cc"
//#include "camera_sensor.cc"

//Movement controllers
#include "motion_controllers.cc"

//Controllers for home behaviours
//#include "home_controls.cc"

//Unclassified functions
#include "misc.cc"

//Simulated sensors for detecting litter and neighbouring robots
#include "litter_sensor.cc"


//including communication handling class for different kinds filters

#include "../comm_models/comm_models.cc"
//Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ModelVel)
