#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>
#include <stdio.h>
#include <stdlib.h> // absolute value computation i.e. abs
#include <iostream> //needed in order to use cout
#include <math.h> //for trig ratios
#include <random> //for generating random numbers.
//#include <gazebo/gui/viewers/TopicView.hh>

//Sensors Palava
#include <string>
#include <vector>
#include <map>
#include <mutex>

#include <gazebo/gazebo_config.h>
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#ifndef H_COMM_MODELS
#define H_COMM_MODELS
    #include "../comm_models/comm_models.hh"
#endif

#include "robot_info.pb.h"

using namespace std;
//using namespace cv;
using namespace gazebo;

namespace gazebo
{
	class ModelVel : public ModelPlugin
	{
		public: 
			void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
			void OnUpdate(const common::UpdateInfo & _info);//Called by the world update start event
			
			
			void ContactInfo(ConstContactsPtr &c);//Contact sensor should set a boolean type that will control robot
			
			
			void stop();
			void rotate(double dxn_error);
			void rotate_v2(double dxn_eror);
			void straight();//Straight motion
			double avoid_obstacle(double x, double y,math::Pose my_pos);
			void my_acTion(std::string acTion);
			double normalize(double angle);
			void CommSignal(ConstAnyPtr &a);
			void my_Init(ConstAnyPtr &any);
			void start_sim_cb(ConstAnyPtr &any);
			
			double dxy(math::Vector3 A, math::Vector3 B);
			bool try_pick(std::string litter_name);// pick litter if I have enough space
			void LitterSensor();
			bool litterInPickingRange(std::string litterName);
			double computeObjectOrientation(math::Vector3 objectPos, math::Vector3 myPos, double myYaw);
			bool testObjectWithinFoV(double objectOrientation, double halffov);
			void deposit_litter();
			
	
		private:
			
			bool start_sim;
			gazebo::transport::SubscriberPtr sub_start_sim;
			gazebo::transport::SubscriberPtr sub_my_Init;
			
			
			std::mutex mutex;
			physics::ModelPtr model,home;
			physics::WorldPtr world;
			std::string acTion;
			std::string ModelName;
			physics::JointPtr rwheel;
			physics::JointPtr lwheel;
			double chassis_diameter;
			double rVel;
			
			
			transport::NodePtr node;
			transport::PublisherPtr pub_info;
			
			
			double wheel_separation;
			double Kp;
			
			
			bool turn_complete;
			bool turn_set;
			
			
			std::string ContactMsg;
			gazebo::transport::SubscriberPtr c_sub;
			bool crashed;
			double x_crash;
			double y_crash;
			int wall_bounces; //keep count of number of times robot hits a wall
			int neighbour_bounces;//keep count of number of times you hit another robot
			
			transport::PublisherPtr pub_robot_status;
			int rep_neighbours;
			int call_neighbours;
			int nei_sensing;
			std::string neighbours_info;
			
			double halffov;
			
			transport::PublisherPtr lit_nei_pub;
			transport::PublisherPtr pub_seen_litter;
			int seen_litter;
			double lit_sensing;
			std::string litter_info;
			gazebo::math::Vector3 litter_pos;
			std::string LitterName;
			double litter_distance;
			bool pick_litter;
			bool picking_lit_wait;
			double picking_lit_dur;
			double pick_lit_time;
			int capacity;
			int litter_count;
			int litter_collected;// all litter a robot has collected so far
			int litter_deposited;// all litter dropped by robot at the nest
			std::set<std::string> litter_db;
			std::string detectedLitterNames;//names of litters currently within view of robot
			gazebo::math::Pose litter_dump_site;
			bool no_litter;
			physics::Model_V myLitterDB;//list of litter this robot can detect
			
			double p_s2s;//probability of seen to seen
			double p_u2s;//probability of unseen to seen
			double detectionDuration;//time taken to perform detection
			double previousVisionTime;
			std::set<std::string> prev_seen;
			
			transport::PublisherPtr pub_robot_info;
			transport::PublisherPtr pub_myDetectedLitterNames;
			transport::PublisherPtr pub_myDetectableLitters;
			transport::PublisherPtr pub_myLitter_DB;//publisher of names of litter picked by robot
			physics::ModelPtr litterModel;
			
			bool go_home;
			bool at_home;
			
			transport::PublisherPtr pub_litter;
			
			int waiting_t;
			
			
			gazebo::math::Pose my_pose;
			math::Angle turn_amt;
			double d_heading;
			
			double escape;
			double escape_dist;
			gazebo::math::Vector3 escape_start;
			
			double turn_prob;
			double turn_prob_max;
			double turn_prob_min;
			
			double abandon_prob;
			
			//handling color of robots based on state
			std::string state;
			std::string prev_state;
			transport::PublisherPtr pub_visual;
			//homing
			msgs::Color *homColMsg;
			msgs::Color *homDiffMsg;
			msgs::Visual homvisMsg;
			msgs::Material *hommaterialMsg;
			
			//searching
			msgs::Color *srchColMsg;
			msgs::Color *srchDiffMsg;
			msgs::Visual srchvisMsg;
			msgs::Material *srchmaterialMsg;
			
			//go4litter
			msgs::Color *go4lColMsg;
			msgs::Color *go4lDiffMsg;
			msgs::Visual go4lvisMsg;
			msgs::Material *go4lmaterialMsg;
			

			//avoid
			msgs::Color *avoidColMsg;
			msgs::Color *avoidDiffMsg;
			msgs::Visual avoidvisMsg;
			msgs::Material *avoidmaterialMsg;

			//RANDOM NUMBERS PALAVA. BOTH UNIFORM AND NORMAL(GAUSSIAN) DISTRIBUTIONS
			std::default_random_engine generator;
			std::uniform_real_distribution<double> uform_rand;
			std::uniform_real_distribution<double> uform_rand_heading;
			double umin,umax;//minimum and maximum for uniform distribution
			//std::normal_distribution<double> normal_rand;
			std::normal_distribution<double> normal_rand_heading;
			double n_stddev,n_mean;//stddev and mean of normal distribution
			
			
			//std::map<std::string,transport::PublisherPtr> pub_repulsion;
			//std::set<std::string> com_sig_set;
			//double cummulative_sig;
			transport::SubscriberPtr sub_comm_signal;
			//transport::PublisherPtr pub_comm_signal;
			double call_scale_mult;
			double call_scale_div;
			double repel_scale_mult;
			double repel_scale_div;
			
		//**************to be replaced by CommModels?*************************//
			double repel_signal;
			double prev_repel_signal;
			bool new_comm_signal;
			double call_signal;
			double prev_call_signal;
			std::deque<double> repel_queue;
			std::deque<double> call_queue;
			double queue_size;
		//***********************CommModels********************************//
			CommModels commModel;//a communication handler that also has inbuild filter processing
			std::string com_model;//string describing if communication is sound/linear/vector based
			std::string filter_type;
			//get time stamp of simulation
			double timeStamp;
		//*****************************************************************//
			std::string correction_mtd;
			
			double rslt_theta;
			
			transport::PublisherPtr pub_log;
			double log_timer;
			double max_step_size;
			double log_rate;
			
			math::Vector3 prev_loc;
			double prev_yaw;
			double linear_dist;
			double rot_dist;


			//logging activity times variables
			double t_obstacle_avoidance;
			double t_litter_processing;
			double t_go4litter;
			double t_oa_go4litter;
			double t_searching;
			double t_oa_searching;
			double t_homing;
			double t_oa_homing;

			
			event::ConnectionPtr updateConnection;//pointer to the update event connection

		
				
	};
}
