#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include <boost/bind.hpp>
#include <iostream>
#include <string>
#include <fstream>
#include <mutex>
#include <map>
#include <vector>
#include <set>

#include <sstream>
#include <iomanip>
#include <ctime>

using namespace std;
namespace gazebo
{
	struct LitterInfo {
		physics::ModelPtr litterModel;
		int numberOfDetections;
		bool picked;
		std::string detectableBy;
		LitterInfo()
		: litterModel(nullptr), numberOfDetections(0), picked(false),detectableBy("")
		{}
		LitterInfo(physics::ModelPtr model)
		: litterModel(model), numberOfDetections(0), picked(false),detectableBy("")
		{}
		void updateDetectableBy(std::string robotID)
		{
			if ((this->detectableBy).empty())
				this->detectableBy += robotID;
			else
				this->detectableBy += ";" + robotID;
		}
	};
	struct RobotInfo {
		physics::ModelPtr robotModel;
		int litterCount;
		std::string seenLitter;
		std::string state;
	};

	class WP_Swarm1 : public WorldPlugin
	{
		//Pointer to the update event connection
		private:
			event::ConnectionPtr updateConnection;
			physics::WorldPtr world;
			std::mutex mutex;
			transport::NodePtr node;
			transport::PublisherPtr pub_litter_pose;
			std::string world_info_string;//should state when simulation starts and ends per world
			bool world_info_bool;//used to indicate the world is loaded and ready to go
			transport::PublisherPtr pub_world_loaded;
			
			bool world_gov_experiment_control;// used by world governor to control start of simulation runs
			transport::SubscriberPtr sub_world_gov_experiment_control;

			double nei_sensing;
			bool param_set;
			gazebo::transport::SubscriberPtr sub_params;
			//This section of parameters are used to handle communication between robots
			std::map<std::string,transport::PublisherPtr> pub_commSignal;
			//std::map<std::string,transport::PublisherPtr> pub_attraction;
			
			std::map<std::string,int> extra_litter_map;
			gazebo::transport::SubscriberPtr sub_seen_litter;//Models publish seen litter to this topic
			
			std::map<std::string,std::string> robot_status_map;
			std::map<std::string,LitterInfo> litterNumberOfDetections;
			transport::SubscriberPtr sub_robotDetectedLitterNames;
			transport::SubscriberPtr sub_robotDetectableLitters;
			std::string logPrefix;

			gazebo::transport::SubscriberPtr sub_robot_status;//Models publish their status to this sub
			
			gazebo::transport::PublisherPtr pub_repel_signal;
			gazebo::transport::PublisherPtr pub_attract_signal;
			gazebo::physics::Model_V robot_ptr;//consider changing it to set
			gazebo::physics::Model_V litter_ptr;
			double lit_threshold;
			
			//*********************START: TO HANDLE PARAMTER UPDATING AND SIMULATION RESETS
			bool start_sim;
			bool start_sim2;
			bool no_litter;
			gazebo::transport::PublisherPtr pub_start_sim;
			gazebo::transport::PublisherPtr pub_my_Init;
			gazebo::transport::PublisherPtr pub_model_log_control;
			transport::PublisherPtr physicsPub;
			bool set_max_step_size;
			std::string params_str;

			int litter_tot;//total litter in world
			int litter_in_nest;//total litter deposited at the nest
			gazebo::transport::SubscriberPtr sub_litter_in_nest;//subscriber to litter in nest topic
			
			//All params to be simulated are stored in a vector
			vector<string> my_params;
			std::vector<string>::iterator param_it;
			
			
			double world_size;
			//double world_sizeY;
			//double psec;
			//*********************END: TO HANDLE PARAMTER UPDATING AND SIMULATION RESETS
			
			//START: EXPERIMENT START STOP CONTROL
			gazebo::transport::PublisherPtr pub_experiment_control;
			bool end_experiment;
			//END: EXPERMEINT START STOP CONTROL
			
			double log_timer;
			double max_step_size;
			double log_rate;
			
			std::string com_model;
			int cap_com;
			// Sound source modelling parameters
			math::Vector3 signal_source_pos;
			double A0; //intensity from source
			double alpha; //attenuation factor
			double Ae; //ambient noise scale
			double noise_mean;//mean of the noise
			double noise_std;//standard deviation of the noise
			std::normal_distribution<double> noise_distro;//distribution for making sound noisy
			std::default_random_engine generator;
		public:
			void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
			{
				// std::cout<<"world loading"<<std::endl;
				this->node = transport::NodePtr(new transport::Node());
				this->node->Init();
				this->world = _parent;
				this->world_info_bool = true;//default value is true when world is loaded
				this->pub_world_loaded = this->node->Advertise<msgs::Any>("/world_loaded");
				

				this->world_gov_experiment_control = false;//default value is false. Set to true from subscribed topic
				this->sub_world_gov_experiment_control = this->node->Subscribe("/world_gov_experiment_control",&WP_Swarm1::world_gov_experiment_control_cb,this);

				this->physicsPub = node->Advertise<msgs::Physics>("~/physics");
				msgs::Physics physicsMsg;
				physicsMsg.set_type(msgs::Physics::ODE);
				physicsMsg.set_max_step_size(0.025);//manually increase the step size
				this->physicsPub->Publish(physicsMsg);

				this->set_max_step_size = true;
				//this->psec = 0;
				//exit(5);
				//load simulation paramters
				// ifstream myfile;
				// string line, header;
				// myfile.open("params/params.csv");
				// if(myfile.is_open())
				// {
				// 	getline(myfile,header);
				// 	while(getline(myfile,line))
				// 	{
				// 		size_t hsep1=0,hsep2=0,psep1=0,psep2=0;
				// 		string hparam,param_value;
				// 		string param_line = "";
				// 		while(header.find(",",hsep1) != string::npos and
				// 				line.find(",",psep1) != string::npos)
				// 		{
				// 			hsep2 = header.find(",",hsep1);
				// 			hparam = header.substr(hsep1,hsep2-hsep1);
				// 			psep2 = line.find(",",psep1);
				// 			param_value = line.substr(psep1,psep2-psep1);
							
				// 			param_line = param_line + hparam + ":" + param_value + ",";
				// 			hsep1 = hsep2 + 1;
				// 			psep1 = psep2 + 1;
				// 			if(hparam.compare("max_step_size") == 0)
				// 			{
				// 				transport::PublisherPtr physicsPub = 
				// 				node->Advertise<msgs::Physics>("~/physics");
				// 				msgs::Physics physicsMsg;
				// 				physicsMsg.set_type(msgs::Physics::ODE);
								
				// 				double value = std::stod(param_value);
				// 				//this->max_step_size = value;
				// 				//value = 1.0/value;
				// 				physicsMsg.set_max_step_size(value);
				// 				physicsPub->Publish(physicsMsg);
				// 			}
				// 		}
				// 		this->my_params.push_back(param_line);
				// 	}
				// 	//iterator set to first parameter list
				// 	this->param_it = this->my_params.begin();
				// 	myfile.close();
				// }
				// else
				// {
				// 	//cout<<"unable to load file"<<endl;
				// 	//exit(5);
				// }
				
				
				
				math::Box boundary = this->world->GetModel("boundary1")->GetBoundingBox();
				this->world_size = boundary.GetXLength();
				//std::cout<<boundary.GetXLength()<<":"<<boundary.GetYLength()<<":"<<boundary.GetZLength()<<std::endl;
				
				this->sub_params = this->node->Subscribe("/params_topic",&WP_Swarm1::Params_cb,this);
				this->sub_seen_litter = this->node->Subscribe("/topic_seen_litter",&WP_Swarm1::seen_litter_cb,this);
				this->sub_robot_status = this->node->Subscribe("/topic_robot_status",&WP_Swarm1::robot_status_cb,this);
				this->pub_litter_pose = this->node->Advertise<msgs::Any>("/topic_litter_pose");
				std::string all_litter_pos = "";
				//////////////////////////////////////////////////////////////
				gazebo::physics::Model_V all_models = this->world->GetModels();
				this->sub_litter_in_nest = this->node->Subscribe("/litter_in_nest",&WP_Swarm1::litter_in_nest_cb,this);
				this->litter_in_nest = 0;
				
				this->litter_tot = 0;
				for(auto m : all_models)
				{
					
					std::string model_name = m->GetName();
					if(model_name.find("m_4wrobot") != std::string::npos)
					{
						this->robot_ptr.push_back(m);
						this->pub_commSignal[model_name] = this->node->Advertise<msgs::Any>("/"+model_name+"/comm_signal");
						//this->pub_attraction[model_name] = this->node->Advertise<msgs::Any>("/"+model_name+"/attract_signal");
						this->extra_litter_map[model_name] = 0;
						this->robot_status_map[model_name] = "searching";
					}
					
					if(model_name.find("litter") != std::string::npos)
					{
						this->litter_tot += 1;
						gazebo::math::Vector3 lit_loc = m->GetWorldPose().pos;
						this->litter_ptr.push_back(m);
						all_litter_pos = all_litter_pos + to_string(lit_loc.x) + ","
														+ to_string(lit_loc.y) + ":";
						this->litterNumberOfDetections[model_name] = LitterInfo(m);
					}
				}
				this->sub_robotDetectedLitterNames = this->node->Subscribe("/robotDetectedLitterNames", &WP_Swarm1::cb_robotDetectedLitterNames, this);
				this->sub_robotDetectableLitters = this->node->Subscribe("/robotDetectableLitterNames", &WP_Swarm1::cb_robotDetectableLitters, this);
				
				msgs::Any all_litter_pos_msg;
				all_litter_pos_msg.set_type(msgs::Any::STRING);
				all_litter_pos_msg.set_string_value(all_litter_pos);
				this->pub_litter_pose->Publish(all_litter_pos_msg);//publish initial pos of all litter
				
				this->pub_repel_signal = this->node->Advertise<msgs::Any>("/repulsion_signal");
				this->pub_attract_signal = this->node->Advertise<msgs::Any>("/attraction_signal");
				
				
				this->pub_start_sim = this->node->Advertise<msgs::Any>("/start_sim");//control start and stop for simulations
				this->pub_my_Init = this->node->Advertise<msgs::Any>("/my_Init");//publish simulation parameters
				this->start_sim = false;
				this->start_sim2 = false;
				this->pub_model_log_control = this->node->Advertise<msgs::Any>("/topic_model_log");
				
				this->pub_experiment_control = this->node->Advertise<msgs::Any>("/experiment_control");
				this->end_experiment = false;
				//my_Init();
				this->param_set = false;//Initially params_str is null. so we know it has not been set.
				//Handling communication among robots section
				/*for (int i = 1; i <= 10;i++)
				{
					std::string name = "m_4wrobot" + to_string(i);
					this->pub_repulsion[name] = this->node->Advertise<msgs::Any>("/"+name+"/comm_signal");
					
				}
				*/
				//////////////////////////////////////////////////////////
				this->updateConnection = event::Events::ConnectWorldUpdateBegin(
							boost::bind(&WP_Swarm1::OnUpdate,this,_1));
				// std::cout<<"world loaded"<<std::endl;
			}
			void cb_robotDetectedLitterNames(ConstAnyPtr &any)
			{
				std::lock_guard<std::mutex> lock(this->mutex);
				std::string detectionStr = any->string_value();
				while(detectionStr.find(",") != std::string::npos)
				{
					std::size_t nameLoc = detectionStr.find(",");
					std::string litterName = detectionStr.substr(0,nameLoc);
					if (litterName.find("litter") != std::string::npos)
					{
						(this->litterNumberOfDetections[litterName]).numberOfDetections += 1;

					}
					detectionStr = detectionStr.substr(nameLoc + 1);
				}
			}
			void cb_robotDetectableLitters(ConstAnyPtr &any)
			{
				std::lock_guard<std::mutex> lock(this->mutex);
				std::string detectabilityStr = any->string_value();
				
				std::size_t robIDloc = detectabilityStr.find(":");
				std::string robotID = detectabilityStr.substr(0,robIDloc);
				detectabilityStr = detectabilityStr.substr(robIDloc + 1);

				while(detectabilityStr.find(",") != std::string::npos)
				{
					std::size_t nameLoc = detectabilityStr.find(",");
					std::string litterName = detectabilityStr.substr(0,nameLoc);
					if (litterName.find("litter") != std::string::npos)
					{
						(this->litterNumberOfDetections[litterName]).updateDetectableBy(robotID);

					}
					detectabilityStr = detectabilityStr.substr(nameLoc + 1);
				}
			}
			void litter_in_nest_cb(ConstAnyPtr &any)
			{
				std::lock_guard<std::mutex> lock(this->mutex);
				std::string litter_in_nest_str = any->string_value();
				size_t n_loc = litter_in_nest_str.find(",");
				int l = std::stoi(litter_in_nest_str.substr(n_loc+1));
				this->litter_in_nest = l;
			}
			void seen_litter_cb(ConstAnyPtr &any)
			{
				std::lock_guard<std::mutex> lock(this->mutex);
				std::string seen_litter_msg = any->string_value();
				size_t n_loc = seen_litter_msg.find(":");
				std::string model_name = seen_litter_msg.substr(0,n_loc);
				int seen_litter_num = std::stoi(seen_litter_msg.substr(n_loc+1));
				//update appropriate model with it's seen litter
				this->extra_litter_map[model_name] = seen_litter_num;
			}
			
			void robot_status_cb(ConstAnyPtr &any)
			{
				std::lock_guard<std::mutex> lock(this->mutex);
				
				std::string robot_status_msg = any->string_value();
				size_t n_loc = robot_status_msg.find(":");
				std::string model_name = robot_status_msg.substr(0,n_loc);
				std::string robot_status = robot_status_msg.substr(n_loc+1);
				
				this->robot_status_map[model_name] = robot_status;
				
				//std::cout<<model_name<<" "<<robot_status<<std::endl;
			}
			// void logDetectionDetails(string litterName,int numberOfDetections,bool picked)
			// {
			// 	ofstream detectionDetails(this->logPrefix + "_litterDetectionDetails.csv",std::ios::app|std::ios::ate);
			// 	detectionDetails << litterName << "," << numberOfDetections << "," << picked << std::endl;
			// 	detectionDetails.close();
			// }
			void my_Init()
			{
				// std::cout<<"Init called"<<std::endl;
				msgs::Any exp_control;
				exp_control.set_type(msgs::Any::STRING);
				std::string sim_params = "param:value pairs";
				
				if(this->params_str.compare("end_simulation") != 0)
				{
					// sim_params = *(this->param_it);
					sim_params = this->params_str;

					int area = (int)(this->world_size)*(this->world_size);
					std::string x = sim_params;
					x = x + 
						"no_of_litter:" + to_string(this->litter_ptr.size())
						+ ",no_of_robots:" + to_string(this->robot_ptr.size())
						+ ",world_size:" +
						 to_string(area) + "sqm";
					exp_control.set_string_value(x);
					this->pub_experiment_control->Publish(exp_control);
				}
				if(this->params_str.compare("end_simulation") == 0)
				{
					// exp_control.set_string_value("end");
					// this->pub_experiment_control->Publish(exp_control);
					this->end_experiment = true;
					return;
				}
				//cout<<"called"<<endl;
				
				//stop all models by sending start_sim = false
				msgs::Any s_sim;
				s_sim.set_type(msgs::Any::BOOLEAN);
				this->start_sim = false;//to be double sure that simulation does not start
				s_sim.set_bool_value(this->start_sim);
				this->pub_start_sim->Publish(s_sim);
				
				//reset world
				this->world->Reset();
				
				//Extract all parameters for this simulation
				/*if(this->param_it == this->my_params.end())
				{
					std::cout<<"Simulations Ended"<<endl;
					exit(0);
				}*/
				
				//insert seed for each robot type.
				srand(std::time(nullptr));
				for (auto m:this->robot_ptr)
				{
					std::string robot_name = m->GetName();
					sim_params = sim_params + robot_name + ":" + to_string(rand()) + ",";
				}
				std::string sim_params_mine = sim_params;
				
				//set parameters of world plugin++++
				while(sim_params_mine.find(",") != std::string::npos)
				{//loop through all parameters and assign to appropriate variable in plugin
					size_t ploc = sim_params_mine.find(",");
					std::string temp = sim_params_mine.substr(0,ploc);
					
					size_t aloc = temp.find(":");
					std::string param_name = temp.substr(0,aloc);
					std::string param_value_str = temp.substr(aloc+1);
					// double value  = std::stod(param_value_str);//double value of parameter
					
					if(param_name.compare("nei_sensing") == 0)
					{
						this->nei_sensing = std::stod(param_value_str);//double value of parameter;
						//std::cout<<"nei_sensing = "<<this->nei_sensing<<endl;
						//break;
					}
					else if(param_name.compare("log_rate") == 0)
					{
						this->log_rate = std::stod(param_value_str);//double value of parameter;
					}
					else if(param_name.compare("max_step_size") == 0)
					{
						this->max_step_size = std::stod(param_value_str);//double value of parameter;
					}
					else if(param_name.compare("lit_threshold") == 0)
					{
						this->lit_threshold = std::stod(param_value_str);//double value of parameter;
					}

					else if(param_name.compare("A0") == 0)
					{
						this->A0 = std::stod(param_value_str);//double value of parameter;
					}
					else if (param_name.compare("Ae") == 0)
					{
						this->Ae = std::stod(param_value_str);//double value of parameter;
					}
					else if(param_name.compare("alpha") == 0)
					{
						this->alpha = std::stod(param_value_str);//double value of parameter;
					}
					else if(param_name.compare("noise_mean") == 0)
					{
						this->noise_mean = std::stod(param_value_str);//double value of parameter;
					}
					else if(param_name.compare("noise_std") == 0)
					{
						this->noise_std = std::stod(param_value_str);//double value of parameter;
					}
					else if(param_name.compare("com_model") == 0)
					{
						this->com_model = param_value_str;//communication model is a string
					}
					else if(param_name.compare("cap_com") == 0)
					{
						this->cap_com = std::stoi(param_value_str);//double value of parameter;
					}
					else if(param_name.compare("max_step_size") == 0 and this->set_max_step_size)
					{
						msgs::Physics physicsMsg;
						physicsMsg.set_type(msgs::Physics::ODE);
						
						double value = std::stod(param_value_str);
						//this->max_step_size = value;
						//value = 1.0/value;
						physicsMsg.set_max_step_size(value);
						this->physicsPub->Publish(physicsMsg);
						this->set_max_step_size = false;
					}
					else if(param_name.compare("logPrefix") == 0)
					{
						this->logPrefix = param_value_str;
					}
					else
					{
						//cout<<temp<<endl;
					}
					//cout<<temp<<endl;
					sim_params_mine = sim_params_mine.substr(ploc+1);
					
						
				}
				//cout<<"params set"<<endl;
				//this->nei_sensing = 5.0;
				
				//publish simulation parameters for this set of simulation.
				msgs::Any sim_params_msg;
				sim_params_msg.set_type(msgs::Any::STRING);
				sim_params_msg.set_string_value(sim_params);
				this->pub_my_Init->Publish(sim_params_msg);
				
				//noisy data based on normal distro
				this->generator = std::default_random_engine(rand());
				this->noise_distro = std::normal_distribution<double>(this->noise_mean,this->noise_std);
				
				
				// (this->param_it)++;//increment iterator to next parameter set;
				
				//this->param_set = false;
				this->world_info_string = "";
				
				this->start_sim2 = true;
				this->no_litter = false;//assume there is litter;
				
				this->log_timer = 0;//reset log timer.
				// std::cout<<"Init exited"<<std::endl;
				
			}
			
			void Params_cb(ConstAnyPtr &a)
			{//set all parameters and set param_set = true when done.
				std::lock_guard<std::mutex> lock(this->mutex);
				this->params_str = a->string_value();
				// std::cout<<this->params_str<<std::endl;
				this->param_set = true;
			}
			
			void world_gov_experiment_control_cb(ConstAnyPtr &a){
				//controls when to start a simulation.
				std::lock_guard<std::mutex> lock(this->mutex);

				this->world_gov_experiment_control = a->bool_value();

				//world governor has been informed that the world is ready to go. Set the world_info_bool to false;
				this->world_info_bool = false;
			}
			void OnUpdate(const common::UpdateInfo &_info)
			{
				std::lock_guard<std::mutex> lock(this->mutex);
				// std::cout<<_info.simTime<<std::endl;
				if(this->param_set)
				{//check if parameters have been initialized.
					
					this->my_Init();
					this->param_set = false;
				}
				if(this->end_experiment)
				{//Publish this->begin_experiment
					//std::cout<<"Simulations Ended"<<endl;
					exit(0); 
				}
				if(this->world_info_bool){
					//At start of simulation, this informs the world governor that the world has been loaded and ready to go
					msgs::Any any;
					//any.set_type(msgs::Any::STRING);
					//any.set_string_value("end");
					any.set_type(msgs::Any::BOOLEAN);
					any.set_bool_value(this->world_info_bool);
					this->pub_world_loaded->Publish(any);
					// this->world_info_bool = false;//to be set to false from world_gov_experiment_control subscriber
				}
				if(!this->world_gov_experiment_control){
					//needed to prevent simulation from counting while world-gov-control is set to false
					this->world->Reset();
				}
				if(this->litter_in_nest >= this->litter_tot)// or st.sec >= 30)//(true and this->no_litter) or  false and (st.sec >= 300 and st.nsec==0))
				{
					// this->litter_in_nest = 0;
					// // this->param_set =  true;
					// msgs::Any s_sim;
					// s_sim.set_type(msgs::Any::BOOLEAN);
					// this->start_sim = false;//to be double sure that simulation does not start
					// s_sim.set_bool_value(this->start_sim);
					// this->pub_start_sim->Publish(s_sim);
					// this->world_gov_experiment_control = false;//set to false since a simulation iteration is ended
					
					//All litter collected. Alert Governor that you have ended current simulation
					msgs::Any exp_control;
					exp_control.set_type(msgs::Any::STRING);
					std::string end_sim_str = "end";
					exp_control.set_string_value(end_sim_str);
					this->pub_experiment_control->Publish(exp_control);
					// std::cout<<"End Published"<<std::endl;
					// return;
					// this->world->Reset();
				}

				if(this->world_gov_experiment_control and !(this->param_set)) {
						//world governor controls when simulation starts by setting the value of world_gov_experiment_control to true
						//Also, if simulation parameters have been set
					//std::cout<<_info.simTime.Double()<<std::endl;
					if(this->start_sim2)
					{
						msgs::Any s_sim;
						s_sim.set_type(msgs::Any::BOOLEAN);
						this->start_sim = true;
						s_sim.set_bool_value(this->start_sim);
						this->pub_start_sim->Publish(s_sim);
						
						this->start_sim2 = false;
						this->log_timer = 0;
						// msgs::Any set_prefix;
						// set_prefix.set_type(msgs::Any::STRING);
						// set_prefix.set_string_value("new_file");
						// this->pub_model_log_control->Publish(set_prefix);
					}
					this->log_timer += this->max_step_size;
					
					
					
					// if(this->param_set)
					// {
						
					// 	this->my_Init();
					// 	this->param_set = false;
					// }
					//std::cout<<this->world->GetModelCount()<<endl;
					gazebo::common::Time st = _info.simTime;
				/*
					if(st.nsec==0)
					{
						ofstream myfile("robot_name.txt",std::ios::app|std::ios::ate);
						myfile << st.nsec<<std::endl;
						myfile.close();
					}*/
					
					
					/*/Start::Testing Seen litter messages
					std::string s="";
					for(auto it = this->extra_litter_map.begin(); it != this->extra_litter_map.end();it++)
					{
						s = s + it->first + ":" + to_string(it->second) + ",";
					}
					std::cout<<s<<std::endl;
					/End :: Test seen litter messages*/
					if(this->litter_in_nest >= this->litter_tot)// or st.sec >= 30)//(true and this->no_litter) or  false and (st.sec >= 300 and st.nsec==0))
					{
						this->litter_in_nest = 0;
						// this->param_set = true;
						msgs::Any s_sim;
						s_sim.set_type(msgs::Any::BOOLEAN);
						this->start_sim = false;//to be double sure that simulation does not start
						s_sim.set_bool_value(this->start_sim);
						this->pub_start_sim->Publish(s_sim);
						this->world_gov_experiment_control = false;//set to false since a simulation iteration is ended
						
						//All litter collected. Alert Governor that you have ended current simulation
						// msgs::Any exp_control;
						// exp_control.set_string_value("end");
						// this->pub_experiment_control->Publish(exp_control);
						this->world->Reset();
							
						
				
					}
					else
					{
						for(auto m : this->robot_ptr)
						{//Get current robot.
							std::string r_name = m->GetName();
							math::Vector3 r_pos = m->GetWorldPose().pos;
							int rep_neighbours = 0;
							int att_neighbours = 0;
							double rep_signal = 0;
							double att_signal = 0;
							std::string rep_data = "";
							std::string att_data = "";
							
							double rslt_x=0.0,rslt_y=0.0;
							
							for(auto n : this->robot_ptr)
							{//Loop through neighbours
								std::string n_name = n->GetName();
								math::Vector3 n_pos = n->GetWorldPose().pos;
								
								std::string n_status = this->robot_status_map[n_name];
								int n_extra_litter = this->extra_litter_map[n_name];
								
								
								//std::cout<<n_name<<" "<<n_status<<std::endl;
								
								
								if(n_name.compare(r_name) !=0)
								{//If not the current robot do this section
									r_pos.z = 0;
									n_pos.z = 0;
									// if(abs(r_pos.x - n_pos.x) < this->nei_sensing and
									// 		abs(r_pos.y - n_pos.y) < this->nei_sensing)
									// {
										double dist = r_pos.Distance(n_pos);
										// if(dist <= this->nei_sensing)//neighbour sensing distance... Consider setting parameter at startup
										// {
											if(n_status.compare("searching")==0 || false)
											{
												rep_neighbours += 1;
												double repulsion_intensity = 0;
												if(this->com_model.compare("linear") == 0){
													repulsion_intensity = (this->nei_sensing - dist)/(this->nei_sensing);
												}
												else if(this->com_model.compare("sound") == 0) {
													repulsion_intensity = this->A0 * exp(-dist*(this->alpha)) + this->Ae;
													//add noise
													repulsion_intensity += this->noise_distro(this->generator);
												
												}
												else if(this->com_model.compare("soundv2") == 0){
													//noise_mean is average of fitError/signalStrength and noise_std is the deviation across experiments
													std::normal_distribution<double> noise_distro_std(this->noise_mean,this->noise_std);
													
													//intensity of pure signal
													repulsion_intensity = this->A0 * exp(-dist*(this->alpha)) + this->Ae;
													
													//noise is proportional to intensity
													double signal_std = repulsion_intensity * noise_distro_std(this->generator);

													//use proportional value computed as standard deviation
													std::normal_distribution<double> noise_distro_signal(repulsion_intensity,signal_std);

													//noisy repulsion intenisity of communicated signal
													repulsion_intensity = noise_distro_signal(this->generator);
													
													
												}
												else if(this->com_model.compare("vector") == 0) {
													//TO DO
												}
												//double repulsion_intensity = (this->nei_sensing - dist)/(this->nei_sensing);
												/*******************************************************/
												// double repulsion_intensity = this->A0 * exp(-dist*(this->alpha)) + this->Ae;
												if(dist > this->nei_sensing and this->cap_com == 1)
												{//If outside comm range and cap_com is set
													repulsion_intensity = 0;
												}
												
												if(repulsion_intensity < 0)
													repulsion_intensity = 0;
												/*******************************************************/

												rep_signal += repulsion_intensity;
												rep_data = rep_data + "," + to_string(dist);
											}
											
											if(n_extra_litter > 0)//If seen litter is greater than space left, send an attraction signal
											{
												//cout<<"n_extra_litter:"<<n_extra_litter<<" this->lit_threshold:"<<this->lit_threshold<<endl;
												att_neighbours += 1;
												double attraction_intensity = 0;
												if(this->com_model.compare("linear") == 0){
													attraction_intensity = (this->nei_sensing - dist)/(this->nei_sensing);
												}
												else if(this->com_model.compare("sound") == 0) {
													attraction_intensity = this->A0 * exp(-dist*(this->alpha)) + this->Ae;
													//add noise
													attraction_intensity += this->noise_distro(this->generator);
												}
												else if(this->com_model.compare("soundv2") == 0){
													//noise_mean is average of fitError/signalStrength and noise_std is the deviation across experiments
													std::normal_distribution<double> noise_distro_std1(this->noise_mean,this->noise_std);
													
													//intensity of pure signal
													attraction_intensity = this->A0 * exp(-dist*(this->alpha)) + this->Ae;
													
													//noise is proportional to intensity
													double signal_std1 = attraction_intensity * noise_distro_std1(this->generator);

													//use proportional value computed as standard deviation
													std::normal_distribution<double> noise_distro_signal1(attraction_intensity,signal_std1);

													//noisy repulsion intenisity of communicated signal
													attraction_intensity = noise_distro_signal1(this->generator);
													
													
												}
												else if(this->com_model.compare("vector") == 0) {
													//TO DO
												}
												// double attraction_intensity = (this->nei_sensing - dist)/(this->nei_sensing);
												/*******************************************************/
												// double attraction_intensity = this->A0 * exp(-dist*(this->alpha)) + this->Ae;

												if(dist > this->nei_sensing and this->cap_com == 1)
												{//If outside comm range and cap_com is set
													attraction_intensity = 0;
												}

												if(attraction_intensity < 0)
													attraction_intensity = 0;
												/*******************************************************/
												att_signal += attraction_intensity;
												att_data = att_data + "," + to_string(dist);
											}
											//Computing Resultant Vector
											double vec_x,vec_y;
											vec_x = (r_pos.x - n_pos.x)/dist;
											vec_y = (r_pos.y - n_pos.y)/dist;
											rslt_x += vec_x;
											rslt_y += vec_y;
									// 	}
									// }
								}
							}
							
							double rslt_theta = atan2(rslt_y,rslt_x);
							
							msgs::Any any;
							any.set_type(msgs::Any::STRING);
							any.set_string_value(to_string(rep_neighbours) + ":" + to_string(rep_signal)
													+ ":" + to_string(rslt_theta) + ":" + 
													to_string(att_neighbours) + ":" + to_string(att_signal));
							this->pub_commSignal[r_name]->Publish(any);
							
							//any.set_string_value(to_string(att_neighbours) + ":" + to_string(att_signal));
							//this->pub_attraction[r_name]->Publish(any);
							
							if(st.nsec==0){//record repulsion once per second.	
								rep_data = r_name + "," + to_string(rep_signal) + "," + to_string(rep_neighbours) + rep_data;
								msgs::Any any2;
								any2.set_type(msgs::Any::STRING);
								any2.set_string_value(rep_data);
								this->pub_repel_signal->Publish(any2);
								
								att_data = r_name + "," + to_string(att_signal) + "," + to_string(att_neighbours) + att_data;
								any2.set_string_value(att_data);
								this->pub_attract_signal->Publish(any2);
							}
						}
						
						//std::cout <<this->log_timer << this->log_rate << this->start_sim <<std::endl;
						if( st.nsec==0 or (this->log_timer >= this->log_rate and this->start_sim))//rate of 100Hz
						{
							this->log_timer = 0;
							this->no_litter = true;//Assume there is no litter within world
							//std::cout<<this->psec<<endl;
							std::string all_litter_pos = "";
							ofstream detectionDetails(this->logPrefix + "_litterDetectionDetails.csv", std::ofstream::out | std::ofstream::trunc);
							detectionDetails << "name,detectableBy,numberOfDetections,picked" << std::endl;
								
							for(auto m: this->litter_ptr)
							{//Get current pose of all litter in world
								gazebo::math::Vector3 lit_loc = m->GetWorldPose().pos;
								
								if(abs(lit_loc.x) < 500 && abs(lit_loc.y) < 500)
								{
									this->no_litter = false;
									
								}
								else
								{
									(this->litterNumberOfDetections[m->GetName()]).picked = true;
								}
								all_litter_pos = all_litter_pos + to_string(lit_loc.x) + ","
																+ to_string(lit_loc.y) + ":";
								
								detectionDetails << m->GetName() << ","
										<< (this->litterNumberOfDetections[m->GetName()]).detectableBy << ","
										<< (this->litterNumberOfDetections[m->GetName()]).numberOfDetections << ","
										<< (this->litterNumberOfDetections[m->GetName()]).picked << std::endl;
								
							}
							detectionDetails.close();
							msgs::Any all_litter_pos_msg;
							all_litter_pos_msg.set_type(msgs::Any::STRING);
							
							all_litter_pos = to_string(_info.simTime.Double())
											+ ":" + all_litter_pos;
							all_litter_pos_msg.set_string_value(all_litter_pos);
							this->pub_litter_pose->Publish(all_litter_pos_msg);//publish current pos of all litter
						}
						//this->psec = st.nsec;
					}
					if(this->log_timer > this->log_rate)
					{
						this->log_timer = 0;
					}
					
					/*if(this->param_set)
					{		
						//msgs::Any any;
						//any.set_type(msgs::Any::INT32);
						//any.set_int_value(this->robot_ptr.size());
						//any.set_type(msgs::Any::BOOLEAN);
						//any.set_bool_value(true);
						if(st.sec >= 30)
						{
							msgs::Any any;
							//any.set_type(msgs::Any::STRING);
							//any.set_string_value("end");
							any.set_type(msgs::Any::BOOLEAN);
							any.set_bool_value(this->world_info_bool);
							this->pub_world_loaded->Publish(any);
							this->world->Reset();
							this->world->Stop();//or this->world->Stop()(or Fini();
							//this->Load(this->world,sdf::ElementPtr sdf);
							//this->world_info_bool = true;
							//this->world->Load
							//this->world->Init();
						}
					}
					if(this->world_info_bool){
						msgs::Any any;
						//any.set_type(msgs::Any::STRING);
						//any.set_string_value("start");
						any.set_type(msgs::Any::BOOLEAN);
						any.set_bool_value(this->world_info_bool);
						this->pub_world_loaded->Publish(any);
						this->world_info_bool = false;
					}
					//this->pub_world_loaded->Publish(any);*/
				}
				
			}
	};
	
	//Register this plugin with the simulator
	GZ_REGISTER_WORLD_PLUGIN(WP_Swarm1)
}
