#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
//#include <gazebo/sdf/sdf.hh>
#include <fstream>
#include <ctime>
#include <time.h>
#include <sstream>
#include <iostream> //needed in order to use cout
#include <mutex>
#include <unistd.h>
#include <boost/filesystem.hpp>

using namespace std;

namespace gazebo
{

	struct RobotInfo {
		physics::ModelPtr robotModel;
		std::string litterCount;
		std::string seenLitter;
		std::string state;
		
		RobotInfo()
		{
			robotModel = nullptr;
			litterCount = "";
			seenLitter = "";
			state = "";
		}
		RobotInfo(physics::ModelPtr model)
		{
			robotModel = model;
			litterCount = "";//topic is /robotPickedLitterNames, msg "robotName:l1,l2,ln,"
			seenLitter = "";//topic is /robotDetectedLitterNames, msg "robotName:l1,l2,ln,"
			state = "";//topic is /topic_robot_status, msg "robotName:status"
		}

	};

	class Nest_Plugin : public ModelPlugin
	{
		private:
			std::vector<std::string> litter_count;
			int numLitter;
			transport::NodePtr node;
			transport::SubscriberPtr sub;
			transport::PublisherPtr pub;
			transport::SubscriberPtr sub_my_Init;
			transport::SubscriberPtr sub_start_sim;
			std::mutex mutex;
			
			double log_timer;
			double max_step_size;
			double log_rate;
			bool start_sim;

			physics::Model_V litters;
			std::map<std::string,RobotInfo> robots;
			ostringstream littersFile,robotsFile,nestFile;

			transport::SubscriberPtr sub_myDetectedLitterNames;
			transport::SubscriberPtr sub_myLitter_DB;
			transport::SubscriberPtr sub_robot_status;

			
			// Pointer to the update event connection
			 event::ConnectionPtr updateConnection;
		public: void writeData(string fileName,string data) {
			ofstream myfile(fileName,std::ios::app|std::ios::ate);
			myfile << data << std::endl;
			myfile.close();
		}
		public : void logDetails(bool header,int t) {
			ostringstream timeStepData;
			if (header) {
				string litterNames = "name",litterx = "x",littery = "y";
				for (auto m : this->litters){
					litterNames += "," + m->GetName();
					litterx += "," + to_string(m->GetWorldPose().pos.x);
					littery += "," + to_string(m->GetWorldPose().pos.y);
				}
				// ofstream myfile(this->littersFile,std::ios::app|std::ios::ate);
				this->writeData(this->littersFile.str(),litterNames);
				// gzdbg << litterNames <<std::endl;
				// gzdbg << this->littersFile.str() <<std::endl;
				this->writeData(this->littersFile.str(),litterx);
				this->writeData(this->littersFile.str(),littery);

				this->writeData(this->nestFile.str(),"time,litterCount,pickedLitter");

				string robotNames="names",robotInfo="info";
				for (auto m : this->robots) {
					robotNames += "," + m.first + "," + m.first + "," + m.first + "," + m.first + "," + m.first + "," + m.first;
					robotInfo += ",x,y,yaw,litterPicked,seenLitter,state";
				}
				this->writeData(this->robotsFile.str(),robotNames);
				this->writeData(this->robotsFile.str(),robotInfo);
				this->writeData(this->robotsFile.str(),"time");
			}
			else {
				string litterInfo = to_string(t);
				string robotsInfo = to_string(t);
				string nestInfo = to_string(t);
				int pickedLitter = 0;

				for (auto m : this->litters) {
					if (m->GetWorldPose().pos.x < 100 and abs(m->GetWorldPose().pos.y) < 100) {//yet to pick this litter
						litterInfo += "," + to_string(1);
					}
					else {//litter has been picked
						litterInfo += "," + to_string(0);
						pickedLitter++;
					}
				}
				this->writeData(this->littersFile.str(),litterInfo);

				for (auto m : this->robots) {
					physics::ModelPtr robotMode = (m.second).robotModel;
					math::Vector3 pos = robotMode->GetWorldPose().pos;
					math::Quaternion rot = robotMode->GetWorldPose().rot;
					robotsInfo += "," + this->setNumDP(pos.x,3);
					robotsInfo += "," + this->setNumDP(pos.y,3);
					robotsInfo += "," + this->setNumDP(rot.GetYaw(),3);
					robotsInfo += "," + (m.second).litterCount;
					robotsInfo += "," + (m.second).seenLitter;
					robotsInfo += "," + (m.second).state;
				}
				this->writeData(this->robotsFile.str(),robotsInfo);
				nestInfo += "," + to_string(this->numLitter) + "," + to_string(pickedLitter);
				this->writeData(this->nestFile.str(),nestInfo);

			}
		}
		public : string setNumDP(const double x, const int decDigits) {
									stringstream ss;
									ss << fixed;
									ss.precision(decDigits); // set # places after decimal
									ss << x;
									return ss.str();
								}
		public : void createFileNames(string logPrefix) {
			this->littersFile << logPrefix << "_littersFile.csv";
			this->robotsFile << logPrefix << "_robotsFile.csv";
			this->nestFile << logPrefix << "_nestFile.csv";


		}
		public: void getModelsLitterRobot(const physics::WorldPtr world) {
			physics::Model_V all_models = world->GetModels();
			for(auto m : all_models) {
				string m_name = m->GetName();
				if (m_name.find("m_4wrobot") != string::npos) {
					this->robots[m->GetName()] = RobotInfo(m);
				}
				else if (m_name.find("litter") != string::npos) {
					this->litters.push_back(m);
				}
			}
		}
		public : void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
		{
			
			
			this->getModelsLitterRobot(_parent->GetWorld());
			
			// this->logDetails(false,0);
			this->numLitter = 0;
			//Initialize the litter vector
			
			//create a subscriber to subscribe to the litter topic
			//Remember to make all robots send litter names to that topic
			
			this->node = transport::NodePtr(new transport::Node());
			this->node->Init();
			this->sub = this->node->Subscribe("/litter_deposits",&Nest_Plugin::Collect_Litter,this);
			this->pub = this->node->Advertise<msgs::Any>("/litter_in_nest");
			this->start_sim = false;


			
			this->sub_my_Init = this->node->Subscribe("/my_Init",&Nest_Plugin::my_Init,this);
			this->sub_start_sim = this->node->Subscribe("/start_sim",&Nest_Plugin::start_sim_cb,this);
			
			this->sub_myDetectedLitterNames = this->node->Subscribe("/robotDetectedLitterNames", &Nest_Plugin::cb_robotDetectedLitterNames, this);
			this->sub_myLitter_DB = this->node->Subscribe("/myLitter_DB",&Nest_Plugin::cb_myLitter_DB,this);
			this->sub_robot_status = this->node->Subscribe("/topic_robot_status",&Nest_Plugin::robot_status_cb,this);

			// Listen to the update event. This event is broadcast every
			// simulation iteration.
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(
				boost::bind(&Nest_Plugin::OnUpdate, this, _1));
		}

		public: std::string extractLitterIDs(std::string detectionStr)
		{
			
			std::string detectionsID = "";
			while(detectionStr.find(",") != std::string::npos)
			{
				std::size_t nameLoc = detectionStr.find(",");
				std::string litterName = detectionStr.substr(0,nameLoc);
				if (litterName.find("litter") != std::string::npos)
				{
					if (detectionsID != ""){
						detectionsID += ";";
					}
					detectionsID += litterName.substr(8);//ignore m_litter part of littername

				}
				detectionStr = detectionStr.substr(nameLoc + 1);
			}
			return detectionsID;
		}
		public: void cb_myLitter_DB(ConstAnyPtr &any)
		{
			std::lock_guard<std::mutex> lock(this->mutex);
			
			std::string detectionStr = any->string_value();

			std::size_t robotNameLoc = detectionStr.find(":");
			std::string robotName = detectionStr.substr(0,robotNameLoc);
			detectionStr = detectionStr.substr(robotNameLoc + 1);
			
			this->robots[robotName].litterCount = this->extractLitterIDs(detectionStr);
			// gzdbg << robotName << ":" << detectionStr << std::endl;
		}
		public: void cb_robotDetectedLitterNames(ConstAnyPtr &any)
		{
			std::lock_guard<std::mutex> lock(this->mutex);
			
			std::string detectionStr = any->string_value();

			std::size_t robotNameLoc = detectionStr.find(":");
			std::string robotName = detectionStr.substr(0,robotNameLoc);
			detectionStr = detectionStr.substr(robotNameLoc + 1);

			this->robots[robotName].seenLitter = this->extractLitterIDs(detectionStr);
			// gzerr << robotName << ":" << detectionStr << std::endl;
		}

		public: void robot_status_cb(ConstAnyPtr &any)
		{
			std::lock_guard<std::mutex> lock(this->mutex);
			
			std::string robot_status_msg = any->string_value();
			size_t n_loc = robot_status_msg.find(":");
			std::string model_name = robot_status_msg.substr(0,n_loc);
			std::string robot_status = robot_status_msg.substr(n_loc+1);
			
			this->robots[model_name].state = robot_status;
			
		}

		public: void my_Init(ConstAnyPtr &any)
		{//used to initialize/reset simulation parameters
			std::lock_guard<std::mutex> lock(this->mutex);
			std::string sim_params_mine = any->string_value();
			
			while(sim_params_mine.find(",") != std::string::npos)
			{
				std::size_t ploc = sim_params_mine.find(",");
				std::string temp = sim_params_mine.substr(0,ploc);
				
				std::size_t aloc = temp.find(":");
				std::string param_name = temp.substr(0,aloc);
				std::string param_value_str = temp.substr(aloc+1);
				// double value = std::stod(param_value_str);
				
				if(param_name.compare("log_rate") == 0)
				{
					this->log_rate = std::stod(param_value_str);;
				}
				else if(param_name.compare("max_step_size") == 0)
				{
					this->max_step_size = std::stod(param_value_str);;
				}
				else if(param_name.compare("logPrefix") == 0){
					// gzdbg << param_value_str;
					this->createFileNames(param_value_str);
					this->logDetails(true,0);
				}
				else
				{
				}
				sim_params_mine = sim_params_mine.substr(ploc+1);
			}
			
			this->log_timer = 0;//reset timer
		}
		
		public: void start_sim_cb(ConstAnyPtr &any)
		{//receives true when simulation is to start and false when simulation stops
			std::lock_guard<std::mutex> lock(this->mutex);
			this->start_sim = any->bool_value();
			if(this->start_sim)
			{//start sim is true, that means a new simulation has started.
				//Therefore, reset litter count.
				this->litter_count.clear();
				
			}
		}
		
		public: void Collect_Litter(ConstAnyPtr &any)
		{
			std::lock_guard<std::mutex> lock(this->mutex);
			msgs::Any a = *any;
			std::string litter_name = a.string_value();
			this->litter_count.push_back(litter_name);
			//msgs::Any b;
			//b.set_type(msgs::Any::STRING);
			//b.set_string_value(to_string(this->litter_count.size()));
			//this->pub->Publish(b);
			//Use this to add litter objects to the vector.
			//Remember what happened when picking litter with contact sensor
			//If names of litter appear multiple times make use of set instead of vector
			//i.e. change the declaration from vector to set container.
		}
		
		// Called by the world update start event
		public: void OnUpdate(const common::UpdateInfo & _info)
		{
			std::lock_guard<std::mutex> lock(this->mutex);
			this->log_timer += this->max_step_size;
			
			gazebo::common::Time st = _info.simTime;
			
			if(/*st.nsec==0 and this->start_sim)//or */(this->log_timer >= this->log_rate and this->start_sim))//rate of 100Hz
			{
				
				this->log_timer = 0;
				std::string log_litter_count(to_string(_info.simTime.Double()));
				log_litter_count += "," + to_string(this->litter_count.size());
				this->numLitter =  (int) this->litter_count.size();
				msgs::Any b;
				b.set_type(msgs::Any::STRING);
				b.set_string_value(log_litter_count);
				this->pub->Publish(b);
				this->logDetails(false,st.sec);
			}
			
			if(this->log_timer > this->log_rate)
			{
				this->log_timer = 0;
			}
		}
		
		
	};
	
	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(Nest_Plugin)
}
