void ModelVel::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
{
	
	
	this->model = _parent;
	this->ModelName = this->model->GetName();
	this->rwheel = this->model->GetJoint("R_joint");
	this->lwheel = this->model->GetJoint("L_joint");
	math::Box chassis = this->model->GetLink("chassis")->GetBoundingBox();
	//cout<<chassis.GetXLength()<<","<<chassis.GetYLength()<<","<<chassis.GetZLength()<<endl;
	this->chassis_diameter = chassis.GetXLength();//Gets the diameter of chassis. Needed for computing the rotational distance in litter_sensor.cc
	//this->slide_joint = this->model->GetJoint("slide_joint");
	this->world = this->model->GetWorld();
	
	
	
	
	
	//*************Litter picked tracking****************/
	
	
	//***************home messags palava*************/
	
	this->node = transport::NodePtr(new transport::Node());
	this->node->Init();
	this->pub_info = this->node->Advertise<msgs::Any>("/"+this->model->GetName()+"_info");
	
	//msgs::Any any;
	//any.set_type(msgs::Any::STRING);
	//any.set_string_value(this->model->GetName());
	//this->pub_info->Publish(any);
	//*****************************************?/
	//********** home exploration problem********/
	this->wheel_separation = this->rwheel->GetAnchor(0).Distance(
							this->lwheel->GetAnchor(0));
	
	
	
	//******Publish model name*********************//
	
	//this->pub_name = this->node->Advertise<msgs::Any>("/model_names");
	
	//this->send_name = true;
	//msgs::Any any;
	//any.set_type(msgs::Any::STRING);
	//any.set_string_value(this->model->GetName());
	//this->pub_name->Publish(any);
	//*********************************************//
	
	
	//Contact Sensor Palava
	this->ContactMsg = "/gazebo/default/"+ this->ModelName 
						+ "/chassis/chassis_contact/contacts";
	this->c_sub = this->node->Subscribe(this->ContactMsg,&ModelVel::ContactInfo,this);
	
	//****************Contact Sensor****************//
	
	//**********************************************//
	
	//***************Robot Status , Litter and neighbour sensing palava*******************//
	std::string sensor_topic = "/" + this->ModelName;
	//this->lit_nei_sub = this->node->Subscribe(sensor_topic,&ModelVel::LitterSensor,this);
	this->lit_nei_pub = this->node->Advertise<msgs::Any>(sensor_topic);
	this->pub_seen_litter = this->node->Advertise<msgs::Any>("/topic_seen_litter");
	this->pub_robot_status = this->node->Advertise<msgs::Any>("/topic_robot_status");
	//*********************************************************************//
	
	this->pub_litter = this->node->Advertise<msgs::Any>("/litter_deposits");
	this->pub_myDetectedLitterNames = this->node->Advertise<msgs::Any>("/robotDetectedLitterNames");
	this->pub_myDetectableLitters = this->node->Advertise<msgs::Any>("/robotDetectableLitterNames");
	this->pub_myLitter_DB = this->node->Advertise<msgs::Any>("/myLitter_DB");
	this->pub_robot_info = this->node->Advertise<custom_msgs::msgs::RobotInfo>("/robot_info");
	this->pub_end_experiment = this->node->Advertise<msgs::Any>("/end_experiment");

	this->litter_dump_site = gazebo::math::Pose(1000.0,1000.0, 0.0, 0.0, 0.0, 0.0);
	/*
	gazebo::physics::Model_V all_models = this->world->GetModels();
	for(auto m : all_models)
	{
		
		std::string model_name = m->GetName();
		if(model_name.find("m_4wrobot") != std::string::npos and this->ModelName.compare(model_name) != 0)
		{
			this->pub_repulsion[model_name] = this->node->Advertise<msgs::Any>("/"+model_name+"/comm_signal");
		}
		
		
	}*/
	this->sub_comm_signal = this->node->Subscribe("/"+this->ModelName+"/comm_signal",&ModelVel::CommSignal,this);
	//this->pub_comm_signal = this->node->Advertis<msgs::Any>("/communication_signal");
	
	
	//Color handling section
	this->pub_visual = this->node->Advertise<msgs::Visual>("/gazebo/default/visual");
	gazebo::physics::LinkPtr link = this->model->GetLink("chassis");
	
	//homing
	gazebo::common::Color homColor(0.0/255,150.0/255,0.0/255,1.0);
	this->homColMsg = new gazebo::msgs::Color(gazebo::msgs::Convert(homColor));
	this->homDiffMsg = new gazebo::msgs::Color(*(this->homColMsg));
	
	
	this->homvisMsg = link->GetVisualMessage("visual");
	this->hommaterialMsg = this->homvisMsg.mutable_material();
	this->hommaterialMsg->clear_ambient();
	this->hommaterialMsg->clear_diffuse();
	
	this->homvisMsg.set_name(link->GetScopedName()+"::visual");
	this->homvisMsg.set_parent_name(this->model->GetScopedName());
	
	this->hommaterialMsg->set_allocated_ambient(this->homColMsg);
	this->hommaterialMsg->set_allocated_diffuse(this->homDiffMsg);
	
	//searching
	// gazebo::common::Color srchColor(170.0/255,110.0/255,110.0/255,1.0);
	gazebo::common::Color srchColor(0.95,0.95,0.95,1.0);
	this->srchColMsg = new gazebo::msgs::Color(gazebo::msgs::Convert(srchColor));
	this->srchDiffMsg = new gazebo::msgs::Color(*(this->srchColMsg));
	
	
	this->srchvisMsg = link->GetVisualMessage("visual");
	this->srchmaterialMsg = this->srchvisMsg.mutable_material();
	this->srchmaterialMsg->clear_ambient();
	this->srchmaterialMsg->clear_diffuse();
	
	this->srchvisMsg.set_name(link->GetScopedName()+"::visual");
	this->srchvisMsg.set_parent_name(this->model->GetScopedName());
	
	this->srchmaterialMsg->set_allocated_ambient(this->srchColMsg);
	this->srchmaterialMsg->set_allocated_diffuse(this->srchDiffMsg);
	
	
	//go4litter
	gazebo::common::Color go4lColor(0.0/255,0.0/255,255.0/255,1.0);
	this->go4lColMsg = new gazebo::msgs::Color(gazebo::msgs::Convert(go4lColor));
	this->go4lDiffMsg = new gazebo::msgs::Color(*(this->go4lColMsg));
	
	
	this->go4lvisMsg = link->GetVisualMessage("visual");
	this->go4lmaterialMsg = this->go4lvisMsg.mutable_material();
	this->go4lmaterialMsg->clear_ambient();
	this->go4lmaterialMsg->clear_diffuse();
	
	this->go4lvisMsg.set_name(link->GetScopedName()+"::visual");
	this->go4lvisMsg.set_parent_name(this->model->GetScopedName());
	
	this->go4lmaterialMsg->set_allocated_ambient(this->go4lColMsg);
	this->go4lmaterialMsg->set_allocated_diffuse(this->go4lDiffMsg);
	
	//avoid obstacle
	gazebo::common::Color avoidColor(0.0,0,0,1.0);
	this->avoidColMsg = new gazebo::msgs::Color(gazebo::msgs::Convert(avoidColor));
	this->avoidDiffMsg = new gazebo::msgs::Color(*(this->avoidColMsg));
	
	
	this->avoidvisMsg = link->GetVisualMessage("visual");
	this->avoidmaterialMsg = this->avoidvisMsg.mutable_material();
	this->avoidmaterialMsg->clear_ambient();
	this->avoidmaterialMsg->clear_diffuse();
	
	this->avoidvisMsg.set_name(link->GetScopedName()+"::visual");
	this->avoidvisMsg.set_parent_name(this->model->GetScopedName());
	
	this->avoidmaterialMsg->set_allocated_ambient(this->avoidColMsg);
	this->avoidmaterialMsg->set_allocated_diffuse(this->avoidDiffMsg);

	//*********************START: TO HANDLE PARAMTER UPDATING AND SIMULATION RESETS
	this->start_sim = false;//It should be false at start in anticipation of simulation parameters.
	this->sub_start_sim = this->node->Subscribe("/start_sim",&ModelVel::start_sim_cb,this);
	this->sub_my_Init = this->node->Subscribe("/my_Init",&ModelVel::my_Init,this);
	//---------------------END: TO HANDLE PARAMETER UPDATING AND SIMULATION RESETS
	//this->my_Init();
	
	
	
	this->pub_log = this->node->Advertise<msgs::Any>("/topic_model_log");
	
	// Listen to the update event. This event is broadcast every
	// Simulation iteration.
	this->updateConnection = event::Events::ConnectWorldUpdateBegin(
		boost::bind(&ModelVel::OnUpdate, this,_1));
		
	//gazebo::client::shutdown;//Are you sure you want this here? Probably disable it.
}
void ModelVel::start_sim_cb(ConstAnyPtr &any)
{//The value usually gotten from the world plugin on this topic is usually false
	std::lock_guard<std::mutex> lock(this->mutex);
	this->start_sim = any->bool_value();
}
void ModelVel::my_Init(ConstAnyPtr &any)
{
	std::lock_guard<std::mutex> lock(this->mutex);
	this->new_comm_signal = false;//At start, there is no communicated information
	//this->model->Reset();
	//*****************START: PARAMTERS THAT VARY BETWEEN ITERATIONS*************************
	std::string sim_params_mine = any->string_value();
	double seed = rand();
	while(sim_params_mine.find(",") != std::string::npos)
	{//loop through all parameters and assign to appropriate variable in plugin
		std::size_t ploc = sim_params_mine.find(",");
		std::string temp = sim_params_mine.substr(0,ploc);
		
		std::size_t aloc = temp.find(":");
		std::string param_name = temp.substr(0,aloc);
		std::string param_value_str = temp.substr(aloc+1);
		// double value = std::stod(param_value_str);//change string to double
		
		if(param_name.compare("nei_sensing")==0)
		{
			this->nei_sensing = std::stod(param_value_str);;
		}
		else if(param_name.compare("lit_sensing")==0)
		{
			this->lit_sensing = std::stod(param_value_str);;
		}
		else if (param_name.compare("FoV") == 0)
		{
			this->halffov = std::stod(param_value_str)/2.0;
		}
		else if(param_name.compare("rVel")==0)
		{
			this->rVel = std::stod(param_value_str);;
		}
		else if(param_name.compare("turn_prob")==0)
		{
			this->turn_prob = std::stod(param_value_str);;
		}
		else if(param_name.compare("abandon_prob")==0)
		{
			this->abandon_prob = std::stod(param_value_str);;
		}
		else if(param_name.compare("capacity")==0)
		{
			this->capacity = std::stod(param_value_str);;
		}
		else if(param_name.compare("umin")==0)
		{
			this->umin = std::stod(param_value_str);;
		}
		else if(param_name.compare("umax")==0)
		{
			this->umax = std::stod(param_value_str);;
		}
		else if(param_name.compare("n_stddev")==0)
		{
			this->n_stddev = std::stod(param_value_str);;
		}
		else if(param_name.compare("n_mean")==0)
		{
			this->n_mean = std::stod(param_value_str);;
		}
		else if(param_name.compare("escape_dist")==0)
		{
			this->escape_dist = std::stod(param_value_str);;
		}
		else if(param_name.compare("call_scale_mult") == 0)
		{
			this->call_scale_mult = std::stod(param_value_str);;
		}else if(param_name.compare("call_scale_div") == 0)
		{
			this->call_scale_div = std::stod(param_value_str);;
		}
		else if(param_name.compare("repel_scale_mult") == 0)
		{
			this->repel_scale_mult = std::stod(param_value_str);;
		}
		
		else if(param_name.compare("repel_scale_div") == 0)
		{
			this->repel_scale_div = std::stod(param_value_str);;
		}
		else if(param_name.compare("turn_prob_max") == 0)
		{
			this->turn_prob_max = std::stod(param_value_str);;
		}
		else if(param_name.compare("turn_prob_min") == 0)
		{
			this->turn_prob_min = std::stod(param_value_str);;
		}
		else if(param_name.compare("max_step_size") == 0)
		{
			this->max_step_size = std::stod(param_value_str);;
		}
		else if(param_name.compare("log_rate") == 0)
		{
			this->log_rate = std::stod(param_value_str);;
		}
		else if(param_name.compare("picking_lit_dur") == 0)
		{
			this->picking_lit_dur = std::stod(param_value_str);;
		}
		else if(param_name.compare("queue_size") == 0)
		{
			this->queue_size = std::stod(param_value_str);
		}
		else if(param_name.compare("correction_mtd") == 0)
		{
			this->correction_mtd = param_value_str;
		}
		else if(param_name.compare(this->model->GetName()) == 0)
		{
			seed = std::stod(param_value_str);;
		}
		else if(param_name.compare("filter_type") == 0)
		{
			this->filter_type = param_value_str;
		}
		else if(param_name.compare("p_s2s") == 0)
		{
			this->p_s2s = std::stod(param_value_str);	
		}
		else if(param_name.compare("p_u2s") == 0)
		{
			this->p_u2s = std::stod(param_value_str);
		}
		else if (param_name.compare("detectionDuration") == 0)
		{
			this->detectionDuration = std::stod(param_value_str);
		}
		else if(param_name.compare("logPrefix") == 0){
			this->log_filename = param_value_str + "_" + this->ModelName + ".csv";
			ofstream myfile(this->log_filename,std::ios::app|std::ios::ate);
			myfile << "time,turncount,total_distance,forward_distance,forward_duration,"
				<<"reverse_distance,reverse_duration,attract_steps,repel_steps,"
				<<"turn_prob,num_aa,num_ar,num_ra,num_rr\n";
			myfile.close();
			
		}
		else
		{
			//std::cout<<temp<<"::"<<value<<std::endl;
		}
		sim_params_mine = sim_params_mine.substr(ploc+1);
	}
	//litter processing variables
	this->picking_lit_wait = false;
	this->pick_lit_time = 0;
	
	//Reset litter capacity
	this->litter_db.clear();
	this->acTion = "Start Sim";
	this->Kp = 10*this->rVel;
	this->no_litter = false;
	//*****************************************/
	
	//***********Explore environment with random turn
	this->turn_complete = true;
	this->turn_set = false;
	//***********************************************
	this->crashed = false;
	this->seen_litter = 0;//count of litters
	this->rep_neighbours = 0;//count of neighbours
	this->call_neighbours = 0;
	this->litter_info = "";
	this->neighbours_info = "";
	this->litter_pos.z=-9000.1;//initialize closest litter at unreasonable distance in z direction
	this->LitterName = "";
	this->pick_litter = false;
	this->litter_count = 0;
	this->previousVisionTime = 0; //initialize time previous vision system was executed
	this->go_home = false;
	this->at_home = false;
	this->waiting_t = 0;
	
	this->my_pose = this->model->GetWorldPose();
	this->stop_pose = this->model->GetWorldPose();
	
	this->turn_amt = math::Angle(0.0);
	
	this->d_heading = this->my_pose.rot.GetYaw();
	
	this->escape = -1.0;//escape is needed to break out of traps due to interference when going home or picking litter
	this->escape_start = gazebo::math::Vector3(-1.0,-1.0,-1.0);
	
	//********************commModel to replace this?****************//
	//reset cummulative sum of the communication signals received
	this->repel_signal = 0.0;
	this->prev_repel_signal = this->repel_signal;
	
	this->call_signal = 0.0;
	this->prev_call_signal = this->call_signal;
	//************create commModel*************************************
	this->commModel = CommModels(this->queue_size,this->filter_type);
	//*****************************************************************
	this->rslt_theta = 0.0;
	
	
	//Color handling sections
	this->state = "searching";//homing,go4litter
	this->prev_state ="";
	
	//RANDOM NUMBERS PALAVA. BOTH UNIFORM AND NORMAL(GAUSSIAN) DISTRIBUTIONS
	
	this->generator = std::default_random_engine(seed);
	this->uform_rand = std::uniform_real_distribution<double>(this->umin,this->umax);
	this->uform_rand_heading = std::uniform_real_distribution<double>(-M_PI,M_PI);//uniform turning between -180 and +180
	
	//Normal distribution to control heading from 0 to 360.
	this->normal_rand_heading = std::normal_distribution<double>(this->n_mean,this->n_stddev);
	
	this->log_timer = 0;//reset log timer at start of every simulation
	//set start sim value to true when all parameters have been initialized
	//this->start_sim = true;
	
	this->prev_loc = this->my_pose.pos;
	this->prev_yaw = this->my_pose.rot.GetYaw();
	this->linear_dist = 0.0;
	this->rot_dist = 0.0;

	//reset litter collected and deposited data
	this->litter_collected = 0;
	this->litter_deposited = 0;

	//reset wall bounces
	this->wall_bounces = 0;
	this->neighbour_bounces = 0;//reset collision with other robots

	

	this->timeStamp = 0;//initialize time stamp to zero

	this->prev_seen.clear();//reset seen litter

	//****** populate litter database***//
	for (auto m : this->world->GetModels())
	{
		std::string m_name = m->GetName();
		if (m_name.find("litter") != std::string::npos)
		{
			this->myLitterDB.push_back(m);
		}
	}

	//*********************************************************************************//
	this->detectedLitterNames = "";//initially no litters were detected

	//starting position
	this->my_pose = this->model->GetWorldPose();
	this->stop_pose = gazebo::math::Pose(this->my_pose);
	this->turn_count = 0;
	this->total_travel_distance = 0;
	this->forward_distance = 0, this->forward_duration = 0;
	this->reverse_distance = 0, this->reverse_duration = 0;
	this->signal.push_back(Signal());
	this->attract_steps = 0; this->repel_steps = 0;
	
	this->num_aa = 0;
	this->num_ar = 0;
	this->num_ra = 0;
	this->num_rr = 0;
}
		
void ModelVel::OnUpdate(const common::UpdateInfo & _info)
{
	
	std::lock_guard<std::mutex> lock(this->mutex);
	//std::cout<<_info.simTime.Double()<<":"<<_info.simTime.sec<<","<<_info.simTime.nsec<<std::endl;
	this->timeStamp = _info.simTime.Double();//update timeStamp

	
	
	if(this->state.compare(this->prev_state) != 0)
	{
		msgs::Any any;
		any.set_type(msgs::Any::STRING);
		if(this->state.compare("go4litter")==0){
			this->pub_visual->Publish(this->go4lvisMsg);
			any.set_string_value(this->ModelName+":go4litter");
		}
		else if(this->state.compare("homing")==0){
			this->pub_visual->Publish(this->homvisMsg);
			any.set_string_value(this->ModelName+":homing");
		}
		else{//in searching state
			this->pub_visual->Publish(this->srchvisMsg);
			any.set_string_value(this->ModelName+":searching");
			
		}
		
		this->pub_info->Publish(any);
		this->pub_robot_status->Publish(any);
	}
	
	this->prev_state = this->state;//set previous state value to state in last iteration
	
	
	if(this->start_sim)
	{
		this->previousVisionTime += this->max_step_size;
		this->log_timer += this->max_step_size;
		
		double alevel = 0,rlevel = 0;
		for (int sig = 0; sig < this->signal.size(); sig++)
		{
			if(this->previousVisionTime >= this->detectionDuration)//modify to be based on detection update rate
			{
				this->previousVisionTime = 0;
				double probability = this->p_u2s;
				if (this->signal[sig].type.compare("attract") == 0)//use p_s2s
				{
					probability = this->p_s2s;
				}

				if(this->uform_rand(this->generator) < probability)
				{
					this->signal[sig].update_type("attract");
				}
				else
				{
					this->signal[sig].update_type("repel");
				}
			}

			//this->signal[sig].update_type("attract");
			this->signal[sig].update_level(this->rVel);
			if (this->signal[sig].type.compare("attract") == 0)
			{
				alevel += this->signal[sig].level;
				this->attract_steps += 1;
			}
			if (this->signal[sig].type.compare("repel") == 0)
			{
				rlevel += this->signal[sig].level;
				this->repel_steps += 1;
			}
			if ((this->signal[sig].prev_type.compare("attract") == 0) and (this->signal[sig].type.compare("attract") == 0))
			{
				this->num_aa += 1;
			}
			else if ((this->signal[sig].prev_type.compare("attract") == 0) and (this->signal[sig].type.compare("repel") == 0))
			{
				this->num_ar += 1;
			}
			else if ((this->signal[sig].prev_type.compare("repel") == 0) and (this->signal[sig].type.compare("attract") == 0))
			{
				this->num_ra += 1;
			}
			else if ((this->signal[sig].prev_type.compare("repel") == 0) and (this->signal[sig].type.compare("repel") == 0))
			{
				this->num_rr += 1;
			}

		}
		this->repel_signal = rlevel;
		this->call_signal = alevel;
		
		
		//update repulsion and attraction signals
		// this->repel_signal = this->commModel.get_value("curr_repel_signal");
		// this->prev_repel_signal = this->commModel.get_value("prev_repel_signal");
		// this->call_signal = this->commModel.get_value("curr_call_signal");
		// this->prev_call_signal = this->commModel.get_value("prev_call_signal");


		//start: modify turn probability based on comm signal
		//this section handles repulsion signals
		// if(this->acTion.compare("straight") == 0)
		// {
			// this->new_comm_signal = false;//turn to false and wait till there is new signal.
			this->turn_prob = this->turn_prob_min;

			if(this->repel_signal < this->prev_repel_signal)
			{
				//this->turn_prob -= this->repel_scale;
				this->turn_prob = this->turn_prob_min / this->repel_scale_div;
			}
			else if (this->repel_signal > this->prev_repel_signal)
			{
				this->turn_prob = this->turn_prob_min * this->repel_scale_mult;
			}
			
		//This section handles attraction signals
			
			if(this->call_signal < this->prev_call_signal)
			{
				this->turn_prob = this->turn_prob * this->call_scale_mult;
			}
			else if(this->call_signal > this->prev_call_signal)
			{
				this->turn_prob = this->turn_prob / this->call_scale_div;
			}
		// }
			this->prev_call_signal = this->call_signal;
			this->prev_repel_signal = this->repel_signal;

			if(this->turn_prob >= this->turn_prob_max)
			{
				this->turn_prob = this->turn_prob_max;
			}
		
		
		if (this->rVel > 0)
		{
			this->forward_distance += this->dxy(this->my_pose.pos,this->model->GetWorldPose().pos);
			this->forward_duration += this->max_step_size;
		}
		else if (this->rVel < 0)
		{
			this->reverse_distance += this->dxy(this->my_pose.pos,this->model->GetWorldPose().pos);
			this->reverse_duration += this->max_step_size;
		}
		
		this->total_travel_distance += this->dxy(this->my_pose.pos,this->model->GetWorldPose().pos);
		
		if (this->timeStamp > 5000) {
			//log data
			ofstream myfile(this->log_filename,std::ios::app|std::ios::ate);
			myfile << this->timeStamp << ","  << this->turn_count << ","
					<< this->total_travel_distance << "," 
					<< this->forward_distance << "," << this->forward_duration << "," 
					 << this->reverse_distance << "," << this->reverse_duration << ","
					 << this->attract_steps << "," << this->repel_steps << "," 
					 << this->turn_prob << "," << this->num_aa<< "," << this->num_ar
					 << "," << this->num_ra<< "," << this->num_rr << "\n";
			myfile.close();
			this->turn_count = 0;
			this->total_travel_distance = 0;
			this->forward_distance = 0;
			this->forward_duration = 0;
			this->reverse_distance = 0;
			this->reverse_duration = 0;
			this->attract_steps = 0;
			this->repel_steps = 0;
			this->num_aa = 0;
			this->num_ar = 0;
			this->num_ra = 0;
			this->num_rr = 0;
			//end experiment
			msgs::Any any;
			any.set_type(msgs::Any::BOOLEAN);
			any.set_bool_value(true);
			this->pub_end_experiment->Publish(any);
		}

		
		this->my_pose = this->model->GetWorldPose();	
		if((this->uform_rand(this->generator) < this->turn_prob))
		{//Make a random turn
			//log data before changing direction
			this->turn_count += 1;
			std::cerr << this->timeStamp <<"," << this->turn_count << ": distance = " << this->total_travel_distance
					<< " velocity = " << this->rVel << std::endl;

			//When making random turn, reset turn_prob
			
			this->rVel *= -1;
			this->waiting_t = 0;
			//reset time or duration of travel
			this->log_timer = 0;

			//log data
			ofstream myfile(this->log_filename,std::ios::app|std::ios::ate);
			myfile << this->timeStamp << ","  << this->turn_count << ","
					<< this->total_travel_distance << "," 
					<< this->forward_distance << "," << this->forward_duration << "," 
					 << this->reverse_distance << "," << this->reverse_duration << ","
					 << this->attract_steps << "," << this->repel_steps << "," 
					 << this->turn_prob << "," << this->num_aa<< "," << this->num_ar
					 << "," << this->num_ra<< "," << this->num_rr << "\n";
			myfile.close();
			this->turn_count = 0;
			this->total_travel_distance = 0;
			this->forward_distance = 0;
			this->forward_duration = 0;
			this->reverse_distance = 0;
			this->reverse_duration = 0;
			this->attract_steps = 0;
			this->repel_steps = 0;
			this->num_aa = 0;
			this->num_ar = 0;
			this->num_ra = 0;
			this->num_rr = 0;

		}
		gazebo::math::Pose p = this->model->GetWorldPose();
		if(this->rVel > 0)
		{
			p.pos.x += 0.00025;
		}
		else
		{
			p.pos.x -= 0.00025;
		}
		this->model->SetWorldPose(p);
		
		
		
	}
	else
	{//if not start sim, the robot should be stationary.
		this->stop();
	}
}

