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
		else if(param_name.compare("att_threshold") == 0)
		{
			this->att_threshold = std::stod(param_value_str);
		}
		else if(param_name.compare(this->model->GetName()) == 0)
		{
			seed = std::stod(param_value_str);;
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
	//int seed = rand();// std::stoi((this->model->GetName()).substr(9));
	//std::cout<<seed<<endl;
	/*this->nei_sensing = 5;//neighbour sensirng range
	this->lit_sensing = 2;//litter sensing range
	this->rVel  = 5; //robot velocity value represents angular velocity of wheel joint
	this->turn_prob = 0.0001;//probability of making a turn during random walk
	this->abandon_prob = 0.5;//probability of abandoning a litter when blocked by another robot
	this->capacity = 3; 									//Max litter_count to pick================3,5
	this->umin = 0.0;
	this->umax = 1.0;//minimum and maximum for uniform distribution
	// Normalize value within code
	this->n_stddev = M_PI/2.0;//stamdard deviation of 90degs
	this->n_mean = M_PI;//mean of 180 degs
	 this->escape_dist = 1;*/
	//----------------END: PARAMTERS THAT VARY BETWEEN DIFFERENT SIMULATIONS----------
	
	
	//this->com_sig_set.clear();
	//this->cummulative_sig = 0;
	
	//Reset litter capacity
	this->litter_db.clear();
	this->acTion = "Start Sim";
	this->Kp = 10*this->rVel;
	this->no_litter = false;
	//*****************************************/
	//******dropping litter handlers***********/
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
	
	this->go_home = false;
	this->at_home = false;
	this->waiting_t = 0;
	
	this->my_pose = this->model->GetWorldPose();
	this->turn_amt = math::Angle(0.0);
	
	this->d_heading = this->my_pose.rot.GetYaw();
	
	this->escape = -1.0;//escape is needed to break out of traps due to interference when going home or picking litter
	this->escape_start = gazebo::math::Vector3(-1.0,-1.0,-1.0);
	
	this->repel_signal = 0.0;//cummulative sum of the communication signals received
	this->prev_repel_signal = this->repel_signal;
	
	this->call_signal = 0.0;
	this->prev_call_signal = this->call_signal;
	
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

	//reset activity times used for logging
	this->t_obstacle_avoidance = 0;
	this->t_litter_processing = 0;
	this->t_go4litter = 0;
	this->t_oa_go4litter = 0;
	this->t_searching = 0;
	this->t_oa_searching = 0;
	this->t_homing = 0;
	this->t_oa_homing = 0;

	//logged data header
	std::string myHeader = this->model->GetName() + ":";
	myHeader = myHeader + "name:t,xc,yc,theta,turn_prob,seen_litter," +
					"rep_neigh,rep_signal,call_neigh,call_signal," +
					"litter_db,linear_dist,rot_dist,litter_collected," +
					"litter_deposited,wall_bounces,neighbour_bounces," +
					"t_obstacle_avoidance,t_searching,t_oa_searching,t_go4litter," +
					"t_oa_go4litter,t_litter_processing,t_homing," +
					"t_oa_homing,action:state";
	msgs::Any b;
	b.set_type(msgs::Any::STRING);
	b.set_string_value(myHeader);
	this->pub_log->Publish(b);
}
		
void ModelVel::OnUpdate(const common::UpdateInfo & _info)
{
	
	std::lock_guard<std::mutex> lock(this->mutex);
	//std::cout<<_info.simTime.Double()<<":"<<_info.simTime.sec<<","<<_info.simTime.nsec<<std::endl;
	
	this->log_timer += this->max_step_size;
	
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
	this->state = "searching";//the default state is searching
	
	if(this->start_sim)
	{
		
		//start: modify turn probability based on comm signal
		//this section handles repulsion signals
		if(this->turn_complete and this->new_comm_signal)
		{//change turning probability and update communication signal only when not turning i.e. only when moving straight
			//also, do this only when there is new communcated information from neighbours

			this->new_comm_signal = false;//turn to false and wait till there is new signal.
			
			if(this->rep_neighbours > 0)
			{
				if(this->prev_repel_signal > this->repel_signal)
				{
					//this->turn_prob -= this->repel_scale;
					this->turn_prob = this->turn_prob_min / this->repel_scale_div;
				}
				else if (this->prev_repel_signal < this->repel_signal)
				{
					this->turn_prob = this->turn_prob_min * this->repel_scale_mult;
				}
				else
				{
					this->turn_prob = this->turn_prob_min;
				}
			}
			else
			{
				this->turn_prob = this->turn_prob_min;
			}
			
			//This section handles attraction signals
			if (this->call_signal <= this->att_threshold)
			{
				if(this->prev_call_signal > this->call_signal)
				{
					this->turn_prob = this->turn_prob * this->call_scale_mult;
				}
				else if(this->prev_call_signal < this->call_signal)
				{
					this->turn_prob = this->turn_prob / this->call_scale_div;
				}
				
				else
				{
					this->turn_prob = this->turn_prob;
				}
			}
			else
			{
				this->turn_prob = this->turn_prob;
			}
			if(this->turn_prob >= this->turn_prob_max)
			{
				this->turn_prob = this->turn_prob_max;
			}
			this->prev_repel_signal = this->repel_signal;
			this->prev_call_signal = this->call_signal;
		}
		else{
			if(this->correction_mtd.compare("prob_reset") == 0 and this->acTion.compare("turning") == 0)
			{//Robot does not have new communication signal and correction method is prob_reset. Reset probability if robot action is turning.
				this->turn_prob = this->turn_prob_min;
			}
		}
		
		/*if(this->turn_prob <= this->turn_prob_min)
		{
			this->turn_prob = this->turn_prob_min;
		}*/
		
		//end: modify turn probability based on comm signal
		
		//msgs::Color *homColMsg;
		////msgs::Color *homDiffMsg;
		////msgs::Visual visMsg;
		////msgs::Material *materialMsg;
		/*gazebo::common::Color newColor(0.0,1.0,0.0,1.0);
		msgs::Color *homColMsg = new gazebo::msgs::Color(gazebo::msgs::Convert(newColor));
		msgs::Color *homDiffMsg = new gazebo::msgs::Color(*homColMsg);
		gazebo::physics::LinkPtr link = this->model->GetLink("chassis");
		
		
		
		msgs::Visual visMsg = link->GetVisualMessage("visual");
		msgs::Material *materialMsg = visMsg.mutable_material();
		materialMsg->clear_ambient();
		materialMsg->clear_diffuse();
		
		visMsg.set_name(link->GetScopedName()+"::visual");
		visMsg.set_parent_name(this->model->GetScopedName());
		
		materialMsg->set_allocated_ambient(homColMsg);
		materialMsg->set_allocated_diffuse(homDiffMsg);
		*/
		//this->pub_visual->Publish(visMsg);
		
		
		//cout<<this->ModelName<<" running"<<endl;
		this->my_pose = this->model->GetWorldPose();
		if(this->escape > 0 && this->dxy(this->my_pose.pos,this->escape_start) >= this->escape)
		{//If desired escape distance has been reached/exceeded, set escape to invalide input
			this->escape = -1.0;
		}
		
		this->LitterSensor();
		if(this->no_litter and not this->litter_db.empty())
		{
			this->go_home = true;
		}
		//static bool xx = true;
		/*if(_info.simTime.nsec == 0)
		{
			cout<<this->ContactMsg<<endl;
		}*/
		if(false && _info.simTime.nsec == 0)//this->send_name)
		{
			//cout<<this->acTion<<endl;
			//msgs::Any any;
			//any.set_type(msgs::Any::STRING);
			//any.set_string_value(this->acTion);
			
			//this->pub_info->Publish(any);
			//any.set_string_value(this->neighbours_info);
			//this->lit_nei_pub->Publish(any);
		}	
		
		if(this->turn_complete and this->litter_db.size() < this->capacity and this->seen_litter <= 0)
		{//If not in the middle of turning, litter storage not full and not seeing any litter
			//double x = rand()/double(RAND_MAX);
			//std::uniform_real_distribution<double> my_rand(0.0,1.0);
			
			
			//cout<<std::scientific;
			//cout<<x<<'\t'<<turn_prob<<endl;
			if(this->uform_rand(this->generator) < this->turn_prob)
			{//Make a random turn
				//When making random turn, reset turn_prob
				//this->turn_prob = this->turn_prob_min;
				//std::uniform_real_distribution<double> my_rand_heading(-M_PI,M_PI);
				//std::normal_distribution<double>my_rand_heading(mean,stddev);
				
				//Using uniform distribution
				//this->d_heading = this->uform_rand_heading(this->generator);
				
				//Using normal distribution
				
				double temp_heading = this->normal_rand_heading(this->generator);
				if(temp_heading > 2*M_PI) temp_heading = 2*M_PI;//cap to 360degs
				
				if(temp_heading < 0.0) temp_heading = 0.0;//cap to 0degs
				
				temp_heading = this->normalize(temp_heading);
				//this->d_heading = temp_heading;/**/
				this->d_heading = this->my_pose.rot.GetYaw() + temp_heading;
				this->d_heading = this->normalize(this->d_heading);
				//this->d_heading = (rand()/double(RAND_MAX) * 2 - 1) * M_PI;
				//string ss ="random turn: " + to_string(this->d_heading);
				//any.set_string_value(ss);
				//this->pub_info->Publish(any);
				this->turn_complete = false;
				
				this->waiting_t = 0;//reset waiting time.
			}
				
				
				//cout<<"random turn: "<<this->d_heading<<endl;
				
			
			/*
			msgs::Pose  p = msgs::Convert(this->model->GetWorldPose().Ign());
			p.set_name(this->model->GetName());
			msgs::Any any;
			
			//msgs::Any any = msgs::ConvertAny(this->model->GetWorldPose().Ign());
			any.set_type(msgs::Any::POSE3D);
			any.mutable_pose3d_value()->CopyFrom(p);
			//any.set_type(msgs::Any::STRING);
			//any.set_string_value(this->model->GetName());
			this->pub_name->Publish(any);
			this->send_name = false;
			*/
			
		}
		
		if(this->rep_neighbours > 0 and false)
		{
			this->d_heading = this->normalize(this->rslt_theta);
			//cout<<this->ModelName<<":"<<this->rslt_theta<<endl;
			//string x;
			//cin>>x;
		}
		
		if(this->seen_litter > 0 && this->litter_pos.z > -100 and this->litter_db.size() < this->capacity)
		{//if litter seen is greater than 0 and the location is valid and there is space for more litter
			//also if not avoiding obstacles and not this->crashed
			double theta = this->my_pose.rot.GetYaw();
			double xc = this->my_pose.pos.x;
			double yc = this->my_pose.pos.y;
			double x = this->litter_pos.x;
			double y = this->litter_pos.y;
			
			double litter_loc = atan2(y-yc,x-xc);//angle of line joining litter and robot center wrt world frame
			//double d_theta = theta - obstacle_loc;//difference between direction and litter
			if(this->escape < 0)
			{//If robot sees litter, and it is not trying to escape a trapped location.
				//change heading toward closest litter
				this->d_heading = this->normalize(litter_loc);
			}
			this->state = "go4litter";	
		}
		
		
		
		if(this->go_home)
		{//if litter capacity is full, go home.
			double theta = this->my_pose.rot.GetYaw();
			double xc = this->my_pose.pos.x;
			double yc = this->my_pose.pos.y;
			double x = 0;
			double y = 0;
			
			double home_loc = atan2(y-yc,x-xc);//angle of line joining home and robot center wrt world frame
			//double d_theta = theta - obstacle_loc;//difference between direction and home
			if(this->escape < 0)
			{//If robot is going home, and it is not trying to escape a trapped location.
				//change heading toward home
				this->d_heading = this->normalize(home_loc);
			}
			if(this->at_home and not this->litter_db.empty())
			{// robot is at home and yet to finish depositing all litter.
				//Deposit litter.
				this->deposit_litter();
			}
			if(this->litter_db.empty())
			{
				this->go_home = false;
				this->at_home = false;
			}
			this->state = "homing";
			
		}
		
		
		
		if(this->turn_complete && this->crashed)
		{//If robot has finished turning, but has crashed into an obstacle
			double x = this->uform_rand(this->generator); //or rand()/double(RAND_MAX);
			
			
			double turn = this->turn_amt.Radian();//Amount of turn returned by the obstacle avoidance behaviour
			if(this->seen_litter > 0 && x <= this->abandon_prob and not this->go_home)
			{//If crash occurs during moving toward a litter and probability of abandoning litter is met
				//compute distance to travel straight
				//std::cout<<"abandon litter"<<endl;
				this->escape = this->uform_rand(this->generator) * this->escape_dist;//or rand()/double(RAND_MAX);//move a certain distance g <=1m
				this->escape_start = this->my_pose.pos;//pose where robot got stuck.
			}
			if(not this->go_home && this->seen_litter > 0 && x > abandon_prob)
			{//If not full, seeing litter and not abandoning litter due to crash
				turn = 0;
			}
			
			
			//When going home i.e. capacity full.
			if(this->go_home and not this->at_home)
			{//when capacity is full and crashed, definitely avoid obstacle.
				this->escape = this->uform_rand(this->generator)  * this->escape_dist;//or rand()/double(RAND_MAX);
				this->escape_start = this->my_pose.pos;
				turn = this->turn_amt.Radian();
			}
			
			
			this->d_heading = this->my_pose.rot.GetYaw() + turn;
			this->d_heading = this->normalize(this->d_heading);
			this->crashed = false;
			this->turn_complete = false;
			//this->waiting_t = 0;//reset waiting time.
			//cout<<this->turn_complete<<" " <<this->normalize(this->d_heading - this->my_pose.rot.GetYaw())<<" Crashed, turn_amt: "<<this->turn_amt.Radian()/M_PI*180<<endl;
		}
		
		
		double dxn_eror = this->d_heading - this->my_pose.rot.GetYaw();
		dxn_eror = this->normalize(dxn_eror);
		
		if(this->pick_litter and not this->picking_lit_wait  and abs(dxn_eror) < M_PI/6.0)
		{//pick litter that is within 30 degrees of robot yaw.
			//cout<<" pick: "<<this->LitterName<<endl;
			if(this->try_pick(this->LitterName))
			{
				double dump_x = rand() % 1000 + 1000;
				double dump_y = rand() % 1000 + 1000;
				gazebo::math::Pose dump_site = gazebo::math::Pose(dump_x,dump_y, 0.0, 0.0, 0.0, 0.0);
				//transport::requestNoReply(this->node,"entity_delete",this->LitterName);
				physics::ModelPtr l = this->world->GetModel(this->LitterName);
				//l->SetWorldPose(this->litter_dump_site);
				
				l->SetWorldPose(dump_site);

		//**************************************************************************************************//
				this->deposit_litter();//essentially empties what has been picked up, making robot capacity unlimited if activated
		//**************************************************************************************************//
				//l->Fini();
				//this->world->RemoveModel(this->LitterName);
				this->LitterName = "";
				if(this->litter_db.size() >= this->capacity)
				{//If litter capacity is full, activate go home behaviour
					this->go_home = true;
				}
				
			}
			this->pick_litter = false;
			this->picking_lit_wait = true;
			this->pick_lit_time = _info.simTime.Double();
		}
		
		
		
		//cout<<"turn_complete: "<<this->turn_complete<<" d_heading: "<<this->d_heading<<" curr_h: "<<this->my_pose.rot.GetYaw()<<" dxn_eror: "<<dxn_eror<<endl;
		if(this->turn_complete and abs(dxn_eror) < 0.09)
		{
	////////////////////////////try and handle home avoidance when searching for litter here
	///////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////
			//cout<<this->acTion<<endl;
			this->waiting_t = 0;//reset waiting time.
			
			double home_distance = this->dxy(this->my_pose.pos,gazebo::math::Vector3(0.0,0.0,0.0));
			if(home_distance < 1.0 and this->go_home)
			{//stop when at home and deposit litter
				this->stop();
				this->acTion = "stop";
				this->at_home = true;
			}
			else
			{
				//Move straight
				this->rotate_v2(dxn_eror);
				//cout<<abs(dxn_eror)<<" "<<this->turn_complete<<endl;
				//this->straight();
				this->acTion = "straight";
				
			}
		}
		else
		{
			//Continue Turning
			if(this->waiting_t <= 10)
			{//stop for a while before turning.
				this->stop();
				this->waiting_t +=1;
				//cout<<"wait"<<endl;
			}
			else
			{
				this->rotate(dxn_eror);
				//cout<<"turning "<<dxn_eror<<endl;
			}
			
			if(abs(dxn_eror) < 0.09)
			{//if within 0.09 (approx 5deg, stop turning)
				this->turn_complete = true;
				this->crashed=false;
				this->waiting_t = 0;//reset waiting time.
				//cout<<dxn_eror<<endl;
			}
			this->acTion = "turning";
			//cout<<this->acTion<<" "<<dxn_eror<<endl;
		}
		
		
		if(this->picking_lit_wait and (_info.simTime.Double() - this->pick_lit_time) < this->picking_lit_dur)
		{// robot is processing litter. So stop
			this->t_litter_processing += this->max_step_size;
			this->stop();
		}
		else
		{
			this->picking_lit_wait = false;
		}
		
		//Process activity times needed for logging
		// this->escape
		if(this->escape >= 0){
			//robot is in obstacle avoidance mode
			this->t_obstacle_avoidance += this->max_step_size;
		}
		if(this->state.compare("searching") == 0){
			//robot in searching state
			this->t_searching += this->max_step_size;
			if(this->escape >= 0){//avoiding obstacle in current state
				this->t_oa_searching += this->max_step_size;
			}
		}
		if(this->state.compare("go4litter") == 0){
			//robot in go4litter state
			this->t_go4litter += this->max_step_size;
			if(this->escape >= 0){//avoiding obstacle in current state
				this->t_oa_go4litter += this->max_step_size;
			}
		}
		if(this->state.compare("homing") == 0){
			//robot in homing state
			this->t_homing += this->max_step_size;
			if(this->escape >= 0){//avoiding obstacle in current state
				this->t_oa_homing += this->max_step_size;
			}
		}
		//compute linear and rotation distance travelled cummulatively.
		this->linear_dist += this->dxy(this->prev_loc,this->my_pose.pos);
		this->prev_loc = this->my_pose.pos;
		
		this->rot_dist += abs((this->prev_yaw - this->my_pose.rot.GetYaw())/2.0 * this->chassis_diameter);
		this->prev_yaw = this->my_pose.rot.GetYaw();
		
		if(this->log_timer >= this->log_rate)//rate of 100Hz
		{
			this->log_timer = 0;
			//log:x,y,yaw,turn_prob,seen_litter,neighbours,comm_signal,this->litter_db,state
			std::string my_log_data;
			double theta = fmod(this->my_pose.rot.GetYaw() * 180.0/M_PI + 360.0, 360.0);
			double xc = this->my_pose.pos.x;
			double yc = this->my_pose.pos.y;
			//this->seen_litter;
			//this->neighbours;
			//this->comm_signal;
			//this->litter_db.size();
			//this->state;
			//std::cout<<this->comm_signal<<" "<<to_string(this->comm_signal)<<std::endl;
			double rep_signal = this->repel_signal;
			double cll_signal = this->call_signal;
			int rep_neigh = (int) this->rep_neighbours;
			int cll_neigh = (int) this->call_neighbours;
			
			if(this->repel_scale_div == 1.0)
			{
				rep_signal = 0;
				rep_neigh = 0;
			}
			
			if(this->call_scale_div == 1.0)
			{
				cll_signal = 0;
				cll_neigh = 0;
			}
			
			my_log_data = this->model->GetName() + ":" + 
							this->model->GetName() + ":" + 
							to_string(_info.simTime.Double()) + "," +
							to_string(xc) + "," + to_string(yc) + "," +
							to_string(theta) + "," +
							to_string(this->turn_prob) + "," + 
							to_string(this->seen_litter) + "," +
							to_string(rep_neigh) + "," + 
							to_string(rep_signal) + "," +
							to_string(cll_neigh) + "," + 
							to_string(cll_signal) + "," +
							to_string(this->litter_db.size()) + "," + 
							to_string(this->linear_dist) + "," + 
							to_string(this->rot_dist) + "," + 
							to_string(this->litter_collected) + "," +
							to_string(this->litter_deposited) + "," +
							to_string(this->wall_bounces) + "," +
							to_string(this->neighbour_bounces) + "," +
							to_string(this->t_obstacle_avoidance) + "," +
							to_string(this->t_searching) + "," +
							to_string(this->t_oa_searching) + "," +
							to_string(this->t_go4litter) + "," +
							to_string(this->t_oa_go4litter) + "," +
							to_string(this->t_litter_processing) + "," +
							to_string(this->t_homing) + "," +
							to_string(this->t_oa_homing) + "," +
							this->acTion + ":" + 
							this->state;
			// std::string myHeader = this->model->GetName() + ":";
			// myHeader = myHeader + "name:t,xc,yc,theta,turn_prob,seen_litter," +
			// 				"rep_neigh,rep_signal,call_neigh,call_signal," +
			// 				"litter_db,linear_dist,rot_dist,litter_collected," +
			// 				"litter_deposited,wall_bounces,neighbour_bounces," +
			// 				"t_obstacle_avoidance,t_searching,t_oa_searching,t_go4litter," +
			// 				"t_oa_go4litter,t_litter_processing,t_homing," +
			// 				"t_oa_homing,action:state";
			msgs::Any b;
			b.set_type(msgs::Any::STRING);
			b.set_string_value(my_log_data);
			this->pub_log->Publish(b);
		}
	}
	else
	{//if not start sim, the robot should be stationary.
		this->stop();
	}
	if(this->log_timer > this->log_rate)
	{
		this->log_timer = 0;
	}
	/**
	 * 
	 * Generate random number, rn
	 * If rn > turn_prob
	 * 		Set desired heading to a random turn between -pi to +pi
	 * else
	 * 		Set desired heading to current heading
	 * 
	 * Apply turn controller till heading approx= desired heading
	 * If desired heading approx= current heading
	 * 		Make straight motion.
	 * 
	**/
	/**********************************************************/
	
	/**********************************************************/
}

