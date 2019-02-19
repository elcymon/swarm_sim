void ModelVel::RepulsionSender(std::string neighbour, double distance)
{
	//transport::PublisherPtr pub_repulsion;
	//this->pub_repulsion = this->node->Advertise<msgs::Any>("/"+neighbour+"/comm_signal");
	double repulsion_intensity = (this->nei_sensing - distance)/((double)this->nei_sensing);
	msgs::Any any;
	any.set_type(msgs::Any::DOUBLE);
	any.set_double_value(repulsion_intensity);
	//this->pub_repulsion[neighbour]->Publish(any);
}

void ModelVel::CommSignal(ConstAnyPtr &a)
{
	std::lock_guard<std::mutex> lock(this->mutex);
	std::string s = a->string_value();
	double repel_queue_tot = 0, call_queue_tot = 0;

	size_t div_loc = s.find(":");
	//Extract repulsion neighbours
	this->rep_neighbours = std::stoi(s.substr(0,div_loc));
	
	//Extract repulsion_signal
	s = s.substr(div_loc+1);
	div_loc = s.find(":");
	this->repel_queue.push_back(std::stod(s.substr(0,div_loc)));
	if(this->repel_queue.size() >= this->queue_size)
	{
		for(auto it = this->repel_queue.cbegin(); it != this->repel_queue.cend(); ++it)
		{
			repel_queue_tot += (*it);
		}
		this->repel_signal = repel_queue_tot / ((int)this->repel_queue.size());
		this->repel_queue.clear();
	}
	// this->repel_signal =  std::stod(s.substr(0,div_loc));
	
	
	//Extract resultant theta
	s = s.substr(div_loc+1);
	div_loc = s.find(":");
	this->rslt_theta = std::stod(s.substr(0,div_loc));
	
	//Extract attraction neighbours
	s = s.substr(div_loc+1);
	div_loc = s.find(":");
	this->call_neighbours = std::stoi(s.substr(0,div_loc));
	
	//Extract attraction signal
	this->call_queue.push_back(std::stod(s.substr(div_loc+1)));
	if(this->call_queue.size() >= this->queue_size)
	{
		for(auto it = this->call_queue.cbegin(); it != this->call_queue.cend(); ++it)
		{
			call_queue_tot += (*it);
		}
		this->call_signal = call_queue_tot / ((int)this->call_queue.size());
		this->call_queue.clear();
	}

	if(this->call_queue.size() == 0 and this->repel_queue.size() == 0){
		//queue limit reached and the queue was cleared. So a new communication signal is available to use
		this->new_comm_signal = true;
	}
	// this->call_signal = std::stod(s.substr(div_loc+1));
	
	/*std::string nei_name = s.substr(0,div_loc);
	double signal = std::stod(s.substr(div_loc+1));
	
	auto result = this->com_sig_set.insert(nei_name);
	
	if(result.second)
	{
		this->cummulative_sig += signal;
	}
	if(this->com_sig_set.size() == this->pub_repulsion.size())
	{
		//std::cout<<this->com_sig_set.size()<<std::endl;
		this->com_sig_set.clear();
		this->com_sig_set.insert(nei_name);
		
		this->comm_signal = this->cummulative_sig;
		this->cummulative_sig = 0;//reset the cummulative signals received
		this->cummulative_sig = signal;//add current signal value
	}
	msgs::Any any;
	any.set_type(msgs::Any::DOUBLE);
	any.set_double_value(this->comm_signal);
	this->pub_info->Publish(any);*/
	
	//std::cout<<this->comm_signal<<std::endl;
	
	//std::cout<<any.double_value()<<" "<<this->comm_signal<<std::endl;
	
	
}

//litter sensing topics
void ModelVel::LitterSensor()
{
	//std::lock_guard<std::mutex> lock(this->mutex);
	this->no_litter = true;
	physics::Model_V models = this->world->GetModels();
	std::string detections = "";
	//this->neighbours = 0;
	this->seen_litter = 0;
	double litter_distance = 1000000;
	double lit_or = M_PI;
	//math::Vector3 litter_pos;
	this->litter_pos.z=-9000.1;//initialize closest litter at unreasonable distance in z direction
	std::string litter_name = "";
	math::Vector3 my_p = this->my_pose.pos;
	std::string my_name = this->ModelName;
	//this->comm_signal = 0; //reset comm_signa at every time step
	for(auto m : models)
	{
		std::string m_name = m->GetName();//model name
	/*	if(m_name.find("m_4wrobot") != std::string::npos and m_name.compare(my_name) !=0)
		{//Detects all neighbouring robots, their distances and how many they are
			double repel_force = 0;
			math::Vector3 m_p = m->GetWorldPose().pos;
			double dist = this->dxy(my_p,m_p);
			
			if(dist <= this->nei_sensing)
			{
				repel_force = (this->nei_sensing - dist)/(this->nei_sensing);
				
				detections = detections + m_name + ":" + to_string(dist) + " ";//consider removing
				//this->neighbours += 1;
				//this->RepulsionSender(m_name,dist);
			}
			else
			{
				repel_force = 0;
			}
			if(true){
				msgs::Any any;
				any.set_type(msgs::Any::STRING);
				any.set_string_value(my_name + ":" + to_string(repel_force));
				this->pub_repulsion[m_name]->Publish(any);
			}
		}*/
		if(m_name.find("litter") != std::string::npos and m_name.compare(my_name) != 0)
		{//Check all litter within the world and count them
			//find the closest litter and orientation.
			
			math::Vector3 m_p = m->GetWorldPose().pos;
			if(abs(m_p.x) < 500 and abs(m_p.y) < 500)
			{//Litter still within arena
				this->no_litter = false;
			}
			if(abs(my_p.x - m_p.x) < this->lit_sensing and
				abs(my_p.y - m_p.y) <  this->lit_sensing)
			{
				double dist = this->dxy(my_p,m_p);//linear distance
				lit_or = this->normalize(atan2(m_p.y - my_p.y,m_p.x - my_p.x)) - this->my_pose.rot.GetYaw();//amount of angle to rotate
				
				if(dist <= this->lit_sensing and (lit_or >= -this->halffov and lit_or <= this->halffov))
				{
					//computing the rotaional distance
					//double rot_dist = lit_or/(2 * M_PI) * (M_PI * this->chassis_diameter);//original formula
					double rot_dist = lit_or/2.0 * this->chassis_diameter;//original formula reduces to this
					dist = dist + abs(rot_dist); //add rotational distance
					if(dist + 0.01 < litter_distance)//if difference between distances is more than 10cm, you can change closest litter
					{
						//cout<<litter_name<<":"<<litter_distance<<"replaced by::: "<<m_name<<":"<<dist<<endl;
						this->litter_pos = m_p;
						litter_name = m_name;
						litter_distance = dist;//update closest litter
					}
					this->seen_litter += 1;
				}
			}
/////////////////////////////////***attract or repel based on litter count (Do it outside the loop)
		}
		
	}
	msgs::Any seen_litter_msg;
	seen_litter_msg.set_type(msgs::Any::STRING);
	seen_litter_msg.set_string_value(my_name + ":" + to_string(this->seen_litter));
	this->pub_seen_litter->Publish(seen_litter_msg);//Publish seen litter
	
	if(!litter_name.empty() and litter_distance <= this->lit_sensing)//this->chassis_diameter/2.0)
	{//Pick litter if the distance to litter is less than robot sensing area.
		this->LitterName = litter_name;
		this->pick_litter = true;
	}
	
	//this->neighbours_info = to_string(this->neighbours) + ": " + to_string(this->comm_signal);
	//if(!litter_name.empty())
	//{
		//cout<<this->seen_litter<<" ::: "<<litter_name + " {" <<litter_pos<<"}"<<endl;
	//}
	////cout<<detections<<endl;
	//msgs::Any any;
	//any.set_type(msgs::Any::STRING);
	//any.set_string_value(this->neighbours_info);
	//this->lit_nei_pub->Publish(any);
	//this->pub_info->Publish(any);
	//std::cout<<"cummulative signal = "<<this->comm_signal<<endl;
}
