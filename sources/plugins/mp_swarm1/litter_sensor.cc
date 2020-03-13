void ModelVel::CommSignal(ConstAnyPtr &a)
{
	std::lock_guard<std::mutex> lock(this->mutex);
	std::string s = a->string_value();
	double repel_queue_tot = 0, call_queue_tot = 0;
	double call_sig,repel_sig;//call and repel signals gotten in the new message
	int call_neigh,repel_neigh;//call and repel neighbours gotten in the new message



	size_t div_loc = s.find(":");
	//Extract repulsion neighbours
	repel_neigh = std::stoi(s.substr(0,div_loc));
	this->rep_neighbours = repel_neigh;
	
	//Extract repulsion_signal
	s = s.substr(div_loc+1);
	div_loc = s.find(":");
	stringstream repel_stream;
	repel_stream << std::fixed << std::setprecision(10) << s.substr(0,div_loc);
	repel_stream >> std::setprecision(10) >> std::fixed >> repel_sig;
	// repel_sig = std::stod(s.substr(0,div_loc));
	this->repel_queue.push_back(repel_sig);

	//Extract resultant theta
	s = s.substr(div_loc+1);
	div_loc = s.find(":");
	this->rslt_theta = std::stod(s.substr(0,div_loc));
	
	//Extract attraction neighbours
	s = s.substr(div_loc+1);
	div_loc = s.find(":");
	call_neigh = std::stoi(s.substr(0,div_loc));
	this->call_neighbours = call_neigh;
	
	//Extract attraction signal
	stringstream call_stream;
	call_stream <<std::fixed << std::setprecision(10) << s.substr(div_loc+1);
	call_stream >> std::setprecision(10) >> std::fixed >> call_sig;
	// call_sig = std::stod(s.substr(div_loc+1));
	this->call_queue.push_back(call_sig);
	
	//using proposed communication update method
	this->commModel.update_comm_signals(call_sig,repel_sig,this->timeStamp);

	
}

//litter sensing topics
void ModelVel::LitterSensor()
{
	//std::lock_guard<std::mutex> lock(this->mutex);
	
	//temporary variable for detected litter
	std::string tempLitterName = "";
	physics::ModelPtr templitterModel = nullptr;
	double templitter_distance = 1000000;
	math::Vector3 templitter_pos= math::Vector3(0,0,-9000.1);//initialize closest litter at unreasonable distance in z direction
	
	this->seen_litter = 0;
	this->numseen_pure = 0;
	this->numseen_u2s = 0;
	this->no_litter = true;
	std::string detections = "";
	//this->neighbours = 0;
	double lit_or = M_PI;
	//math::Vector3 litter_pos;
	
	math::Vector3 my_p = this->my_pose.pos;
	
	for(auto name_modelptr : this->myLitterDB)
	{
		std::string m_name = name_modelptr.first;
		physics::ModelPtr m = name_modelptr.second;
		math::Vector3 m_p = m->GetWorldPose().pos;
		bool seen = false;
		auto p_seen = this->prev_seen.find(m_name);
		double probability = this->p_u2s; //assume not previously seen
		
		if (p_seen != this->prev_seen.end())
		{//seen previously
			probability = this->p_s2s;
		}

		if(abs(m_p.x) < 500 and abs(m_p.y) < 500)
		{//Litter still within arena
			this->no_litter = false;
			double dist = this->dxy(my_p,m_p);//linear distance
			lit_or = this->computeObjectOrientation(m_p,my_p,this->my_pose.rot.GetYaw());
			
			if((dist <= this->lit_sensing) and 
			    (this->testObjectWithinFoV(lit_or,this->halffov))
			  )
			{
				this->numseen_pure += 1;
				if (this->uform_rand(this->generator) < probability)
				{
					this->numseen_u2s += 1;
					seen = true;
					//computing the rotaional distance
					//double rot_dist = lit_or/(2 * M_PI) * (M_PI * this->chassis_diameter);//original formula
					double rot_dist = lit_or/2.0 * this->chassis_diameter;//original formula reduces to this
					dist = dist + abs(rot_dist); //add rotational distance
					if(dist + 0.01 < templitter_distance)//if difference between distances is more than 10cm, you can change closest litter
					{
						//cout<<litter_name<<":"<<this->litter_distance<<"replaced by::: "<<m_name<<":"<<dist<<endl;
						templitter_pos = m_p;
						tempLitterName = m_name;
						templitter_distance = dist;//update closest litter
						templitterModel = m;
					}
					this->seen_litter += 1;
					if (detections != "")
					{
						detections += ";";
					}
					detections += (m_name).substr(8);
				}
			}
		}
		// else{//delete from DB because litter is not within world area
		// this->myLitterDB.erase(m);

		// }

		if(seen)
		{//insert litter name to previously seen (data type is set, so it will fail if it already exists)
			this->prev_seen.insert(m_name);
		}
		else
		{//remove litter name from previously seen
			this->prev_seen.erase(m_name);
		}
		
		
	}

	//update detected litter information data
	if(!(this->LitterName.empty()) and false)
	{
		this->litter_pos = this->litterModel->GetWorldPose().pos;
		this->litter_distance = this->dxy(my_p,this->litter_pos);//linear distance
		double old_lit_or = this->computeObjectOrientation(templitter_pos,my_p,this->my_pose.rot.GetYaw());
		this->litter_distance += abs(old_lit_or/2.0 * this->chassis_diameter);//original formula reduces to this
		if((this->litter_distance > 500) or
			(this->litter_distance + 0.01 > templitter_distance))
		{//litter distance is beyond world dimension
		//new litter is closer than target
			this->litter_pos = templitter_pos;
			this->LitterName = tempLitterName;
			this->litter_distance = templitter_distance;//update closest litter
			this->litterModel = templitterModel;
		}
		else
		{
			this->seen_litter += 1;
		}
		
		
	}
	else {
		//cout<<litter_name<<":"<<this->litter_distance<<"replaced by::: "<<m_name<<":"<<dist<<endl;
		this->litter_pos = templitter_pos;
		this->LitterName = tempLitterName;
		this->litter_distance = templitter_distance;//update closest litter
		this->litterModel = templitterModel;
	}
	

	// this->LitterName = tempLitterName;
	// this->litterModel = templitterModel;
	// this->litter_distance = templitter_distance;
	// this->litter_pos = templitter_pos;

	msgs::Any seen_litter_msg;
	seen_litter_msg.set_type(msgs::Any::STRING);
	seen_litter_msg.set_string_value(this->ModelName + ":" + to_string(this->seen_litter - (this->capacity - this->litter_count)));
	this->pub_seen_litter->Publish(seen_litter_msg);//Publish seen litter
	
	msgs::Any detectedLitterMsg;
	detectedLitterMsg.set_type(msgs::Any::STRING);
	detectedLitterMsg.set_string_value(detections);
	this->pub_myDetectedLitterNames->Publish(detectedLitterMsg);
	

	this->detectedLitterNames = detections;

	this->pick_litter = this->litterInPickingRange(this->LitterName);
	
}
