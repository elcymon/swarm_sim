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
	double call_sig,repel_sig;//call and repel signals gotten in the new message
	int call_neigh,repel_neigh;//call and repel neighbours gotten in the new message



	size_t div_loc = s.find(":");
	//Extract repulsion neighbours
	repel_neigh = std::stoi(s.substr(0,div_loc));
	this->rep_neighbours = repel_neigh;
	
	//Extract repulsion_signal
	s = s.substr(div_loc+1);
	div_loc = s.find(":");
	repel_sig = std::stod(s.substr(0,div_loc));
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
	call_sig = std::stod(s.substr(div_loc+1));
	this->call_queue.push_back(call_sig);
	
	//using proposed communication update method
	this->commModel.update_comm_signals(call_sig,repel_sig,this->timeStamp);

	
}

//litter sensing topics
void ModelVel::LitterSensor()
{
	//std::lock_guard<std::mutex> lock(this->mutex);
	this->no_litter = true;
	this->LitterName = "";
	this->litterModel = nullptr;
	std::string detections = "";
	//this->neighbours = 0;
	this->seen_litter = 0;
	this->litter_distance = 1000000;
	double lit_or = M_PI;
	//math::Vector3 litter_pos;
	this->litter_pos.z=-9000.1;//initialize closest litter at unreasonable distance in z direction
	
	math::Vector3 my_p = this->my_pose.pos;
	
	for(auto m : this->myLitterDB)
	{
		if((this->visionModel).compare("initialization") == 0 || //if initialization based, litter has already been filtered
			((this->visionModel).compare("instantaneous") == 0 && //if instantaneous based, perform litter filtering now
			  this->uform_rand(this->generator) <= this->detectionProbability //filter based on detection probability
			)
		  )
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
				lit_or = this->computeObjectOrientation(m_p,my_p,this->my_pose.rot.GetYaw());
				
				if(dist <= this->lit_sensing and this->testObjectWithinFoV(lit_or,this->halffov))
				{
					//computing the rotaional distance
					//double rot_dist = lit_or/(2 * M_PI) * (M_PI * this->chassis_diameter);//original formula
					double rot_dist = lit_or/2.0 * this->chassis_diameter;//original formula reduces to this
					dist = dist + abs(rot_dist); //add rotational distance
					if(dist + 0.01 < this->litter_distance)//if difference between distances is more than 10cm, you can change closest litter
					{
						//cout<<litter_name<<":"<<this->litter_distance<<"replaced by::: "<<m_name<<":"<<dist<<endl;
						this->litter_pos = m_p;
						this->LitterName = m->GetName();
						this->litter_distance = dist;//update closest litter
						this->litterModel = m;
					}
					this->seen_litter += 1;
					detections += m->GetName() + ",";
				}
			}
		}
		
	}
	msgs::Any seen_litter_msg;
	seen_litter_msg.set_type(msgs::Any::STRING);
	seen_litter_msg.set_string_value(this->ModelName + ":" + to_string(this->seen_litter - (this->capacity - this->litter_count)));
	this->pub_seen_litter->Publish(seen_litter_msg);//Publish seen litter
	
	msgs::Any detectedLitterMsg;
	detectedLitterMsg.set_type(msgs::Any::STRING);
	detectedLitterMsg.set_string_value(detections);
	this->pub_myDetectedLitterNames->Publish(detectedLitterMsg);


	this->pick_litter = this->litterInPickingRange(this->LitterName);
	
}
