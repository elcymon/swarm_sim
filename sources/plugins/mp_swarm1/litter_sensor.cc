void ModelVel::CommSignal(ConstComDataPtr &a)
{
	std::lock_guard<std::mutex> lock(this->mutex);
	if (this->ModelName.compare(a->robot_name()) != 0)
		gzerr << "Expected: " << this->ModelName << "; but got: " << a->robot_name() <<std::endl;
	


	this->rep_neighbours = a->rep_neighbours();
	this->repel_queue.push_back(a->rep_signal());
	this->call_neighbours = a->att_neighbours();
	this->call_queue.push_back(a->att_signal());

	//using proposed communication update method
	this->commModel.update_comm_signals(a->att_signal(),a->rep_signal(),this->timeStamp,
					a->att_x(),a->att_y(),a->rep_x(),a->rep_y());


	//handle vector based communication computation of resultant values
	double att_x = 0;
	double att_y = 0;
	if (this->call_scale_mult > 0)
	{
		att_x = this->call_scale_mult * this->commModel.get_vector_component("att_x");
		att_y = this->call_scale_mult * this->commModel.get_vector_component("att_y");
	}
	double rep_x = 0;
	double rep_y = 0;
	if (this->repel_scale_mult > 0)
	{
		rep_x = this->repel_scale_mult * this->commModel.get_vector_component("rep_x");
		rep_y = this->repel_scale_mult * this->commModel.get_vector_component("rep_y");
	}

	double rslt_x = att_x + rep_x;
	double rslt_y = att_y + rep_y;

	this->rslt_theta = atan2(rslt_y,rslt_x);
	// if(this->ModelName.compare("m_4wrobot10") == 0)
	// 	gzdbg << this->rslt_theta << ": nrep="<<this->rep_neighbours
	// 			<<", natt="<<this->call_neighbours 
	// 			// << " " <<att_x <<" "<<att_y
	// 			//<<" "<<rep_x <<" "<<rep_y 
	// 			<<endl;	
}

//litter sensing topics
void ModelVel::LitterSensor()
{
	//std::lock_guard<std::mutex> lock(this->mutex);
	this->LitterName = "";
	this->litterModel = nullptr;
	this->litter_distance = 1000000;
	this->litter_pos.z=-9000.1;//initialize closest litter at unreasonable distance in z direction
	
	this->seen_litter = 0;
	this->no_litter = true;
	std::string detections = "";
	//this->neighbours = 0;
	double lit_or = M_PI;
	//math::Vector3 litter_pos;
	
	math::Vector3 my_p = this->my_pose.pos;
	
	for(auto m : this->myLitterDB)
	{
		
		math::Vector3 m_p = m->GetWorldPose().pos;
		bool seen = false;
		auto p_seen = this->prev_seen.find(m->GetName());
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
			    (this->testObjectWithinFoV(lit_or,this->halffov)) and
				(this->uform_rand(this->generator) < probability)
			  )
			{
				seen = true;
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
				if (detections != "")
				{
					detections += ";";
				}
				detections += (m->GetName()).substr(8);
			}
		}
		// else{//delete from DB because litter is not within world area
		// this->myLitterDB.erase(m);

		// }

		if(seen)
		{//insert litter name to previously seen (data type is set, so it will fail if it already exists)
			this->prev_seen.insert(m->GetName());
		}
		else
		{//remove litter name from previously seen
			this->prev_seen.erase(m->GetName());
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
	

	this->detectedLitterNames = detections;

	this->pick_litter = this->litterInPickingRange(this->LitterName);
	
}
