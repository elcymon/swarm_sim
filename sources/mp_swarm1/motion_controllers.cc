void ModelVel::stop()
{
	this->lwheel->SetVelocity(0,0);
	this->rwheel->SetVelocity(0,0);
	this->acTion = "stop";
	//cout<<"stopping"<<endl;
	//return this->model->GetName()+": "+"stop \n";
}

void ModelVel::rotate(double dxn_error)
{
	
	if(dxn_error > 0)
	{//Object on the right. Turn right
		this->lwheel->SetVelocity(0,-(this->rVel/2));
		this->rwheel->SetVelocity(0,this->rVel/2);
		this->acTion = "right turn";
		//return this->model->GetName()+": "+"right turn";
	}
	else if(dxn_error < 0)
	{//Object on left. Turn left.
		this->lwheel->SetVelocity(0,this->rVel/2);
		this->rwheel->SetVelocity(0,-(this->rVel/2));
		this->acTion = "left turn";
		//return this->model->GetName()+": "+"left turn";
	}
	else
	{//This means object is straight ahead. This should not happen
		//because This function is only called when |dxn_error| >tolerance
		//return straight();
	}
}
		
void ModelVel::rotate_v2(double dxn_eror)
{
	//this->tot_e = this->tot_e + dxn_eror;
	double va = dxn_eror * this->Kp ;//+ this->tot_e * this->Kp/100000 + (dxn_error - this->prev_e) * this->Kp;
	//this->prev_e = dxn_eror;
	double r = this->rVel + va * this->wheel_separation/2;
	double l = this->rVel - va * this->wheel_separation/2;
	
	if (r > this->rVel)
		r = this->rVel;
	if (r < 0)
		r = 0;
	
	if (l > this->rVel)
		l = this->rVel;
	if (l < 0)
		l = 0;
	
	this->rwheel->SetVelocity(0,r);
		this->lwheel->SetVelocity(0,l);
		
	if(r < l)
	{
		
		this->acTion = "right turn";
		//return this->model->GetName()+": "+"right turn";
	}
	else if(r>l)
	{
		
		this->acTion = "left turn";
		//return this->model->GetName()+": "+"left turn";
	}
	else
	{this->acTion = "straight";}
}
		
void ModelVel::straight()
{//Straight motion
	this->rwheel->SetVelocity(0,this->rVel);
	this->lwheel->SetVelocity(0,this->rVel);
	this->acTion = "straight";
	//return this->model->GetName()+": "+"straight";
}


		
double ModelVel::avoid_obstacle(double x, double y,math::Pose my_pos)
{
	double turn_reqd = 0;
	double theta = my_pos.rot.GetYaw();
	double xc = my_pos.pos.x;
	double yc = my_pos.pos.y;
	double obstacle_loc = atan2(y-yc,x-xc);//angle of line joining litter and robot center wrt world frame
	double d_theta = theta - obstacle_loc;//difference between direction and collision
	d_theta = this->normalize(d_theta);
	
	if(abs(d_theta) < M_PI/2.0)
	{//turn only when collision is less than 90 degrees
		if(d_theta < - M_PI/6.0)
		{
			turn_reqd = -M_PI/4.0;
		}
		else if(d_theta > M_PI/6.0)
		{
			turn_reqd = M_PI/4.0;
		}
		else
		{
			//double d90 = M_PI/2.0;
			//double x = rand()/double(RAND_MAX) * M_PI + M_PI/2.0;
			double x = this->uform_rand(this->generator) * M_PI + M_PI/2.0;
			turn_reqd = this->normalize(x);
			/*if(x < 0)
			{
				x -= d90;
			}
			else
			{
				x += d90;
			}*/
		}
	}
	
	
	return turn_reqd;
	//double new_x = (x-xc) * cos(theta) - (y-yc) * sin(theta);
	//double new_y = (x-xc) * sin(theta) + (y-yc) * cos(theta);
	/*cout<<"Crash Details"<<endl;
		cout<<"Crash pt"<<endl;
		cout<<x<<"\t"<<y<<endl;
		
		cout<<"Crash Transformed by "<<theta * 180/M_PI<<endl;
		cout<<new_x<<"\t"<<new_y<<endl;
		
		cout<<"Center"<<endl;
		cout<<xc<<"\t"<<yc<<endl;
		
		cout<<"Left contact"<<endl;
		cout<<this->lx_contact<<"\t"<<this->ly_contact<<endl;
		
		cout<<"Right contact"<<endl;
		cout<<this->rx_contact<<"\t"<<this->ry_contact<<endl;
		
		cin.get();
	if (new_x > 0 )
	{
		
		if( (new_y >= (this->ly_contact)) && (new_x <= (this->lx_contact)))
		{//Left crash. Turn right 45 i.e. -45
			//using +45 because for camera control, dxn_error > 0 for right turn
			
			turn_reqd = M_PI/4.0;
			//cout<<"Left crash. Turn = "<<turn_reqd*180/M_PI<<endl;
		}
		else if( (new_y <= (this->ry_contact)) && (new_x <= (this->rx_contact)))
		{//Right crash. Turn left 45
			
			turn_reqd = - M_PI/4.0;
			//cout<<"Right crash. Turn = "<<turn_reqd*180/M_PI<<endl;
		}
		else if(new_x >= (this->rx_contact))
		{//Front crash. Turn greater than 90 degrees
			double d90 = M_PI/2.0;
			double x = rand()/double(RAND_MAX) * M_PI - d90;
			if(x < 0)
			{
				x -= d90;
			}
			else
			{
				x += d90;
			}
			//cout<<"Front crash. Turn = "<<turn_reqd*180/M_PI<<endl;
		}
		else
		{
			//cout<<"Error. This crash should not count"<<endl;
		}
		this->crashed = true;
	}
	else
	{
		//cout<<"back contact"<<endl;
	}*/
}/*

bool ModelVel::turn_control()
{//Controls the rotation of the robot. So it turns the right amount
	math::Quaternion q1 = this->model->GetWorldPose().rot;
	math::Quaternion q2 = this->my_pose.rot;
	double turn_tot = q1.GetYaw() - q2.GetYaw();
	math::Angle current_turn(turn_tot);
	current_turn.Normalize();
	this->turn_amt.Normalize();

	if(abs(current_turn.Radian()) < abs(this->turn_amt.Radian()))
	{
		//cout<<abs(current_turn.Radian()) - abs(this->turn_amt.Radian()) << endl;
		this->dxn_error = this->turn_amt.Radian();
		rotate(this->dxn_error);
		return false;
	}
	else
	{
		//this->crashed = false;
		stop();
		return true;
	}
}
*/

bool ModelVel::try_pick(std::string litter_name)
{// pick litter if I have enough space
	int my_capacity = this->capacity;
	int my_litter_count = this->litter_db.size();
	int litter_cost = 0;
	//set cost of this litter
	/*if(litter_name.find("m_litter") != std::string::npos)
	{
		litter_cost = 1;				
	}
	else
	{//this is not a recognized litter. 
		litter_cost = 999;
	}*/
	//cin.get();
	//check if I have enough space to pick it
	if(my_litter_count < my_capacity)
	{
		if(this->litterInPickingRange(litter_name)){
			auto result = this->litter_db.insert(litter_name);
			//cout<<this->litter_db.size()<<endl;
			
			
			//***********Test contents of the robot***********************//
			/*std::string ss = to_string(litter_db.size()) + ": ";
			for(auto it : this->litter_db)
			{
				std::string litter_name = it;
				ss = ss + litter_name + ",";
			}
			ss = ss + "inserting: " + litter_name;
			msgs::Any any;
			any.set_type(msgs::Any::STRING);
			any.set_string_value(ss);
			this->pub_info->Publish(any);*/
			//************************************************************//
			
			
			if(result.second)
			{//insertion successful
				this->litter_collected += 1;
				this->litter_count += 1;// litter_cost;
				//this->litter_db.insert(litter_name);
				
				
				//transport::requestNoReply(this->node,"entity_delete",litter_name);
				//physics::ModelPtr l = this->world->GetModel(this->LitterName);
				//l->Fini();
				//this->LitterName = "";
				return true;
			}
			
		}
	}
	
	return false;//unsuccessful because not in picking range, full or not in FoV
}


void ModelVel::deposit_litter()
	{//deposit all litter picked at home
		
		std::set<std::string>::iterator it;
		it = this->litter_db.begin();
		std::string litter_name = *it;
		//::gazebo::msgs::Any::ValueType vtype;
		
		msgs::Any any;
		any.set_type(msgs::Any::STRING);
		any.set_string_value(litter_name);
		this->pub_litter->Publish(any);
		this->litter_db.erase(it);
		this->litter_deposited += 1;
		//cout<<" 370 deposit "<<litter_name<<" count "<<this->litter_db.size()<<endl;
	}
