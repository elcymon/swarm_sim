double ModelVel::home_distance()
{
	math::Vector3 my_xyz = this->model->GetWorldPose().pos;
	double dx = this->home_x - my_xyz.x;
	double dy = this->home_y - my_xyz.y;
	double distance = pow(pow(dx,2)+pow(dy,2),0.5);
	return distance;
	
	
}

void ModelVel::leave_home()
{//leave home function helps free up a space/dock when a robot is leaving home
	//reimpliment it to use keep_away() i.e. no longer using home_distance()
	
	if(!keep_away())
	{
		this->litter_deptime = -999;
		//cout<<"327: left home"<<endl;
		msgs::JointCmd joint_cmd;
		joint_cmd.set_name(this->dock_name);
		joint_cmd.set_force(10);
		this->pub_joint->Publish(joint_cmd);
		this->at_home = false;
		this->exit_home = false;
		this->home_mode = false;
	}
	else
	{
		//cout<<"335: leaving home"<<endl;
		straight();
	}
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
	//cout<<" 370 deposit "<<litter_name<<" count "<<this->litter_db.size()<<endl;
}

void ModelVel::circle_home()
{
	//int x = NearestLitter(this->keypoints);
	double e =1 - (this->goal_x)/(this->ImgWidth/2);
	if(this->home_kp)
	{
		e =1.65 - (this->goal_x)/(this->ImgWidth/2);
	}
	if(this->dock_kp)
	{
		e =1 - (this->goal_x)/(this->ImgWidth/2);
	}
	
	this->dxn_error = e;
	rotate_v2(this->dxn_error);
}
double ModelVel::angle_from_home()
{// returns the angle made by the robot x,y location and the home
	
	double my_x = this->model->GetWorldPose().pos.x;
	double my_y = this->model->GetWorldPose().pos.y;
	double dy = (this->home_y - my_y),dx = (this->home_x - my_x);
	double dist_to_home = sqrt(pow(dx,2) + pow(dy,2));
	return atan2(dy,dx);
}
bool ModelVel::home_turn()
{//This is to face home when going home and away from home when leaving home
		//cout<<"go home"<<endl;
		math::Quaternion rot = this->model->GetWorldPose().rot;
		double my_yaw = rot.GetYaw();
		double theta = angle_from_home();
		double yaw_diff = theta - my_yaw;
		if(this->at_home || this->exit_home)
		{//if at home, face out
			yaw_diff -= M_PI;
		}
		//cout<<yaw_diff<<endl;
		//Normalize yaw_diff
		math::Angle p(yaw_diff); p.Normalize(); yaw_diff = p.Radian();
		
		//turn and face home
		if(abs(yaw_diff) > this->rStop)
		{
			//cout<<yaw_diff<<endl;
			//cout<<"face home"<<endl;
			rotate(-yaw_diff);
			return false;
		}
		else
		{
			return true;
		}
}
void ModelVel::go_home()
{//responsible for going home to deposit picked litter when full
	if(this->crashed && !this->dock_found)
	{//if crashed, turn and move a bit then face home again and move straight
		//cout<<"avoid obstacle"<<endl;
		char a1='w',b1= ' ';
		this->straight_50cm = turn_control();
		if(this->straight_50cm)
		{
			b1 = 'v';
			this->crashed = false;
			this->straight_rand_cm = 1 * rand()/double(RAND_MAX);
		}
		//cout<<a1<<b1<<endl;
		return;//so that the bottom won't be executed
	}
	
	if(this->straight_50cm)
	{
		char a= ' ',b=' ',c=' ',d=' ',e=' ',f = ' ';
		a='a';
		double travelled = this->my_pose.pos.Distance(this->model->GetWorldPose().pos);
		if(travelled >= this->straight_rand_cm)
		{b='b';


					this->straight_50cm = false;

		}
		//cout<<a<<b<<c<<d<<e<<f<<endl;
		straight();
		return;
	}
	if(!this->search_dock)
	{
		

		if(home_turn())
		{
			this->face_home = true;
			if(this->at_home && this->dock_found)
			{
				this->exit_home = true;
				//cout<<"at home stop"<<endl;
				stop();
			}
			else
			{
				//cout<<"not at home go straight"<<endl;
				//go straight to home
				straight();
			}
		}
	}
		//cout<<"circle_home"<<endl;
		//if(this->keypoints.size()>0)
	if(this->dock_found)
	{
		//cout<<"dock found"<<endl;
		this->search_dock = false;
		this->face_home = false;
	}
	if(this->search_dock && !this->dock_found && this->face_home)
	{
		//cout<<"search dock"<<endl;
		circle_home();
	}
		/*
		if(this->at_home)
		{
			stop();
			
		}
		else
		{
			straight();
		}*/
	
	
}
bool ModelVel::keep_away()
{//This function prevents robots from crowding in the home region when not
	//in go home mode
	double my_x = this->model->GetWorldPose().pos.x;
	double my_y = this->model->GetWorldPose().pos.y;
	double dy = (this->home_y - my_y),dx = (this->home_x - my_x);
	double dist_to_home = sqrt(pow(dx,2) + pow(dy,2));
	if(dist_to_home <= 1.2)
	{//too close. keep away
		return true;
	}
	return false;
}	/**/
		
