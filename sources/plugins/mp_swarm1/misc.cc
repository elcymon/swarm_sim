double ModelVel::dxy(math::Vector3 A, math::Vector3 B)
{
	A.z = 0;
	B.z = 0;
	return A.Distance(B);
}
void ModelVel::my_acTion(std::string acTion)
{
	if ( acTion != " .")
	{
		//cout<<"\n"<<acTion<<"\n";
	}
	
}

double ModelVel::normalize(double angle)
{
	math::Angle D_angle(angle);
	D_angle.Normalize();
	return D_angle.Radian();
}
double ModelVel::computeObjectOrientation(math::Vector3 objectPos, math::Vector3 myPos, double myYaw)
{
	return this->normalize(atan2(objectPos.y - myPos.y,objectPos.x - myPos.x)) - myYaw;
}
bool ModelVel::testObjectWithinFoV(double objectOrientation, double halffov)
{
	return (objectOrientation >= -halffov and objectOrientation <= halffov);
}

bool ModelVel::litterInPickingRange(std::string litterName)
{
	
	
	if(!(litterName).empty())
	{//Pick litter if the distance to litter is less than robot radius.
		
		physics::ModelPtr litterModel = this->myLitterDB[litterName];
		math::Vector3 litterPos = (litterModel->GetWorldPose()).pos;
		math::Vector3 myPos = (this->my_pose).pos;
		double litterDistance = this->dxy(litterPos, myPos);

		double litter_or = this->computeObjectOrientation(litterPos,myPos,this->my_pose.rot.GetYaw());
	
		if(this->testObjectWithinFoV(litter_or,this->halffov))
		{
			double rot_dist = litter_or / 2.0 * this->chassis_diameter;
			if ((litterDistance + rot_dist) < this->chassis_diameter / 2.0)
			{
				return true;
			}

		}
		
	}
	return false;//not in picking range because no litterName, not in FoV or not within picking distance
}
