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
