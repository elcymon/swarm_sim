void ModelVel::ContactInfo(ConstContactsPtr &c)
{//Contact sensor should set a boolean type that will control robot
	//movement based on the collision type. i.e between robots, litter, wall.
	std::lock_guard<std::mutex> lock(this->mutex);
	//cout<<"Contracts Called"<<endl;
	if(!(this->crashed))
	{
		msgs::Contacts contacts = *c;
		//cout<<contacts.contact_size()<<endl;
		for (unsigned int i = 0; i < (int)contacts.contact_size(); ++i)
		{
			//cout<<contacts.contact_size()<<endl;
			std::string col1 = contacts.contact(i).collision1();
			std::string col2 = contacts.contact(i).collision2();
			std::string collision = col1 + " " + col2;
			bool Me = (collision.find(this->ModelName) != std::string::npos);
			//bool otherRobot = (collision.find('m_4wrobot') != std::string::npos);
			bool _obstacle = collision.find("chassis::collision") != std::string::npos;
			bool _litter = collision.find("litter") != std::string::npos;
			if(Me && _obstacle && not _litter)// && not otherRobot)
			{//Obstacle Hit. Also ignore litter collision.
				
				//store pose at which crash occured
				for (unsigned int j = 0; j < (int)contacts.contact(i).position_size(); ++j)
				{
					if(contacts.contact(i).position(j).z() > 0.004)
					{

							this->x_crash = contacts.contact(i).position(j).x();
							this ->y_crash = contacts.contact(i).position(j).y();
							this->turn_amt = avoid_obstacle(this->x_crash,this->y_crash,this->my_pose);
							if(abs(this->turn_amt.Radian()) > 0)
							{
								this->crashed = true;
							}
							/*if(_litter and this->litter_db.size() < this->capacity)
							{//If collision is with litter and I have space and litter not behind me. pick it up.
								//You need to put check if capacity is full here.
								//You need to stoor litter name here.
								if(abs(this->turn_amt.Radian()) >= M_PI/2.0)
								{
									std::size_t sep1 = collision.find("m_litter");
									std::size_t sep2 = collision.find("::",sep1);
									this->LitterName = collision.substr(sep1,sep2 - sep1);
									this->pick_litter = true;
								}
								this->turn_amt = 0;
								this->crashed = false;
								
							}*/
							
							
							return;
					}
				}
				
			}
			//std::cout << "Collision between [" << contacts.contact(i).collision1()
			//		<<"] and ["<< contacts.contact(i).collision2() << "]\n";
		}
	}
}

