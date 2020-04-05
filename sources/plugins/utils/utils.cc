struct RobotInfo {
		double x = 0;
        double y = 0;
        double yaw = 0;
        double litter_count = 0;
        double extra_litter = 0;//seen litter excess of robot's capacity
        std::string litter_db = "";
        std::string seen_litter = "";
        std::string state = "searching";
        std::string robot_name = "";
        std::string detectable_pure = "";

        double numseen_u2s = 0;
        double numseen_pure = 0;
        std::string com_pure = "";
        std::string com_u2s = "";

        void init_info(gazebo::physics::ModelPtr model)
        {
            auto pose = model->GetWorldPose();

            this->x = pose.pos.x;
            this->y = pose.pos.y;
            this->yaw = pose.rot.GetYaw();

            this->robot_name = model->GetName();
        }
        void update_data(custom_msgs::msgs::RobotInfo info)
        {
            if(this->robot_name == info.robot_name())
            {
                this->x = info.x();
                this->y = info.y();
                this->yaw = info.yaw();
                this->extra_litter = info.extra_litter();
                this->litter_count = info.litter_count();
                this->litter_db = info.litter_db();
                this->seen_litter = info.seen_litter();
                this->state = info.state();
                this->numseen_pure = info.numseen_pure();
                this->numseen_u2s = info.numseen_u2s();
                this->com_pure = info.com_pure();
                this->com_u2s = info.com_u2s();
                this->detectable_pure = info.detectable_pure();
            }
            else
            {
                gzerr << "Expected '" << this->robot_name <<"' but got"
                    << info.robot_name() <<"'" <<std::endl;
            }
            
            
            
        }

	};