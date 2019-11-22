void ModelVel::CameraInfo( ConstImageStampedPtr &msg)
{
	std::lock_guard<std::mutex> lock(this->mutex);
	if(this->exit_home)
	{
		return;
	}
	
	int width;
	int height;
	char *data;
	
	width = (int) msg->image().width();
	height = (int) msg->image().height();
	data = new char[msg->image().data().length() + 1];
	
	memcpy(data, msg->image().data().c_str(), msg->image().data().length());
	cv::Mat image(height, width, CV_8UC3, data);
	cv::Mat hsv_image; //to store image as hsv
	this->cam_image = image;
	cvtColor(image,hsv_image,CV_RGB2HSV);
	//cvtColor(hsv_image,image,CV_HSV2BGR);
	// Get Binary image showing only red pixels
	Mat b_image1,b_image2,b_image3,b_image_home,b_image_dock;
	if(this->home_mode)
	{//find home using camera
		//cout<< "do not call me"<<endl;
		if(!this->face_home)
		{
			return;
		}
		std::vector<KeyPoint> keypoints_home, keypoints_dock,keypoints_goal;
		inRange(hsv_image,Scalar(12, 100, 80), Scalar(20, 255, 255),b_image_home);
		//imshow(this->ModelName+"_home", b_image_home);
		this->detector->detect(b_image_home,keypoints_home);
		KeyPoint y_h = NearestLitter(keypoints_home);
		
		
		//Also need to locate docking station
		inRange(hsv_image,Scalar(115, 100, 80), Scalar(125, 255, 255), b_image_dock);
		this->detector->detect(b_image_dock,keypoints_dock);
		KeyPoint y_d = NearestLitter(keypoints_dock);
		
		//if(y_h != -999)
		//{
			//keypoints_goal = keypoints_home;
		this->keypoints.clear();	
		if(keypoints_dock.empty())
		{
			keypoints_goal = keypoints_home;
			this->home_kp = true;
			this->dock_kp = false;
		}
		else
		{
			keypoints_goal = keypoints_dock;
			this->home_kp = false;
			this->dock_kp = true;
		}
		if(!keypoints_goal.empty())
		{
			this->keypoints = keypoints_goal;
			if(!this->dock_found)
			{
				this->search_dock = true;
			}
			else
			{
				this->search_dock = false;
			}
		}
		else
		{
			this->search_dock = false;
			this->home_kp = false;
			this->dock_kp = false;
		}
		
		/*if(keypoints_home.size() > 0)
		{//search for the right-most x location for home
			int y = NearestLitter(keypoints_home);
			if(y != -999)
			{
				//KeyPoint kp = this->keypoints_home.at(y);
				draw_goal(keypoints_home.at(y));
			}
		}
		
		//imshow(this->ModelName+"_dock", b_image_dock);
		if(keypoints_dock.size() > 0)
		{//search for the left-most x location for dock
			//cout<<this->ModelName<<keypoints_dock.size()<<endl;
			int w = NearestLitter(keypoints_dock);
			if(w != -999)
			{
				//KeyPoint kp = this->keypoints_dock.at(w);
				draw_goal(keypoints_dock.at(w));
			}

		}*/
	}
	else
	{
		inRange(hsv_image,Scalar(0,100,80),Scalar(10,255,255),b_image1);
		inRange(hsv_image,Scalar(160,100,80),Scalar(179,255,255),b_image2);
		cv::addWeighted(b_image1, 1.0, b_image2, 1.0, 0.0, b_image3);
		//imshow(this->ModelName+"_litter",b_image3);
		//clear all values within keypoints vector
		this->keypoints.clear();
		
		//store new keypoints values from detector
		this->detector->detect(b_image3,this->keypoints);
		if (this->keypoints.size() > 0)
		{//There is a litter in view
			this->litter_present = true;
		}
	}
	
	
	//Debugging purposes only.
	
	//int l = NearestLitter((this->keypoints));
	//std::vector<KeyPoint> k;
	//k.push_back(this->keypoints.at(0));
	//Mat image2;
	//drawKeypoints(image,k,image2,Scalar(0,255,0),DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	
	KeyPoint x = NearestLitter(this->keypoints);
	if(x.size != -999)
	{
		//KeyPoint kp = this->keypoints.at(x);
		draw_goal(x);
		Point2f mpt = x.pt;
		this->goal_x = mpt.x;
		this->goal_y = mpt.y;
		this->goal_size = x.size;
	}
	//double dxn_eror = blob_hor_offset(kp);
	//What do you need this->goal_x, this->goal_y and this->goal_size for?
	
	
	cv::cvtColor(this->cam_image,this->cam_image,cv::COLOR_RGB2BGR);
	//cv::imshow(this->ModelName,this->cam_image);
	cv::waitKey(1);
	delete data; //DO NOT FORGET TO DELETE THIS,
				// ELSE GAZEBO WILL TAKE ALL YOUR MEMORY
	
	
	this->msg = height;
	if(!this->NewFrame)
	{
		this->NewFrame = true;//There is a new Image frame
	}

}

void ModelVel::BlobData(KeyPoint my_keypoint)
{
	Point2f mpt = my_keypoint.pt;
	double kSize = my_keypoint.size/2;
	ostringstream si1,si2,si3;
	si1<<"C:("<<mpt.x<<","<<mpt.y<<")";
	si2<<"R:"<<kSize;
	si3<<"D="<<mpt.y+kSize;
	Point2f mpt2 = mpt;
	Size sz = getTextSize(si1.str(),FONT_HERSHEY_SIMPLEX,0.4,1,0);
	mpt2.x = (mpt.x - sz.width/2 > 0) ? mpt.x - sz.width/2 : 0;
	mpt2.y = mpt.y - kSize - 3 * sz.height;
	putText(this->cam_image,si1.str(),mpt2,FONT_HERSHEY_SIMPLEX,
			0.3,Scalar(0,0,255),1,8,false);
	mpt2.y = mpt.y - kSize - 1.5 * sz.height;
	putText(this->cam_image,si2.str(),mpt2,FONT_HERSHEY_SIMPLEX,
			0.3,Scalar(0,0,255),1,8,false);
	mpt2.y = mpt.y - kSize;
	putText(this->cam_image,si3.str(),mpt2,FONT_HERSHEY_SIMPLEX,
			0.3,Scalar(0,0,255),1,8,false);
}
KeyPoint ModelVel::NearestLitter(std::vector<KeyPoint> kpt)
{//Find the nearest blob
	if(kpt.size()>0)
	{
		int Nearest = 0;
		float best_pos, temp_pos;
		for (int i = 0; i < kpt.size(); i++)
		{
			KeyPoint kp_temp = kpt.at(i);
			KeyPoint kp = kpt.at(Nearest);
			
			best_pos = kp.pt.y + kp.size / 2;
			temp_pos = kp_temp.pt.y + kp_temp.size / 2;
			
			//if ((kp_temp.pt.y > kp.pt.y) && (kp_temp.size > kp.size))//(temp_pos > best_pos))
			
			//if(!((kp_temp.pt.x) >=(kp.pt.x - kp.size/2) && (kp_temp.pt.x) <= (kp.pt.x + kp.size/2)))
			if(temp_pos > best_pos)
			{
				//if((temp_pos - best_pos) > 0.2 * kp.size)
			
				Nearest = i; // temp_pos is closer than best_pos
			
			}
			//BlobData(kp_temp);
		}
		/*KeyPoint kp = kpt.at(Nearest);
		double dxn_eror = blob_hor_offset(kp);
		draw_goal(kp);
		Point2f mpt = kp.pt;
		this->goal_x = mpt.x;
		this->goal_y = mpt.y;
		this->goal_size = kp.size;
		*///double dxn_eror = (this->deg_step) * (this->goal_x)
			//				- (this->h_fov)/2;
		//cout<<"best_pos: "<<best_pos<<" temp_pos: "<<temp_pos<<endl;
		
		//std::vector<KeyPoint> k;
		//k.push_back(kp);
		//Mat image2 = this->cam_image;
		//drawKeypoints(image2,k,this->cam_image,Scalar(0,255,0),DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
		//cv::imshow(this->ModelName,this->cam_image);
		return kpt.at(Nearest);
	}
	else
	{
		//cout<<"This shouldn't happen"<<endl;
		KeyPoint kp;
		kp.size = -999;
		return kp;
	}
}
double ModelVel::blob_hor_offset(KeyPoint kp)
{//compute offset of blob centre from middle of hfov
	Point2f mpt = kp.pt;
	double x = mpt.x;
	double y = mpt.y;
	double dxn_eror = x;
	return dxn_eror;
}
void ModelVel::draw_goal(KeyPoint kp)
{
	std::vector<KeyPoint> k;
	k.push_back(kp);
	Mat image2 = this->cam_image;
	drawKeypoints(image2,k,this->cam_image,Scalar(0,255,0),DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
}
		
