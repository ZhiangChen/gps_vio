// GPSVIO.tpp
// A Class that fuses GPS and VIO
// Zhiang Chen, Aug 2020, zch@asu.edu

template <class T> 
GPSVIO<T>::GPSVIO(const ros::NodeHandle& nh): nh_(nh)
{
	// initialize ros interfaces
	gps_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_, "/gps/odom", Queue_Size);
	vio_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_, "/vio/odom", Queue_Size);
	gps_vio_sync_ = new message_filters::Synchronizer<GPS_VIO_POLICY>(GPS_VIO_POLICY(Queue_Size), *gps_sub_, *vio_sub_);
	gps_vio_sync_->registerCallback(boost::bind(&GPSVIO::gpsVioCallback_, this, _1, _2));
	
	odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/gpsvio/odom", 1);
	sub_gps_ = nh_.subscribe("/gps/odom", 1, &GPSVIO::gpsCallback, this, ros::TransportHints().tcpNoDelay());
	sub_vio_ = nh_.subscribe("/vio/odom", 1, &GPSVIO::vioCallback, this, ros::TransportHints().tcpNoDelay());

	// wait first gps+vio
	while(!flag_gpsvio_ && ros::ok())
	{
		ros::spinOnce();
		ROS_INFO("GPSVIO: Wait for gps+vio");
		ros::Duration(0.1).sleep();

	}
	// initialize graph
	graph_.updateGPSVIO(newGPSVIO_GPS_, newGPSVIO_VIO_);
	// update process variables
	gpsVioVarUpdate_();
	// start timer to update graph
	timer_ = nh_.createTimer(ros::Duration(1.0/RATE), &GPSVIO::timerCallback, this);  // RATE is defined in param.cpp
	ROS_INFO("GPSVIO has been initialized!");
}

template <class T> 
void GPSVIO<T>::gpsVioCallback_(const nav_msgs::Odometry::ConstPtr &gps_odom, const nav_msgs::Odometry::ConstPtr &vio_odom)
{
	newGPSVIO_GPS_ = *gps_odom;
	newGPSVIO_VIO_ = *vio_odom;
	flag_gpsvio_ = true;
}


template <class T>
void GPSVIO<T>::gpsCallback(const nav_msgs::Odometry::ConstPtr &gps_odom)
{
	newGPS_ = *gps_odom;
	flag_gps_ = true;
}


template <class T>
void GPSVIO<T>::vioCallback(const nav_msgs::Odometry::ConstPtr &vio_odom)
{
	newVIO_ = *vio_odom;
	flag_vio_ = true;
}

template <class T>
void GPSVIO<T>::timerCallback(const ros::TimerEvent& event)
{
	if (flag_gpsvio_)  // highest priority
	{
		// update graph with node type gps_vio
		graph_.updateGPSVIO(newGPSVIO_GPS_, newGPSVIO_VIO_);
		// update process variables
		gpsVioVarUpdate_();
	}
	else
	{
		if (flag_gps_ && flag_vio_)
		{
			// it's odd if this happens
			// we only utilize vio in this situation
			// update graph with node type vio
			graph_.updateVIO(newVIO_);
			// update process variables
			vioVarUpdate_();
		}
		else
		{
			if (flag_vio_)
			{
				// update graph with node type vio
				graph_.updateVIO(newVIO_);
				// update process variables
				vioVarUpdate_();

			}
			if (flag_gps_)
			{
				// update graph with node type gps
				graph_.updateGPS(newGPS_);
				// update process variables
				gpsVarUpdate_();
			}

		}
	}
	publishOdom_();
}

template <class T>
void GPSVIO<T>::gpsVioVarUpdate_()
{
	// all other variables will be updated because GPS+VIO has the highest priority
	oldGPS_ = newGPSVIO_GPS_;
	oldVIO_ = newGPSVIO_VIO_;
	oldGPSVIO_GPS_ = newGPSVIO_GPS_;
	oldGPSGPS_VIO_ = newGPSVIO_VIO_;
	flag_gpsvio_ = false;
	flag_gps_ = false;
	flag_vio_ = false;
	last_node_type_ = NODE_TYPE::GPS_VIO;
}

template <class T>
void GPSVIO<T>::gpsVarUpdate_()
{
	oldGPS_ = newGPS_;
	flag_gps_ = false;
	last_node_type_ = NODE_TYPE::GPS;
}

template <class T>
void GPSVIO<T>::vioVarUpdate_()
{
	oldVIO_ = newVIO_;
	flag_vio_ = false;
	last_node_type_ = NODE_TYPE::VIO;
}

template <class T>
void GPSVIO<T>::publishOdom_()
{
	odom_pub_.publish(graph_.getOdom());
}
