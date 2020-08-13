// GPSVIO.tpp
// A Class that fuses GPS and VIO
// Zhiang Chen, Aug 2020, zch@asu.edu

template <class T> GPSVIO<T>::GPSVIO(const ros::NodeHandle& nh): nh_(nh)
{
	gps_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_, "/gps/odom", Queue_Size);
	vio_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_, "/vio/odom", Queue_Size);
	gps_vio_sync_ = new message_filters::Synchronizer<GPS_VIO_POLICY>(GPS_VIO_POLICY(Queue_Size), *gps_sub_, *vio_sub_);
	gps_vio_sync_->registerCallback(boost::bind(&GPSVIO::gpsVioCallback_, this, _1, _2));
	
}

template <class T> void GPSVIO<T>::gpsVioCallback_(const nav_msgs::Odometry::ConstPtr &gps_odom, const nav_msgs::Odometry::ConstPtr &vio_odom)
{
}
