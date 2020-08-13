// GPSVIO.h
// A Class that fuses GPS and VIO
// Zhiang Chen, Aug 2020, zch@asu.edu

#ifndef GPSVIO_H
#define GPSVIO_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "gps_vio/SWGraph.h"
#include "gps_vio/ISAMGraph.h"
#include "gps_vio/ISAM2Graph.h"
#include "gps_vio/TDGraph.h"

#include "gps_vio/param.cpp"

template <class T> class GPSVIO
{
public:
	GPSVIO(const ros::NodeHandle& nh);
	

protected:
	//ros
	ros::NodeHandle nh_;
	ros::Subscriber sub_gps_;
	ros::Subscriber sub_vio_;
	typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> GPS_VIO_POLICY;
	message_filters::Subscriber<nav_msgs::Odometry> *gps_sub_; // message filter
	message_filters::Subscriber<nav_msgs::Odometry> *vio_sub_;
	message_filters::Synchronizer<GPS_VIO_POLICY> *gps_vio_sync_;
	void gpsVioCallback_(const nav_msgs::Odometry::ConstPtr &gps_odom, const nav_msgs::Odometry::ConstPtr &vio_odom);

	//graph
	T graph_;
};

#include "gps_vio/GPSVIO.tpp"

#endif 

