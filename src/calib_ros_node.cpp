// calib_ros_node.cpp
// A ros node for tracking camera external calibration
// Zhiang Chen, Nov 2020, zch@asu.edu

#include <ros/ros.h>
#include <numeric>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>


#include <gps_vio/param.cpp>

using namespace gtsam;

bool g_calib = false;
std::vector<nav_msgs::Odometry> g_GPS_vec;
std::vector<nav_msgs::Odometry> g_VIO_vec;


Pose3 odom_vec_2_Pose3(std::vector<nav_msgs::Odometry> odom_vec)
{
	std::vector<double> pos_Xs, pos_Ys, pos_Zs, orien_Xs, orien_Ys, orien_Zs, orien_Ws;
	int idx = odom_vec.size();
	while (--idx >= 0)
	{
		nav_msgs::Odometry gps_odom = odom_vec[idx];
		pos_Xs.push_back(gps_odom.pose.pose.position.x);
		pos_Ys.push_back(gps_odom.pose.pose.position.y);
		pos_Zs.push_back(gps_odom.pose.pose.position.z);
		orien_Xs.push_back(gps_odom.pose.pose.orientation.x);
		orien_Ys.push_back(gps_odom.pose.pose.orientation.y);
		orien_Zs.push_back(gps_odom.pose.pose.orientation.z);
		orien_Ws.push_back(gps_odom.pose.pose.orientation.w);
	}
	double c_pos_x = std::accumulate( pos_Xs.begin(), pos_Xs.end(), 0.0)/pos_Xs.size();
	double c_pos_y = std::accumulate( pos_Ys.begin(), pos_Ys.end(), 0.0)/pos_Ys.size();
	double c_pos_z = std::accumulate( pos_Zs.begin(), pos_Zs.end(), 0.0)/pos_Zs.size();
	double c_orien_x = std::accumulate( orien_Xs.begin(), orien_Xs.end(), 0.0)/orien_Xs.size();
	double c_orien_y = std::accumulate( orien_Ys.begin(), orien_Ys.end(), 0.0)/orien_Ys.size();
	double c_orien_z = std::accumulate( orien_Zs.begin(), orien_Zs.end(), 0.0)/orien_Zs.size();
	double c_orien_w = std::accumulate( orien_Ws.begin(), orien_Ws.end(), 0.0)/orien_Ws.size();
	//Rot3 rot(c_orien_w, c_orien_x, c_orien_y, c_orien_z);
	Rot3 rot(1, 0, 0, 0);
	Point3 point(c_pos_x, c_pos_y, c_pos_z);
	return  Pose3(rot, point);
}



void gpsVioCallback(const nav_msgs::Odometry::ConstPtr &gps_odom, const nav_msgs::Odometry::ConstPtr &vio_odom)
{
	if (g_GPS_vec.size() <= CALIB_NM )
		{
			g_GPS_vec.push_back(*gps_odom);
			g_VIO_vec.push_back(*vio_odom);
		}
		else
		{

			Pose3 gps_pose = odom_vec_2_Pose3(g_GPS_vec);
			Pose3 vio_pose = odom_vec_2_Pose3(g_VIO_vec);
			Pose3 vio2gps = gps_pose*vio_pose.inverse();

			Quaternion qua = vio2gps.rotation().toQuaternion();
			Point3 t = vio2gps.translation();

			g_calib = true;
			ROS_INFO("GPSVIO: camera external calibration done");
			std::cout<<"Position: (x,y,z) = ("<<t.x()<<" "<<t.y()<<" "<<t.z()<<")"<<std::endl;
			std::cout<<"Orientation: (x,y,z,w) = ("<<qua.x()<<" "<<qua.y()<<" "<<qua.z()<<" "<< qua.w()<<")"<<std::endl;

		}
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "gpsvio_calib");
	ros::NodeHandle nh;


	typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> GPS_VIO_POLICY;
	message_filters::Subscriber<nav_msgs::Odometry> *gps_sub; // message filter
	message_filters::Subscriber<nav_msgs::Odometry> *vio_sub;
	message_filters::Synchronizer<GPS_VIO_POLICY> *gps_vio_sync;

	int Queue_size = 10;
	gps_sub = new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/gps/odom", Queue_Size);
	vio_sub = new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/vio/odom", Queue_Size);
	gps_vio_sync = new message_filters::Synchronizer<GPS_VIO_POLICY>(GPS_VIO_POLICY(Queue_Size), *gps_sub, *vio_sub);
	gps_vio_sync->registerCallback(boost::bind(gpsVioCallback, _1, _2));

	ROS_INFO("GPSVIO: Translate your device and try to avoid rotation movement.");
	while(!g_calib && ros::ok())
	{

		//ros::spinOnce();
		//std::cout<<"."<<std::flush;
		//ros::Duration(0.1).sleep();
	}

	return 0;
}
