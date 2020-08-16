// SWGraph.cpp
// Sliding Window Graph with offline inference
// Zhiang Chen, Aug 2020, zch@asu.edu

#include "gps_vio/SWGraph.h"

SWGraph::SWGraph()
{

}

void SWGraph::updateGPSVIO(nav_msgs::Odometry gps_odom, nav_msgs::Odometry vio_odom)
{
	Pose3 pose;
	noiseModel::Gaussian::shared_ptr cov;


}

void SWGraph::updateGPS(nav_msgs::Odometry gps_odom)
{

}

void SWGraph::updateVIO(nav_msgs::Odometry vio_odom)
{

}

nav_msgs::Odometry SWGraph::getOdom()
{
	nav_msgs::Odometry currentOdom;
	return currentOdom;
}

Pose3Gaussian SWGraph::odom_to_pose3_(nav_msgs::Odometry odom)
{
	geometry_msgs::Quaternion quat = odom.pose.pose.orientation;
	geometry_msgs::Point p = odom.pose.pose.position;
	Rot3 rot(quat.w, quat.x, quat.y, quat.z);
	Point3 point(p.x, p.y, p.z);
	Pose3 pose(rot, point);

	noiseModel::Gaussian::shared_ptr cov = noiseModel::Diagonal::Variances(
			(Vector(6) << odom.pose.covariance[0], odom.pose.covariance[7], odom.pose.covariance[14],
	    		  odom.pose.covariance[21], odom.pose.covariance[28], odom.pose.covariance[35]).finished());

	return Pose3Gaussian(pose, cov);

}
