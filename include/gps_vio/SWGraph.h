// SWGraph.h
// Sliding Window Graph with offline inference
// Zhiang Chen, Aug 2020, zch@asu.edu

#ifndef SWGRAPH_H
#define SWGRAPH_H

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <nav_msgs/Odometry.h>

#include <gps_vio/param.cpp>

using namespace gtsam;

class SWGraph
{
public:
	SWGraph();
	void updateGPSVIO(nav_msgs::Odometry gps_odom, nav_msgs::Odometry vio_odom);
	void updateGPS(nav_msgs::Odometry gps_odom);
	void updateVIO(nav_msgs::Odometry vio_odom);
	nav_msgs::Odometry getOdom();

private:
	NonlinearFactorGraph graph_;
};



#endif 

