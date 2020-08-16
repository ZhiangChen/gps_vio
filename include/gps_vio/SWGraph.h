// SWGraph.h
// Sliding Window Graph with offline inference
// Zhiang Chen, Aug 2020, zch@asu.edu

#ifndef SWGRAPH_H
#define SWGRAPH_H

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/dataset.h>
#include <queue>

#include <gps_vio/param.cpp>

using namespace gtsam;
using namespace std;

struct Pose3Gaussian{
	Pose3 mean;
	noiseModel::Gaussian::shared_ptr cov;
	Pose3Gaussian(Pose3 m, noiseModel::Gaussian::shared_ptr c): mean(m), cov(c){}
};

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
	queue<Pose3> nodes_;
	queue<Value> inits_;
	queue<int> types_;

	Pose3Gaussian odom_to_pose3_(nav_msgs::Odometry odom);
};



#endif 

