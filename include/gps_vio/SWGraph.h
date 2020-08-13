// SWGraph.h
// Sliding Window Graph with offline inference
// Zhiang Chen, Aug 2020, zch@asu.edu

#ifndef SWGRAPH_H
#define SWGRAPH_H

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
using namespace gtsam;

class SWGraph
{
public:
	SWGraph();

private:
	NonlinearFactorGraph graph_;
};



#endif 

