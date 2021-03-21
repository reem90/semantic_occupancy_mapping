/*
 * Copyright 2015 Andreas Bircher, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef SEMCORE_H_
#define SEMCORE_H_

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <kdtree/kdtree.h>
#include <nav_msgs/Odometry.h>
#include <octomap_generator/octomap_generator.h>
#include <octomap_msgs/Octomap.h>
#include <ros/package.h>
#include <ros/ros.h>
//#include <semantic_exploration/SelectPose.h>
#include <semantic_occupancy_mapping_3d/rrt_tree.h>
#include <semantics_octree/semantics_octree.h>
#include <eigen3/Eigen/Dense>
#include <sstream>
#include "semantic_occupancy_mapping_3d/common.h"

#define SQ(x) ((x) * (x))
#define SQRT2 0.70711

namespace semMAP
{
class semCore : public MapBase
{
  public:
    semCore();
    semCore(OctomapGeneratorBase *manager_);
    ~semCore();
    virtual void setStateFromPoseStampedMsg(const geometry_msgs::PoseStamped &pose);
    virtual void setStateFromPoseMsg(const geometry_msgs::PoseWithCovarianceStamped &pose);
    virtual void setStateFromOdometryMsg(const nav_msgs::Odometry &pose);
    virtual void setup();
    //virtual void clear();

    void publishDebugNode(StateVec node , int Nodetype) ;
    void publishDebugGain(StateVec node ,  double gain, int type) ;
    void publishDebugStatus(StateVec node , int status) ;

  protected:
    kdtree *kdTree_;
    std::stack<StateVec> history_;
    std::vector<StateVec> bestBranchMemory_;
    int g_ID_;
    int iterationCount_;
    std::fstream fileTree_;
    std::fstream fileResponse_;
    std::fstream fileCoverageAndGain_;
    std::string logFilePath_;
    std::vector<double> inspectionThrottleTime_;
    std::ofstream outfile;

    visualization_msgs::MarkerArray sample_points_array  ;
    int marker_id ;
    bool  debugParam;
    
    std::string orientationDebugFile_;

/*private:
  friend class boost::serialization::access;

  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & g_ID_;
    //ar & history_;
    /*ar & bestBranchMemory_;
    ar & iterationCount_;
    ar & oneViewObjectFound;
    ar & markerCounter;
    ar & utilityFunction;
    ar & alphaGain;
    ar & betaGain;
    ar & marker_id ; 
    ar & debugParam; 
  }
  */
};
}  // namespace rrtNBV


#endif
