/**
BSD 3-Clause License

Copyright (c) 2018, Khalifa University Robotics Institute
Copyright (c) 2018, Tarek Taha tarek@tarektaha.com
Copyright (c) 2018, Reem Ashour reemashour1@gmail.com
Copyright (c) 2020, Mohamed Abdelkader mohamedashraf123@gmail.com
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef SEMANTIC_MAPPER_H
#define SEMANTIC_MAPPER_H
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <kdtree/kdtree.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/Odometry.h>
#include <octomap/ColorOcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap_generator/octomap_generator.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <semantic_occupancy_mapping_3d/GetPath.h>
#include <semantic_occupancy_mapping_3d/sem_core.h>
#include <semantic_occupancy_mapping_3d/rrt_tree.h>
#include <semantics_octree/semantics_octree.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
#include <pcl_ros/impl/transforms.hpp>
#include <sstream>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include "utilities/time_profiler.h"


namespace semMAP
{

class semanticMapper
{
  public:
    semanticMapper(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    semanticMapper(){};

    ~semanticMapper();
    void posStampedCallback(const geometry_msgs::PoseStamped& pose);
    void posCallback(const geometry_msgs::PoseWithCovarianceStamped& pose);
    void odomCallback(const nav_msgs::Odometry& pose);

    bool mapperCallbackStart();

    void insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
    bool isReady();
    void setHandlers(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    bool setParams();
    void setupLog();
    void getSemanticLabelledColors();

    semMAP::Params getParams();
    //void drawPath(geometry_msgs::Pose p, int id);
    //std::istream input_file ; 
    //std::ostream output_file ; 

  protected:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber posClient_;
    ros::Subscriber posStampedClient_;
    ros::Subscriber odomClient_;

    ros::ServiceServer mapperService_;
    ros::ServiceServer toggleSemanticService;  ///<ROS service to toggle semantic color display

    bool toggleUseSemanticColor(
        std_srvs::Empty::Request& request,
        std_srvs::Empty::Response&
            response);  ///<Function to toggle whether write semantic color or rgb color as when serializing octree

    ros::Publisher fullmapPub_;  ///<ROS publisher for octomap message
    ros::Subscriber pointcloud_sub_;

    OctomapGeneratorBase* octomap_generator_;
    semMAP::semCore* mapObject;

    bool ready_;
    semMAP::Params params_;
    std::string logFilePathName_;
    std::ofstream file_path_;
    std::ofstream objects_file_path_;

    // Global variables
    octomap_msgs::Octomap map_msg_;  ///<ROS octomap message
    std::map<std::string,octomap::ColorOcTreeNode::Color> semanticColoredLabels;
    std::vector<std::string> objectsOfInterest;
    std::vector<std::string> datasetObjects;
    float confidenceThreshold;
    float numOfVisitsThreshold;
    //double globalObjectGain;
    //double globalVolumetricGain;
    //std::vector<geometry_msgs::Pose> selected_poses;
    bool debug_save_state_param ; 
    bool debug_load_state_param ; 
    std::string output_file_path_;
    std::string  input_file_path_;
    std::vector<std::array<int, 3> > colorArray ; 

 
};

}  // namespace semMAP
#endif  // SEMANTIC_MAPPER_H
