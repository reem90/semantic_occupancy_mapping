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
#include <chrono>
#include <thread>

#include <semantic_occupancy_mapping_3d/sem_core.h>
#include <cstdlib>


#include <semantic_occupancy_mapping_3d/semantic_mapper.h>
#include <semantic_occupancy_mapping_3d/rrt_tree.h>
#include <semantic_cloud/GetSemanticColoredLabels.h>
#include <semantic_cloud/SemanticColoredLabels.h>

#include <std_srvs/Empty.h>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
#include "semantic_occupancy_mapping_3d/common.h"

using namespace std;
using namespace Eigen;



semMAP::semanticMapper::semanticMapper(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private)
{
    if (!setParams())
    {
        ROS_ERROR("Could not start the mapper. Parameters missing!");
    }

    // Set up the topics and services
    params_.transfromedPoseDebug = nh_.advertise<geometry_msgs::PoseStamped>("transformed_pose", 100);
    // Either use perfect positioning from gazebo, or get the px4 estimator position through mavros // OR Pose of MRS System
    // todo: add the topic as a param in config file
    if (params_.use_gazebo_ground_truth_)
    {
        odomClient_ = nh_.subscribe("odometry", 10, &semMAP::semanticMapper::odomCallback, this);
    }
    else
    {
        posStampedClient_ = nh_.subscribe("/mavros/local_position/pose", 10,&semMAP::semanticMapper::posStampedCallback, this);
        posClient_ = nh_.subscribe("pose", 10, &semMAP::semanticMapper::posCallback, this);
    }
    
    // todo: add the topic as a param in config file

    // Getting params
    debug_load_state_param = false ; 
    if (!ros::param::get("debug_load_state", debug_load_state_param))
    {
        ROS_WARN("No option for debug_load_state. Looking for debug_load_state");
    }

    debug_save_state_param = false ; 
    if (!ros::param::get("debug_save_state", debug_save_state_param))
    {
        ROS_WARN("No option for debug_save_state. Looking for debug_load_state");
    }

    output_file_path_ = "~/map.bt";
    if (!ros::param::get("debug_write_file", output_file_path_))
    {
        ROS_WARN("No option for function. Looking for debug_write_file. Default is ~/map.bt.");
    }
    input_file_path_ = "~/map.bt";
    if (!ros::param::get("debug_read_file", input_file_path_))
    {
        ROS_WARN("No option for function. Looking for debug_read_file. Default is ~/map.bt.");
    }

    // Initiate octree
    if (params_.treeType_ == SEMANTICS_OCTREE_BAYESIAN || params_.treeType_ == SEMANTICS_OCTREE_MAX)
    {
        if (params_.treeType_ == SEMANTICS_OCTREE_BAYESIAN)
        {
            ROS_INFO("Semantic octomap generator [bayesian fusion]");
            if (debug_load_state_param)
                octomap_generator_ =  new OctomapGenerator<PCLSemanticsBayesian, SemanticsOctreeBayesian>(input_file_path_.c_str());
            else
                octomap_generator_ =  new OctomapGenerator<PCLSemanticsBayesian, SemanticsOctreeBayesian>();
        }
        else
        {
            ROS_INFO("Semantic octomap generator [max fusion]");
            if (debug_load_state_param)
                octomap_generator_ =  new OctomapGenerator<PCLSemanticsMax, SemanticsOctreeMax>(input_file_path_.c_str());
            else
                octomap_generator_ = new OctomapGenerator<PCLSemanticsMax, SemanticsOctreeMax>();
        }
        toggleSemanticService = nh_.advertiseService(
            "toggle_use_semantic_color", &semMAP::semanticMapper::toggleUseSemanticColor, this);
    }
    else
    {
        ROS_INFO("Color octomap generator");
        if (debug_load_state_param)
            octomap_generator_ =  new OctomapGenerator<PCLColor, ColorOcTree>(input_file_path_.c_str());
        else
            octomap_generator_ = new OctomapGenerator<PCLColor, ColorOcTree>();

    }

    fullmapPub_ = nh_.advertise<octomap_msgs::Octomap>("octomap_full", 1, true);
    pointcloud_sub_ = nh_.subscribe(params_.pointCloudTopic_, 1, &semMAP::semanticMapper::insertCloudCallback, this);


    setupLog();
    getSemanticLabelledColors();

    octomap_generator_->setClampingThresMin(params_.clampingThresMin_);
    octomap_generator_->setClampingThresMax(params_.clampingThresMax_);
    octomap_generator_->setResolution(params_.octomapResolution_);
    octomap_generator_->setOccupancyThres(params_.occupancyThres_);
    octomap_generator_->setProbHit(params_.probHit_);
    octomap_generator_->setProbMiss(params_.probMiss_);
    octomap_generator_->setRayCastRange(params_.rayCastRange_);
    octomap_generator_->setMaxRange(params_.maxRange_);
    octomap_generator_->setSematicColoredLabels(semanticColoredLabels);
    octomap_generator_->setObjectsOfInterest(objectsOfInterest);
    //octomap_generator_->setDatasetObjects(datasetObjects);
    octomap_generator_->setConfidenceThreshold(confidenceThreshold);
    octomap_generator_->setNumOfVisitsThreshold(numOfVisitsThreshold);
    
    //debug
    //params_.camboundries_        getBestEdgeDeep= nh_.advertise<visualization_msgs::Marker>("camBoundries", 10);
    //params_.fovHyperplanes       = nh_.advertise<visualization_msgs::MarkerArray>( "hyperplanes", 100 );

    std::string ns = ros::this_node::getName();
    ROS_INFO("********************* The topic name is:%s", posStampedClient_.getTopic().c_str());
    // Map Initialization Instance.
    ROS_INFO("*************************** Start mapping ******************************");
    // ###########################################################
    //mapObject = new semMAP::semCore(octomap_generator_);
    //mapObject->setParams(params_);
    // ###########################################################
    // Not yet ready. need a position msg first.
    ready_ = false;
}

semMAP::semanticMapper::~semanticMapper()
{
    if (octomap_generator_)
    {
        octomap_generator_->save(params_.octomapSavePath_.c_str());
        ROS_INFO("OctoMap saved.");
       
        if (debug_save_state_param)
            octomap_generator_->writeFile(output_file_path_.c_str()) ; 

        delete octomap_generator_;
    }
    if (file_path_.is_open())
    {
        file_path_.close();
    }
    if (objects_file_path_.is_open())
    {
        objects_file_path_.close();
    }
}

void semMAP::semanticMapper::getSemanticLabelledColors()
{
    semantic_cloud::GetSemanticColoredLabels getSemanticColoredLabels;
    ROS_INFO("Getting Semantic Colored Labels");

    ros::service::waitForService("get_semantic_colored_labels",ros::Duration(5.0));
    if(ros::service::call("get_semantic_colored_labels", getSemanticColoredLabels))
    {
        semantic_cloud::SemanticColoredLabels res = getSemanticColoredLabels.response.semantic_colored_labels;
        int j = 0 ; 
        for(auto i = res.semantic_colored_labels.begin(); i != res.semantic_colored_labels.end(); i++ )
        {
            octomap::ColorOcTreeNode::Color color = octomap::ColorOcTreeNode::Color((*i).color_r,
                                                                                    (*i).color_g,
                                                                                    (*i).color_b);
            std::array<int, 3> a = {int(color.b),int(color.g),int(color.r)};
            colorArray.push_back(a) ;
            ROS_INFO("%d %d %d ",a[0],a[1],a[2]);                                                                        
            ROS_WARN("%d %d %d ",colorArray[j][0],colorArray[j][1],colorArray[j][2]);  
            ROS_ERROR("%d %d %d ",color.b,color.g,color.r ); 
            j = j + 1;
            semanticColoredLabels.insert(std::make_pair((*i).label,color));
        }
    }
    else
    {
        ROS_WARN("Failed to get Semantic Colored Labels");
    }

    octomap::ColorOcTreeNode::Color bookColor = semanticColoredLabels["book"];
    ROS_INFO("Book Colors are:%d %d %d",bookColor.r,bookColor.g,bookColor.b);
}


void semMAP::semanticMapper::setupLog()
{
	//return true; 
}

bool semMAP::semanticMapper::toggleUseSemanticColor(std_srvs::Empty::Request& request,
                                                std_srvs::Empty::Response& response)
{
    octomap_generator_->setUseSemanticColor(!octomap_generator_->isUseSemanticColor());
   
    if (octomap_generator_->isUseSemanticColor())
        ROS_INFO("Using semantic color");
    else
        ROS_INFO("Using rgb color");

    if (octomap_msgs::fullMapToMsg(*octomap_generator_->getOctree(), map_msg_))
        fullmapPub_.publish(map_msg_);
    else
        ROS_ERROR("Error serializing OctoMap");
    return true;
}

void semMAP::semanticMapper::insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
    //ROS_INFO("Received PointCloud");
    static double last = ros::Time::now().toSec();
    if (ros::Time::now().toSec() - last > params_.pcl_throttle_)
    {
        //ROS_INFO_THROTTLE(1.0, "inserting point cloud into rrtTree");
        ros::Time tic = ros::Time::now();
        octomap_generator_->insertPointCloud(cloud_msg, params_.worldFrameId_);
        // Publish octomap
        map_msg_.header.frame_id = params_.worldFrameId_;
        map_msg_.header.stamp = cloud_msg->header.stamp;
        if (octomap_msgs::fullMapToMsg(*octomap_generator_->getOctree(), map_msg_))
            fullmapPub_.publish(map_msg_);
        else
            ROS_ERROR("Error serializing OctoMap");

        ros::Time toc = ros::Time::now();
        //ROS_INFO("PointCloud Insertion Took: %f", (toc - tic).toSec());
        last = ros::Time::now().toSec();
    }
}

bool semMAP::semanticMapper::mapperCallbackStart()
{
    
    if (!ros::ok())
    {
        ROS_INFO_THROTTLE(1, "Mapping finished.");
        return true;
    }

    if (!ready_)
    {
        ROS_ERROR_THROTTLE(1, "Mapper not set up: Mapper not ready!");
        return true;
    }

    if (octomap_generator_ == nullptr)
    {
        ROS_ERROR_THROTTLE(1, "Mapper not set up: No octomap available!");
        return true;
    }

    if (octomap_generator_->getMapSize().norm() <= 0.0)
    {
        ROS_ERROR_THROTTLE(1, "Mapper not set up: Octomap is empty!");
        return true;
    }
    return true;
}

void semMAP::semanticMapper::posStampedCallback(const geometry_msgs::PoseStamped& pose)
{
    //mapObject->setStateFromPoseStampedMsg(pose);
    // Mapper is now ready to plan.
    ready_ = true;
}

void semMAP::semanticMapper::posCallback(const geometry_msgs::PoseWithCovarianceStamped& pose)
{
    //mapObject->setStateFromPoseMsg(pose);
    // Mapper is now ready to plan.
    ready_ = true;
}

void semMAP::semanticMapper::odomCallback(const nav_msgs::Odometry& pose)
{
 
    //mapObject->setStateFromOdometryMsg(pose);
    // Mapper is now ready to plan.
    ready_ = true;
}

bool semMAP::semanticMapper::isReady()
{
    return this->ready_;
}

semMAP::Params semMAP::semanticMapper::getParams()
{
    return this->params_;
}

bool semMAP::semanticMapper::setParams()
{
    //std::string ns = ros::this_node::getName();
    //std::cout<<"Node name is:"<<ns<<"\n";
    std::string ns = "";
    bool ret = true;
    params_.use_gazebo_ground_truth_ = false;
    if (!ros::param::get(ns + "/system/localization/use_gazebo_ground_truth",
                         params_.use_gazebo_ground_truth_))
    {
        ROS_WARN("using localization ground truth is not specified in the parameters while Looking "
                 "for %s. Default is false",
                 (ns + "/system/localization/use_gazebo_ground_truth").c_str());
    }
    params_.log_ = false;
    if (!ros::param::get(ns + "/nbvp/log/on", params_.log_))
    {
        ROS_WARN("Logging is off by default. Turn on with %s: true", (ns + "/nbvp/log/on").c_str());
    }

    params_.log_throttle_ = 0.5;
    if (!ros::param::get(ns + "/nbvp/log/throttle", params_.log_throttle_))
    {
        ROS_WARN("No throttle time for logging specified. Looking for %s. Default is 0.5s.",
                 (ns + "/nbvp/log/throttle").c_str());
    }
    params_.navigationFrame_ = "world";
    if (!ros::param::get(ns + "/tf_frame", params_.navigationFrame_))
    {
        ROS_WARN("No navigation frame specified. Looking for %s. Default is 'world'.",
                 (ns + "/tf_frame").c_str());
    }
    params_.pcl_throttle_ = 0.333;
    if (!ros::param::get(ns + "/pcl_throttle", params_.pcl_throttle_))
    {
        ROS_WARN("No throttle time constant for the point cloud insertion specified. Looking for "
                 "%s. Default is 0.333.",
                 (ns + "/pcl_throttle").c_str());
    }
   
    params_.treeType_ = 1;
    if (!ros::param::get(ns + "/octomap/tree_type", params_.treeType_))
    {
        ROS_WARN("No option for function. Looking for %s. Default is 1.",
                 (ns + "/octomap/tree_type").c_str());
    }
    params_.octomapSavePath_ = "~/map.bt";
    if (!ros::param::get(ns + "/octomap/save_path", params_.octomapSavePath_))
    {
        ROS_WARN("No option for function. Looking for %s. Default is ~/map.bt.",
                 (ns + "/octomap/save_path").c_str());
    }
    params_.pointCloudTopic_ = "/semantic_pcl/semantic_pcl";
    if (!ros::param::get(ns + "/octomap/pointcloud_topic", params_.pointCloudTopic_))
    {
        ROS_WARN("No option for function. Looking for %s. Default is /semantic_pcl/semantic_pcl.",
                 (ns + "/octomap/pointcloud_topic").c_str());
    }
    params_.worldFrameId_ = "world";
    if (!ros::param::get(ns + "/octomap/world_frame_id", params_.worldFrameId_))
    {
        ROS_WARN("No option for function. Looking for %s. Default is world.",
                 (ns + "/octomap/world_frame_id").c_str());
    }
    
    params_.octomapResolution_ = static_cast<float>(0.02);
    if (!ros::param::get(ns + "/octomap/resolution", params_.octomapResolution_))
    {
        ROS_WARN("No option for function. Looking for %s. Default is 0.02",
                 (ns + "/octomap/resolution").c_str());
    }
    params_.maxRange_ = 5.0f;
    if (!ros::param::get(ns + "/octomap/max_range", params_.maxRange_))
    {
        ROS_WARN("No option for function. Looking for %s. Default is 5.0",
                 (ns + "/octomap/max_range").c_str());
    }
    params_.rayCastRange_ = 2.0f;
    if (!ros::param::get(ns + "/octomap/raycast_range", params_.rayCastRange_))
    {
        ROS_WARN("No option for function. Looking for %s. Default is 2.0",
                 (ns + "/octomap/raycast_range").c_str());
    }
    params_.clampingThresMin_ = 0.12f;
    if (!ros::param::get(ns + "/octomap/clamping_thres_min", params_.clampingThresMin_))
    {
        ROS_WARN("No option for function. Looking for %s. Default is 0.12",
                 (ns + "/octomap/clamping_thres_min").c_str());
    }
    params_.clampingThresMax_ = 0.97f;
    if (!ros::param::get(ns + "/octomap/clamping_thres_max", params_.clampingThresMax_))
    {
        ROS_WARN("No option for function. Looking for %s. Default is 0.97",
                 (ns + "/octomap/clamping_thres_max").c_str());
    }
    params_.occupancyThres_ = 0.5f;
    if (!ros::param::get(ns + "/octomap/occupancy_thres", params_.occupancyThres_))
    {
        ROS_WARN("No option for function. Looking for %s. Default is 0.5",
                 (ns + "/octomap/occupancy_thres").c_str());
    }
    params_.probHit_ = 0.7f;
    if (!ros::param::get(ns + "/octomap/prob_hit", params_.probHit_))
    {
        ROS_WARN("No option for function. Looking for %s. Default is 0.7",
                 (ns + "/octomap/prob_hit").c_str());
    }
    params_.probMiss_ = 0.4f;
    if (!ros::param::get(ns + "/octomap/prob_miss", params_.probMiss_))
    {
        ROS_WARN("No option for function. Looking for %s. Default is 0.4",
                 (ns + "/octomap/prob_miss").c_str());
    }

    if (!ros::param::get(ns + "/objects_of_interest", objectsOfInterest))
    {
        ROS_WARN("No option for function. Looking for %s. Default is empty",
                 (ns + "/objects_of_interest").c_str());
    }

   /* if (!ros::param::get(ns + "/dataset_labels_ade20k", datasetObjects))
    {
        ROS_WARN("No option for function. Looking for %s. Default is empty",
                 (ns + "/dataset_labels_ade20k").c_str());
    }*/

    if (!ros::param::get(ns + "/confidence_threshold", confidenceThreshold))
    {
        ROS_WARN("No option for function. Looking for %s. Default is empty",
                 (ns + "/confidence_threshold").c_str());
    }
    //std::cout << "confidence_threshold " << confidenceThreshold<<std::endl ;

    if (!ros::param::get(ns + "/num_of_visits_threshold", numOfVisitsThreshold))
    {
        ROS_WARN("No option for function. Looking for %s. Default is empty",
                 (ns + "/num_of_visits_threshold").c_str());
    }


    return ret;
}

