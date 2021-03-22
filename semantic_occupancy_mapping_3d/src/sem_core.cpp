/*
 * Copyright 2015 Andreas Bircher, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef SEMCORE_HPP_
#define SEMCORE_HPP_
#include <math.h>
#include <semantic_occupancy_mapping_3d/sem_core.h>
#include <semantic_occupancy_mapping_3d/rrt_tree.h>
#include <std_srvs/Empty.h>
#include <stdio.h>
#include <tf/transform_datatypes.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <thread>
TimeProfiler timer;

using namespace std;
std::ofstream outfile;
semMAP::semCore::semCore() : semMAP::MapBase::MapBase()
{
    setup();
}

semMAP::semCore::semCore(OctomapGeneratorBase *octomap_generator_)
    : semMAP::MapBase::MapBase(octomap_generator_)
{
    //srand(time(0)); 
    setup();
}

void semMAP::semCore::setup()
{
    
    //iterationCount_ = 0;
    for (int i = 0; i < 4; i++)
    {
        inspectionThrottleTime_.push_back(ros::Time::now().toSec());
    }
    // If logging is required, set up files here
    bool ifLog = false;
    std::string ns = ros::this_node::getName();

    ros::param::get(ns + "/nbvp/log/on", ifLog);
    if (ifLog)
    {
        time_t rawtime;
        struct tm *ptm;
        time(&rawtime);
        ptm = gmtime(&rawtime);
    }

    debugParam = false;
    if (!ros::param::get("/nbvp/debug", debugParam))
    {
        ROS_WARN("Debugging is off by default. Turn on with /nbvp/debug " ) ;
    }
}

semMAP::semCore::~semCore()
{

}

void semMAP::semCore::setStateFromPoseMsg(const geometry_msgs::PoseWithCovarianceStamped &pose)
{
    // Get latest transform to the planning frame and transform the pose
    static tf::TransformListener listener;
    tf::StampedTransform transform;
    try
    {
        listener.lookupTransform(params_.navigationFrame_, pose.header.frame_id, pose.header.stamp,
                                 transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        return;
    }

    tf::Pose poseTF;
    tf::poseMsgToTF(pose.pose.pose, poseTF);
    tf::Vector3 position = poseTF.getOrigin();
    position = transform * position;
    tf::Quaternion quat = poseTF.getRotation();
    quat = transform * quat;
    /*root_[0] = position.x();
    root_[1] = position.y();
    root_[2] = position.z();
    root_[3] = tf::getYaw(quat);
*/
    // Log the vehicle response in the planning frame
    static double logThrottleTime = ros::Time::now().toSec();
    /*if (ros::Time::now().toSec() - logThrottleTime > params_.log_throttle_)
    {
        logThrottleTime += params_.log_throttle_;
        if (params_.log_)
        {
            for (int i = 0; i < root_.size() - 1; i++)
            {
                fileResponse_ << root_[i] << ",";
            }
            fileResponse_ << root_[root_.size() - 1] << "\n";
        }
    }*/
    // Update the inspected parts of the mesh using the current position
    if (ros::Time::now().toSec() - inspectionThrottleTime_[0] > params_.inspection_throttle_)
    {
        inspectionThrottleTime_[0] += params_.inspection_throttle_;
    }
}

void semMAP::semCore::setStateFromPoseStampedMsg(const geometry_msgs::PoseStamped &pose)
{
    // Get latest transform to the planning frame and transform the pose
    static tf::TransformListener listener;
    tf::StampedTransform transform;

    try
    {
        listener.lookupTransform(params_.navigationFrame_, pose.header.frame_id, pose.header.stamp,
                                 transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        return;
    }

    tf::Pose poseTF;
    tf::poseMsgToTF(pose.pose, poseTF);
    tf::Vector3 position = poseTF.getOrigin();
    position = transform * position;
    tf::Quaternion quat = poseTF.getRotation();
    quat = transform * quat;
    /*root_[0] = position.x();
    root_[1] = position.y();
    root_[2] = position.z();
    root_[3] = tf::getYaw(quat);
*/
    // debug
    geometry_msgs::PoseStamped poseMsg;
    poseMsg.header.stamp = ros::Time::now();
    poseMsg.header.frame_id = params_.navigationFrame_;
  //  poseMsg.pose.position.x = root_[0];
   // poseMsg.pose.position.y = root_[1];
  //  poseMsg.pose.position.z = root_[2];
    tf::Quaternion quat2;
  //  quat2.setEuler(0.0, 0.0, root_[3]);
    poseMsg.pose.orientation.x = quat2.x();
    poseMsg.pose.orientation.y = quat2.y();
    poseMsg.pose.orientation.z = quat2.z();
    poseMsg.pose.orientation.w = quat2.w();
    params_.transfromedPoseDebug.publish(poseMsg);

    // Log the vehicle response in the planning frame
    static double logThrottleTime = ros::Time::now().toSec();
    /*if (ros::Time::now().toSec() - logThrottleTime > params_.log_throttle_)
    {
        logThrottleTime += params_.log_throttle_;
        if (params_.log_)
        {
            for (int i = 0; i < root_.size() - 1; i++)
            {
                fileResponse_ << root_[i] << ",";
            }
            fileResponse_ << root_[root_.size() - 1] << "\n";
        }
    }*/

    // Update the inspected parts of the mesh using the current position
    if (ros::Time::now().toSec() - inspectionThrottleTime_[0] > params_.inspection_throttle_)
    {
        inspectionThrottleTime_[0] += params_.inspection_throttle_;
    }
}

void semMAP::semCore::setStateFromOdometryMsg(const nav_msgs::Odometry &pose)
{
    // Get latest transform to the planning frame and transform the pose
    static tf::TransformListener listener;
    tf::StampedTransform transform;
    try
    {
        listener.lookupTransform(params_.navigationFrame_, pose.header.frame_id, pose.header.stamp,
                                 transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        return;
    }
    tf::Pose poseTF;
    tf::poseMsgToTF(pose.pose.pose, poseTF);
    tf::Vector3 position = poseTF.getOrigin();
    position = transform * position;
    tf::Quaternion quat = poseTF.getRotation();
    quat = transform * quat;
    /*root_[0] = position.x();
    root_[1] = position.y();
    root_[2] = position.z();
    root_[3] = tf::getYaw(quat);
*/
    // Log the vehicle response in the planning frame
    static double logThrottleTime = ros::Time::now().toSec();
   /* if (ros::Time::now().toSec() - logThrottleTime > params_.log_throttle_)
    {
        logThrottleTime += params_.log_throttle_;
        if (params_.log_)
        {
            for (int i = 0; i < root_.size() - 1; i++)
            {
                fileResponse_ << root_[i] << ",";
            }
            fileResponse_ << root_[root_.size() - 1] << "\n";
        }
    }*/
    // Update the inspected parts of the mesh using the current position
    if (ros::Time::now().toSec() - inspectionThrottleTime_[0] > params_.inspection_throttle_)
    {
        inspectionThrottleTime_[0] += params_.inspection_throttle_;
    }
}


#endif
