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
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <math.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/Bool.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <deque>
#include "geometry_msgs/PoseStamped.h"
#include "message_filters/subscriber.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "tf/transform_listener.h"

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>

#include <octomap/ColorOcTree.h>
#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <tf/transform_broadcaster.h>
#include <string>

#include <octomap/OcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <chrono>
#include <thread>

#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

//#include <mav_msgs/conversions.h>
//#include <mav_msgs/default_topics.h>
#include <semantic_occupancy_mapping_3d/GetPath.h>
//#include <waypoint_navigator/GoToPoseWaypoints.h>
#include <geometry_msgs/PoseArray.h>
//#include <controller_msgs/FlatTarget.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#define SQ(x) ((x) * (x))
#define SQRT2 0.70711

using namespace std;
using namespace octomap;

struct Params
{
    double dt;
    int numIterations;
    bool iflog;
    //int root_note_delay_indicator ; 
};

class SemanticMapper3D
{
protected:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    Params params_;
    geometry_msgs::PoseStamped currentPose, explorationViewPointPose;
    ros::Publisher explorationViewpointPub;
    //ros::Publisher rotationPublisher;
    ros::Subscriber localPoseSub;
    ros::Subscriber odomSub;
    ros::Subscriber stateSub;
    bool currentPositionUpdated;
    mavros_msgs::State currentState;

public:
    SemanticMapper3D(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    ~SemanticMapper3D();
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void stateCallback(const mavros_msgs::State::ConstPtr& msg);
    bool SetParams();
    void RunStateMachine();
    void rotateOnSpot(float duration);
    void moveOnSpot(float duration);
    void odomCallback(const nav_msgs::Odometry& pose);
};

SemanticMapper3D::~SemanticMapper3D()
{
    
}

SemanticMapper3D::SemanticMapper3D(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private)
{
    currentPositionUpdated = false;
    std::string droneName = ros::this_node::getName();
    std::string droneNameSpace = ros::this_node::getNamespace();

    ROS_INFO("Drone Name:%s NameSpace:%s", droneName.c_str(), droneNameSpace.c_str());

    explorationViewpointPub =
            nh_.advertise<geometry_msgs::PoseStamped>("semantic_viewpoint", 10);
    
    //TODO: make this a parameter and make it global
    bool use_gazebo_ground_truth_ = false;
    if(!ros::param::get("/system/localization/use_gazebo_ground_truth", use_gazebo_ground_truth_)){
        ROS_WARN("Could not get param  /system/localization/use_gazebo_ground_truth . Setting default to true");
        use_gazebo_ground_truth_ = true;
    }

    // Either use perfect positioning from gazebo, or get the px4 estimator position through mavros
    if (use_gazebo_ground_truth_)
    {
        odomSub = nh_.subscribe("odometry", 10, &SemanticMapper3D::odomCallback, this);
    }
    else
    {
        localPoseSub = nh_.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &SemanticMapper3D::poseCallback, this);
    }
    
    //rotationPublisher = nh_.advertise<controller_msgs::FlatTarget>("reference/flatsetpoint", 100);
    stateSub = nh_.subscribe("/mavros/state", 10, &SemanticMapper3D::stateCallback, this);

    if (!SemanticMapper3D::SetParams())
    {
        ROS_ERROR("Could not start. Parameters missing!");
    }
}

void SemanticMapper3D::stateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    currentState = *msg;
}

void SemanticMapper3D::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    this->currentPose = *msg;
    currentPositionUpdated = true;
}

void SemanticMapper3D::odomCallback(const nav_msgs::Odometry& pose)
{
    this->currentPose.pose = pose.pose.pose;
    currentPositionUpdated = true;
}

void SemanticMapper3D::RunStateMachine()
{
    ros::Rate loopRate(10);

    std_srvs::Empty srv;
    bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    unsigned int i = 0;

    // Trying to unpause Gazebo for 10 seconds.
    while (i <= 10 && !unpaused)
    {
        ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
        std::this_thread::sleep_for(std::chrono::seconds(1));
        unpaused = ros::service::call("/gazebo/unpause_physics", srv);
        ++i;
    }

    if (!unpaused)
    {
        ROS_FATAL("Could not wake up Gazebo.");
        return;
    }
    else
    {
        ROS_INFO("Unpaused the Gazebo simulation.");
    }

    while (!currentPositionUpdated)
    {
        ROS_INFO_THROTTLE(1, "Waiting for Pose");
        ros::spinOnce();
        loopRate.sleep();
    }

    ROS_INFO("initial Position %f , %f , %f", currentPose.pose.position.x,
             currentPose.pose.position.y, currentPose.pose.position.z);
    int iteration = 0;
    int seqNum = 0;
    
  

    // simulate the camera position publisher by broadcasting the /tf
    tf::TransformBroadcaster br;
    tf::StampedTransform transform;
    tf::TransformListener listener;
    std::string targetFrame = "local_origin";
    geometry_msgs::PoseStamped transformedPose;

    // Start mapping: The mapper is called 
    ros::Time startTime = ros::Time::now();
    while (ros::ok())
    {
        ROS_INFO("Mapping iteration %i", iteration);

        /* TODO: handling the frames need improvement.
         * Either the service call should pass the target frame and recieved the already transformed
         * target frames back, or make target frame something that could be modified in a yaml file.
         * Currently it's hard coded above
         */
        //std::vector<geometry_msgs::Pose> waypoints;
        //geometry_msgs::PoseStamped poseMsg_;
        semantic_occupancy_mapping_3d::GetPath planSrv;
        planSrv.request.header.stamp = ros::Time::now();
        planSrv.request.header.seq = static_cast<uint>(iteration);
        planSrv.request.header.frame_id = "world";
        //waypoint_navigator::GoToPoseWaypoints gotoPoseWaypointsSrv;

        ROS_INFO("Mapper Call");
        ros::service::waitForService("sem_mapper", ros::Duration(3.0));
        if (ros::service::call("sem_mapper", planSrv))
        {            
            ROS_INFO("Planner Call Successfull");
            seqNum++;    
            //ros::Duration(params_.dt).sleep();
            // planSrv.response.path[i].position.x;
           
        }
        else
        {
            ROS_WARN_THROTTLE(1, "Planner not reachable");
            ros::Duration(1.0).sleep();
        }

        if (++iteration > params_.numIterations)
        {
            ROS_INFO("Finished all iterations:%d, exiting", iteration);
            break;
        }
        ros::spinOnce();
        loopRate.sleep();
    }

    ros::Time endTime = ros::Time::now();
    double elapsed = endTime.toSec() - startTime.toSec();
    std::cout << "The mapping Time " << elapsed << std::endl;
}

bool SemanticMapper3D::SetParams()
{
    //std::string ns = ros::this_node::getName();
    //std::cout<<"Node name is:"<<ns<<"\n";
    std::string ns = "";
    bool ret = true;

    // Exploration Algorithm Params
    // 1- Termination

    params_.numIterations = 500;
    if (!ros::param::get(ns + "/mapping/num_iterations", params_.numIterations))
    {
        ROS_WARN("No number of iteration for termination specified. Looking for %s",
                 (ns + "/mapping/num_iterations").c_str());
    }

    params_.dt = 1.0;
    if (!ros::param::get(ns + "/mapping/dt", params_.dt))
    {
        ROS_WARN("Delta T between Iterations not found. Looking for %s",
                 (ns + "/mapping/dt").c_str());
    }
    params_.iflog = false;
    if (!ros::param::get(ns + "/mapping/log/on", params_.iflog))
    {
        ROS_WARN("No log is specified. Looking for %s.", (ns + "/mapping/log/on").c_str());
    }

    return ret;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "semantic_mapping_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    SemanticMapper3D mapObj(nh, nh_private);
    mapObj.RunStateMachine();
    return 0;
}
