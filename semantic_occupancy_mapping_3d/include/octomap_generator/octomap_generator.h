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
#ifndef OCTOMAP_GENERATOR_H
#define OCTOMAP_GENERATOR_H

#include <octomap_generator/octomap_generator_base.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <semantics_octree/semantics_bayesian.h>
#include <semantics_octree/semantics_max.h>
#include <semantics_octree/semantics_octree.h>
#include <semantics_point_type/semantics_point_type.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>
#include <pcl_ros/impl/transforms.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include "semantic_occupancy_mapping_3d/common.h"

#define COLOR_OCTREE 0
#define SEMANTICS_OCTREE_MAX 1
#define SEMANTICS_OCTREE_BAYESIAN 2

typedef pcl::PointCloud<PointXYZRGBSemanticsBayesian> PCLSemanticsBayesian;
typedef pcl::PointCloud<PointXYZRGBSemanticsMax> PCLSemanticsMax;
typedef pcl::PointCloud<pcl::PointXYZRGB> PCLColor;

typedef octomap::ColorOcTree ColorOcTree;
typedef octomap::SemanticsOcTree<octomap::SemanticsMax> SemanticsOctreeMax;
typedef octomap::SemanticsOcTree<octomap::SemanticsBayesian> SemanticsOctreeBayesian;

typedef octomap::SemanticsOcTreeNode<octomap::SemanticsMax> SemanticsOcTreeNodeMax;
typedef octomap::SemanticsOcTreeNode<octomap::SemanticsBayesian> SemanticsOcTreeNodeBayesian;

/**
 * Templated octomap generator to generate a color octree or a semantic octree (with different fusion methods)
 * See base class for details
 * \author Xuan Zhang
 * \data Mai-July 2018
 */
template <class CLOUD, class OCTREE>
class OctomapGenerator : public OctomapGeneratorBase
{
  public:
    /**
     * \brief Constructor
     * \param nh The ros node handler to be used in OctomapGenerator
     */
    OctomapGenerator();
    OctomapGenerator(const char* filename);

    virtual ~OctomapGenerator();
    virtual void readFile(const char* filename) ; 
    virtual void writeFile(const char* filename) ; 

    virtual void setMaxRange(float max_range)
    {
        max_range_ = max_range;
    }

    virtual void setRayCastRange(float raycast_range)
    {
        raycast_range_ = raycast_range;
    }

    virtual void setClampingThresMin(float clamping_thres_min)
    {
        octomap_.setClampingThresMin(clamping_thres_min);
    }

    virtual void setClampingThresMax(float clamping_thres_max)
    {
        octomap_.setClampingThresMax(clamping_thres_max);
    }

    virtual void setResolution(float resolution)
    {
        octomap_.setResolution(resolution);
    }

    virtual void setOccupancyThres(float occupancy_thres)
    {
        octomap_.setOccupancyThres(occupancy_thres);
    }

    virtual void setProbHit(float prob_hit)
    {
        octomap_.setProbHit(prob_hit);
    }

    virtual void setProbMiss(float prob_miss)
    {
        octomap_.setProbMiss(prob_miss);
    }

    /**
     * \brief Callback to point cloud topic. Update the octomap and publish it in ROS
     * \param cloud ROS Pointcloud2 message in arbitrary frame (specified in the clouds header)
     */
    virtual void insertPointCloud(const pcl::PCLPointCloud2::Ptr& cloud,
                                  const Eigen::Matrix4f& sensorToWorld);

    virtual void insertPointCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud,
                                  const std::string& to_frame);

    virtual void setUseSemanticColor(bool use);

    virtual bool isUseSemanticColor();

    virtual octomap::AbstractOcTree* getOctree()
    {
        return &octomap_;
    }

    /**
     * \brief Save octomap to a file. NOTE: Not tested
     * \param filename The output filename
     */
    virtual bool save(const char* filename);

    virtual double getResolution() const
    {
        return octomap_.getResolution();
    }

    virtual VoxelStatus getBoundingBoxStatus(const Eigen::Vector3d& center,
                                             const Eigen::Vector3d& bounding_box_size,
                                             bool stop_at_unknown_voxel);

    virtual VoxelStatus getLineStatus(const Eigen::Vector3d& start, const Eigen::Vector3d& end);

    virtual VoxelStatus getLineStatusBoundingBox(const Eigen::Vector3d& start,
                                                 const Eigen::Vector3d& end,
                                                 const Eigen::Vector3d& bounding_box_size);

    virtual VoxelStatus getVisibility(const Eigen::Vector3d& view_point,
                                      const Eigen::Vector3d& voxel_to_test,
                                      bool stop_at_unknown_cell);

    virtual VoxelStatus getCellProbabilityPoint(const Eigen::Vector3d& point, double* probability);
    virtual octomap::ColorOcTreeNode::Color getVoxelColor(const Eigen::Vector3d& point);

    virtual int getCellConfidence(const Eigen::Vector3d& point) ;

    virtual Eigen::Vector3d getMapSize();

    virtual double getVisibilityLikelihood(const Eigen::Vector3d& view_point,
                                           const Eigen::Vector3d& voxel_to_test);

    virtual bool getRearSideVoxel(const Eigen::Vector3d& view_point,
                                  const Eigen::Vector3d& voxel_to_test);

    virtual int getCellIneterestCellType(double x, double y, double z);
  
    //virtual void octomapReadData( std::ostream& s)  ; 

    virtual double getCellIneterestGain(const Eigen::Vector3d& point);

    virtual uint getCellNumOfVisits(const Eigen::Vector3d& point);

    virtual bool lookupTransformation(const std::string& from_frame, const std::string& to_frame,
                                      const ros::Time& timestamp, Transformation* transform);
    virtual void setSematicColoredLabels(std::map<std::string,octomap::ColorOcTreeNode::Color> scl)
    {
        this->semanticColoredLabels = scl;
    }

    virtual void setObjectsOfInterest(std::vector<std::string> ooi)
    {
        this->objectsOfInterest = ooi;
    }
    virtual void setConfidenceThreshold(float ooi)
    {
        this->confidenceThreshold = ooi;
    }
    virtual void setNumOfVisitsThreshold(int ooi)
    {
        this->numOfVisitsThreshold = ooi;
    }

  protected:
    OCTREE octomap_;       ///<Templated octree instance
    float max_range_;      ///<Max range for points to be inserted into octomap
    float raycast_range_;  ///<Max range for points to perform raycasting to free unoccupied space
    void updateColorAndSemantics(CLOUD* pcl_cloud);
    tf::TransformListener tf_listener_;
    octomap::KeyRay key_ray;
    std::map<std::string,octomap::ColorOcTreeNode::Color> semanticColoredLabels;
    std::vector<std::string> objectsOfInterest;
    float confidenceThreshold;
    int numOfVisitsThreshold;


};





#endif  //OCTOMAP_GENERATOR
