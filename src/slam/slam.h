//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    slam.h
\brief   SLAM Interface
\author  Joydeep Biswas, (C) 2018
*/
//========================================================================

// GTSam includes.
// #include "gtsam/geometry/Pose2.h"
// #include "gtsam/inference/Key.h"
// #include "gtsam/slam/BetweenFactor.h"
// #include "gtsam/nonlinear/NonlinearFactorGraph.h"
// #include "gtsam/nonlinear/GaussNewtonOptimizer.h"
// #include "gtsam/nonlinear/Marginals.h"
// #include "gtsam/nonlinear/Values.h"
#include <algorithm>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#ifndef SRC_SLAM_H_
#define SRC_SLAM_H_
namespace slam {

struct PoseScan
{
  // map frame pose- this should be the optimized pose
  Eigen::Vector2f state_loc;
  float state_angle;

  // map or base_link referenced point cloud 
  std::vector<Eigen::Vector2f> point_cloud;

};

struct Voxel
{
  Eigen::Vector2f delta_loc;
  float delta_angle;
};

class SLAM {
  public:
    // Default Constructor.
    SLAM();
    // Observe a new laser scan.
    void ObserveLaser( const std::vector<float>& ranges,
                       float range_min,
                       float range_max,
                       float angle_min,
                       float angle_max );

    // Observe new odometry-reported location.
    void ObserveOdometry( const Eigen::Vector2f& odom_loc,
                          const float odom_angle );

    // Get latest map.
    std::vector<Eigen::Vector2f> GetMap();

    // Get latest robot pose.
    void GetPose (Eigen::Vector2f* loc, float* angle ) const;

    // Get latest point_cloud from map_pose_scan.
    void GetCloud ( std::vector<Eigen::Vector2f>* point_cloud ) const;

    // Get the raster of the latest point_cloud from map_pose_scan.
    void GetRaster( float* resolution, Eigen::MatrixXf* raster );

    // TODO
    void ProcessMPSwithGTSAM( std::vector<PoseScan>* mps );
    
  private:
    
    // Previous odometry-reported locations.
    Eigen::Vector2f prev_odom_loc_;
    float prev_odom_angle_;
    bool odom_initialized_;

    // List of poses and their associated scan point cloud
    std::vector<PoseScan> map_pose_scan_;
    bool map_initialized_;

    // List of poses and their associated scan point cloud of multiple runs
    std::vector<PoseScan> opt_rel_trans_;

    // Robot's maximum likelihood pose estimate
    Eigen::Vector2f state_loc_;
    float state_angle_;

    // Robot's odom at previous node
    Eigen::Vector2f prev_state_loc_;
    float prev_state_angle_;

    // Minumum translation before new scan will be registered
    float const min_trans_ = 1.25;
    // Minumum rotation before new scan will be registered
    float const min_rot_ = M_PI/6;

    // Raster dimensions
    float const raster_height_ = 8.5; // m 8.5
    float const raster_width_ = 5.5;  // m 5.5
    
    // Raster
    // Robot is at 0,0 of raster
    float const resolution_ = 0.075; // m 0.075
    int const rows =  2*raster_height_/resolution_;
    int const cols = 2*raster_width_/resolution_;
    Eigen::MatrixXf raster_{ rows+1, cols+1 };  // 2n+!
    
    // Sensor noise
    float const sigma_s_ = 0.2; // ~ 0.1-0.2

    // Voxel parameters
    int const loc_samples_ = 10; // Total samples will be 2n+1 20
    float const loc_std_dev = 0.4; // 0.2
    int const angle_samples_ = 15; // 20
    float const angle_std_dev = 0.3; // 0.2

    // Voxel cube- can be made at construction time if the odom noise is the same throughout 
    // which we assume is true
    std::vector<Voxel> voxel_cube_;

    // Factor graph container that contains relative successive poses ( read: the optimzed odom from aditional runs from CSM)
    //gtsam::NonlinearFactorGraph nlfg_;
    
    // Initial optimized value container of global pose from the first run of CSM 
    //gtsam::Values nlfg_init_;
   
};

void GenerateRaster( const std::vector<Eigen::Vector2f>& pcl,
                     const float& resolution,
                     const float sensor_noise,
                     Eigen::MatrixXf* raster_ptr);

std::vector<Eigen::Vector2f> ScanToPointCloud( const std::vector<float>& ranges,
                                               const float angle_min,
                                               const float angle_max );

std::vector<Eigen::Vector2f> TransformPointCloud( const std::vector<Eigen::Vector2f>& in,
                                                  const Eigen::Vector2f translation,
                                                  const float rotation );

double RasterWeighting( const Eigen::MatrixXf& raster,
                        const float resolution,
                        const std::vector<Eigen::Vector2f>& point_cloud );

}  // namespace slam

// void AddOdomFactor( const Eigen::Vector2f state_loc,
//                     const float state_angle , 
//                     const int index,
//                     gtsam::NonlinearFactorGraph* nlfg_ptr );

// void AddPoseInit( const Eigen::Vector2f state_loc,
//                   const float state_angle , 
//                   const int index,
//                   gtsam::Values* nlfg_init_ptr );

// void OptimizeGtsam( std::vector<slam::PoseScan> opt_rel_trans,
//                     gtsam::Values* nlfg_init_ptr );

#endif   // SRC_SLAM_H_
