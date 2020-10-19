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

#include <algorithm>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#ifndef SRC_SLAM_H_
#define SRC_SLAM_H_

namespace slam {

struct PoseScan
{
  // map frame pose
  Eigen::Vector2f state_loc;
  float state_angle;

  // base_link referenced point cloud 
  // TODO- why shouldn't this be in the map frame?
  std::vector<Eigen::Vector2f> point_cloud;

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

  private:
    // Previous odometry-reported locations.
    Eigen::Vector2f prev_odom_loc_;
    float prev_odom_angle_;
    bool odom_initialized_;

    // List of poses and their associated scan point cloud
    std::vector<PoseScan> map_pose_scan_;
    bool map_initialized_;

    // Robot's maximum likelihood pose estimate
    Eigen::Vector2f state_loc_;
    float state_angle_;

    // Minumum translation before new scan will be registered
    float const min_trans_ = 0.5;
    // Minumum rotation before new scan will be registered
    float const min_rot_ = M_PI/6;

    // Raster dimensions
    float const raster_height_ = 10.0; // m
    float const raster_width_ = 10.0;  // m
    
    // High resolution raster
    float const high_resolution_ = 0.05; // m
    int const high_res_rows =  raster_height_/high_resolution_;
    int const high_res_cols = raster_width_/high_resolution_;
    Eigen::MatrixXf high_res_raster_{ high_res_rows, high_res_cols };

    // Low resolution raster
    float const low_resolution_ = 0.3; // m
    int const low_res_rows =  raster_height_/low_resolution_;
    int const low_res_cols = raster_width_/low_resolution_;
    Eigen::MatrixXf low_res_raster_{ low_res_rows, low_res_cols };


};

std::vector<Eigen::Vector2f> ScanToPointCloud( const std::vector<float>& ranges,
                                               const float angle_min,
                                               const float angle_max );

}  // namespace slam

#endif   // SRC_SLAM_H_
