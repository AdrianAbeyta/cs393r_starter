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
\file    slam.cc
\brief   SLAM Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================
#include <algorithm>
#include <cmath>
#include <iostream>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include <numeric>

#include "slam.h"

#include "vector_map/vector_map.h"

// GTSam includes.
#include "gtsam/geometry/Pose2.h"
#include "gtsam/inference/Key.h"
#include "gtsam/slam/BetweenFactor.h"
#include "gtsam/nonlinear/NonlinearFactorGraph.h"
#include "gtsam/nonlinear/GaussNewtonOptimizer.h"
#include "gtsam/nonlinear/Marginals.h"
#include "gtsam/nonlinear/Values.h"
#include <gtsam/slam/PriorFactor.h>

using namespace math_util;
using Eigen::Affine2f;
using Eigen::Rotation2Df;
using Eigen::Translation2f;
using Eigen::Vector2f;
using Eigen::Vector2i;
using Eigen::MatrixXf;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using vector_map::VectorMap;

namespace slam {

SLAM::SLAM() 
  : prev_odom_loc_( 0, 0 ),
    prev_odom_angle_( 0 ),
    odom_initialized_( false ),
    map_initialized_( false ) 
  {
    // Construct voxel cube
    for( int a = -angle_samples_; a <= angle_samples_; ++a )  // Iterate over angle
    {
      float relative_angle_sample = a*angle_std_dev/angle_samples_;

      for( int ix = -loc_samples_; ix <= loc_samples_; ++ix )   // Iterate over x
      {

        for( int iy = -loc_samples_; iy <= loc_samples_; ++iy )   // Iterate over y
        {
          Vector2f relative_loc_sample( ix*loc_std_dev/loc_samples_,
                                        iy*loc_std_dev/loc_samples_ );
          
          Voxel temp{relative_loc_sample, relative_angle_sample};
          
          voxel_cube_.push_back( temp );
        }
      }
    }
  }



void SLAM::GetPose( Eigen::Vector2f* loc, float* angle ) const 
{
  if( !loc || !angle )
  {
    std::cout<<" SLAM::GetPose() was passed a nullptr! What the hell man...\n";
    return;
  }

  // Return the latest pose estimate of the robot.
  if( map_initialized_ &&
      odom_initialized_ )
  {
    *loc = state_loc_;
    *angle = state_angle_;
  }
}

void SLAM::GetCloud ( vector<Vector2f>* point_cloud ) const
{
  if( !point_cloud )
  {
    std::cout<<" SLAM::GetCloud() was passed a nullptr! What the hell man...\n";
    return;
  }
  if( map_initialized_ &&
      odom_initialized_ )
  {
    *point_cloud = map_pose_scan_.back().point_cloud;
  }

  return;
}

void SLAM::GetRaster( float* resolution, MatrixXf* raster )
{
  if( !resolution || !raster )
  {
    std::cout<<" SLAM::GetRaster() was passed a nullptr! What the hell man...\n";
    return;
  }

  *resolution = resolution_;
  *raster = raster_;
                
  return;
}


void SLAM::ObserveLaser( const vector<float>& ranges,
                         float range_min,
                         float range_max,
                         float angle_min,
                         float angle_max ) 
{
  // A new laser scan has been observed. Decide whether to add it as a pose
  // for SLAM. If decided to add, align it to the scan from the last saved pose,
  // and save both the scan and the optimized pose.
  if( !map_initialized_ &&
      odom_initialized_ )
  {
    map_pose_scan_.clear();

    // Note that the first cloud does not need to be transformed to world frame becasue it defines the world frame!
    PoseScan origin{ state_loc_, 
                     state_angle_, 
                     ScanToPointCloud(ranges, angle_min, angle_max) };

    map_pose_scan_.push_back( origin );

    map_initialized_ = true;

    prev_state_loc_ = state_loc_;
    prev_state_angle_ = state_angle_;

    GenerateRaster( origin.point_cloud,
                    resolution_,
                    sigma_s_,
                    &raster_ );

    return;
  }

  // Default && short circuits so we wont get segfault if map_pose_scan doesnt have its first element
  if( odom_initialized_ &&
      map_initialized_ &&
      ((prev_state_loc_ - state_loc_).norm() > min_trans_ ||
      fabs(prev_state_angle_ - state_angle_) > min_rot_) )
  {
    
    // Collect global pose information from the first run of CSM 
    AddPoseInit(  map_pose_scan_.back().state_loc,
                  map_pose_scan_.back().state_angle,
                  map_pose_scan_.size(),
                  &nlfg_init_ );


    // This is the raster for the scan at our last update. The goal is to maximize the correlation between
    // the scan we just got, and this raster
    GenerateRaster( map_pose_scan_.back().point_cloud,
                    resolution_,
                    sigma_s_,
                    &raster_ );

    // This converts the scan we just got to a pointcloud
    vector<Vector2f> pcl = ScanToPointCloud( ranges,
                                             angle_min,
                                             angle_max);
    // This is the mean value of our relative transform- this is the center of our voxel cube, and 
    // our goal is to find the cell in that cube that is the best relative transform
    Vector2f const relative_loc_mle = state_loc_ - prev_state_loc_ ; // mle = maximum likelihood estimate
    float const relative_angle_mle = state_angle_ - prev_state_angle_;

    prev_state_loc_ = state_loc_;
    prev_state_angle_ = state_angle_;

    // We use these as progress capture devices (read: keep most likely relative transform)
    Vector2f relative_loc( 0, 0 );
    float relative_angle = 0;
    float likelihood = -1000000000;

    for(const auto& v: voxel_cube_)
    {
     
      double const raster_likelihood = RasterWeighting( raster_,
                                                        resolution_,
                                                        TransformPointCloud(pcl, 
                                                                           (relative_loc_mle + v.delta_loc), 
                                                                           (relative_angle_mle + v.delta_angle)) );
      if( likelihood < raster_likelihood )
      { 
        likelihood = raster_likelihood;
        relative_loc = relative_loc_mle + v.delta_loc;
        relative_angle = relative_angle_mle + v.delta_angle;
      }
    }

    // DELETE
    // relative_loc = relative_loc_mle;
    // relative_angle = relative_angle_mle;
    // // DELETE
    //std::cout<< likelihood << " mle x: " << relative_loc_mle.x() << " opt x: " << relative_loc.x() << " mle y: " << relative_loc_mle.y() << " opt y: " << relative_loc.y()<< " mle a: " << relative_angle_mle << " opt a: " << relative_angle << std::endl;

    PoseScan node{ map_pose_scan_.back().state_loc + relative_loc, 
                   map_pose_scan_.back().state_angle + relative_angle, 
                   pcl };

    map_pose_scan_.push_back( node );

    return;
  }
}

void SLAM::ProcessMPSwithGTSAM(std::vector<PoseScan>* mps_ptr)
{
  if( !mps_ptr )
  {
    std::cout<<"ProcessMPSwithGTSAM was passed a nullptr! What the hell man...\n";
    return;
  }

  std::vector<PoseScan>& mps = *mps_ptr; // mps = map_pose_scan
  
  std::vector<PoseScan> opt_rel_trans; //put at end 

  //Gather mle estimate for every i to i+2 key in the dic. (read: mle = maximum likelihood estimate )
  for( size_t i = 1 ; i < mps.size()-2 ; ++i)
  {
    Eigen::Vector2f  relative_loc_mle = mps[i].state_loc  - mps[i+2].state_loc ; 
    float relative_angle_mle = mps[i].state_angle - mps[i+2].state_angle;
  
    // Raster from n pointcloud
     GenerateRaster( mps[i].point_cloud,
                     resolution_,
                     sigma_s_,
                     &raster_ );

    // We use these as progress capture devices (read: keep most likely relative transform)
    Vector2f relative_loc( 0, 0 );
    float relative_angle = 0.0;
    float likelihood = 0.0;

    for(const auto& v: voxel_cube_)
    {

      float raster_likelihood = RasterWeighting( raster_,
                                                 resolution_,
                                                 mps[i+2].point_cloud);
        if( likelihood < raster_likelihood )
        { 
          likelihood = raster_likelihood;
          relative_loc = relative_loc_mle + v.delta_loc;
          relative_angle = relative_angle_mle + v.delta_angle;
        }
    }

        PoseScan node{ mps.back().state_loc + relative_loc, 
                       mps.back().state_angle + relative_angle, 
                       mps[i+2].point_cloud  };

        opt_rel_trans.push_back( node );

        
        

    return;
  }
  
}

void SLAM::ObserveOdometry( const Vector2f& odom_loc, const float odom_angle ) 
{
  if ( !odom_initialized_ ) 
  {
    prev_odom_angle_ = odom_angle;
    prev_odom_loc_ = odom_loc;
    odom_initialized_ = true;

    return;
  }
  // Keep track of odometry to estimate how far the robot has moved between 
  // poses.
  
  const Rotation2Df base_link_rot( -prev_odom_angle_ );        // THIS HAS TO BE NEGATIVE :)
  Vector2f delta_T_bl = base_link_rot*( odom_loc - prev_odom_loc_ );  // delta_T_base_link: pres 6 slide 14
  double const delta_angle_bl = odom_angle - prev_odom_angle_;        // delta_angle_base_link: pres 6 slide 15
  
  Rotation2Df map_rot( state_angle_ );
  state_loc_ += map_rot*delta_T_bl;
  state_angle_ += delta_angle_bl;

  prev_odom_loc_ = odom_loc;
  prev_odom_angle_ = odom_angle;

  return;
}


vector<Vector2f> SLAM::GetMap() 
{
  vector<Vector2f> map;
  // Reconstruct the map as a single aligned point cloud from all saved poses
  // and their respective scans.
  
 
  for(const auto& mps: map_pose_scan_)
  {
     vector<Vector2f> temp = TransformPointCloud( mps.point_cloud,
                                                  mps.state_loc,
                                                  mps.state_angle );
    for(const auto& p: temp)
    {
      map.push_back(p);
    }
  }                         
  return map;
}


void GenerateRaster( const vector<Vector2f>& pcl,
                     const float& resolution,
                     const float sensor_noise,
                     MatrixXf* raster_ptr )
{
  // Pointcloud should be in the map frame
  if( !raster_ptr )
  {
    std::cout<<"GenerateRaster() was passed a nullptr! What the hell man...\n";
    return;
  }
  
  MatrixXf& raster = *raster_ptr;

  // Column first iteration is supposed to reduce cache misses and speed things up
  // but i didn't do benchmarks so who really cares?
  for( int j=-(raster.cols()-1)/2; j<=(raster.cols()-1)/2; ++j ) 
  {
    for( int i=-(raster.rows()-1)/2; i<=(raster.rows()-1)/2; ++i )
    {
      // Make this loop a lambda+algorithm- hand rolled loops are bad!
      int const x = i+raster.rows()/2;
      int const y = j+raster.cols()/2;
      raster( x, y ) = -100000000;  

      for(const auto& p: pcl)
      { 
        Vector2f temp( i*resolution, j*resolution );
        double const prob = ( -0.5*(temp-p).norm()*(temp-p).norm()/(sensor_noise*sensor_noise) );
        
        if( prob > raster( x, y ))
        {
          raster( x, y ) = prob;
        }

      }
    }
  }

  return;
}


vector<Vector2f> ScanToPointCloud( const vector<float>& ranges,
                                   const float angle_min,
                                   const float angle_max )
{
  // Adopted from Adrian Abeyta's work in "navigation.cc"

  Vector2f const kLaserLoc( 0.2, 0 );

  vector<Vector2f> point_cloud;
  point_cloud.reserve( ranges.size() );

  float const angle_increment = ( angle_max - angle_min )/ranges.size();
  for( size_t i=0; i<ranges.size(); ++i )
  {
    double const theta = angle_min + angle_increment*i; 
    Vector2f point( cos(theta)*ranges[i], sin(theta)*ranges[i] ); 
    point += kLaserLoc; 

    point_cloud.push_back( point );
  }
  
  return point_cloud;
}


vector<Vector2f> TransformPointCloud( const vector<Vector2f>& in,
                                      const Vector2f translation,
                                      const float rotation )
{
  vector<Vector2f> out;
  out.reserve( in.size() );

  const Rotation2Df rot( rotation );

  for(auto& p: in)
  {
    out.push_back( translation + rot*p ); // This should not be rot*( translation + p ) FML
  }

  return out;
}


double RasterWeighting( const MatrixXf& raster,
                        const float resolution,
                        const vector<Vector2f>& point_cloud )
{
  // The point cloud given to this function should be transformed back to the rasters base_link
  // It is this reverse relative transfrom that we are essentially evaluating here
  double likelihood = 0.0;
  int i, j; // row, col

  // std::cout << "rows: " << raster.rows() << " cols: " << raster.cols() << std::endl;

  for(auto& p: point_cloud)
  {
    // Check if the point is within the rasters dimensions
    if( fabs( p.x() ) < resolution*(raster.rows()-1)/2 &&
        fabs( p.y() ) < resolution*(raster.cols()-1)/2 )
    {
      i = p.x()/resolution;
      j = p.y()/resolution;

      likelihood += raster( i+(raster.rows()-1)/2, j+(raster.cols()-1)/2 );
    }
  }

  return likelihood;
}

}  // namespace slam

void AddPoseInit( const Eigen::Vector2f state_loc,
                  const float state_angle ,
                  const int index,
                  gtsam::Values* nlfg_init_ptr )
{
   if( !nlfg_init_ptr )
  {
    std::cout<<"AddPoseInit() was passed a nullptr! What the hell man...\n";
    return;
  }
  
  gtsam::Values& nlfg_init = *nlfg_init_ptr;
  
  nlfg_init.insert( index, gtsam::Pose2(state_loc[0], state_loc[1], state_angle));
  //nlfg_init.print("\nInitial Estimate:\n"); // print
}
