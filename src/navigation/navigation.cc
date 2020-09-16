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
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;

using namespace math_util;
using namespace ros_helpers;

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;  
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
} //namespace

namespace navigation {

Navigation::Navigation(const string& map_file, ros::NodeHandle* n) :
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0) {
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);

  ////HELMS DEEP ADDITIONS////
  ////HELMS DEEP ADDITIONS////
  ////HELMS DEEP ADDITIONS////
  fr_ = { length_-(length_-wheel_base_)/2, -width_/2 }; // front right
  br_ = { -(length_-wheel_base_)/2, -width_/2 };  // back right
  fl_ = { length_-(length_-wheel_base_)/2, width_/2 }; // front left 
  bl_ = { -(length_-wheel_base_)/2, width_/2 }; // back left


  GenerateCurvatureSamples();

  
  //TODO check that car dimensions are logical
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
  nav_goal_loc_ = loc;
  nav_goal_angle_ = angle;

  nav_complete_ = 0;
  
  return;
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) { 
  return;
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
  odom_loc_ = loc;
  odom_angle_ = angle;                             
  robot_vel_ = vel;
  robot_omega_ = ang_vel;

  if( nav_goal_loc_[0]-loc[0] < nav_goal_loc_tol_ &&
      nav_complete_ == 0)
  {
    nav_complete_ = 1;
  }
  return;
}

void Navigation::ObservePointCloud( const vector<Vector2f>& point_cloud,double time ) {
  // visualization::ClearVisualizationMsg( local_viz_msg_ );
  // for ( const auto& point: point_cloud_ )
  // {
  //   visualization::DrawPoint( point, 252, local_viz_msg_ );
  // }

  point_cloud_ = point_cloud; 
  
  // viz_pub_.publish( local_viz_msg_ );
  
  return;
}

double Navigation::PredictedRobotVelocity(){
  auto commandIsOld = [ this ]
                      ( const AccelerationCommand& command ) 
                      { return ros::Time::now() - command.stamp > actuation_lag_time_; }; 

  command_history_.remove_if( commandIsOld );

  float predicted_velocity = robot_vel_[0];
  // Integrate acceleration to get the velocity we can expect after the actuation lag
  for( const auto& command: command_history_)
  {
    predicted_velocity += command.acceleration * time_step_;
  }

  return predicted_velocity;
}

void Navigation::GenerateCurvatureSamples(){
  path_options_.resize( curvature_sample_count_ );
  //Calculate path for positive curvatures because the math is simple
  for( size_t i=0; i <  path_options_.size(); ++i )
  {
    path_options_[i].first.curvature = curvature_limit_ - i*(curvature_limit_/curvature_sample_count_);

    Vector2f const pole( 0, 1/path_options_[i].first.curvature ); 
    float const lookahead_theta = path_options_[i].first.curvature * lookahead_distance_;

    for(int j=0; j<arc_samples_ + 1; ++j)
    {
      const float theta = j*lookahead_theta/arc_samples_;

      Eigen::Rotation2D<float> rot2(theta);

      Vector2f base_link;
      base_link[0] = pole[0]+sin(theta)*pole.norm();
      base_link[1] = pole[1]-cos(theta)*pole.norm();
      
      VehicleCorners temp;
      temp.fr = base_link + rot2*fr_;
      temp.fl = base_link + rot2*fl_;
      temp.bl = base_link + rot2*bl_;
      temp.br = base_link + rot2*br_;

      path_options_[i].second.push_back( temp );    
    }
  }
  // Zero curvature option, unique case so its calculated individually
  PathOption zero_curvature{};
  zero_curvature.curvature = 0.0;
  std::vector<VehicleCorners> zero_curvature_corners(arc_samples_+1);
  for(int j=0; j< arc_samples_+1; ++j)
  {
    const float lookahead = j*lookahead_distance_/arc_samples_;

    Vector2f base_link;
    base_link[0] = lookahead;
    base_link[1] = 0;
    VehicleCorners temp;
    temp.fr = base_link + fr_;
    temp.fl = base_link + fl_;
    temp.bl = base_link + bl_;
    temp.br = base_link + br_;
    zero_curvature_corners[j]=temp;
  }
  path_options_.push_back(std::make_pair(zero_curvature, zero_curvature_corners));

  // Reflect across x axis for negative curvatures instead of actually calculating it
  for( int i=curvature_sample_count_; i >=  0; --i )
  {
    PathOption reflected_curvature{};
    reflected_curvature.curvature = -path_options_[i].first.curvature; 
    std::vector<VehicleCorners> reflected_corners;
    for(int j=0; j< arc_samples_+1; ++j)
    {
      // Multiplying the y coordinate by negative 1 is a reflection about the x axis
      VehicleCorners temp;
      temp.fr = path_options_[i].second[j].fr;
      temp.fr[1] *=-1;
      temp.fl = path_options_[i].second[j].fl;
      temp.fl[1] *=-1;
      temp.bl = path_options_[i].second[j].bl;
      temp.bl[1] *=-1;
      temp.br = path_options_[i].second[j].br;
      temp.br[1] *=-1;
      reflected_corners.push_back(temp);
    }
    path_options_.push_back(std::make_pair(reflected_curvature, reflected_corners));
  }

  return;
}

void Navigation::EvaluatePathOption( std::pair< PathOption, std::vector<VehicleCorners> >& path_option, const float& lookahead_distance ){
  Vector2f const pole( 0, 1/path_option.first.curvature ); 
  float corner_curvatures[4] = { 1/(pole - fr_).norm(),
                                 1/(pole - br_).norm(), 
                                 1/(pole - fl_).norm(), 
                                 1/(pole - bl_).norm() };
  // Sorts the curvatures from fastest (smallest curvature) to slowest (largest curvature) corners
  std::sort(corner_curvatures, corner_curvatures+4);

  visualization::ClearVisualizationMsg( local_viz_msg_ );
  vector<Vector2f> collision_set;
  for(const auto& point: point_cloud_)
  {
    float const curvature = 1/(pole - point).norm();
    if( curvature < corner_curvatures[3] &&
        curvature > corner_curvatures[0] ) 
    {
      collision_set.push_back(point);
      visualization::DrawPoint( point, 255, local_viz_msg_ );
    } 
  }

  path_option.first.free_path_length = lookahead_distance_;
  int index = 0;
  for(const VehicleCorners& corners: path_option.second)
  {
    if( Collision(collision_set, corners ) )
    {
      // visualization::DrawLine(corners.fr, corners.fl, 255, local_viz_msg_ );
      // visualization::DrawLine(corners.fr, corners.br, 255, local_viz_msg_ );
      // visualization::DrawLine(corners.fl, corners.bl, 255, local_viz_msg_ );
      path_option.first.free_path_length = index * lookahead_distance_/arc_samples_;
      break;  // If there is a collision then break, because we dont need to look any further
    }else{
      // visualization::DrawLine(corners.fr, corners.fl, 0, local_viz_msg_ );
      // visualization::DrawLine(corners.fr, corners.br, 0, local_viz_msg_ );
      // visualization::DrawLine(corners.fl, corners.bl, 0, local_viz_msg_ );
    }
    ++index;
  }

  viz_pub_.publish( local_viz_msg_ );

  return;
}

void Navigation::TOC( const float& curvature, const float& robot_velocity, const float& distance_to_local_goal, const float& distance_needed_to_stop ){
  AccelerationCommand commanded_acceleration{0.0, ros::Time::now()}; //Defaults to "Cruise"- means acceleration = 0.0

  if( distance_to_local_goal > distance_needed_to_stop &&
      robot_velocity < max_velocity_ )
  {
    commanded_acceleration.acceleration = max_acceleration_;  // Accelerate
  }else if( distance_to_local_goal <= distance_needed_to_stop )
  {
    float const predicted_velocity = robot_velocity + min_acceleration_*time_step_;
    commanded_acceleration.acceleration = predicted_velocity<0.0 ? 0.0 : min_acceleration_;    // Decelerate
  }

  drive_msg_.header.frame_id = "base_link";
  drive_msg_.header.stamp = commanded_acceleration.stamp;
  drive_msg_.velocity = robot_velocity + commanded_acceleration.acceleration*time_step_;
  drive_msg_.curvature = curvature;

  command_history_.push_back(commanded_acceleration);  

  drive_pub_.publish(drive_msg_);
}

void Navigation::Run() {
  int path_option_id = 5;
  EvaluatePathOption(path_options_[path_option_id], 2.0);
  if(!nav_complete_)
  {
    float const predicted_robot_vel = PredictedRobotVelocity();
    float const distance_to_local_goal = fabs(odom_loc_[0]-nav_goal_loc_[0]);
    float const distance_needed_to_stop = 
      (predicted_robot_vel*predicted_robot_vel)/(2*-min_acceleration_) + predicted_robot_vel*actuation_lag_time_.nsec/1e9; //dnts = dynamic distance + lag time distance
    
    TOC(path_options_[path_option_id].first.curvature, predicted_robot_vel, distance_to_local_goal, distance_needed_to_stop );   
  }

  return;
}

// Create Helper functions here
// Milestone 1 will fill out part of this class.
bool Collision(const vector<Vector2f>& obstacle_set, const VehicleCorners& rectangle)
{
  for(const auto& obstacle: obstacle_set)
  {
    //https://stackoverflow.com/questions/2752725/finding-whether-a-point-lies-inside-a-rectangle-or-not
    float const ABAM = (rectangle.br-rectangle.fr).dot(obstacle-rectangle.fr);
    float const ABAB = (rectangle.br-rectangle.fr).dot(rectangle.br-rectangle.fr);
    float const BCBM = (rectangle.bl-rectangle.br).dot(obstacle-rectangle.br);
    float const BCBC = (rectangle.bl-rectangle.br).dot(rectangle.bl-rectangle.br);
    if (0 <= ABAM && ABAM <= ABAB && 0 <= BCBM && BCBM <= BCBC)
    {
      return 1;
    }
  }
  return 0;
}
// Milestone 3 will complete the rest of navigation.


}  // namespace navigation




