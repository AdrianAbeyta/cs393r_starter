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
  GenerateCurvatureSamples();

  fr_ = { length_-(length_-wheel_base_)/2, -width_/2 }; // front right
  br_ = { -(length_-wheel_base_)/2, -width_/2 };  // back right
  fl_ = { length_-(length_-wheel_base_)/2, width_/2 }; // front left 
  bl_ = { -(length_-wheel_base_)/2, width_/2 }; // back left

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
  path_options_.resize( 2*curvature_sample_count_ + 1 );
  for( size_t i=0; i <  path_options_.size(); ++i )
  {
    path_options_[i].curvature = -curvature_limit_ + i*(curvature_limit_/curvature_sample_count_);
  }

  return;
}

void Navigation::EvaluatePathOption( PathOption& path_option, const float& lookahead_distance ){
  // For each point in pointcloud
  // Check if its less than the lookahead distance away and if it is:
    // Check inner side collision
    // Check frontal collision
    // Check outer side collision
  Vector2f const pole( 0, 1/path_option.curvature ); 
  std::vector<float> corner_curvatures{ 1/(pole-fr_).norm(),
                                        1/(pole-br_).norm(), 
                                        1/(pole-fl_).norm(), 
                                        1/(pole-bl_).norm() };
  // Sorts the curvatures from fastest (smallest curvature) to slowest (largest curvature) corners
  sort(corner_curvatures.begin(), corner_curvatures.end());

  visualization::ClearVisualizationMsg( local_viz_msg_ );
  for(const auto& point: point_cloud_)
  { 
    float const point_curvature = 1/(pole-point).norm();
    float const point_position = atan2( point[1], point[0] ); // Thinking in polar coords!!!

    // Only consider points that we can reach on the arc given the lookahead distance
    if(fabs(lookahead_distance*path_option.curvature) > fabs(point_position))
    {
      if( point_curvature < corner_curvatures[3] &&
          point_curvature > corner_curvatures[2]) 
      {
        visualization::DrawCross( point, 0.02, 16711680, local_viz_msg_ );
      }else if( point_curvature < corner_curvatures[2] &&
                point_curvature > corner_curvatures[1] ) 
      {
        visualization::DrawPoint( point, 255, local_viz_msg_ );
      }else if( point_curvature < corner_curvatures[1] &&
                point_curvature > corner_curvatures[0] ) 
      {
        visualization::DrawCross( point, 0.02, 16711935, local_viz_msg_ );
      }
    }
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
  int path_option_id = 0;
  EvaluatePathOption(path_options_[path_option_id], 0.5);
  if(!nav_complete_)
  {
    float const predicted_robot_vel = PredictedRobotVelocity();
    float const distance_to_local_goal = fabs(odom_loc_[0]-nav_goal_loc_[0]);
    float const distance_needed_to_stop = 
      (predicted_robot_vel*predicted_robot_vel)/(2*-min_acceleration_) + predicted_robot_vel*actuation_lag_time_.nsec/1e9; //dnts = dynamic distance + lag time distance
    
    TOC(path_options_[path_option_id].curvature, predicted_robot_vel, distance_to_local_goal, distance_needed_to_stop );   
  }

  return;
}

// Create Helper functions here
// Milestone 1 will fill out part of this class.
// Milestone 3 will complete the rest of navigation.

}  // namespace navigation
