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

void Navigation::ObservePointCloud( const vector<Vector2f>& point_cloud_,double time ) {
  visualization::ClearVisualizationMsg( local_viz_msg_ );
  for ( const auto& point: point_cloud_ )
  {
    visualization::DrawPoint( point, 252, local_viz_msg_ );
  }

  viz_pub_.publish( local_viz_msg_ );
  
  return;
}

double Navigation::PredictedRobotVelocity(){
  auto commandIsOld = [ this ]
                      ( const AccelerationCommand& command ) 
                      { return ros::Time::now() - command.stamp > actuation_lag_time_; }; 

  command_history_.remove_if( commandIsOld );

  float predicted_velocity = robot_vel_[0];
  for( const auto& command: command_history_)
  {
    predicted_velocity += command.acceleration * time_step_;
  }

  return predicted_velocity;
}

void Navigation::GenerateCurvatureSamples(){
  curvature_samples_.resize( 2*curvature_sample_count_ + 1 );
  for( size_t i=0; i < curvature_samples_.size(); ++i )
  {
    curvature_samples_[i] = ( -curvature_limit_+ i*(curvature_limit_/curvature_sample_count_) );
  }

  return;
}

void Navigation::Run() {
  if(!nav_complete_)
  {
    float const predicted_robot_vel = PredictedRobotVelocity();
    float const distance_to_goal = fabs(odom_loc_[0]-nav_goal_loc_[0]);
    float const distance_needed_to_stop = 
      (predicted_robot_vel*predicted_robot_vel)/(2*-min_acceleration_) + predicted_robot_vel*actuation_lag_time_.nsec/1e9; //dnts = dynamic distance + lag time distance
    
    AccelerationCommand commanded_acceleration{0.0, ros::Time::now()}; //Default "Cruise" means acceleration = 0.0

    if( distance_to_goal > distance_needed_to_stop &&
        predicted_robot_vel < max_velocity_ )
    {
      commanded_acceleration.acceleration = max_acceleration_;  // Accelerate
    }else if( distance_to_goal <= distance_needed_to_stop )
    {
      float const predicted_velocity = predicted_robot_vel + min_acceleration_*time_step_;
      commanded_acceleration.acceleration = predicted_velocity<0.0 ? 0.0 : min_acceleration_;    // Decelerate
    }

    drive_msg_.header.frame_id = "base_link";
    drive_msg_.header.stamp = commanded_acceleration.stamp;
    drive_msg_.velocity = predicted_robot_vel + commanded_acceleration.acceleration*time_step_;
    drive_msg_.curvature = 0.0;

    command_history_.push_back(commanded_acceleration);  

    drive_pub_.publish(drive_msg_);        
  }

  return;
}

// Create Helper functions here
// Milestone 1 will fill out part of this class.
// Milestone 3 will complete the rest of navigation.

}  // namespace navigation
