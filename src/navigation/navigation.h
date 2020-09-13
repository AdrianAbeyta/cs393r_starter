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
\file    navigation.h
\brief   Interface for reference Navigation class.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <vector>
#include <list> 
  
#include "eigen3/Eigen/Dense"

#ifndef NAVIGATION_H
#define NAVIGATION_H

namespace ros {
  class NodeHandle;
}  // namespace ros

namespace navigation {

struct PathOption {
  float curvature;
  float clearance;
  float free_path_length;
  Eigen::Vector2f obstruction;
  Eigen::Vector2f closest_point;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};
struct AccelerationCommand {
  float acceleration;
  ros::Time stamp;
};

class Navigation {
 public:

   // Constructor
  explicit Navigation(const std::string& map_file, ros::NodeHandle* n);

  // Used in callback from localization to update position.
  void UpdateLocation(const Eigen::Vector2f& loc, float angle);

  // Used in callback for odometry messages to update based on odometry.
  void UpdateOdometry(const Eigen::Vector2f& loc,
                      float angle,
                      const Eigen::Vector2f& vel,
                      float ang_vel);

  // Updates based on an observed laser scan
  void ObservePointCloud(const std::vector<Eigen::Vector2f>& cloud,
                         double time);

  // Main function called continously from main
  void Run();
  // Used to set the next target pose.
  void SetNavGoal(const Eigen::Vector2f& loc, float angle);

  ////HELMS DEEP ADDITIONS//// //TODO make additional functions private
  ////HELMS DEEP ADDITIONS////
  ////HELMS DEEP ADDITIONS////
  // Predict the velocity the car will have after the lagged commands are executed
  double PredictedRobotVelocity();
  // Generate curvature samples
  /**
  * @note Should always produce one sample at -curvature_limit_, 0, and curvature_limit_
  *
  * @brief Calculate the discrete curvature samples which represent potential path direction options
  * @see Mutates curvature_samples_
  **/
  void GenerateCurvatureSamples();
  // Generate curvature samples
  /**
  * @note If there is no collision on the path the lookahead distance will equal the calculated float FreePathLength();
  *
  * @brief Calculates how far down an arc the car can travel
  * @param curvature The curvature of the path being considered
  * @param lookahead_distance How far along the arc to consider collision risk
  * @return The collision free distance the vehicle could travel along a path with given curvature
  * @see Mutates None
  **/
  float FreePathLength( const float& curvature, const float& lookahead_distance ) const;

  /**
  * @note 
  *
  * @brief Publish velocity commands based on TOC logic
  * @param curvature The commanded curvature
  * @param robot_velocity The current robot velocity- should be lag compensated
  * @param distance_to_local_goal The distance along the arc to the end of the arc
  * @param distance_needed_to_stop The distancce needed to stop given kinematic limits 
  * @see Mutates drive_msg_ and command_history_
  **/
  void TOC( const float& curvature, const float& robot_velocity, const float& distance_to_local_goal, const float& distance_needed_to_stop  );

  ////HELMS DEEP ADDITIONS////
  ////HELMS DEEP ADDITIONS////
  ////HELMS DEEP ADDITIONS////

 private:

  // Current robot location.
  Eigen::Vector2f robot_loc_;
  // Current robot orientation.
  float robot_angle_;
  // Current robot velocity.
  Eigen::Vector2f robot_vel_;
  // Current robot angular speed.
  float robot_omega_;
  // Odometry-reported robot location.
  Eigen::Vector2f odom_loc_;
  // Odometry-reported robot angle.
  float odom_angle_;

  // Whether navigation is complete.
  bool nav_complete_;
  // Navigation goal location.
  Eigen::Vector2f nav_goal_loc_;
  // Navigation goal angle.
  float nav_goal_angle_;

 
  // Navigation goal location tolerance
  float const nav_goal_loc_tol_ = 0.05; // m
  // Navigation goal angle tolerance
  float const nav_goal_angle_tol_ = 0.1; // rad

  // Max velocity
  float const max_velocity_ = 1.0; // m/s
  // Max acceleration
  float const max_acceleration_ = 4.0; // m/s2
  // Max deceleration
  float const min_acceleration_ = -4.0; // m/s2
   // Command history
  std::list<AccelerationCommand> command_history_;
  // Controller+actuation lag time
  ros::Duration const actuation_lag_time_ = ros::Duration(0.15); // s

  // Curvature - assume symmetry (i.e. max=-min)
  float const curvature_limit_ = 1.0;
  // How many samples you want on each side of zero (i.e. min to 0 and then 0 to max)
  int const curvature_sample_count_= 10; // 
  // Curvature samples
  std::vector<float> curvature_samples_;
  // Vehicle dimensions
  float const length_ = 0.75;  // m
  float const wheel_base_ = 0.60;  // m
  float const width_ = 0.45;  // m
  float const track_ = 0.35;  // m
  float const margin_ = 0.1; // m
  // Vehicle coordinates in base_link frame
  Eigen::Vector2f fr_; // front right
  Eigen::Vector2f br_;  // back right
  Eigen::Vector2f fl_; // front left 
  Eigen::Vector2f bl_; // back left

  // carrot
  float const carrot_ = 3.0; //m
  
  // Point cloud
  std::vector<Eigen::Vector2f> point_cloud_;

  // Run function call rate
  float const time_step_ = 1.0/20; // s


  ////HELMS DEEP ADDITIONS////
  ////HELMS DEEP ADDITIONS////
  ////HELMS DEEP ADDITIONS////
  
 
  
};

}  // namespace navigation

#endif  // NAVIGATION_H
