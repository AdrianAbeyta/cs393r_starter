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
\file    particle-filter.h
\brief   Particle Filter Interface
\author  Joydeep Biswas, (C) 2018
*/
//========================================================================

#include <algorithm>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "shared/math/line2d.h"
#include "shared/util/random.h"
#include "vector_map/vector_map.h"

#ifndef SRC_PARTICLE_FILTER_H_
#define SRC_PARTICLE_FILTER_H_

namespace particle_filter {

struct Particle {
  Eigen::Vector2f loc;
  float angle;
  double weight;
};

class ParticleFilter {
 public:
  // Default Constructor.
   ParticleFilter();

  // Observe a new laser scan.
  void ObserveLaser(const std::vector<float>& ranges,
                    float range_min,
                    float range_max,
                    float angle_min,
                    float angle_max);

  // Observe new odometry-reported location.
  void ObserveOdometry(const Eigen::Vector2f& odom_loc,
                       const float odom_angle);

  // Initialize the robot location.
  void Initialize(const std::string& map_file,
                  const Eigen::Vector2f& loc,
                  const float angle);

  // Return the list of particles.
  void GetParticles(std::vector<Particle>* particles) const;

  // Get robot's current location.
  void GetLocation(Eigen::Vector2f* loc, float* angle) const;
 
  // Update particle weight based on laser.
  void Update(const std::vector<float>& ranges,
              float range_min,
              float range_max,
              float angle_min,
              float angle_max,
              std::vector<Particle> *particle_set_ptr);

  // Resample particles.
  void Resample();
  // Measurement Liklihood for weight calculation
  double MeasurementLikelihood( const std::vector<float>& ranges, const std::vector<float>& predicted_ranges, const float& gamma, const float& beam_count );
  // For debugging: get predicted point cloud from current location.
  void GetPredictedPointCloud(const Eigen::Vector2f& loc,
                              const float angle,
                              int num_ranges,
                              float range_min,
                              float range_max,
                              float angle_min,
                              float angle_max,
                              std::vector<Eigen::Vector2f>* scan);

  void logLikelihoodReweight(const double &max_weight, std::vector <Particle> *particle_set );

  bool isDegenerate();

 private:

  // List of particles being tracked.
  std::vector<Particle> particles_;

  // Map of the environment.
  vector_map::VectorMap map_;

  // Random number generator.
  util_random::Random rng_;

  // Previous odometry-reported locations.
  Eigen::Vector2f prev_odom_loc_;
  float prev_odom_angle_;
  bool odom_initialized_;

///HELMS DEEP ADDITIONS///

  // Initialization spread covariance
  float const I_xx_ = 0.15;
  float const I_yy_ = 0.15;
  float const I_aa_ = 0.15;
  Eigen::Matrix3f I_;

  // Process noise (prediction) variance
  float const Q_vxvx_ = 0.15;
  float const Q_vyvy_ = 0.15;
  float const Q_vava_ = 0.15;
  Eigen::Matrix3f Q_;

  // Resempling covariance noise
  float const R_xx_ = 0.15;
  float const R_yy_ = 0.15;
  float const R_aa_ = 0.15;
  Eigen::Matrix3f R_;

  // How many beams to calculate p_z_x with
  int const num_beams_= 15;

  // Correlation between laser beams
  float const gamma_ = 1.0;
  
  // Distance after which we update particles with laser
  float const min_update_dist_=  0.1;

  // Location where we last did laser update
  Eigen::Vector2f  prev_update_loc_;
  
};
}  // namespace slam

#endif   // SRC_PARTICLE_FILTER_H_
