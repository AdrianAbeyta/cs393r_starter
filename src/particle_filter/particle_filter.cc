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
\file    particle-filter.cc
\brief   Particle Filter Starter Code
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
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "config_reader/config_reader.h"
#include "particle_filter.h"

#include "vector_map/vector_map.h"

using geometry::line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using Eigen::Vector2f;
using Eigen::Vector2i;
using vector_map::VectorMap;

DEFINE_double(num_particles, 50, "Number of particles");

namespace particle_filter {

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false)
{
  particles_.resize(FLAGS_num_particles); 
  I_<< I_xx_, 0,     0,
       0,     I_yy_, 0,
       0,     0,     I_aa_;

  Q_<< Q_vxvx_, 0,       0,
       0,       Q_vyvy_, 0,
       0,       0,       Q_vava_;

  R_<< R_xx_, 0,     0,
       0,     R_yy_, 0,
       0,     0,     R_aa_;

}

void ParticleFilter::GetParticles(vector<Particle>* particles) const {
  *particles = particles_;
}

void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc,
                                            const float angle,
                                            int num_ranges,
                                            float range_min,
                                            float range_max,
                                            float angle_min,
                                            float angle_max,
                                            vector<Vector2f>* scan_ptr) {
Vector2f const laser_link(0.2,0);
vector<Vector2f>& scan = *scan_ptr;
scan.resize(num_ranges);

for (size_t i = 0; i < scan.size(); ++i) { 
    
    float const laser_angle = angle_min + i*(angle_max - angle_min)/num_ranges;
    float const line_x0 = loc[0] + laser_link.x()*cos(angle) + range_min*cos(angle + laser_angle);
    float const line_y0 = loc[1] + range_min*sin(angle + laser_angle);
    float const line_x1 = loc[0] + laser_link.x()*cos(angle) + range_max*cos(angle + laser_angle);
    float const line_y1 = loc[1] + range_max*sin(angle + laser_angle);
    
    // Intersection_final is updating as line shortens
    Vector2f intersection_final (line_x1,line_y1); 
    Vector2f intersection_point;

    for (size_t j = 0; j < map_.lines.size(); ++j) 
    {
      const line2f map_line = map_.lines[j];
      line2f my_line(line_x0, line_y0, intersection_final.x(), intersection_final.y());
      const bool intersects = map_line.Intersection(my_line, &intersection_point);

      if (intersects) {
        // Replace intersection_final with closer obstacle point
        intersection_final = intersection_point; 
      } 
    }  

    scan[i] = intersection_final;
  }
}

void ParticleFilter::Update(const vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                            Particle* p_ptr) {
  // Implement the update step of the particle filter here.
  // You will have to use the `GetPredictedPointCloud` to predict the expected
  // observations for each particle, and assign weights to the particles based
  // on the observation likelihood computed by relating the observation to the
  // predicted point cloud.
}

void ParticleFilter::Resample() {
  
  // Create a variable to store the new particles 
  vector <Particle> new_particle_set(FLAGS_num_particles);

  // Find incremental sum of weight vector
  vector <double> weight_sum{0.0};
  for ( const auto& particle: particles_)
  {
    weight_sum.push_back(weight_sum.back() + particle.weight);
  }

  assert(weight_sum.back() == 1.0 );
  
  // Pick new particles from old particle set
  for ( auto& new_particle: new_particle_set)
  {
    const double pick = rng_.UniformRandom(0,1);
    for(int i=0; i< FLAGS_num_particles; i++)
    {
      if (pick > weight_sum[i] && pick < weight_sum[i+1])
      {
        new_particle = particles_[i];
        new_particle.weight = 1.0/FLAGS_num_particles;
        new_particle.loc.x() += rng_.Gaussian( 0, R_(0,0) ); 
        new_particle.loc.y() += rng_.Gaussian( 0, R_(1,1) );   
        new_particle.angle += rng_.Gaussian( 0, R_(2,2) ); 
      }
    }
  }

  particles_ = move( new_particle_set );
  return;
}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
  // A new laser scan observation is available (in the laser frame)
  // Call the Update and Resample steps as necessary.
}

void ParticleFilter::ObserveOdometry(const Vector2f& odom_loc,
                                     const float odom_angle) {
  // A new odometry value is available (in the odom frame)
  // Implement the motion model predict step here, to propagate the particles
  // forward based on odometry.
  if(!odom_initialized_)
  {
    prev_odom_loc_ = odom_loc;
    prev_odom_angle_ = odom_angle;
    odom_initialized_ = true;

    return;
  }

  else
  {
    const float delta_x = odom_loc.x() - prev_odom_loc_.x();
    const float delta_y = odom_loc.y() - prev_odom_loc_.y();
    const float delta_a = odom_angle - prev_odom_angle_;

    for(auto& particle: particles_)
    {
      particle.loc.x() += rng_.Gaussian( delta_x, Q_(0,0)*fabs(delta_x) ); 
      particle.loc.y() += rng_.Gaussian( delta_y, Q_(1,1)*fabs(delta_y) ); 
      particle.angle += rng_.Gaussian( delta_a, Q_(2,2)*fabs(delta_a) ); 
    }

    prev_odom_loc_ = odom_loc;
    prev_odom_angle_ = odom_angle;

    return;
  }
}

void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
  odom_initialized_ = false;

  map_ = VectorMap("maps/"+ map_file +".txt");

  // TODO- what do you do with the map_file name?
  for(auto& particle: particles_)
  {
    particle.loc.x() = rng_.Gaussian( loc.x(), I_(0,0) );
    particle.loc.y() = rng_.Gaussian( loc.y(), I_(1,1) );
    particle.angle = rng_.Gaussian( angle, I_(2,2) );
    particle.weight = 1/FLAGS_num_particles;
  }

  return;
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, 
                                 float* angle_ptr) const {
  if(!loc_ptr || !angle_ptr)
  {
    std::cout<<"GetLocation() was passed a nullptr! What the hell man...\n";
    return;
  }

  Vector2f& loc = *loc_ptr;
  float& angle = *angle_ptr;

  loc = Vector2f(0, 0);
  angle = 0;

  for( const auto& particle: particles_ )
  {
    loc.x() += particle.loc.x();
    loc.y() += particle.loc.y();
    angle += particle.angle;
  }

  loc /= FLAGS_num_particles;
  angle /= FLAGS_num_particles;
  
  return;
}


}  // namespace particle_filter
