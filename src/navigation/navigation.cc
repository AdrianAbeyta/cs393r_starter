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
#include "simple_queue.h"
#include "shared/math/geometry.h"
#include "vector_map/vector_map.h"

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

  map_ = vector_map::VectorMap(map_file);

  
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
  nav_goal_loc_ = loc;
  nav_goal_angle_ = angle;
  nav_complete_ = 0;
  
  path_.clear();
  MakePlan( robot_loc_, nav_goal_loc_, &path_ );

  visualization::ClearVisualizationMsg( global_viz_msg_ );
  //VisualizePath( robot_loc_, nav_goal_loc_, path_ );
  
  return;
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) { 
   robot_loc_ = loc;
   robot_angle_ = angle ;

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
  for(auto& path_option: path_options_)
  {
    const Vector2f pole( 0, 1/path_option.first.curvature ); 

    float corner_curvatures[4] = { 1/(pole - fr_).norm(),
                                 1/(pole - br_).norm(), 
                                 1/(pole - fl_).norm(), 
                                 1/(pole - bl_).norm() };
    // Sorts the curvatures from fastest (smallest curvature) to slowest (largest curvature) corners
    std::sort(corner_curvatures, corner_curvatures+4);
    // How far in radians we are looking down the arc
    float const lookahead_theta = fabs(path_option.first.curvature * lookahead_distance_);  //todo move these inside the non zero curvature option
    //Find all the points we could collide with for a given curvature
    vector<Vector2f> collision_set;
    vector<Vector2f> clearance_set;
    for(const auto& point: point_cloud)
    {
      if( path_option.first.curvature != 0){
        const bool point_in_lookahead_arc = PointInAreaOfInterestCurved(point, lookahead_theta, pole);
        const float curvature = 1/(pole - point).norm();
        if( point_in_lookahead_arc && 
            curvature < corner_curvatures[3] &&
            curvature > corner_curvatures[0] ) 
        {
          collision_set.push_back( point );
        }
        else if( point_in_lookahead_arc ){
          clearance_set.push_back(point);
        }
      }else{
        const bool point_in_lookahead_distance = PointInAreaOfInterestStraight(point, lookahead_distance_);
        //TODO change width_ to account for margin
        if( point_in_lookahead_distance &&
            fabs(point[1]) < width_ )
        {
          collision_set.push_back( point );
        }
        else if( point_in_lookahead_distance )
        {
          clearance_set.push_back(point);
        }
      }
    }

    //Find free path length by evaluating the collisions of the arc samples
    path_option.first.free_path_length = lookahead_distance_;
    int index = 0;
    for(const VehicleCorners& corners: path_option.second)
    {
      if( Collision(collision_set, corners ) )
      {
        //visualization::DrawLine(corners.fr, corners.fl, 255, local_viz_msg_ );
        //visualization::DrawLine(corners.fr, corners.br, 255, local_viz_msg_ );
        //visualization::DrawLine(corners.fl, corners.bl, 255, local_viz_msg_ );
        path_option.first.free_path_length = (index-1) * lookahead_distance_/arc_samples_;
        break;  // If there is a collision then break, because we dont need to look any further
      }else{
        //visualization::DrawLine(corners.fr, corners.fl, 0, local_viz_msg_ );
        //visualization::DrawLine(corners.fr, corners.br, 0, local_viz_msg_ );
        //visualization::DrawLine(corners.fl, corners.bl, 0, local_viz_msg_ );
      }
      ++index;
    }

    //Calculate closest point- i.e. the base link location at the end of the arc
    const float theta = (index-1)*lookahead_theta/arc_samples_;
    path_option.first.clearance = 10;
    for( const auto& point: clearance_set )
    {
      if( path_option.first.curvature != 0 )
      {
        const bool point_in_lookahead_arc = PointInAreaOfInterestCurved(point, theta, pole);
        if( point_in_lookahead_arc )
        {
          const float clearance = fabs( 1/fabs(path_option.first.curvature) - (pole - point).norm() );
          if( clearance < path_option.first.clearance ) path_option.first.clearance = clearance;
        }
      }
      else
      {
        const bool point_in_lookahead_distance = PointInAreaOfInterestStraight(point, path_option.first.free_path_length);
        if ( point_in_lookahead_distance )
        {
          const float clearance = fabs(point[1]);
          if (clearance < path_option.first.clearance ) path_option.first.clearance = clearance;
        }
      }
    }
  
    if( path_option.first.curvature != 0 )
    {
      path_option.first.closest_point = BaseLinkPropagationCurve( theta, path_option.first.curvature );
    }else{
      path_option.first.closest_point = BaseLinkPropagationStraight( path_option.first.free_path_length );
    }    
  }
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
  Vector2f base_link;

  // For each arc
  for(size_t i=0; i<path_options_.size(); ++i)  
  {
    path_options_[i].first.curvature = -curvature_limit_ + i*(curvature_limit_/curvature_sample_count_);  //Set the curvature, this is essentially the "thing" which defines an arc

    // If the arc is curved (i.e. curvature != 0)
    if(path_options_[i].first.curvature != 0){
      float const lookahead_theta = fabs(path_options_[i].first.curvature )* lookahead_distance_;

      // For each position along curved the arc
      for(int j=0; j<arc_samples_ + 1; ++j)
      {
        const float theta = j*lookahead_theta/arc_samples_;
     
        base_link = BaseLinkPropagationCurve( theta, path_options_[i].first.curvature );

        Eigen::Rotation2D<float> rot(fabs(theta));

        VehicleCorners temp;
        temp.fr = rot*fr_;
        temp.fl = rot*fl_;
        temp.bl = rot*bl_;
        temp.br = rot*br_;
        if(path_options_[i].first.curvature < 0)
        {
          temp.fr[1] *= -1.0;
          temp.fl[1] *= -1.0;
          temp.bl[1] *= -1.0;
          temp.br[1] *= -1.0;
        }
        temp.fr += base_link;
        temp.fl += base_link;
        temp.bl += base_link;
        temp.br += base_link;

        path_options_[i].second.push_back( temp );  
      }
    }else{

      // For each position along the zero curvature arc
      for(int j=0; j<arc_samples_ + 1; ++j)
      {
        const float lookahead = j*lookahead_distance_/arc_samples_;

        base_link = BaseLinkPropagationStraight( lookahead );

        VehicleCorners temp;
        temp.fr = base_link + fr_;
        temp.fl = base_link + fl_;
        temp.bl = base_link + bl_;
        temp.br = base_link + br_;

        path_options_[i].second.push_back( temp );  
      }
    }
  }

  return;
}

Vector2f Navigation::BaseLinkPropagationStraight(const float& lookahead_distance ) const {
  Vector2f base_link_location( 0, 0 );
  base_link_location[0] += lookahead_distance;
  return base_link_location; 
}

Vector2f Navigation::BaseLinkPropagationCurve(const float& theta, const float& curvature) const {
  const float radius = 1.0/fabs(curvature);

  Vector2f base_link_location (0, radius);
  base_link_location[0] += sin(theta)*radius;
  base_link_location[1] += -cos(theta)*radius;

  if (curvature < 0 ) base_link_location[1] *= -1.0;

  return base_link_location;
}

bool Navigation::PointInAreaOfInterestStraight(const Eigen::Vector2f point, const float& lookahead_distance ) const{
  if( 0 < point[0] &&
      point[0] < lookahead_distance) 
  {
    return true;
  }
  return false;
}

bool Navigation::PointInAreaOfInterestCurved(const Vector2f& point, const float& theta, const Vector2f& pole) const{
  Vector2f pole_local_point = pole-point;
  float point_theta=theta;
  if(pole[1]>0)
  {
    point_theta = atan2(pole_local_point[1], pole_local_point[0]) - M_PI/2;
  }else if(pole[1]<0){
    point_theta =  atan2(-1.0*pole_local_point[1], pole_local_point[0]) - M_PI/2;
  }else{
    std::cout<< "You gave PointInAreaOfInterestCurved() a bad pole"<<std::endl;
  }

  return 0<point_theta  && point_theta<theta ? 1 : 0;
  
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
  
  GetCarrot( &carrot_ );

  if( !nav_complete_ )
  {

    //Eigen::Vector2f carrot= {4,7};
    //std::cout << "carrot stick is: " << carrot_[0] << "," << carrot_[1] << std::endl;
    visualization::ClearVisualizationMsg( local_viz_msg_ );
    VisualizePath( robot_loc_, nav_goal_loc_, path_ );
    //visualization::DrawPoint( carrot_, 255, global_viz_msg_ ); 
    visualization::DrawPoint( carrot_, 0, local_viz_msg_ ); 
    //TODO: Check if car needs to plan a new nabigation path based on its previous plan and current location. 
    
    PathOption selected_path{path_options_[0].first}; 
    for( auto& path_option: path_options_ )
    {
  
      // -3*path_option.first.free_path_length+ // -0.5*path_option.first.clearance;
      path_option.first.cost = 0.5*(path_option.first.closest_point - carrot_).norm();
      
       if(path_option.first.cost < selected_path.cost)
       {
         selected_path = path_option.first;
       }
    }

    float const predicted_robot_vel = PredictedRobotVelocity();
    float const distance_to_local_goal = fabs(odom_loc_[0]-nav_goal_loc_[0]);
    float const distance_needed_to_stop = 
    (predicted_robot_vel*predicted_robot_vel)/(2*-min_acceleration_) + predicted_robot_vel*actuation_lag_time_.nsec/1e9; //dnts = dynamic distance + lag time distance
    
    TOC(selected_path.curvature, predicted_robot_vel, distance_to_local_goal, distance_needed_to_stop );
    viz_pub_.publish( global_viz_msg_ );
    viz_pub_.publish( local_viz_msg_ );
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

// Accepts a grid cell row and colum (cell) this function will output its map frame coordnate (x, y) (m)
Vector2f  Navigation::CellToCoord( std::pair< int, int > cell ) const
{
  Vector2f coord{cell.second*res_, cell.first*res_};

  return coord +  grid_offset_ ;
}


// Accepts a map frame coordnate (x, y) (m) this function will output a grid cell row and colum (cell)  
std::pair< int, int > Navigation::CoordToCell( Eigen::Vector2f coord ) const
{
  coord -= grid_offset_;

  const int row = coord.y()/res_;
  const int col = coord.x()/res_;

  return std::make_pair( row, col ); 
}

// Accepts a grid cell row and colum (cell) returns a node_ID 
int Navigation::CellToID( std::pair< int, int > cell ) const
{
  return cols_*cell.first + cell.second;
}

// Accepts a node_ID and returns a grid cell row and colum (cell)
std::pair< int, int >Navigation::IDToCell( int ID ) const
{
  int row = ID/cols_;
  int col = ID%cols_;

  return std::make_pair( row, col ); 
}

// https://stackoverflow.com/questions/43816484/finding-the-neighbors-of-2d-array
bool Navigation::InGrid( int row, int col ) const
{
  //Return false if row and col are negative
  if( row < 0 || 
      col < 0 ) 
  {
    return false;  
  }
  
  //Return false if values are greater than given grid.
  if( row >= rows_ || 
      col >= cols_ )
  {
    return false;    
  }

  return true;
}


std::vector<std::pair< int, int >> Navigation::FindValidNeighboors( std::pair<int,int> cell) const
{
  std::vector<std::pair< int, int >> valid_neighboors;
  // Find all surrounding cells by adding +/- 1 to col and row 
  for ( int col = cell.second-1; col <= cell.second+1; ++col)
  {
    for ( int row = cell.first-1; row <= cell.first+1; ++row)
    {
      // If the cell given is not center and its within the grid.
      if ( !((col == cell.second) && (row == cell.first)) &&
              InGrid(row, col) )
      {
        //std::cout << " cell is not center cell and its in grid!" <<std::endl;
        std::pair< int, int > center{ cell.first, cell.second };

        //std::pair< int, int > center_inflate{ cell.first+(res_*0.5), cell.second+(res_*0.5) };
        
        std::pair< int, int > neighboor{ row, col };
        const geometry::line2f line_to_neighboor{ CellToCoord(center), CellToCoord(neighboor) };
        
        // Inflate lines top and bottom.
        //const geometry::line2f line_to_neighboor_inflate{ CellToCoord(center_inflate), CellToCoord(neighboor) };
        
        
        bool intersects = false;
        //bool intersects_top = false;
        

        // Check if the line between center and neighboor intersects with map line.
        for (size_t i = 0; i < map_.lines.size(); ++i)
        {
          intersects = map_.lines[i].Intersects( line_to_neighboor );
          //intersects_top = map_.lines[i].Intersects( line_to_neighboor_inflate_up );

          if( intersects == true )
          {
            break;
          }
        }

        if( intersects == false )
        {
          valid_neighboors.push_back( neighboor );
        }

      }
    }
  }

  return valid_neighboors;
}



void Navigation::MakePlan( Eigen::Vector2f start , Eigen::Vector2f finish, std::vector<std::pair< int, int >>* path_ptr)
{
  if( !path_ptr )
  {
    std::cout<<"MakePlan was passed a nullptr! What the hell man...\n";
    return;
  }
 
  std::vector<std::pair< int, int >> &path = *path_ptr;

  // SL is set to current robo_loc and converted into an ID
  uint64_t start_ID = CellToID( CoordToCell(start) );
  uint64_t goal_ID = CellToID( CoordToCell(finish) );

  // Construct the priority queue, to use uint64_t to represent node ID, and float as the priority type.
  SimpleQueue<uint64_t, float> frontier;
  frontier.Push( start_ID, 0 );
  
  // Key: Parent Cell --- Value: Child Cell
  std::map< uint64_t, uint64_t >came_from{ {start_ID, start_ID} }; 

  // Key: Cell --- Value: Cost
  std::map< uint64_t, double >cost_so_far{ {start_ID, 0.0} };

  
  while( !frontier.Empty() )
  {
    uint64_t current_ID = frontier.Pop();
    
    if( current_ID == goal_ID )
    {
      std::cout << "Found Path!" << std::endl;
      break;
    }
    std::vector<std::pair< int, int >> valid_neighboors = FindValidNeighboors( IDToCell(current_ID) );
    for( const auto& valid_neighboor: valid_neighboors )
    { 
      float const new_cost = cost_so_far.at( current_ID ) + ( CellToCoord(IDToCell(current_ID)) - CellToCoord( valid_neighboor) ).norm();
      uint64_t valid_neighboor_ID = CellToID( valid_neighboor );
      
      if( cost_so_far.find( valid_neighboor_ID ) == cost_so_far.end() ||
          new_cost < cost_so_far.at(valid_neighboor_ID)  )
      {
        cost_so_far[valid_neighboor_ID] = new_cost;
      
        float priority = new_cost + ( CellToCoord(IDToCell(goal_ID)) - CellToCoord(valid_neighboor) ).norm();

        frontier.Push( valid_neighboor_ID, priority );
        came_from[valid_neighboor_ID] = current_ID;
      }
    }
  }
  
  uint64_t current = goal_ID;
  while( current != start_ID ) 
  {
    path.push_back( IDToCell(current) );
    current = came_from[current];
  }
  reverse( path.begin(), path.end() );
  return; 
}

void Navigation::VisualizePath( const Eigen::Vector2f start, const Eigen::Vector2f finish, std::vector<std::pair< int, int >> path )
{
  // Visualize Start
  //visualization::DrawCross( start, .25, 255, global_viz_msg_ );
  
  // Visualize Goal 
  visualization::DrawCross( finish, .25, 255, global_viz_msg_ ); // Inflated x to make issue more apparent. 

  // Itterate through path define parent child and viualize by converting cell to coordnate. 
  for ( std::size_t i=0; i<path.size()-1; ++i )  
  {
    std::pair< int, int > parent_cell = path[i];
    std::pair< int, int > child_cell = path[i+1];

    visualization::DrawLine( CellToCoord(child_cell), CellToCoord(parent_cell), 0xCB00FF, global_viz_msg_ );
  }

  return;
}

 void Navigation::GetCarrot( Eigen::Vector2f* carrot_ptr )
  {
    if( !carrot_ptr )
    {
    std::cout<<"GetCarrot was passed a nullptr! What the hell man...\n";
    return;
    }

     Eigen::Vector2f &carrot = *carrot_ptr;
     double circumference = 2*M_PI*radius_;

      for( const auto& node:path_ )
      {

        // Calculate distance between center of the circle and the given node.
        double dist_cent_to_node = ( robot_loc_-(CellToCoord(node)) ).norm();
        double dist_cent_to_endgoal = ( robot_loc_- nav_goal_loc_ ).norm();
        double dist_node_to_endgoal = ( CellToCoord(node) - nav_goal_loc_ ).norm();

        if( dist_cent_to_node > circumference && 
            dist_cent_to_node-circumference <= res_ &&
            dist_node_to_endgoal < dist_cent_to_endgoal )
        {
            Eigen::Rotation2Df rot( nav_goal_angle_ );
            Eigen::Vector2f node_to_bl = rot*CellToCoord(node) - robot_loc_;
            carrot={ node_to_bl };  
        }
      }
    return;
  }

}  // namespace navigation

//dist_node_to_endgoal > dist_cent_to_endgoal