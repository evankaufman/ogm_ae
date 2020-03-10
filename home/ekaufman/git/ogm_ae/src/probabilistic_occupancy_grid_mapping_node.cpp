/* Probabilistic Occupancy Grid Mapping
 * by Evan Kaufman, Kuya Takami, and Taeyoung Lee
 * Flight Dynamics and Control Laboratory (FDCL)
 * Mechanical and Aerospace Engineering (MAE)
 * The George Washington University (GWU)
 * Contracted under the US Naval Research Laboratory (NRL)
*/

#include <ogm_ae/ProbabilisticOccupancyGridMapping.hpp>

using namespace probabilistic_occupancy_grid_mapping;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "probabilistic_occupancy_grid_mapping");

  ProbabilisticOccupancyGridMapping mapping;

  try{
    ros::spin();
  }catch(std::runtime_error& e){
    ROS_ERROR("%s\nQuitting Probabilistic Occupancy Grid Mapping...", e.what());
    return -1;
  }
  return 0;
}



