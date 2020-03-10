/* Probabilistic Occupancy Grid Mapping
 * by Evan Kaufman, Kuya Takami, and Taeyoung Lee
 * Flight Dynamics and Control Laboratory (FDCL)
 * Mechanical and Aerospace Engineering (MAE)
 * The George Washington University (GWU)
*/

#include "ogm_ae/Robot.hpp"
#include "ogm_ae/Mapping.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "probabilistic_occupancy_grid_mapping");
  Mapping map("mapping");

  // Let PCL TFs & Filtering Start
  usleep(1000000);

  // Multithreaded Spin: Publish Full Map & Changes
  if(map.numThreads > 1){
    ros::MultiThreadedSpinner spinner(map.numThreads);
    try{
      spinner.spin();
    }catch(std::runtime_error& e){
      ROS_ERROR("%s\nQuitting probabilistic occupancy grid mapping with %d threads...", e.what(), map.numThreads);
      ros::shutdown();
      return -1;
    }
  }

  // Single Threaded Spin: Publishing Map Changes
  else{
    try{
      ros::spin();
    }catch(std::runtime_error& e){
      ROS_ERROR("%s\nQuitting probabilistic occupancy grid mapping with 1 thread...", e.what());
      ros::shutdown();
      return -1;
    }
  }

  return 0;
}



