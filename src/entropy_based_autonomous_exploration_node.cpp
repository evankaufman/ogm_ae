//#include <ogm_ae/EntropyBasedAutonomousExploration.hpp>

//using namespace entropy_based_autonomous_exploration;

#include "ogm_ae/Robot.hpp"
#include "ogm_ae/Mapping.hpp"
#include "ogm_ae/Exploration.hpp"
#include "Robot.cpp"
#include "Mapping.cpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "entropy_based_autonomous_exploration");

  Exploration explore;
  explore.StartNode();

  ROS_INFO("Starting exploration with %d threads...", explore.numThreads);
  ros::MultiThreadedSpinner spinner(explore.numThreads); // multi-threaded spinner

  try{
    spinner.spin();
  }catch(std::runtime_error& e){
    ROS_ERROR("%s\nQuitting exploration...", e.what());
    return -1;
  }
  return 0;
}



