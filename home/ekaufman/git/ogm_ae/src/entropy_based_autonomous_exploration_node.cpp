#include <ogm_ae/EntropyBasedAutonomousExploration.hpp>

using namespace entropy_based_autonomous_exploration;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "entropy_based_autonomous_exploration");

  EntropyBasedAutonomousExploration exploration;
  ros::MultiThreadedSpinner spinner(2); // Use 2 threads
  spinner.spin(); // spin() will not return until the node has been shutdown

//  try{
//    ros::spin();
//  }catch(std::runtime_error& e){
//    ROS_ERROR("%s\nQuitting Entropy-Based Autonomous Exploration...", e.what());
//    return -1;
//  }
  return 0;
}



