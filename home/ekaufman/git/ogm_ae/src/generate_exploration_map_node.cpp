#include "ogm_ae/voxel.h"

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "generate_exploration_map");

  // Voxel Object
  Voxel voxel;
  string objective = "mapping";
  int numSensors;

  // Sleep for External Initializations/Parameter Settings
  usleep(3000000);

  // Private Parameter: Map Goal (collision or entropy)
  ros::NodeHandle privateNH("~");
  privateNH.param("map_goal", voxel.reduction.goal, voxel.reduction.goal);

  // Public Parameters/Initializations for both Mapping & Exploration
  voxel.GetRobotParameters(objective, numSensors);
  voxel.ReduceMapParamsInit();
  ros::spinOnce();

  // Publishers
  voxel.map.mapProbsPub = voxel.nh.advertise
      <std_msgs::Float64MultiArray>(voxel.reduction.map2DTopic, 1, true);

  voxel.ogm2DPub = voxel.nh.advertise
      <nav_msgs::OccupancyGrid>(voxel.reduction.ogmTopic, 1, true);

  // Subscription
  if(voxel.map.trackDiff)// update map by changes
    voxel.mapProbsSub = voxel.nh.subscribe
        (voxel.map.changesTopic    , 1, &Voxel::MapProbChangesReductionCallback, &voxel);
  else// update entire copy of the map
    voxel.mapProbsSub = voxel.nh.subscribe
        (voxel.reduction.map3DTopic, 1, &Voxel::MapProbsReductionCallback      , &voxel);

  // ROS Spin
  try{
    ros::spin();
  }catch(std::runtime_error& e){
    ROS_ERROR("%s", e.what());
    return -1;
  }
  return 0;

}




