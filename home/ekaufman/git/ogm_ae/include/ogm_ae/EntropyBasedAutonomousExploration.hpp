#ifndef EntropyBasedAutonomousExploration_HPP
#define EntropyBasedAutonomousExploration_HPP

// Voxel Shared Among ROS Packages
#include "voxel.h"

// Namespaces
using namespace std;

namespace entropy_based_autonomous_exploration{
class EntropyBasedAutonomousExploration{

public:

  typedef const std_msgs::Float64MultiArray::ConstPtr mapMsg;
  EntropyBasedAutonomousExploration();
  virtual ~EntropyBasedAutonomousExploration() = default;

protected:

  Voxel voxel;
  string objective;
  ros::Subscriber entropyMapSub, collMapSub;
  tf::TransformListener listener;
  bool collisionMapFirstLoopDone, entropyMapFirstLoopDone;
  bool explorationOK;
  ros::Time rosTimeStartNextTraj;



  // __ Functions __ \\

  void CollisionMapCallback(mapMsg& msg);
  void EntropyMapCallback  (mapMsg& msg);
  void CollisionAndEntropyMapCallback(mapMsg& msg);
  void ExploreInitParams(string& objective, Voxel::mapInfo& mapInput);

  void ExplorationCollisionUpdate(mapMsg& collMap);
  void ExplorationEntropyUpdate(Voxel::mapInfo& collisionMap, Voxel::mapInfo& entropyMap);
  void GetSensorLoc(int& robotNum);


private:


};
}

#endif // EntropyBasedAutonomousExploration_HPP
