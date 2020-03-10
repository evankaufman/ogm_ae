#include "ogm_ae/voxel.h"

using namespace std;

class EtreExploration{

public:
  Voxel::mapInfo fullMap, rdcdMap;
  void MapProbChangesReductionCallback(const ogm_ae::UpdatedMapCells::ConstPtr& msg);//, mapInfo& rdcdMap);

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "generate_exploration_map");

  // Voxel Object
  Voxel voxel;
  string objective = "mapping";
//  TODO: add this
//  Voxel::mapInfo map3D;
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

  // TODO: add this
//  Voxel::mapInfo mapCollision;

//  map3D = voxelPtr->map;
////  *mapCollision = *voxel.reducedMap;

  // Publishers
  voxel.map.mapProbsPub = voxel.nh.advertise
      <std_msgs::Float64MultiArray>(voxel.reduction.map2DTopic, 1, true);

  voxel.ogm2DPub = voxel.nh.advertise
      <nav_msgs::OccupancyGrid>(voxel.reduction.ogmTopic, 1, true);

//  // Subscribers
//  GenSubscriber<ogm_ae::UpdatedMapCells> mapChangesSubscription;
//  voxel.mapProbsSub = voxel.nh.subscribe
//      (voxel.map.changesTopic, 1, &GenSubscriber<ogm_ae::UpdatedMapCells>::callback, &mapChangesSubscription);

//  int dummyInt = 5;
//  int* anIntPtr = &dummyInt;

  EtreExploration ee;

  // Subscription
  voxel.mapProbsSub = voxel.nh.subscribe
      (voxel.map.changesTopic, 1, &EtreExploration::MapProbChangesReductionCallback, &ee);//, voxel.reducedMap));

  ros::spinOnce();

  // ROS Spin
  while(ros::ok()){

//    cout << "sterling1" << endl;
//    // 3D Map Message (only potential changes)
//    voxel.map.changedCells = mapChangesSubscription.msg;
//    voxel.map.UpdateChangedMapCells();
//    cout << "sterling2" << endl;

//    // Min/Max Locations of Block to Update
//    // TODO: remove reduction...
//    vector<double> minCorner(voxel.reducedMap.dimension), maxCorner(voxel.reducedMap.dimension);
//    voxel.reduction.loc3D = voxel.map.MapLocationFromInd(voxel.map.changedCells.inds.front());
//    for(int i(0); i < voxel.reducedMap.dimension; i++){
//      minCorner[i] = voxel.reduction.loc3D[i];
//      if(minCorner[i] > voxel.reducedMap.minimumLocations[i])
//        minCorner[i] = voxel.reducedMap.minimumLocations[i];
//    }
//    minCorner = voxel.reducedMap.SnapLocationToMap(minCorner);
//    voxel.reduction.loc3D = voxel.map.MapLocationFromInd(voxel.map.changedCells.inds.back ());
//    for(int i(0); i < voxel.reducedMap.dimension; i++){
//      maxCorner[i] = voxel.reduction.loc3D[i];
//      if(maxCorner[i] > voxel.reducedMap.maximumLocations[i])
//        maxCorner[i] = voxel.reducedMap.maximumLocations[i];
//    }
//    maxCorner = voxel.reducedMap.SnapLocationToMap(maxCorner);
//    cout << "sterling3" << endl;

//    // Number of Cells to Update
//    vector<int> numUpdatedReducedCellsThisDim(voxel.reducedMap.dimension);
//    int numReducedCellsToUpdate(1);
//    for(int i(0); i < voxel.reducedMap.dimension; i++){
//      numUpdatedReducedCellsThisDim[i] = floor((maxCorner[i]-minCorner[i])/voxel.reducedMap.alpha+0.5)+1;
//      numReducedCellsToUpdate *= numUpdatedReducedCellsThisDim[i];
//    }
//    cout << "sterling4" << endl;

//    // Determine Which Reduced Cells Might Change
//    int ind(0);
//    vector<int> indsUpdatedReducedCellsThisDim(numReducedCellsToUpdate);
//    voxel.reducedMap.FindCellsComposingBlock(voxel.reduction.loc2D, minCorner, numUpdatedReducedCellsThisDim, indsUpdatedReducedCellsThisDim, ind);
//    cout << "sterling5" << endl;

//    // Combine Full Map Into Reduced Map
//    for(int i(0) ; i < numReducedCellsToUpdate; i++){
//      ind = indsUpdatedReducedCellsThisDim[i];
//      voxel.reducedMap.occData.data[ind] = voxel.reduction.ProbReducedMapCell(voxel.map, voxel.reducedMap, ind);
//    }
//    cout << "sterling6" << endl;

//    // Clear Robot Location & Publish the Reduced Map
//    voxel.reduction.ClearRobotLoc(voxel.explore.reqNumFreeNeighbors, voxel.map, voxel.reducedMap, voxel.numRobots, voxel.robotInformation);
//    cout << "sterling7" << endl;

//    // Update Occupancy Grid Message
//    for(int i(0); i < voxel.reducedMap.numCellsTotal; i++)
//      voxel.reduction.OGM.data[i] = floor(voxel.reducedMap.occData.data[i]*100+0.5);
//    voxel.reduction.OGM.header.stamp = ros::Time::now();
//    voxel.reduction.OGM.header.frame_id = voxel.map.frame;
//    cout << "sterling8" << endl;

//    // Publish Reduced Map
//    voxel.map.mapProbsPub.publish(voxel.reducedMap.occData);
//    voxel.ogm2DPub.   publish(voxel.reduction.OGM          );
//    cout << "sterling9" << endl;


    ros::spinOnce();
  }




  return 0;

}

void EtreExploration::MapProbChangesReductionCallback(const ogm_ae::UpdatedMapCells::ConstPtr &msg){
  return;
}




