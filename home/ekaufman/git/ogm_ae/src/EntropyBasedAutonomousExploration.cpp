#include <ogm_ae/EntropyBasedAutonomousExploration.hpp>

#include <tf/transform_listener.h>

// Namespaces
using namespace std;

namespace entropy_based_autonomous_exploration {
EntropyBasedAutonomousExploration::EntropyBasedAutonomousExploration():
  objective("exploration")
{

  // Parameters & Initializations
  voxel.GetRobotParameters(objective, voxel.collisionMap.numSensors);
  voxel.GetExplorationParameters();
  voxel.collisionMap.GetSensorParameters(objective, voxel.numRobots, voxel.robotInformation);
  ros::param::get("/exploration/cell_comb_process",  voxel.reduction.cellCombProcess);

  // multi-threaded example: https://answers.ros.org/question/235144/nodehandler-in-multiple-threads/

//  voxel.mapProbsSub = voxel.nh.subscribe
//      (voxel.map.changesTopic, 1, &Voxel::MapProbChangesCallback, &voxel);


  if(voxel.reduction.cellCombProcess == "free_entropy"){
    voxel.GetRobotParameters(objective, voxel.entropyMap.numSensors);
    voxel.entropyMap.GetSensorParameters(objective, voxel.numRobots, voxel.robotInformation);
    objective = "collision";
    ExploreInitParams(objective, voxel.collisionMap);
    collMapSub
        = voxel.nh.subscribe(voxel.collisionMap.occTopic, 1,
                             &EntropyBasedAutonomousExploration::CollisionMapCallback, this);
    objective = "entropy"  ;
    ExploreInitParams(objective, voxel.entropyMap);
    entropyMapSub
        = voxel.nh.subscribe(voxel.entropyMap.occTopic, 1,
                             &EntropyBasedAutonomousExploration::EntropyMapCallback, this);
    ROS_INFO("Both collision & entropy maps being used.");
  }
  else{// use variables with "collision" exclusively
    objective = "collision";
    ExploreInitParams(objective, voxel.collisionMap);
    collMapSub
        = voxel.nh.subscribe(voxel.collisionMap.occTopic, 1,
                             &EntropyBasedAutonomousExploration::CollisionAndEntropyMapCallback, this);
    ROS_INFO("Same map for collision & entropy being used.");
  }
  voxel.ExplorationInit();
  for(int i(0); i < voxel.numRobots; i++)
    voxel.collisionMap.sensorInformation[i].frame = voxel.robotInformation[i].exploreFrame;
  collisionMapFirstLoopDone = false; entropyMapFirstLoopDone = false;


  // __ Publishers __ \\

  // Robot Non-Specific: Entropy Marker Visualization
  ros::param::get("/exploration/visualization/pub_paths", voxel.explore.publishPaths);
  if(voxel.explore.vizCandEntropies)
    voxel.explore.candidatesPub  = voxel.nh.advertise
        <visualization_msgs::MarkerArray>(voxel.explore.entropyMarkerTopic , 1, true);
  if(voxel.explore.vizCandEntropiesDists)
    voxel.explore.candidatesDiscountedPub = voxel.nh.advertise
        <visualization_msgs::MarkerArray>(voxel.explore.distInfoMarkerTopic, 1, true);

  // Robot Specific
  for(int i(0); i < voxel.numRobots; i++){

    // Desired Paths
//    voxel.robotInformation[i].waypointsPub           = voxel.nh.advertise<nav_msgs::Path>
//        (voxel.robotInformation[i].djkTopic         , 1, true);// TODO: make msg robot-specific
    voxel.robotInformation[i].trajCoeffsPub          = voxel.nh.advertise<ogm_ae::PolyLeastSquaresTraj>
        (voxel.robotInformation[i].trajInfoTopic    , 1, true);

    if(voxel.explore.publishPaths){
      voxel.robotInformation[i].cameraPositionPub      = voxel.nh.advertise<nav_msgs::Path>
          (voxel.robotInformation[i].cameraPathTopic  , 1, true);
      voxel.robotInformation[i].desiredTrajPositionPub = voxel.nh.advertise<nav_msgs::Path>
          (voxel.robotInformation[i].positionPathTopic, 1, true);
    }

    // Visualize Cost Map
    if(voxel.explore.visualizeCostMap)
      voxel.robotInformation[i].costMapPub = voxel.nh.advertise
          <nav_msgs::OccupancyGrid>(voxel.robotInformation[i].costMapTopic, 1, true);
  }
}

void EntropyBasedAutonomousExploration::ExploreInitParams(
    string& objective, Voxel::mapInfo& mapInput){

  mapInput.sqrt2pi = sqrt(2*M_PI);
  mapInput.GetMapParameters(objective);
  mapInput.BasicMapInfoInit(objective);
  mapInput.SensorUpdateInit();
  cout << "For " << objective << ", subscribing to " << mapInput.occTopic << endl;
  return;
}

void EntropyBasedAutonomousExploration::ExplorationCollisionUpdate(mapMsg& collMap){

  // Collision Map Message
  voxel.collisionMap.occData = *collMap;

  // Find Free Spaces
  voxel.FindFreeCells(voxel.explore.reqNumFreeNeighbors, voxel.collisionMap);

  // Flag & Sensor Initial TF
  if(!collisionMapFirstLoopDone){
    // Find Sensor Location from TFs
    for(int iRobot(0); iRobot < voxel.numRobots; iRobot++)
      GetSensorLoc(iRobot);
    rosTimeStartNextTraj = ros::Time::now();
    voxel.explore.tStartTraj = rosTimeStartNextTraj.toSec();// (?) need voxel, could be this object?
  }

  collisionMapFirstLoopDone = true;

  return;
}

void EntropyBasedAutonomousExploration::ExplorationEntropyUpdate(Voxel::mapInfo& collisionMap, Voxel::mapInfo& entropyMap){

  // Initializations
  bool runAutonomousExploration(false),  haltExploration(false);
  geometry_msgs::PoseStamped robotPose, cameraPose; geometry_msgs::TwistStamped robotVel; geometry_msgs::AccelStamped robotAccel;
  ros::param::get("run_autonomous_exploration", runAutonomousExploration);
  double timeAfterEntropy, sleepTime, recHorizonNow, durEntropy;

  // Starting Trajectory Time & Sleep Time
  voxel.explore.tStartTraj = rosTimeStartNextTraj.toSec();
  sleepTime = voxel.explore.tStartTraj-ros::Time::now().toSec();
  if(sleepTime > 0)
    ros::Duration(sleepTime).sleep();

  // Boolean for Exploration
  if(runAutonomousExploration && collisionMapFirstLoopDone){

    // Determine Candidate Expected Entropies
    voxel.explore.FindViableCandidateInfoGains(entropyMap);

    // Update Receding Horizon
    timeAfterEntropy = ros::Time::now().toSec();
    durEntropy = timeAfterEntropy-voxel.explore.tStartTraj;
    recHorizonNow = durEntropy+voxel.explore.maxComputationTime;
    rosTimeStartNextTraj = rosTimeStartNextTraj+ros::Duration(recHorizonNow);

    // Get Next Pose at Receding Horizon (next trajectory starts here)
    if(entropyMapFirstLoopDone){
      for(int iRobot(0); iRobot < voxel.numRobots; iRobot++){
        voxel.GetXYTrajFromCoeff(iRobot,
              voxel.robotInformation[iRobot].trajInfo, rosTimeStartNextTraj,
              cameraPose, robotPose, robotVel, robotAccel);
        voxel.robotInformation[iRobot].sensorLoc
            = {cameraPose.pose.position.x, cameraPose.pose.position.y, cameraPose.pose.position.z};
        voxel.robotInformation[iRobot].sensorAtt = cameraPose.pose.orientation;
      }
    }
    else{
      for(int iRobot(0); iRobot < voxel.numRobots; iRobot++)
        GetSensorLoc(iRobot);
      entropyMapFirstLoopDone = true;
    }

    // Cycle Through All Robots for Cost Maps
    bool allRobotsOK;
    do{

      // Ensure All Robot Future Poses are Collision-Free
      allRobotsOK = true;
      for(int iRobot(0); iRobot < voxel.numRobots; iRobot++){
        if(!voxel.explore.SetUpCostMap(collisionMap, voxel.robotInformation, iRobot, voxel.robotInformation[iRobot].indCurrent)){
          if(haltExploration == true){
//            ROS_ERROR("Any further movement of Robot %d is dangerous... exploration terminated.", iRobot);
//            ros::shutdown();
            return;
          }
          else{
            cout << "Robot " << iRobot << " haltExploration was false, now turned true." << endl;
            haltExploration = true;
            ros::param::set("exploration/halt", haltExploration);
          }
          allRobotsOK = false;
          break;
        }
      }

      // If Not, Halt Exploration at Current Pose
      if(!allRobotsOK){
//        ros::spinOnce();// get an updated collision map
        for(int iRobot(0); iRobot < voxel.numRobots; iRobot++)
          GetSensorLoc(iRobot);
      }

      // Initial Pose of Trajectory is OK, Proceed with Cost Map and Entropy Optimization
      else{
        for(int iRobot(0); iRobot < voxel.numRobots; iRobot++){

          cout << "Starting costmap " << iRobot << "..." << endl;

          // Generate Cost Map
          if(voxel.explore.GenerateCostMap(collisionMap, voxel.robotInformation, iRobot, voxel.robotInformation[iRobot].indCurrent)){

            cout << "voxel.robotInformation[" << iRobot << "].indCurrent = " << voxel.robotInformation[iRobot].indCurrent << endl;
            // Select Candidate Accounting for Travel Time
            if(voxel.explore.FindOptimalReachablePose(voxel.robotInformation[iRobot])){
              voxel.entropyBumpsAllRobots[iRobot] = voxel.robotInformation[iRobot].entropyBumpMax;
              allRobotsOK = true; haltExploration = false;
              cout << "FindOptimalReachablePose done." << endl;
            }
            else{
              ROS_WARN("Bumped entropies for Robot %d are all <= 0.", iRobot);
//              ros::shutdown();
              return;
            }
          }
          else
            return;
        }
      }
    } while(!allRobotsOK);

    cout << "done costmap" << endl;
    // Find Optimal Poses with a Bidding-Based Approach
    if(!voxel.RobotExplorationBidding())
      return;
    cout << "done bid" << endl;

    // Visualize Candidates
    if(voxel.explore.vizCandEntropiesDists)
      voxel.explore.VisualizeCandidatesMultiRobot(voxel.robotInformation, voxel.numRobots);

    // Generate Robot Trajectories
    for(int iRobot(0); iRobot < voxel.numRobots; iRobot++){

      // Find Dijkstra's Points Along Cost Map & Publish
      if(voxel.explore.FindDijkstraTrajectory(collisionMap, voxel.robotInformation, iRobot)){

        // Segment Dijkstra's Waypoints for Patched Curves
        if(voxel.GenerateSegmentsForPatching(iRobot)){

          // Generate a Smooth Trajectory for Control & Publishing
          voxel.GenerateSmoothTrajectory(collisionMap, iRobot, rosTimeStartNextTraj);

        }
      }
    }

    // Publish Path (position, velocity, acceleration) & Update Next Pose
    if(voxel.explore.publishPaths){
      for(int iRobot(0); iRobot < voxel.numRobots; iRobot++)
        voxel.PublishPaths(iRobot);
    }

  }
  entropyMapFirstLoopDone = true;

  return;
}

void EntropyBasedAutonomousExploration::CollisionMapCallback(mapMsg& msg){
  ExplorationCollisionUpdate(msg);
  return;
}

void EntropyBasedAutonomousExploration::EntropyMapCallback(mapMsg& msg){
  voxel.entropyMap.occData = *msg;
  ExplorationEntropyUpdate(voxel.collisionMap, voxel.entropyMap);
  return;
}

void EntropyBasedAutonomousExploration::CollisionAndEntropyMapCallback(mapMsg& msg){
  ExplorationCollisionUpdate(msg);
  voxel.entropyMap.occData = voxel.collisionMap.occData;
  ExplorationEntropyUpdate(voxel.collisionMap, voxel.collisionMap);
  return;
}

void EntropyBasedAutonomousExploration::GetSensorLoc(int& robotNum){

  // Obtain Sensor TF
  tf::StampedTransform transform
      = LoopUntilTransformAcquired(
        voxel.explore.tfListener, voxel.collisionMap.frame,
        voxel.robotInformation[robotNum].exploreFrame);

  // Sensor Position
  voxel.robotInformation[robotNum].sensorLoc
      = {transform.getOrigin().x(),
         transform.getOrigin().y(),
         transform.getOrigin().z()};

  // Sensor Attitude
  tf::Quaternion quatTF = transform.getRotation();
  voxel.robotInformation[robotNum].sensorAtt.x = quatTF.x();
  voxel.robotInformation[robotNum].sensorAtt.y = quatTF.y();
  voxel.robotInformation[robotNum].sensorAtt.z = quatTF.z();
  voxel.robotInformation[robotNum].sensorAtt.w = quatTF.w();

  return;

}

}
