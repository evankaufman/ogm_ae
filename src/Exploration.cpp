#include "ogm_ae/Robot.hpp"
#include "ogm_ae/Mapping.hpp"
#include "ogm_ae/Exploration.hpp"

#define sqrt2pi 2.50662827463


// Initializations

Exploration::Exploration(){}

Exploration::Exploration(vector<Robot>& robotVec, int& numRobots){

  // Parameters Only for Path Command Information
  GetExplorationParameters(robotVec, numRobots);

}

void Exploration::StartNode(){

  // Initialize Robot Information
  ros::param::get("num_robots", numRobots);
  numSensorsTotal = 0; robotVec.resize(0);
  objective = "exploration";
  for(int i(0); i < numRobots; i++){
    Robot robot(objective, i);
    robot.numSensors = 1;
    numSensorsTotal += robot.numSensors;
    robotVec.push_back(robot);
  }
  robotVec.resize(numRobots);

  ExplorationInit();

  // Misc.
  mapChanged = true;
  numThreads = 0;

  // Subscribers

  // Thread 1: Robotic Motions
  TFSubRoboticMotions = nh.subscribe
      ("/tf", 1, &Exploration::RoboticMotionsThreadCallback, this);
  numThreads += 1;
  lockExploreTrajThread = false;

  // Thread 2: Visualize 2D Collision Map
  TFSubVisualizeRdcdMap = nh.subscribe
      ("/tf", 1, &Exploration::TFCallbackOccGridMsgPublish , this);
  numThreads += 1;
  lockOGMPublish = false;

  // Thread 3: Occupancy Grid Map Changes (designed for no locks)
  changedMapProbsSub = nh.subscribe
      (fullMap.changesTopic, 1, &Exploration::MapProbChangesReductionCallback, this);
  numThreads += 1;

  // Thread 4: Full Map Subscriptions (degradation only)
  if(fullMap.degrade){
    allMapProbsSub = nh.subscribe
        ("/map_probabilities", 1, &Exploration::MapProbFullReductionCallback , this);
    numThreads += 1;
    lockFullMapThread = false;
  }

  for(int i(0); i < numRobots; i++){
    rdcdMap.sensors[i].frame = robotVec[i].exploreFrame;
    tempMap.sensors[i].frame = robotVec[i].exploreFrame;
  }
  explorationStarted = false;

  // __ Publishers __ \\

  // Robot Non-Specific: Entropy Marker Visualization
  ros::param::get("/exploration/visualization/pub_paths", visualizePaths);
  if(vizCandEntropies)
    candidatesPub  = nh.advertise
        <visualization_msgs::MarkerArray>(entropyMarkerTopic , 1, true);
  if(vizCandEntropiesDists)
    candidatesDiscountedPub = nh.advertise
        <visualization_msgs::MarkerArray>(distInfoMarkerTopic, 1, true);

  ogm2DPub = nh.advertise
      <nav_msgs::OccupancyGrid>        ("the_2D_reduced_map", 1, true);

  ogm3DPub = nh.advertise
      <visualization_msgs::MarkerArray>(ogmTopic, 1, true);



  // Robot Specific
  for(int i(0); i < numRobots; i++){

    robotVec[i].trajCoeffsPub          = nh.advertise<ogm_ae::PolyLeastSquaresTraj>
        (robotVec[i].trajInfoTopic    , 1, true);

    if(visualizePaths){
      robotVec[i].cameraPositionPub      = nh.advertise<nav_msgs::Path>
          (robotVec[i].cameraPathTopic  , 1, true);
      robotVec[i].desiredTrajPositionPub = nh.advertise<nav_msgs::Path>
          (robotVec[i].positionPathTopic, 1, true);
    }

    // Visualize Cost Map
    if(visualizeCostMap)
      robotVec[i].costMapPub = nh.advertise
          <nav_msgs::OccupancyGrid>(robotVec[i].costMapTopic, 1, true);

    robotVec[i].waypointsPub = nh.advertise<nav_msgs::Path>
        ("DJK", 1, true);

  }

  return;

}

void Exploration::RdcdMapParamsInit(Mapping& mapInput, vector<Robot>& robotVec, int& numRobots, int dimRdcdMap){

  // Dimension Check
  if(!(dimRdcdMap == 2 || dimRdcdMap == 3)){
    ROS_ERROR("Dimension of a reduced map is %d", dimRdcdMap);
    ros::shutdown();
  }

  // Initialize Map
  string paramNameStart("/mapping/");
  ros::param::get(paramNameStart+"occ_min", mapInput.occProbMin);
  ros::param::get(paramNameStart+"occ_max", mapInput.occProbMax);
  ros::param::get(paramNameStart+"frame"  , mapInput.frame     );
  mapInput.dimension = dimRdcdMap;
  mapInput.numCellsEachDimension.resize(dimRdcdMap);
  mapInput.minimumLocations.resize(dimRdcdMap);
  mapInput.maximumLocations.resize(dimRdcdMap);
  mapInput.alpha = alphaXY*fullMap.alpha;
  mapInput.numCellsTotal = 1;
  for(int i(0); i < dimRdcdMap; i++){
    mapInput.numCellsEachDimension[i] = floor(1.0*fullMap.numCellsEachDimension[i]/alphaXY);
    mapInput.minimumLocations[i] = fullMap.minimumLocations[i]
        +0.5*fullMap.alpha*(alphaXY-1);
    mapInput.maximumLocations[i] = mapInput.minimumLocations[i]
        +mapInput.alpha*(mapInput.numCellsEachDimension[i]-1);
    mapInput.numCellsTotal *= mapInput.numCellsEachDimension[i];
  }

  mapInput.initP = fullMap.initP;
  mapInput.occData.data.resize(mapInput.numCellsTotal, fullMap.initP);
  mapInput.stride.resize(dimRdcdMap);
  mapInput.stride[0] = 1;
  for(int i(1); i < dimRdcdMap; i++)
    mapInput.stride[i] = mapInput.stride[i-1]*mapInput.numCellsEachDimension[i-1];

  // Visualization Initializations
  mapInput.OccupancyGridMsgInit(OGM, height);

  // Other Misc. Parameters/Initializations
  mapInput.GetSensorParameters(objective, numRobots, robotVec, numSensorsTotal);
  mapInput.BasicMappingInit(objective);
  mapInput.SensorUpdateInit(numSensorsTotal);

  return;
}

void Exploration::GetExplorationParameters(vector<Robot>& robotVec, int& numRobots){

  // Helpful Strings
  string paramNameStart("/exploration/"), nsParamStart, vizParamStart;
  vizParamStart = paramNameStart+"visualization/";

  // Robot-Nonspecific
  ros::param::get(paramNameStart+"dim_entropy_map"     , dimEntropyMap          );
  ros::param::get(paramNameStart+"use_rdcd_map_entropy", useRdcdMapEntropy      );
  ros::param::get(paramNameStart+"height"              , height                 );
  ros::param::get(paramNameStart+"acceptable_coll_prob", acceptableCollisionProb);
  ros::param::get(paramNameStart+"candidate_separation", candidateSeparation    );
  ros::param::get(paramNameStart+"min_info_gain_thresh", minInfoGainThresh      );
  ros::param::get(paramNameStart+"num_cells_consider"  , numCellsToConsider     );
  ros::param::get(paramNameStart+"max_speed"           , maxSpeed               );
  ros::param::get(paramNameStart+"poly_order"          , polyOrder              );
  ros::param::get(paramNameStart+"points_per_segment"  , pointsPerSegment       );
  ros::param::get(paramNameStart+"traj_time_step"      , trajTimeStep           );
  ros::param::get(paramNameStart+"timers_on"           , timersOn               );
  ros::param::get(paramNameStart+"simulate_gazebo"     , simulateGazebo         );
  ros::param::get(paramNameStart+"gazebo_topic"        , gazeboTopic            );
  ros::param::get(paramNameStart+"free_nbrs_x"         , reqNumFreeNeighborsX   );
  ros::param::get(paramNameStart+"free_nbrs_y"         , reqNumFreeNeighborsY   );
  ros::param::get(paramNameStart+"free_nbrs_z"         , reqNumFreeNeighborsZ   );
  ros::param::get(paramNameStart+"num_rays_per_cand_z" , numRaysPerCandidateInZ );
  ros::param::get(paramNameStart+"num_rays_per_cand_y" , numRaysPerCandidateInY );
  ros::param::get(paramNameStart+"recompute_radius"    , recomputeRadius        );
  ros::param::get(paramNameStart+"FOV_angle_z"         , angleFOVInZ            );
  ros::param::get(paramNameStart+"FOV_angle_y"         , angleFOVInY            );
  ros::param::get(paramNameStart+"deg_cam_pitch"       , degreeCamPitch         );
  ros::param::get(paramNameStart+"bump_dist"           , bumpDist               );
  ros::param::get(paramNameStart+"bump_val_at_dist"    , bumpValAtDist          );
  ros::param::get(paramNameStart+"min_bump_val"        , minBumpFunVal          );
  ros::param::get(paramNameStart+"discount_radius"     , discountRadius         );
  ros::param::get(paramNameStart+"max_computation_time", maxComputationTime     );
  ros::param::get(paramNameStart+"max_angular_vel"     , maxRadPerSec           );
  ros::param::get(paramNameStart+"alpha_xy_factor"     , alphaXY                );
  ros::param::get(paramNameStart+"alpha_z_factor"      , alphaZ                 );
  ros::param::get(paramNameStart+"limit_cost_map"      , limitCostMap           );
  ros::param::get(paramNameStart+"num_cells_cost_map"  , maxNumCostMapCells     );
  ros::param::get(paramNameStart+"distance_bump_BMax"  , BMax                   );
  ros::param::get(paramNameStart+"distance_bump_BFar"  , BFar                   );
  ros::param::get(paramNameStart+"distance_bump_beta"  , beta                   );
  ros::param::get(paramNameStart+"min_max_thresh"      , minMaxThresh           );
  ros::param::get(paramNameStart+"full_map_pass_time"  , fullMapTransferTime    );
  ros::param::get(paramNameStart+"edge_cells_visited"  , edgeCellsVisited       );



  // Robot-Nonspecific Exploration Visualization
  ros::param::get(vizParamStart+"ogm_topic"             , ogmTopic             );
  ros::param::get(vizParamStart+"viz_cost_map"          , visualizeCostMap     );
  ros::param::get(vizParamStart+"viz_cand_entropies"    , vizCandEntropies     );
  ros::param::get(vizParamStart+"arrow_scale"           , arrowScale           );
  ros::param::get(vizParamStart+"non_optimal_opacity"   , nonOptimalOpacity    );
  ros::param::get(vizParamStart+"viz_cand_entr_dists"   , vizCandEntropiesDists);
  ros::param::get(vizParamStart+"entropy_marker_topic"  , entropyMarkerTopic   );
  ros::param::get(vizParamStart+"dist_info_marker_topic", distInfoMarkerTopic  );
  ros::param::get(vizParamStart+"arrow_length"          , arrowLength          );
  ros::param::get(vizParamStart+"arrow_shaft_diameter"  , arrow.scale.x        );
  ros::param::get(vizParamStart+"arrow_head_diameter"   , arrow.scale.y        );
  ros::param::get(vizParamStart+"arrow_head_length"     , arrow.scale.z        );


  // Robot-Specific Topics with General Parameters: Update with Namespaced Topics
  ros::param::get(paramNameStart+"camera_path_topic",   robotVec.back().cameraPathTopic  );
  ros::param::get(paramNameStart+"position_path_topic", robotVec.back().positionPathTopic);
  ros::param::get(paramNameStart+"traj_info_topic",     robotVec.back().trajInfoTopic    );
  ros::param::get(paramNameStart+"des_pose_topic",      robotVec.back().desiredPoseTopic );
  ros::param::get(paramNameStart+"des_cam_topic",       robotVec.back().desiredCamTopic  );
  ros::param::get(paramNameStart+"des_twist_topic",     robotVec.back().desiredTwistTopic);
  ros::param::get(paramNameStart+"des_accel_topic",     robotVec.back().desiredAccelTopic);
  ros::param::get(vizParamStart +"djk_topic",           robotVec.back().djkTopic         );
  ros::param::get(vizParamStart +"cost_map_topic",      robotVec.back().costMapTopic     );

  for(int i(0); i < numRobots; i++){
    if(i < robotVec.size()){// protection for single-element robotVec
      nsParamStart = robotVec[i].ns+"/";
      robotVec[i].cameraPathTopic     = nsParamStart+robotVec.back().cameraPathTopic  ;
      robotVec[i].positionPathTopic   = nsParamStart+robotVec.back().positionPathTopic;
      robotVec[i].trajInfoTopic       = nsParamStart+robotVec.back().trajInfoTopic    ;
      robotVec[i].desiredPoseTopic    = nsParamStart+robotVec.back().desiredPoseTopic ;
      robotVec[i].desiredCamTopic     = nsParamStart+robotVec.back().desiredCamTopic  ;
      robotVec[i].desiredTwistTopic   = nsParamStart+robotVec.back().desiredTwistTopic;
      robotVec[i].desiredAccelTopic   = nsParamStart+robotVec.back().desiredAccelTopic;
      robotVec[i].djkTopic            = nsParamStart+robotVec.back().djkTopic         ;
      robotVec[i].costMapTopic        = nsParamStart+robotVec.back().costMapTopic     ;
    }
  }

  return;
}

void Exploration::ExplorationInit(){

  // Exploration-Specific Parameters
  GetExplorationParameters(robotVec, numRobots);

  // Full Probilistic Map, Collision Map, & Collision Map Temporary Copy
  fullMap.GeneralInit(numSensorsTotal, numRobots, robotVec);// full 3D map, used for copying small data sets and storing parameters
  RdcdMapParamsInit(rdcdMap, robotVec, numRobots, dimEntropyMap);// reduced map, constantly updating with mapping node updates & degradation
  RdcdMapParamsInit(tempMap, robotVec, numRobots, dimEntropyMap);// temporary map locked during an exploration update


  // Initialize Marker Array for 3D Reduced Map Visualization
  if(rdcdMap.dimension == 3){
    rdcdMap.RvizMarkerArrayMsgInit(marker, markerBuff, occupancyVizMin, probRange);
    rdcdMapOccDataBuff = rdcdMap.occData;
  }

  // Vector Offsets (small cell centers to the centers of big cells covering them)
  if(rdcdMap.dimension == 3 && alphaZ != alphaXY){
    ROS_WARN("Requested %d cells in each x & y and %d cells in z: now corrected to %d in x, y, & z", alphaXY, alphaZ, alphaXY);
    alphaZ = alphaXY;
  }
  offset.resize(fullMap.dimension);
  for(int iDim(0); iDim < fullMap.dimension; iDim++){

    // X-Direction and Y-Direction (square projection)
    if(iDim == 0 || iDim == 1){
      offset[iDim].resize(alphaXY);
      offset[iDim][0] = 0.5*(1-alphaXY)*fullMap.alpha;
      for(int i = 1; i < alphaXY; i++){
        offset[iDim][i] = offset[iDim][i-1]+fullMap.alpha;
      }
    }

    // Z-Direction (can be different for 2D projection)
    else{
      offset[iDim].resize(alphaZ);
      offset[iDim][0] = 0.5*(1-alphaZ)*fullMap.alpha;
      for(int i = 1; i < alphaZ; i++){
        offset[iDim][i] = offset[iDim][i-1]+fullMap.alpha;
      }
    }
  }

  // Robot Vicinity
  sqrdRecomputeRadius = pow(2*rdcdMap.sensors[0].maxRange, 2);
  sqrdRecomputeRadius = pow(recomputeRadius, 2);

  // Temporary Variables
  numCellsThisDim = {alphaXY, alphaXY, alphaZ};
  numCellsInside = alphaXY*alphaXY*alphaZ;

  // Update from Full Map or Changes
  timeProbsUpdated.resize(rdcdMap.numCellsTotal, 0.0);

  // Number of Cost Map Cells
  if(!limitCostMap)
    maxNumCostMapCells = rdcdMap.numCellsTotal;

  initTraj = true;
  freeCells.resize(rdcdMap.numCellsTotal, false);

  // Ensure candidate selection occurs on first loop pass
  tNow = 0.0;

  // Use TFs for Offset Robot/Camera Offset
  for(int i(0); i < numRobots; i++)
    GetRobotCameraTFs(robotVec[i]);

  // Candidate Indices
  int numCellsBtwnCandidates(ceil(candidateSeparation/rdcdMap.alpha));
  candidateSeparation = numCellsBtwnCandidates*rdcdMap.alpha;// correction
  cout << "Candidates are separated " << candidateSeparation
       << "m apart with\ncells with edge length " << rdcdMap.alpha
       << "m\nsuch that pose candidates are "
       << numCellsBtwnCandidates
       << " cells apart." << endl;
  vector<int> numCandidatesEachDirection(3, 1);
  vector<double> firstCandidateLocation(3), evalCandidateLocation(3);
  numCandidatesTotal = 1;
  for(int i(0); i < rdcdMap.dimension; i++){
    numCandidatesEachDirection[i] = floor(1.0*(rdcdMap.numCellsEachDimension[i]-2)/(numCellsBtwnCandidates));
    if(numCandidatesEachDirection[i] > 0)
      ROS_INFO ("Exploration: number of candidates in dimension %i is %i", i, numCandidatesEachDirection[i]);
    else
      ROS_ERROR("Exploration: number of candidates in dimension %i is %i", i, numCandidatesEachDirection[i]);
    firstCandidateLocation[i] = rdcdMap.minimumLocations[i]+rdcdMap.alpha;
    numCandidatesTotal *= numCandidatesEachDirection[i];
  }
  ROS_INFO("There are a total of %d candidates.", numCandidatesTotal);
  candidateLocations   .resize(numCandidatesTotal, vector<double>(3));
  optimalUVecs         .resize(numCandidatesTotal, vector<double>(rdcdMap.dimension));
  optimalUVecIndsAboutZ.resize(numCandidatesTotal                                   );
  candidateIndices.resize(numCandidatesTotal);
  candidateDone.resize(numCandidatesTotal, false);
  candidateExpectedInfoGain.resize(numCandidatesTotal, 0.0);

  int indCandidate(0);
  if(rdcdMap.dimension == 2){
    numCandidatesEachDirection[2] = 1;
    firstCandidateLocation[2] = height;
  }
  for(int iX = 0; iX < numCandidatesEachDirection[0]; iX++){
    evalCandidateLocation[0] = firstCandidateLocation[0]+iX*numCellsBtwnCandidates*rdcdMap.alpha;
    for(int iY = 0; iY < numCandidatesEachDirection[1]; iY++){
      evalCandidateLocation[1] = firstCandidateLocation[1]+iY*numCellsBtwnCandidates*rdcdMap.alpha;
      for(int iZ = 0; iZ < numCandidatesEachDirection[2]; iZ++){
        evalCandidateLocation[2] = firstCandidateLocation[2]+iZ*numCellsBtwnCandidates*rdcdMap.alpha;
        candidateLocations[indCandidate] = evalCandidateLocation;
        candidateIndices[indCandidate] = rdcdMap.IndFromMapLocation(evalCandidateLocation);
        indCandidate++;
      }
    }
  }

  // Sensor Unit Vectors
  Eigen::Vector3d uVec;
  Eigen::Matrix3d uVecR;
  double angleZ(0.0), angleY(0.0), angleX(0.0);
  numRaysPerCandidateTotal = numRaysPerCandidateInZ*numRaysPerCandidateInY;
  rayUVecs.resize(numRaysPerCandidateTotal, vector<double>(rdcdMap.dimension));

  if(rdcdMap.dimension == 2 && numRaysPerCandidateInY != 1){
    ROS_ERROR("Desired number of entropy ray rotations in y-direction is %d\nfor a 2D map. Shutting down exploration node...", numRaysPerCandidateInY);
    ros::shutdown();
  }
  else if(!(rdcdMap.dimension == 2 || rdcdMap.dimension == 3)){
    ROS_ERROR("Exploration is not designed for %d dimensions as specified. Shutting down exploration node...", rdcdMap.dimension);
    ros::shutdown();
  }


  // Z-Component of Expected Measurement Rays (y-axis rotation)
  if(degreeCamPitch >= 180.0 || degreeCamPitch <= -180.0 || (rdcdMap.dimension < 3 && degreeCamPitch != 0)){
    ROS_ERROR("Exploration camera pitch is %e for an exploration map with %d dimensions.", degreeCamPitch, rdcdMap.dimension);
    ros::shutdown();
  }
  radiansCamPitch = degreeCamPitch*M_PI/180;
  optimalUVecZComp = -sin(radiansCamPitch);
  magnitudeXY = sqrt(1.0-pow(optimalUVecZComp, 2));
  angleYStart = radiansCamPitch-angleFOVInY/2+angleFOVInY/(2*numRaysPerCandidateInY);
  RotMatrixMinPitchOptimalPitch = Euler321ToMatrix(0.0, radiansCamPitch-angleYStart, 0.0);

  int rayUVecInd(0);
  for(int iZ(0); iZ < numRaysPerCandidateInZ; iZ++){
    angleZ = (2.0*M_PI*iZ/numRaysPerCandidateInZ);
    for(int iY(0); iY < numRaysPerCandidateInY; iY++){
      angleY = angleYStart+iY*angleFOVInY/numRaysPerCandidateInY;
      uVecR = Euler321ToMatrix(angleZ, angleY, angleX);
      uVec = uVecR*e1;
      for(int dimInd(0); dimInd < rayUVecs[rayUVecInd].size(); dimInd++)
        rayUVecs[rayUVecInd][dimInd] = uVec(dimInd);
      rayUVecInd++;
    }
  }
  numRaysEachSideAboutZ = floor(0.5*numRaysPerCandidateInZ*angleFOVInZ/(2*M_PI));
  infoGainEachRayAboutZ.resize(numRaysPerCandidateInZ);
  infoGainEachDirectionAboutZ.resize(numRaysPerCandidateInZ);

  // Path
  bumpSigSqrd = -0.5*pow(bumpDist, 2)/log(bumpValAtDist);
  costMapInitValue = (double)(rdcdMap.numCellsTotal*rdcdMap.alpha);
  visited        .resize(rdcdMap.numCellsTotal, true);
  visitedUpdating.resize(rdcdMap.numCellsTotal, true);
  costClosestCell = rdcdMap.alpha;
  costDiagonal2D  = sqrt(2.0)*rdcdMap.alpha;
  costDiagonal3D  = sqrt(3.0)*rdcdMap.alpha;
  pathwayBuff.resize(rdcdMap.dimension, vector<int>(rdcdMap.numCellsTotal));
  djkDistBuff.resize(rdcdMap.numCellsTotal);
  fixedPoseStampedParams.pose.position.z = height;
  fixedPoseStampedParams.pose.orientation.x = 0.0;
  fixedPoseStampedParams.pose.orientation.y = 0.0;
  fixedPoseStampedParams.pose.orientation.z = 0.0;
  fixedPoseStampedParams.pose.orientation.w = 1.0;
  fixedPoseStampedParams.header.frame_id = rdcdMap.frame;

  // Robot-Specific
  for(int iRobot(0); iRobot < numRobots; iRobot++){
    robotVec[iRobot].bestCandInd = -1;
    robotVec[iRobot].sensorLoc    .resize(3);
    robotVec[iRobot].nextSensorLoc.resize(3);
    robotVec[iRobot].costMap.resize(rdcdMap.numCellsTotal, costMapInitValue);
    robotVec[iRobot].candidateBumps           .resize(numCandidatesTotal, 0.0);
    robotVec[iRobot].candidateInfoGainWithBump.resize(numCandidatesTotal, 0.0);
    robotVec[iRobot].candidateDiscounts       .resize(numCandidatesTotal, 1.0);
  }
  entropyBumpsAllRobots.resize(numRobots, 0.0);

  // Visualization
  if(vizCandEntropies
  || vizCandEntropiesDists){

    // Future Pose Candidates (Rviz marker arrays)
    arrow.header.frame_id = rdcdMap.frame;
    arrow.ns = "basic_shapes";
    arrow.action = visualization_msgs::Marker::ADD;
    arrow.type   = visualization_msgs::Marker::ARROW;
    arrow.color.g = 0.0;
    arrow.color.a = 1.0;
    arrow.lifetime = ros::Duration();
    arrow.points.resize(2);
    arrowLoc.z = height;
    entropyArrowArray.markers.resize(numCandidatesTotal, arrow);
    entropyDistArrowArray = entropyArrowArray;

    rdcdMap.OccupancyGridMsgInit(costMapViz, height);

  }

  return;
}


// ROS Callbacks

void Exploration::RoboticMotionsThreadCallback(const tf2_msgs::TFMessage::ConstPtr& msg){

  // Thread Locked for a Single Set of Robot Trajectories
  if(!lockExploreTrajThread){
    lockExploreTrajThread = true;


    // Initializations
    bool runAutonomousExploration(false);
    geometry_msgs::PoseStamped robotPose, cameraPose;
    geometry_msgs::TwistStamped robotVel;
    geometry_msgs::AccelStamped robotAccel;
    ros::param::get("run_autonomous_exploration", runAutonomousExploration);
    double timeAfterEntropy, recHorizonNow, durEntropy, tStart = ros::Time::now().toSec();
    double tTest;
    geometry_msgs::Vector3 x3Dot, x4Dot;



    for(int i(0); i < numRobots; i++)
      fill(robotVec[i].candidateDiscounts.begin(), robotVec[i].candidateDiscounts.end(), 1.0);


    // First Horizon at Current Time & Position
    if(!explorationStarted){
      rosTimeStartNextTraj = ros::Time::now();
      for(int iRobot(0); iRobot < numRobots; iRobot++)
        GetSensorPose(iRobot);
    }

    if(runAutonomousExploration){

      // Lock Collision Map, Robot Locations, and Free/Visited Cells
      UpdateLockedExplorationMap();

      // Determine Candidate Expected Entropies
      if(!FindViableCandidateInfoGains(robotVec)){
        HaltTrajectories();
        return;
      }

      // Update Receding Horizon
      timeAfterEntropy = ros::Time::now().toSec();
      durEntropy = timeAfterEntropy-rosTimeStartNextTraj.toSec();
      recHorizonNow = durEntropy+maxComputationTime;
      rosTimeStartNextTraj = rosTimeStartNextTraj+ros::Duration(recHorizonNow);
      optimalTravelDistance = maxComputationTime*maxSpeed;
      cout << "Optimal travel distance: " << optimalTravelDistance << "m." << endl;

      // During Exploration, Use Pose at the Receding Horizon
      if(explorationStarted){

        // Update Next Pose
        for(int iRobot(0); iRobot < numRobots; iRobot++){
          GetTrajFromCoeff(tempMap.dimension, robotVec[iRobot], rosTimeStartNextTraj,
                cameraPose, robotPose, robotVel, robotAccel, x3Dot, x4Dot);
          robotVec[iRobot].sensorLoc
              = {cameraPose.pose.position.x, cameraPose.pose.position.y, cameraPose.pose.position.z};
          robotVec[iRobot].sensorAtt = cameraPose.pose.orientation;
        }
      }
      else{
        ROS_WARN("Exploration has not started yet! Getting sensor pose...");

        explorationStarted = true;
      }

      // Cycle Through All Robots for Cost Maps
      if(timersOn){tTest = ros::Time::now().toSec();}

      // Update and Re-Lock Collision Map, Robot Locations, and Free/Visited Cells
      UpdateLockedExplorationMap();

      // Mark Current Cell Index and Other Robot Locations as Visited
      SetUpCostMap(tempMap, robotVec);

      // Cost Maps & First Bids
      for(int iRobot(0); iRobot < numRobots; iRobot++){

        // Generate Cost Map
        if(GenerateCostMap(tempMap, robotVec[iRobot], maxNumCostMapCells)){

          // Select Candidate Bid Accounting for Travel Time
          if(FindOptimalReachablePose(robotVec[iRobot]))
            entropyBumpsAllRobots[iRobot] = robotVec[iRobot].entropyBumpMax;
          else{
            ROS_WARN("Bumped entropies for Robot %d are all <= 0.", iRobot);
            HaltTrajectories();
            return;
          }
        }
        else{
          HaltTrajectories();
          return;
        }
      }

      // Find Optimal Poses with a Bidding-Based Approach
      if(!RobotExplorationBidding(tempMap, robotVec)){
        HaltTrajectories();
        return;
      }

      // Visualize Candidates
      if(vizCandEntropiesDists)
        VisualizeCandidatesMultiRobot(robotVec, numRobots);// TODO: eliminate single version?

      if(timersOn){tTest = ros::Time::now().toSec();}
      // Generate Robot Trajectories
      for(int iRobot(0); iRobot < numRobots; iRobot++){

        // Find Dijkstra's Points Along Cost Map & Publish
        if(FindDijkstraTrajectory(tempMap, robotVec[iRobot])){

          // Segment Dijkstra's Waypoints for Patched Curves
          if(GenerateSegmentsForPatching(robotVec[iRobot])){

            // Generate a Smooth Trajectory for Control & Publishing
            GenerateSmoothTrajectory(tempMap, robotVec[iRobot], rosTimeStartNextTraj);

          }
        }
      }
      if(timersOn){ROS_INFO("Trajectories total time is %e sec", ros::Time::now().toSec()-tTest);}// 0.15

      // Publish Path (position, velocity, acceleration) & Update Next Pose
      if(visualizePaths){
        for(int iRobot(0); iRobot < numRobots; iRobot++)
          PublishPaths(robotVec[iRobot]);
      }

      // Computation Time Analysis
      double tEnd = ros::Time::now().toSec();
      ROS_INFO("Total exploration computation time: %e sec.", tEnd-tStart);

      double sleepTime = rosTimeStartNextTraj.toSec()-ros::Time::now().toSec();
      if(sleepTime >= 0){
        ROS_INFO("Trajectory computation time is %e sec: sleeping for %e sec...",
                 tEnd-timeAfterEntropy, sleepTime);
        ros::Duration(sleepTime).sleep();
      }
      else{
        ROS_WARN("Trajectory computation time is %e sec, but only %e sec was allotted (may cause trajectory discontinuity).",
                 tEnd-timeAfterEntropy, maxComputationTime);
        maxComputationTime = 1.1*(maxComputationTime-sleepTime);// negative sleep time
        ROS_INFO("Maximum computation time is adjusted to %e sec.", maxComputationTime);
      }

      // Printout Spacing
      cout << "\n\n" << endl;
    }

    lockExploreTrajThread = false;
  }

  return;
}

void Exploration::TFCallbackOccGridMsgPublish(const tf2_msgs::TFMessage::ConstPtr& msg){

  // Only 1 Update at a Time
  if(!lockOGMPublish){
    lockOGMPublish = true;

    // Only Update with 1+ Map Changes
    if(mapChanged){
      mapChanged = false;

      // 2D Reduced Map Message
      if(rdcdMap.dimension == 2){
        for(int i(0); i < rdcdMap.numCellsTotal; i++)
          OGM.data[i] = floor(sqrt(rdcdMap.occData.data[i])*100+0.5);
      }
      else{// 3D Cells at 'height'
        int i(0); double prob;
        vector<double> testLoc(3);

        // 2D Map Height
        if(robotVec.size() == 1)
          testLoc[2] = OGM.info.origin.position.z = robotVec[0].sensorLoc[2];// follow a single-robot
        else
          testLoc[2] = height;// consistent for multi-robots

        for(int iY(0); iY < rdcdMap.numCellsEachDimension[1]; iY++){
          testLoc[1] = rdcdMap.minimumLocations[1]+iY*rdcdMap.alpha;
          for(int iX(0); iX < rdcdMap.numCellsEachDimension[0]; iX++){
            testLoc[0] = rdcdMap.minimumLocations[0]+iX*rdcdMap.alpha;
            prob = rdcdMap.ProbFromMapLocation(testLoc);
            OGM.data[i] = floor(sqrt(prob)*100+0.5);
            i++;
          }
        }

        // 3D Reduced Map Message
        if(changedCellsQueue.size() != 0){
          if(rdcdMap.GenerateRvizMarkerArrayMsg(changedCellsQueue[0], marker, markerArrayMsg, markerBuff, occupancyVizMin, probRange, rdcdMapOccDataBuff, rdcdMap.cellsColored))
            ogm3DPub.publish(markerArrayMsg);
          changedCellsQueue.pop_front();
        }
      }

      // 2D Reduced Map Info & Publishing
      OGM.header.frame_id = fullMap.frame;
      OGM.header.stamp = ros::Time::now();
      ogm2DPub.publish(OGM);
    }

    lockOGMPublish = false;
  }

  return;
}

void Exploration::MapProbFullReductionCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){

  if(!lockFullMapThread){
    lockFullMapThread = true;// lock

    // Initializations
    vector<int> IndsInside(numCellsInside);
    vector<double> rdcdCellLoc(fullMap.dimension), probsInside(numCellsInside);
    double tNow = ros::Time::now().toSec();
    int indFullMap;

    // Combine Full Map Into Reduced Map
    for(int indRdcd(0); indRdcd < rdcdMap.numCellsTotal; indRdcd++){

      // Only Update if not Recently Updated by Map Changes (could be updating with past info)
      if(tNow-timeProbsUpdated[indRdcd] > fullMapTransferTime){

        // Reduced Map Cell Location
        if(rdcdMap.dimension == 2){
          vector<double> loc2D = rdcdMap.MapLocationFromInd(indRdcd);
          rdcdCellLoc[0] = loc2D[0]; rdcdCellLoc[1] = loc2D[1]; rdcdCellLoc[2] = height;
        }
        else if(rdcdMap.dimension == 3)
          rdcdCellLoc = rdcdMap.MapLocationFromInd(indRdcd);
        else{
          ROS_ERROR("Reduced map dimension: %d. Shutting down...", rdcdMap.dimension);
          ros::shutdown();
        }

        // 3D Cell Indices & Probabilities Projected to 2D Location
        IndsInsideLargerVoxel(rdcdCellLoc, fullMap, IndsInside);
        for(int indFull(0); indFull < numCellsInside; indFull++){
          indFullMap = IndsInside[indFull];
          fullMap.occData.data[indFullMap] = msg->data[indFullMap];
          probsInside[indFull] = fullMap.occData.data[indFullMap];
        }

        // Reduced Map Probability Selection
        rdcdMap.occData.data[indRdcd] = ProbReducedMapCell(probsInside);

      }
    }

    // Update Expected Transfer Time
    fullMapTransferTime = ros::Time::now().toSec()-tNow;

    // Flag Updated Map
    mapChanged = true;

    lockFullMapThread = false;// unlock

  }

  return;
}

void Exploration::UpdateChangedCellsOfReducedMap(Mapping& mapInput){


  // Start Time
  double tNow = ros::Time::now().toSec();

  // Min/Max Locations of Block to Update
  vector<double> minCorner(mapInput.dimension), maxCorner(mapInput.dimension), loc3D(fullMap.dimension);
  loc3D = fullMap.MapLocationFromInd(fullMap.changedCells.inds.front());
  for(int i(0); i < mapInput.dimension; i++){
    minCorner[i] = loc3D[i];
    if(minCorner[i] < mapInput.minimumLocations[i])
      minCorner[i] = mapInput.minimumLocations[i];
  }
  minCorner = mapInput.SnapLocationToMap(minCorner);
  loc3D = fullMap.MapLocationFromInd(fullMap.changedCells.inds.back ());
  for(int i(0); i < mapInput.dimension; i++){
    maxCorner[i] = loc3D[i];
    if(maxCorner[i] > mapInput.maximumLocations[i])
      maxCorner[i] = mapInput.maximumLocations[i];
  }
  maxCorner = mapInput.SnapLocationToMap(maxCorner);

  // Number of Cells to Update
  vector<int> numUpdatedReducedCellsThisDim(mapInput.dimension);
  int numReducedCellsToUpdate(1);
  for(int i(0); i < mapInput.dimension; i++){
    numUpdatedReducedCellsThisDim[i] = floor((maxCorner[i]-minCorner[i])/mapInput.alpha+0.5)+1;
    numReducedCellsToUpdate *= numUpdatedReducedCellsThisDim[i];
  }

  // Determine Which Reduced Cells Might Change
  int ind(0); vector<double> rdcdCellLoc(fullMap.dimension);
  vector<int> indsReducedCellsUpdated(numReducedCellsToUpdate);
  mapInput.FindCellsComposingBlock(rdcdCellLoc, minCorner, numUpdatedReducedCellsThisDim, indsReducedCellsUpdated, ind);
  ogm_ae::UpdatedMapCells changedCellsReducedMap;
  changedCellsReducedMap.header = fullMap.changedCells.header;
  changedCellsReducedMap.inds.resize(numReducedCellsToUpdate);
  changedCellsReducedMap.probs.resize(numReducedCellsToUpdate);
  changedCellsReducedMap.colors.resize(3*numReducedCellsToUpdate, 128);


  // Combine Full Map Into Reduced Map
  for(int i(0) ; i < numReducedCellsToUpdate; i++){

    ind = changedCellsReducedMap.inds[i] = indsReducedCellsUpdated[i];

    if(mapInput.dimension == 2){

      // 2D Cell Location
      vector<double> loc2D = mapInput.MapLocationFromInd(ind);
      rdcdCellLoc[0] = loc2D[0]; rdcdCellLoc[1] = loc2D[1]; rdcdCellLoc[2] = height;
    }
    else if(mapInput.dimension == 3){

      // 3D Cell Location
      rdcdCellLoc = rdcdMap.MapLocationFromInd(ind);
    }

    // Full Map Cell Indices & Probabilities Inside Reduced Cell
    vector<int> IndsInside(numCellsInside);
    vector<double> probsInside(numCellsInside);
    IndsInsideLargerVoxel(rdcdCellLoc, fullMap, IndsInside);
    for(int i(0); i < numCellsInside; i++)
      probsInside[i] = fullMap.occData.data[IndsInside[i]];

    // Reduced Map Probability Selection
    mapInput.occData.data[ind] = changedCellsReducedMap.probs[i] = ProbReducedMapCell(probsInside);
    timeProbsUpdated[ind] = tNow;

  }

  // Update Reduced Map Change
  if(mapInput.dimension == 3)
    changedCellsQueue.push_back(changedCellsReducedMap);

  return;
}

void Exploration::MapProbChangesReductionCallback(const ogm_ae::UpdatedMapCells::ConstPtr& msg){

  // Update Full Map with Changed Cells
  fullMap.changedCells = *msg;
  fullMap.UpdateChangedMapCells();

  // Update Reduced Map with Changed Cells
  UpdateChangedCellsOfReducedMap(rdcdMap);

  // Allow a Visualization Loop
  mapChanged = true;

  return;
}


// Map Updating

void Exploration::UpdateLockedExplorationMap(){

  // Lock Free/Visted Collision Map Cells
  tempMap.occData.data = rdcdMap.occData.data;

  // Clear Robot Locations
  ClearRobotLoc(fullMap, tempMap, numRobots, robotVec);

  // Label Free/Visited Cells
  for(int indRdcd(0); indRdcd < tempMap.numCellsTotal; indRdcd++){
    if     (tempMap.dimension == 2)
      freeCells[indRdcd] = ReducedMapCellIsFree2D(indRdcd, tempMap);
    else if(tempMap.dimension == 3)
      freeCells[indRdcd] = ReducedMapCellIsFree3D(indRdcd, tempMap);
    else
      ros::shutdown();
    visited[indRdcd] = !freeCells[indRdcd];
  }

  return;
}

void Exploration::IndsInsideLargerVoxel(vector<double>& loc, Mapping& mapToReduce, vector<int>& IndsInside){

  // Dimensions X,Y
  vector<double> minCorner(mapToReduce.dimension);

  for(int i(0); i < fullMap.dimension; i++)
    minCorner[i] = loc[i]+offset[i].front();

  // Extract Indices Composing Reduced Map Cell
  mapToReduce.FindCellsComposingBlock3D(minCorner, numCellsThisDim, IndsInside);

  return;
}

double Exploration::ProbReducedMapCell(vector<double>& probsInside){

  // Find Projected Cell Value
  int ind;
  double output = MaxOfVector(probsInside, ind);
  if(output < minMaxThresh)
    output = MinOfVector(probsInside, ind);

  return output;
}

bool Exploration::ReducedMapCellIsFree2D(int& cellIndex, Mapping& mapInput){

  // Initialize Temporary Indices
  int indTest, iX, iY;

  // Cycle Through Neighbors
  for(iX = -reqNumFreeNeighborsX; iX <= reqNumFreeNeighborsX; iX++){
    for(iY = -reqNumFreeNeighborsY; iY <= reqNumFreeNeighborsY; iY++){

      // Index in Question
      indTest = cellIndex+iX*mapInput.stride[0]+iY*mapInput.stride[1];

      // Mark as Occupied IF: Neighbor is Off Map or Current Cell and/or Neighbor is Occupied
      if(indTest < 0 || indTest >= mapInput.numCellsTotal)// avoid possible segmentation fault
        return false;
      else if(mapInput.occData.data[indTest] > acceptableCollisionProb)
        return false;

    }
  }

  // Passed All Tests
  return true;
}

bool Exploration::ReducedMapCellIsFree3D(int& cellIndex, Mapping& mapInput){

  // Initialize Temporary Indices
  int indTest, iX, iY, iZ;

  // Cycle Through Neighbors
  for(iX = -reqNumFreeNeighborsX; iX <= reqNumFreeNeighborsX; iX++){
    for(iY = -reqNumFreeNeighborsY; iY <= reqNumFreeNeighborsY; iY++){
      for(iZ = -reqNumFreeNeighborsZ; iZ <= reqNumFreeNeighborsZ; iZ++){

//      iZ = 0;
        // Index in Question
        indTest = cellIndex+iX*mapInput.stride[0]+iY*mapInput.stride[1]+iZ*mapInput.stride[2];

        // Mark as Occupied IF: Neighbor is Off Map or Current Cell and/or Neighbor is Occupied
        if(indTest < 0 || indTest >= mapInput.numCellsTotal){// avoid possible segmentation fault
          if(edgeCellsVisited)
            return false;
        }
        else if(mapInput.occData.data[indTest] > acceptableCollisionProb)
          return false;
      }
    }
  }

  // Passed All Tests
  return true;
}


// Entropy-Based Information Gain

vector<double> Exploration::SumScalarQuantityOfRays(vector<double>& input, int& raysEachSide){
  int size(input.size());
  vector<double> output(size, 0.0);
  int rayInd;

  // Get First Index
  for(rayInd = -raysEachSide; rayInd <= raysEachSide; rayInd++){
    output[0] += input[MakeCyclicIndexInt(rayInd, size)];
  }

  // Modify from Last Index
  for(int i = 1; i < size; i++){

    // Add a ray in the positive direction
    rayInd = i+raysEachSide;
    output[i] = output[i-1]+input[MakeCyclicIndexInt(rayInd, size)];

    // Remove a ray in the negative direction
    rayInd = i-raysEachSide-1;
    output[i] -= input[MakeCyclicIndexInt(rayInd, size)];

  }

  return output;
}

int Exploration::SumExpectedInfoGainsEachLocation(Mapping& mapInput, vector<Robot>& robotVec){

  // Initializations
  int numCandConsidered(0);
  int numValidSkipped(0);
  bool considerCandidate;

  // Cycle Through Viable Candidates
  for(int iCand = 0; iCand < numCandidatesTotal; iCand++){

    // Candidate is Free & Warrants Updating if Close to a Robot
    if(!candidateDone[iCand] && freeCells[candidateIndices[iCand]]){

      // Assume that Candidates Far Away are Unchanged
      for(int iRobot(0); iRobot < robotVec.size(); iRobot++){
        if(fullMap.degrade
        || Norm2VecsSqrd(candidateLocations[iCand], robotVec[iRobot].sensorLoc) < sqrdRecomputeRadius)
//        || (!candidateDone[iCand] && candidateExpectedInfoGain[iCand] < minInfoGainThresh))// TODO: check if needed
          considerCandidate = true;
        else{
          considerCandidate = false;
          break;// not close enough for this robot: check the remaining robots
        }
        if(considerCandidate)
          break;// within close proximity of >= 1 robot: consider
      }

      // If Free, Incomplete, & Close Enough, Update Expected Information Gain
      if(considerCandidate){

        // Candidate Information Gain
        UpdateExpectedInfoGain(iCand, mapInput);// TODO: mapInput check

        // Check Threshold
        if(candidateExpectedInfoGain[iCand] < minInfoGainThresh){
          candidateExpectedInfoGain[iCand] = 0.0;
          candidateDone[iCand] = true;
        }
        else
          numCandConsidered++;
      }
      else
        numValidSkipped++;
    }
  }

  cout << "Number skipped:    " << numValidSkipped << endl;
  cout << "Number considered: " << numCandConsidered << endl;
  return numCandConsidered;

}

void Exploration::UpdateExpectedInfoGain(int& iCand, Mapping& mapInput){

  // Obtain Each Ray Expected Info Gain & Take Summation
  int indUVec(0);
  for(int iRayAboutZ = 0; iRayAboutZ < numRaysPerCandidateInZ; iRayAboutZ++){

    // Initialize Information Gain at Each Angle About Z
    infoGainEachRayAboutZ[iRayAboutZ] = 0.0;

    // Negative Entropy Changes for Each Angle About Y
    for(int iRayAboutY = 0; iRayAboutY < numRaysPerCandidateInY; iRayAboutY++){
      if(useRdcdMapEntropy)
        infoGainEachRayAboutZ[iRayAboutZ] -= rdcdMap.SingleRayExpectedEntropyChange(candidateLocations[iCand], rayUVecs[indUVec], numCellsToConsider);
      else
        infoGainEachRayAboutZ[iRayAboutZ] -= fullMap.SingleRayExpectedEntropyChange(candidateLocations[iCand], rayUVecs[indUVec], numCellsToConsider);
      indUVec++;
    }
  }

  // Entropy Sum of Rays
  infoGainEachDirectionAboutZ = SumScalarQuantityOfRays(infoGainEachRayAboutZ, numRaysEachSideAboutZ);

  // Maximum Info Gain Magnitude & Direction
  candidateExpectedInfoGain[iCand] = MaxOfVector(infoGainEachDirectionAboutZ, optimalUVecIndsAboutZ[iCand]);
  optimalUVecs[iCand] = rayUVecs[numRaysPerCandidateInY*optimalUVecIndsAboutZ[iCand]];

  // Projection onto Plane Normal to Z
  optimalUVecs[iCand][2] = 0.0;
  NormalizeVec(optimalUVecs[iCand]);

  return;
}

bool Exploration::FindViableCandidateInfoGains(vector<Robot>& robotVec){

  // Start Timer
  if(timersOn){tStartSection = ros::Time::now().toSec();}

  // Expected Information Gain for Each Candidate Location at the Best Direction
  int numCandConsidered = SumExpectedInfoGainsEachLocation(tempMap, robotVec);

  // Terminate for No Viable Candidates
  if(numCandConsidered == 0){
    ROS_WARN("No candidates are considered... either exploration is done or the initial value of parameter 'run_autonomous_exploration' is incorrectly true.");
    ROS_INFO("Setting parameter 'run_autonomous_exploration' to false...");
    ros::param::set("run_autonomous_exploration", false);
    return false;
  }

  // Visualize Candidates
  if(vizCandEntropies){
    int bestCandInd;
    double bestCandInfoGain = MaxOfVector(candidateExpectedInfoGain, bestCandInd);
    VisualizeCandidates(bestCandInfoGain, candidateExpectedInfoGain, candidatesPub, bestCandInd,
                        minInfoGainThresh);
  }

  // End Timer
  if(timersOn){
    tEndSection = ros::Time::now().toSec();
    ROS_INFO("Exploration optimal pose discovery & its visualization time: %e", tEndSection-tStartSection);
  }

  return true;
}

double Exploration::MaximizeCandidateObjective(Robot& robot){

  while(true){

    // Maximum Information Gain Location
    double reward = MaxOfVector(robot.candidateInfoGainWithBump, robot.bestCandInd);
    int candIndex = candidateIndices[robot.bestCandInd];

    // Verify the Robot Not Here Already
    if(robot.costMap[candIndex] == 0.0){
      robot.candidateInfoGainWithBump[robot.bestCandInd] = 0.0;
      if(!fullMap.degrade){// eliminate candidate for exploration, keep candidate for patrol
        candidateDone[robot.bestCandInd] = true;
        cout << "Cost map is 0 at index " << robot.bestCandInd << " so it is eliminated." << endl;
      }
    }

    // Optimal Pose Requires Translational Motion
    else
      return reward;

  }
}

bool Exploration::FindOptimalReachablePose(Robot& robot){

  // Starting Time
  double tStart = ros::Time::now().toSec();

  // Clear Old Information & Bump Information
  fill(robot.candidateInfoGainWithBump.begin(),
       robot.candidateInfoGainWithBump.end()  ,
       0.0);
  robot.candidateBumps = robot.candidateInfoGainWithBump;

  // Penalize Candidates by a Travel Distance Cost with a Bump Function (options below)
  int candIndex;// double distAway;
  for(int i(0); i < numCandidatesTotal; i++){
    candIndex = candidateIndices[i];
    if(!candidateDone[i]// not done
     && robot.costMap[candIndex] < costMapInitValue){// reachable

      // Single Vehicle: Domain [0, optimalTravelDistance] @ Max Functional Value
      robot.candidateBumps[i]
          = FlatPeakHill(
            robot.costMap[candIndex], optimalTravelDistance, BMax, BFar, beta);

//      // Multi-Vehicle: Sinusoidal-Gaussian Bump
//      robot.candidateBumps[i]
//          = DistanceBump(
//            robot.costMap[candIndex], optimalTravelDistance, BMax, BFar, beta);

//     // Gaussian-Like Bump Function to Prioritize Shorter Actions (fair results)
//      distAway = robot.costMap[candIndex];
//      robot.candidateBumps[i]
//          = GaussianLikeBump(
//            robot.costMap[candIndex], bumpSigSqrd, minBumpFunVal);

//      // No Bump
//      robot.candidateBumps[i] = robot.costMap[candIndex];

      // Entropy-Distance Objective
      robot.candidateInfoGainWithBump[i]
          = robot.candidateBumps[i]
            *candidateExpectedInfoGain[i];

    }
  }

  // Select the Location Index with Maximum Information Gain
  robot.entropyBumpMax = MaximizeCandidateObjective(robot);

  // Checks & Returns
  if(robot.entropyBumpMax <= 0.0){
    ROS_WARN("Optimal candidate at index %d provides %e gain and %e adjusted gain.",
             robot.bestCandInd, candidateExpectedInfoGain[robot.bestCandInd], robot.entropyBumpMax);
    robot.bestCandInd = -1;
    return false;
  }
  else{
    if(timersOn)
      ROS_INFO("Finding optimal reachable pose for Robot %s: %e sec", robot.ns.c_str(), ros::Time::now().toSec()-tStart);
    return true;
  }
}

bool Exploration::RobotExplorationBidding(Mapping& tempMap, vector<Robot>& robotVec){

  double tStart = ros::Time::now().toSec();
  double bestBid;
  int numOptimalPosesFound(0), optimalRobotInd;
  int candIndex, numRobots = robotVec.size();
  vector<bool> optimalPosesFound(numRobots, false);
  vector<double> distsToEachPose(numRobots), mapLocOptimal(tempMap.dimension), mapLocNeighbor(tempMap.dimension);
  while(true){

    // Get Highest Bid for Optimal Pose Among Those Remaining
    bestBid = MaxOfVector(entropyBumpsAllRobots, optimalRobotInd);

    // Terminate for Insufficient Available Candidates
    if(bestBid <= 0.0){
      for(int i(0); i < entropyBumpsAllRobots.size(); i++){
        if(entropyBumpsAllRobots[i] == 0.0)
          cout << "Robot " << i << " has no further viable candidate options." << endl;
      }
      ros::param::set("run_autonomous_exploration", false);
      ROS_WARN("Candidate(s) no longer provide\nenough information gain for all robots.");
      return false;
    }

    // Update Information for Hightest Bid
    int bestBidInd = robotVec[optimalRobotInd].bestCandInd;
    candIndex = candidateIndices[bestBidInd];
    distsToEachPose[optimalRobotInd] = robotVec[optimalRobotInd].costMap[candIndex];
    mapLocOptimal = tempMap.MapLocationFromInd(candIndex);

    // Remove Highest Bid from Future Consideration
    optimalPosesFound[optimalRobotInd] = true;
    entropyBumpsAllRobots[optimalRobotInd] = -1.0;
    numOptimalPosesFound++;
    if(numOptimalPosesFound >= numRobots)
      break;

    // Functionality of Updating Temporary Map only with 2D Projected Map (bad scalability with large 3D maps)

    // Update 'tempMap' with Expected Measurement Rays
    int size(rayUVecs.size());
    for(int rayInd =  optimalUVecIndsAboutZ[bestBidInd]-numRaysEachSideAboutZ;
            rayInd <= optimalUVecIndsAboutZ[bestBidInd]+numRaysEachSideAboutZ; rayInd++)
      tempMap.UpdateCellsWithExpectedRays(candidateLocations[bestBidInd], rayUVecs[MakeCyclicIndexInt(rayInd, size)]);

    // Update Information Gains in Winning Bid's Neighborhood
    for(int iCand = 0; iCand < numCandidatesTotal; iCand++){
      if(Norm2VecsSqrd(candidateLocations[iCand], candidateLocations[robotVec[optimalRobotInd].bestCandInd]) < sqrdRecomputeRadius)
        UpdateExpectedInfoGain(iCand, tempMap);
    }

    // Compute Best Candidate Indices For Remaining Robots
    for(int iRobot(0); iRobot < numRobots; iRobot++){
      if(!optimalPosesFound[iRobot]){

        for(int iCand = 0; iCand < numCandidatesTotal; iCand++){

          // Only Consider if Not Done & Reachable
          candIndex = candidateIndices[iCand];
          if(!candidateDone[iCand]// not done
             && robotVec[iRobot].costMap[candIndex] < costMapInitValue// reachable
             ){

            // Map Location of Candidate
            mapLocNeighbor = tempMap.MapLocationFromInd(candIndex);

            if(abs(mapLocOptimal[0]-mapLocNeighbor[0]) <= discountRadius
            && abs(mapLocOptimal[1]-mapLocNeighbor[1]) <= discountRadius){

              // Neighbor Robot Discounting Factor
              double distToOptimalPose = NormBtwn2Vecs(mapLocOptimal, mapLocNeighbor);
              if(distToOptimalPose < discountRadius)
                robotVec[iRobot].candidateDiscounts[iCand] *= pow(distToOptimalPose/discountRadius, 2);

              // Apply Bump & Discount
              robotVec[iRobot].candidateInfoGainWithBump[iCand] =
                   robotVec[iRobot].candidateDiscounts  [iCand]
                  *robotVec[iRobot].candidateBumps      [iCand]
                  *candidateExpectedInfoGain            [iCand];

            }
          }
        }

        // Update Desired Pose Accounting for Discounts
        entropyBumpsAllRobots[iRobot] = MaximizeCandidateObjective(robotVec[iRobot]);
      }
    }
  }

  if(timersOn)
    ROS_INFO("Bidding time: %e", ros::Time::now().toSec()-tStart);
  return true;
}


// Trajectories

void Exploration::HaltTrajectories(){


  for(int iRobot(0); iRobot < numRobots; iRobot++){

    // Make a Trajectory To & From Current Pose
    GetSensorPose(iRobot);

    // Custom Message
    robotVec[iRobot].trajInfo.header.frame_id = rdcdMap.frame;
    robotVec[iRobot].trajInfo.header.stamp = ros::Time::now();
    robotVec[iRobot].trajInfo.numSegments = 1;
    robotVec[iRobot].trajInfo.numCoeff = 1;
    robotVec[iRobot].trajInfo.timeStartSeg.resize(2);
    robotVec[iRobot].trajInfo.timeStartSeg[0] = 0.0;
    robotVec[iRobot].trajInfo.timeStartSeg[1] = 1.0;
    robotVec[iRobot].trajInfo.xCoeff.resize(1);
    robotVec[iRobot].trajInfo.xCoeff[0] = robotVec[iRobot].sensorLoc[0];
    robotVec[iRobot].trajInfo.yCoeff.resize(1);
    robotVec[iRobot].trajInfo.yCoeff[0] = robotVec[iRobot].sensorLoc[1];
    robotVec[iRobot].trajInfo.fixedZVal  = height;
    Eigen::Matrix3d R; QuatToRotMatrix(robotVec[iRobot].sensorAtt, R);
    Eigen::Vector3d b1SensorBody, b1SensorWorld; b1SensorBody << 1.0, 0.0, 0.0;
    b1SensorWorld = R*b1SensorBody;
    Eigen::Vector3d b1Sensor;
    MakeRollAndPitch0(robotVec[iRobot].sensorAtt, b1Sensor, 1.0);
    double theta = atan2(b1Sensor(1), b1Sensor(0));

    robotVec[iRobot].trajInfo.thetaStart = theta;
    robotVec[iRobot].trajInfo.thetaEnd   = theta;
    robotVec[iRobot].trajCoeffsPub.publish(robotVec[iRobot].trajInfo);

  }
  explorationStarted = false;
  lockExploreTrajThread = false;// unlock thread

  return;
}
void Exploration::ClearRobotLoc(Mapping& mapToReduce, Mapping& reducedMap, int& numRobots, vector<Robot>& robotVec){

  // Clear Robot Immediate Vicinity
  tf::TransformListener tfListener;
  for(int iTF(0); iTF < numRobots; iTF++){
    string fromFrame = "/"+mapToReduce.frame;
    string toFrame   = "/"+robotVec[iTF].robotFrame;
    tf::StampedTransform robotTF = LoopUntilTransformAcquired(tfListener, fromFrame, toFrame);

    if     (reducedMap.dimension == 2){
      vector<double> robotPt = {robotTF.getOrigin().x(), robotTF.getOrigin().y()};
      reducedMap.SetValToSpaceAroundCellLoc2D<double>(
          robotPt, reqNumFreeNeighborsX, reqNumFreeNeighborsY,
            mapToReduce.occProbMin, reducedMap.occData.data);
    }
    else if(reducedMap.dimension == 3){
      vector<double> robotPt = {robotTF.getOrigin().x(), robotTF.getOrigin().y(), robotTF.getOrigin().z()};
      reducedMap.SetValToSpaceAroundCellLoc3D<double>(
          robotPt, reqNumFreeNeighborsX, reqNumFreeNeighborsY, reqNumFreeNeighborsZ,
            mapToReduce.occProbMin, reducedMap.occData.data);
    }
    else
      ros::shutdown();
  }

  return;
}

void Exploration::SetUpCostMap(Mapping& mapInput, vector<Robot>& robotVec){

  int numRobots = robotVec.size();
  for(int iThisRobot(0); iThisRobot < numRobots; iThisRobot++){
    robotVec[iThisRobot].indCurrent = mapInput.IndFromMapLocation(robotVec[iThisRobot].sensorLoc);
    robotVec[iThisRobot].visited = visited;
    for(int iOtherRobot(0); iOtherRobot < numRobots; iOtherRobot++){
      if(iOtherRobot != iThisRobot){
        if(mapInput.dimension == 2)
          mapInput.SetValToSpaceAroundCellLoc2D<bool>(
              robotVec[iOtherRobot].sensorLoc, reqNumFreeNeighborsX, reqNumFreeNeighborsY,
                true, robotVec[iThisRobot].visited);
        if(mapInput.dimension == 3)
          mapInput.SetValToSpaceAroundCellLoc3D<bool>(
              robotVec[iOtherRobot].sensorLoc, reqNumFreeNeighborsX, reqNumFreeNeighborsY, reqNumFreeNeighborsZ,
                true, robotVec[iThisRobot].visited);
      }
    }
  }
  return;
}

bool Exploration::GenerateCostMap(
    Mapping& mapInput, Robot& robot, int& maxNumCostMapCells){

  // Start Timer
  if(timersOn){tStartSection = ros::Time::now().toSec();}

  // Initializations
  fill(robot.costMap.begin(), robot.costMap.end(), costMapInitValue);
  vector<int> dimInd(mapInput.dimension);
  vector<double> tempCosts; vector<int> tempInds;
  bool replaceInd(false); int indToReplace;
  double nodeValBuff;
  int numIter(0);
  double currentCost(0.0);
  int zMinLim(-1), zMaxLim(1);
  if(mapInput.dimension == 2){
    zMinLim = 0; zMaxLim = 0;
  }
  if(mapInput.stride.size() == 2)
    mapInput.stride.push_back(0);
  robot.costMap[robot.indCurrent] = currentCost;


  // Check All Reachable Cells
  while(true){

    // Current Cell Dimensional Index
    dimInd = mapInput.DimensionalIndicesFromMapIndex(robot.indCurrent);
    if(mapInput.dimension == 2)
      dimInd.push_back(0);

    // Cycle Through All Neighboring Cells
    for(int iX = -1; iX <= 1; iX++){
      for(int iY = -1; iY <= 1; iY++){
        for(int iZ = zMinLim; iZ <= zMaxLim; iZ++){

          // Neighboring Cell Type
          int cellNeighborCase = (iX != 0)+(iY != 0)+(iZ != 0);// 1: face, 2: edge, 3: corner

          // Verify the Neighbor on the Map is Not the Cell in Question
          if(cellNeighborCase != 0
            && iX+dimInd[0] > -1 && iX+dimInd[0] < mapInput.numCellsEachDimension[0]
            && iY+dimInd[1] > -1 && iY+dimInd[1] < mapInput.numCellsEachDimension[1]
            && iZ+dimInd[2] > -1 && iZ+dimInd[2] < mapInput.numCellsEachDimension[2]){

            // Map Index of this Neighbor
            int indPt(
                  (dimInd[0]+iX)*mapInput.stride[0]
                 +(dimInd[1]+iY)*mapInput.stride[1]
                 +(dimInd[2]+iZ)*mapInput.stride[2]);

            // Verify the Cell is Unvisted
            if(!robot.visited[indPt]){

              // Cell Shares a Face
              if(cellNeighborCase == 1)
                nodeValBuff = robot.costMap[robot.indCurrent]+costClosestCell;

              // Cell Shares an Edge
              if(cellNeighborCase == 2)
                nodeValBuff = robot.costMap[robot.indCurrent]+costDiagonal2D ;

              // Cell Shares a Corner
              if(cellNeighborCase == 3)
                nodeValBuff = robot.costMap[robot.indCurrent]+costDiagonal3D ;

              // Assign New Distance Cost to the Node if Possible
              if(nodeValBuff < robot.costMap[indPt]){
                robot.costMap[indPt] = nodeValBuff;

                // Update Temporary Costs & Indices While Minimizing Vector Resizings
                bool foundExistingIndex(false);
                for(int i(0); i < tempInds.size(); i++){
                  if(tempInds[i] == indPt){
                    tempCosts[i] = nodeValBuff;
                    foundExistingIndex = true;
                    break;
                  }
                }
                if(!foundExistingIndex){
                  if(replaceInd){
                    tempCosts[indToReplace] = nodeValBuff;
                    tempInds [indToReplace] = indPt;
                    replaceInd = false;
                  }
                  else{
                    tempCosts.push_back(nodeValBuff);
                    tempInds .push_back(indPt);
                  }
                }
              }
            }
          }
        }
      }
    }

    // Mark Cell as Visited & Ensure it Never Minimizes 'tempCosts'
    if(tempCosts.size() == 0){
      ROS_WARN("Cost Map of %s cannot generate a single cell.\nCheck that the initial value of parameter 'run_autonomous_exploration' is incorrectly true.", robot.ns.c_str());
      return false;
    }
    robot.visited[robot.indCurrent] = true;
    currentCost = MinOfVector(tempCosts, indToReplace);
    robot.indCurrent = tempInds[indToReplace];
    tempCosts[indToReplace] = costMapInitValue;// reset to max value
    replaceInd = true;

    // Repeat if Possible
    numIter++;
    if(currentCost < costMapInitValue && numIter < maxNumCostMapCells)
      maxCostMapAsgndVal = currentCost;
    else{
      if(numIter == 1){
        ROS_ERROR("%s: Moving to any neighboring cells risks collision.", robot.ns.c_str());
        return false;
      }
      break;// cost map complete
    }
  }

  // Visualize (candidates showing entropies only)
  if(visualizeCostMap){
    costMapViz.header.stamp    = ros::Time::now();
    costMapViz.header.frame_id = fullMap.frame;
    GenerateOccupancyGridCostMap(mapInput, robot.costMap, maxCostMapAsgndVal, costMapViz, robot);
    robot.costMapPub.publish(costMapViz);
  }

  cout << "The number of costmap cells is " << numIter << endl;
  // End Timer
  if(timersOn){ROS_INFO("Costmap generation time for %s is %e", robot.ns.c_str(), ros::Time::now().toSec()-tStartSection);}

  return true;
}

bool Exploration::FindDijkstraTrajectory(Mapping& mapInput, Robot& robot){

  if(timersOn){tStartSection = ros::Time::now().toSec();}

  // Convert Candidate Index to Map Index
  int desMapInd = candidateIndices[robot.bestCandInd];

  // Desired (final) & Starting (sensor location) Indices
  vector<int> dimIndDes(mapInput.dimension), dimStart(mapInput.dimension);
  dimIndDes = mapInput.DimensionalIndicesFromMapIndex(desMapInd);
  vector<double> locStart = robot.sensorLoc;
  mapInput.MapLocToDimensionalIndex(locStart, dimStart);

  // Dijkstra's Waypoints Inserted in Reverse with 'pathwayBuff'
  int wpInd(0);
  for(int i(0); i < mapInput.dimension; i++)
    pathwayBuff[i][wpInd] = dimIndDes[i];

  // Map Index of Ending Point
  int indPt(0);
  for(int i(0); i < mapInput.dimension; i++)
    indPt += pathwayBuff[i][wpInd]*mapInput.stride[i];
  djkDistBuff[wpInd] = robot.costMap[indPt];

  // Cycle Through Neighbors, Saving Minimum Cost Neighbor for Next Loop
  int iXSave, iYSave, iZSave, mapIndSave;
  while(true){
    double minVal(costMapInitValue);
    if(mapInput.dimension == 2){
      for(int iX = -1; iX <= 1; iX++){
        for(int iY = -1; iY <= 1; iY++){

          // Verify Cell is a Neighbor on the Map
          if(!(iX == 0 && iY == 0)
            && iX+pathwayBuff[0][wpInd] > -1
            && iX+pathwayBuff[0][wpInd] < mapInput.numCellsEachDimension[0]
            && iY+pathwayBuff[1][wpInd] > -1
            && iY+pathwayBuff[1][wpInd] < mapInput.numCellsEachDimension[1]){

            // Extract Neighbor Cost & Save
            indPt =
                (pathwayBuff[0][wpInd]+iX)*mapInput.stride[0]
               +(pathwayBuff[1][wpInd]+iY)*mapInput.stride[1];
            if(robot.costMap[indPt] < minVal){
              minVal = robot.costMap[indPt];
              iXSave = iX; iYSave = iY;
              mapIndSave = indPt;
            }
          }
        }
      }
    }
    else if(mapInput.dimension == 3){
      for(int iX = -1; iX <= 1; iX++){
        for(int iY = -1; iY <= 1; iY++){
          for(int iZ = -1; iZ <= 1; iZ++){

            // Neighboring Cell Type
            int cellNeighborCase = (iX != 0)+(iY != 0)+(iZ != 0);// 0: same, 1: face, 2: edge, 3: corner

            // Verify Cell is a Neighbor on the Map
            if(cellNeighborCase != 0
              && iX+pathwayBuff[0][wpInd] > -1
              && iX+pathwayBuff[0][wpInd] < mapInput.numCellsEachDimension[0]
              && iY+pathwayBuff[1][wpInd] > -1
              && iY+pathwayBuff[1][wpInd] < mapInput.numCellsEachDimension[1]
              && iZ+pathwayBuff[2][wpInd] > -1
              && iZ+pathwayBuff[2][wpInd] < mapInput.numCellsEachDimension[2]){

              // Extract Neighbor Cost & Save
              indPt =
                  (pathwayBuff[0][wpInd]+iX)*mapInput.stride[0]
                 +(pathwayBuff[1][wpInd]+iY)*mapInput.stride[1]
                 +(pathwayBuff[2][wpInd]+iZ)*mapInput.stride[2];
              if(robot.costMap[indPt] < minVal){
                minVal = robot.costMap[indPt];
                iXSave = iX; iYSave = iY; iZSave = iZ;
                mapIndSave = indPt;
              }
            }
          }
        }
      }
    }


    // Ensure that the Pose is Reachable
    if(minVal == costMapInitValue){
      ROS_ERROR("Dijkstra's algorithm cannot be completed: no viable paths along cost map.");
      return false;
    }

    if(mapInput.dimension == 2){

      // Save Pathway Point to Buffers
      wpInd++;
      pathwayBuff[0][wpInd] = pathwayBuff[0][wpInd-1]+iXSave;
      pathwayBuff[1][wpInd] = pathwayBuff[1][wpInd-1]+iYSave;
      djkDistBuff[wpInd] = robot.costMap[mapIndSave];

      // Dijkstra's Waypoints Successfully Reached Current Robot Pose
      if(pathwayBuff[0][wpInd] == dimStart[0]
      && pathwayBuff[1][wpInd] == dimStart[1]){

        // Initialize Variables to Insert Data from Buffers
        wayPoints.header.stamp = ros::Time::now();
        wayPoints.header.frame_id = mapInput.frame;
        numDjkPts = wpInd+1;
        wayPoints.poses.resize(numDjkPts);
        djkLocations.resize(numDjkPts, vector<double>(mapInput.dimension));
        djkTimes.resize(numDjkPts);
        fixedPoseStampedParams.header.stamp = ros::Time::now();
        fill(wayPoints.poses.begin(), wayPoints.poses.end(), fixedPoseStampedParams);

        // Unsnap Initial Location from Grid to Measured Value
        djkLocations[0] = locStart;
        wayPoints.poses[0].pose.position.x = djkLocations[0][0];
        wayPoints.poses[0].pose.position.y = djkLocations[0][1];
        djkTimes[0] = 0.0;
        wayPoints.poses[0].header.stamp
            = fixedPoseStampedParams.header.stamp;

        // Offsets/Max Speeds to Avoid Quick Rotations
        double theta; QuatToYawRot(theta, robot.sensorAtt);
        vector<double> uvecNow = {cos(theta), sin(theta)};
        theta = acos(DotProd(optimalUVecs[robot.bestCandInd], uvecNow));
        double minTrajTime = theta/maxRadPerSec;
        double distOffset;// distance offset for unsnapped initial location

        // Insert Buffer Data for Remaining Waypoints
        for(int i = 1; i <= wpInd; i++){

          // Waypoint Locations
          djkLocations[i][0] = mapInput.minimumLocations[0]+mapInput.alpha*pathwayBuff[0][wpInd-i];
          wayPoints.poses[i].pose.position.x = djkLocations[i][0];
          djkLocations[i][1] = mapInput.minimumLocations[1]+mapInput.alpha*pathwayBuff[1][wpInd-i];
          wayPoints.poses[i].pose.position.y = djkLocations[i][1];

          // Distance Offset & Robot Speed with Upper Rotational Threshold
          if(i == 1){
            distOffset = NormBtwn2Vecs(djkLocations[1], djkLocations[0])// (djkLocations[1]-djkLocations[0]).norm()
                               -djkDistBuff[wpInd-i];
            double totTrajDist = distOffset+djkDistBuff[0];
            if(totTrajDist/robot.speed < minTrajTime)
              robot.speed = totTrajDist/minTrajTime;
            else
              robot.speed = maxSpeed;

          }

          // Waypoint Times
          djkTimes[i] = (distOffset+djkDistBuff[wpInd-i])/robot.speed;
          wayPoints.poses[i].header.stamp
              = fixedPoseStampedParams.header.stamp
              +ros::Duration(djkTimes[i]);

        }

        // Uncomment for Publishing (publishes when computed, prior to execution)
        robot.waypointsPub.publish(wayPoints);

        if(timersOn){ROS_INFO("Finding Dijkstra waypoints for %s is %e sec.", robot.ns.c_str(), ros::Time::now().toSec()-tStartSection);}

        return true;
      }
      else if(minVal <= 0){
        ROS_ERROR("Costmap for %s is minimized at an incorrect local minimum", robot.ns.c_str());
        ros::shutdown();
        return false;
      }

    }
    else{

      // Save Pathway Point to Buffers
      wpInd++;
      pathwayBuff[0][wpInd] = pathwayBuff[0][wpInd-1]+iXSave;
      pathwayBuff[1][wpInd] = pathwayBuff[1][wpInd-1]+iYSave;
      pathwayBuff[2][wpInd] = pathwayBuff[2][wpInd-1]+iZSave;

      djkDistBuff[wpInd] = robot.costMap[mapIndSave];

      // Dijkstra's Waypoints Successfully Reached Current Robot Pose
      if(pathwayBuff[0][wpInd] == dimStart[0]
      && pathwayBuff[1][wpInd] == dimStart[1]
      && pathwayBuff[2][wpInd] == dimStart[2]){

        // Initialize Variables to Insert Data from Buffers
        wayPoints.header.stamp = ros::Time::now();
        wayPoints.header.frame_id = mapInput.frame;
        numDjkPts = wpInd+1;
        wayPoints.poses.resize(numDjkPts);
        djkLocations.resize(numDjkPts, vector<double>(mapInput.dimension));
        djkTimes.resize(numDjkPts);
        fixedPoseStampedParams.header.stamp = ros::Time::now();
        fill(wayPoints.poses.begin(), wayPoints.poses.end(), fixedPoseStampedParams);

        // Unsnap Initial Location from Grid to Measured Value
        djkLocations[0] = locStart;
        wayPoints.poses[0].pose.position.x = djkLocations[0][0];
        wayPoints.poses[0].pose.position.y = djkLocations[0][1];
        wayPoints.poses[0].pose.position.z = djkLocations[0][2];
        djkTimes[0] = 0.0;
        wayPoints.poses[0].header.stamp
            = fixedPoseStampedParams.header.stamp;

        // Offsets/Max Speeds to Avoid Quick Rotations
        Eigen::Vector3d b1Sensor;
        MakeRollAndPitch0(robot.sensorAtt, b1Sensor, 1.0);
        double theta = atan2(b1Sensor(1), b1Sensor(0));
        vector<double> uvecNow = {cos(theta), sin(theta)};
        theta = acos(DotProd(optimalUVecs[robot.bestCandInd], uvecNow));
        double minTrajTime = theta/maxRadPerSec;
        double distOffset;// distance offset for unsnapped initial location

        // Insert Buffer Data for Remaining Waypoints
        for(int i = 1; i <= wpInd; i++){

          // Waypoint Locations
          djkLocations[i][0] = mapInput.minimumLocations[0]+mapInput.alpha*pathwayBuff[0][wpInd-i];
          wayPoints.poses[i].pose.position.x = djkLocations[i][0];
          djkLocations[i][1] = mapInput.minimumLocations[1]+mapInput.alpha*pathwayBuff[1][wpInd-i];
          wayPoints.poses[i].pose.position.y = djkLocations[i][1];
          djkLocations[i][2] = mapInput.minimumLocations[2]+mapInput.alpha*pathwayBuff[2][wpInd-i];
          wayPoints.poses[i].pose.position.z = djkLocations[i][2];

          // Distance Offset & Robot Speed with Upper Rotational Threshold
          if(i == 1){

            distOffset = NormBtwn2Vecs(djkLocations[1], djkLocations[0])// (djkLocations[1]-djkLocations[0]).norm()
                               -djkDistBuff[wpInd-i];
            double totTrajDist = distOffset+djkDistBuff[0];
            if(totTrajDist/robot.speed < minTrajTime)
              robot.speed = totTrajDist/minTrajTime;
            else
              robot.speed = maxSpeed;

          }

          // Waypoint Times
          djkTimes[i] = (distOffset+djkDistBuff[wpInd-i])/robot.speed;
          wayPoints.poses[i].header.stamp
              = fixedPoseStampedParams.header.stamp
              +ros::Duration(djkTimes[i]);
        }

        // Uncomment for Publishing (publishes when computed, prior to execution)
        robot.waypointsPub.publish(wayPoints);

        if(timersOn){ROS_INFO("Finding Dijkstra waypoints for %s is %e sec.", robot.ns.c_str(), ros::Time::now().toSec()-tStartSection);}

        return true;
      }
      else if(minVal <= 0){
        ROS_ERROR("Costmap for %s is minimized at an incorrect local minimum", robot.ns.c_str());
        ros::shutdown();
        return false;
      }
    }
  }
  return false;
}

bool Exploration::GenerateSegmentsForPatching(Robot& robot){

  if(timersOn){tStartSection = ros::Time::now().toSec();}

  // At Least 2 Points Required
  if(numDjkPts < 2){
      ROS_ERROR("At least 2 waypoints are required for trajectory generation.");
      return false;
  }

  // The Minimum Required Points for Constrained Least Squares
  int reqPts = pointsPerSegment+2;// pts/(1 seg) + (start & terminal points)

  // If More Points are Needed
  if(numDjkPts < reqPts){

    // Insert 'numPtsAdd' >= 0 by Interpolation if Necessary
    int numPtsAdd(reqPts-numDjkPts);

    // Obtain the Time & Distance Between the Terminal Point & Point Preceeding for a Unit Vector
    double delT = djkTimes[numDjkPts-1]-djkTimes[numDjkPts-2];
    double dist = robot.speed*delT, normUVec;
    vector<double> uVec = GetUVec(djkLocations[numDjkPts-2], djkLocations[numDjkPts-1], normUVec);
//    Eigen::Vector2d uVec = djkLocations[numDjkPts-1]-djkLocations[numDjkPts-2];
//    uVec /= uVec.norm();

    // Insert Interpolated Points
    djkLocations.resize(reqPts, vector<double>(tempMap.dimension));
    djkTimes    .resize(reqPts);
    for(int i = numDjkPts-1; i < reqPts; i++){
      for(int iDim(0); iDim < tempMap.dimension; iDim++)
        djkLocations[i][iDim] = djkLocations[i-1][iDim]+(dist/(numPtsAdd+1))*uVec[iDim];
      djkTimes[i] = djkTimes[i-1]+delT/(numPtsAdd+1);
    }
    numDjkPts = reqPts;

    // This Case Corresponds to a Single Segment
    numSegments = 1;
    remainingPts = 0;
  }

  // Dijkstra's Waypoints Alone are Enough for Constrained Least Squares
  else{

    // -1: for shared waypoints between segments (num & den), -2: for endpoints (num) (-3 together)
    numSegments = floor((numDjkPts-3)/(pointsPerSegment-1));

    // -1: shared waypoint, +1: last unshared waypoint, -2: endpoints
    remainingPts = numDjkPts-(numSegments*(pointsPerSegment-1)+1)-2;

  }
  if(timersOn){ROS_INFO("Segmenting the trajectory waypoints for %s is %e sec.", robot.ns.c_str(), ros::Time::now().toSec()-tStartSection);}

  return true;
}

void Exploration::GenerateSmoothTrajectory(Mapping& mapInput, Robot& robot, ros::Time& rosTimeStartTraj){

  // Start Time
  if(timersOn){tStartSection = ros::Time::now().toSec();}

  // State Matrix 'AMatrix' Relates Polynomial Coefficients of 'tVec' to 'xVec'/'yVec'/'zVec'
  int numRows = numDjkPts-2+(numSegments-1);// -2: endpoints (constraints), numSegments-1: shared waypoints
  int numCoeff = polyOrder+1;
  Eigen::MatrixXd AMatrix = Eigen::MatrixXd::Zero(numRows, numSegments*(numCoeff));
  Eigen::VectorXd xVec =    Eigen::VectorXd::Zero(numRows);
  Eigen::VectorXd yVec =    Eigen::VectorXd::Zero(numRows);
  Eigen::VectorXd zVec =    Eigen::VectorXd::Zero(numRows);
  Eigen::VectorXd tVec =    Eigen::VectorXd::Zero(numRows);

  // Constraints
  int numConstraints(2*numSegments);
  Eigen::MatrixXd CMatrix = Eigen::MatrixXd::Zero(numConstraints, numSegments*(numCoeff));

  // Polulate Matrices & Vectors
  int numPts, iStartRow, iRow, iStartCol, CMatRow(0), numSgn(1), tmp;
  vector<double> timeStartSeg(numSegments+1, 0.0);// start time, shared times among segments, terminal time

  for(int iSegs = 0; iSegs < numSegments; iSegs++){

    // Row & Column Indices Shared Among Time, State, & Constraint Matrices/Vectors
    iStartRow = iSegs*(pointsPerSegment);
    iStartCol = iSegs*(numCoeff);

    // Time Vector
    if(iSegs > 0)
      timeStartSeg[iSegs] = djkTimes[iStartRow-iSegs+1];

    // Determine the Number of Points on this Segment
    if(iSegs < numSegments-1)
      numPts = pointsPerSegment;
    else
      numPts = pointsPerSegment+remainingPts;


    // __ State Matrix __ \\

    for(int iPts = 0; iPts < numPts; iPts++){
      iRow = iStartRow+iPts;
      tVec(iRow) = djkTimes[iRow+1-iSegs]-timeStartSeg[iSegs];
      xVec(iRow) = djkLocations[iRow+1-iSegs][0];
      yVec(iRow) = djkLocations[iRow+1-iSegs][1];
      if(mapInput.dimension == 3){zVec(iRow) = djkLocations[iRow+1-iSegs][2];}
      PopulateStateCoeffMatrixPosition(
            iRow, iStartCol, numCoeff, AMatrix, tVec(iRow), numSgn);
    }


    // __ Constraint Matrix __ \\

    // Fixed Starting Point
    if(CMatRow == 0){
      CMatrix(0,0) = 1.0;
      CMatRow++;
    }

    // Shared Waypoints Have the Same Position & Velocity
    else if(CMatRow < numConstraints-1){// patched spline position & velocity constraints (-1*last+current = 0)

      // Position (taken from 'AMatrix')
      CMatrix.block(CMatRow,iStartCol-(numCoeff),1,numCoeff)
          = -1*(AMatrix.block(iStartRow-1,iStartCol-(numCoeff),1,numCoeff));
      CMatrix.block(CMatRow,iStartCol,1,numCoeff)
          = (AMatrix.block(iStartRow,iStartCol,1,numCoeff));
      CMatRow++;

      // Velocity
      tmp = iStartCol-numCoeff;
      double t = timeStartSeg[iSegs]-timeStartSeg[iSegs-1];
      PopulateStateCoeffMatrixVelocity(
            CMatRow, tmp, numCoeff, CMatrix, t, -numSgn);
      CMatrix(CMatRow,iStartCol+1) = 1.0;// t = 0 => 1, 0, 0, ...
      CMatRow++;
    }
  }

  // Fixed Terminal Point
  if(CMatRow == numConstraints-1){
    timeStartSeg[numSegments] = djkTimes.back();
    double t = timeStartSeg[numSegments]-timeStartSeg[numSegments-1];
    tmp =  numConstraints-1;
    PopulateStateCoeffMatrixPosition(
          tmp, iStartCol, numCoeff, CMatrix, t, numSgn);
  }

  // Generate State & Constraint Matrix to be Inverted
  int dimCCM = AMatrix.cols()+CMatrix.rows();
  Eigen::MatrixXd ConstrainedCostsMatrix = Eigen::MatrixXd::Zero(dimCCM,dimCCM);
  ConstrainedCostsMatrix.block(0,0,AMatrix.cols(),AMatrix.cols()) = 2*AMatrix.transpose()*AMatrix;
  ConstrainedCostsMatrix.block(AMatrix.cols(),0,CMatrix.rows(),CMatrix.cols()) = CMatrix;
  ConstrainedCostsMatrix.block(0,AMatrix.cols(),CMatrix.cols(),CMatrix.rows()) = CMatrix.transpose();

  // Solution Vector
  VectorXd locAndConstraintsVec = Eigen::VectorXd::Zero(AMatrix.cols()+numConstraints);
  VectorXd xCoeff(dimCCM), yCoeff(dimCCM), zCoeff(dimCCM);

  // X Version of Solution Vector
  locAndConstraintsVec.block(0,0,AMatrix.cols(),1) = 2*AMatrix.transpose()*xVec;
  locAndConstraintsVec(AMatrix.cols()) = djkLocations[0][0];
  locAndConstraintsVec(AMatrix.cols()+numConstraints-1) = djkLocations.back()[0];
  xCoeff = ConstrainedCostsMatrix.colPivHouseholderQr().solve(locAndConstraintsVec);

  // Y Version of Solution Vector
  locAndConstraintsVec.block(0,0,AMatrix.cols(),1) = 2*AMatrix.transpose()*yVec;
  locAndConstraintsVec(AMatrix.cols()) = djkLocations[0][1];
  locAndConstraintsVec(AMatrix.cols()+numConstraints-1) = djkLocations.back()[1];
  yCoeff = ConstrainedCostsMatrix.colPivHouseholderQr().solve(locAndConstraintsVec);

  if(mapInput.dimension == 3){

    // Z Version of Solution Vector
    locAndConstraintsVec.block(0,0,AMatrix.cols(),1) = 2*AMatrix.transpose()*zVec;
    locAndConstraintsVec(AMatrix.cols()) = djkLocations[0][2];
    locAndConstraintsVec(AMatrix.cols()+numConstraints-1) = djkLocations.back()[2];
    zCoeff = ConstrainedCostsMatrix.colPivHouseholderQr().solve(locAndConstraintsVec);

  }

  // Initialize Path Message
  std_msgs::Header header;
  header.frame_id = mapInput.frame;
  header.stamp = rosTimeStartTraj;
  int numTimeSteps = ceil(timeStartSeg[numSegments]/trajTimeStep);
  double dt = timeStartSeg[numSegments]/numTimeSteps;
  numTimeSteps += 1;// include time step for 0

  // Set Initial Attitudes s.t. b1 Exists on a Plane Normal to e3
  Eigen::Vector3d b1Sensor;
  MakeRollAndPitch0(robot.sensorAtt, b1Sensor, 1);

  // Populate 'trajInfo' Object
  robot.trajInfo.header = header;
  robot.trajInfo.numSegments = numSegments;
  robot.trajInfo.numCoeff = numCoeff;
  robot.trajInfo.timeStartSeg = timeStartSeg;
  unsigned numUsefulCoeff = numSegments*numCoeff;
  robot.trajInfo.xCoeff.resize(numUsefulCoeff); EigenToStack(xCoeff, robot.trajInfo.xCoeff, numUsefulCoeff);
  robot.trajInfo.yCoeff.resize(numUsefulCoeff); EigenToStack(yCoeff, robot.trajInfo.yCoeff, numUsefulCoeff);

  // 2D: fixed height, 3D: variable height
  if(mapInput.dimension == 3){
    robot.trajInfo.zCoeff.resize(numUsefulCoeff);
    EigenToStack(zCoeff, robot.trajInfo.zCoeff, numUsefulCoeff);
  }
  else{
    robot.trajInfo.zCoeff.resize(0);
    robot.trajInfo.fixedZVal  = height;
  }

  // Starting/Ending Camera Angle
  robot.trajInfo.thetaStart
      = atan2(b1Sensor(1)                       , b1Sensor(0)                       );
  robot.trajInfo.thetaEnd
      = atan2(optimalUVecs[robot.bestCandInd][1], optimalUVecs[robot.bestCandInd][0]);

  // Publish Trajectory Information
  robot.trajCoeffsPub.publish(robot.trajInfo);

  // Publish Paths if Requested
  if(visualizePaths){

    // Initializations
    robot.camPosPath.header = header;
    robot.robPosPath.header = header;
    robot.camPosPath.poses.resize(numTimeSteps);
    robot.robPosPath.poses.resize(numTimeSteps);
    trajTimeVec.resize(numTimeSteps);

    // Temporary Stamped ROS Message
    geometry_msgs::PoseStamped  desCamMsg;
    geometry_msgs::PoseStamped  desPosMsg;
    geometry_msgs::TwistStamped desVelMsg;
    geometry_msgs::AccelStamped desAclMsg;

    // Cycle Through All Time Steps: Obtain Poses, Velocities, & Accelerations
    for(int iTimeStep(0); iTimeStep < numTimeSteps; iTimeStep++){

      // Time under consideration
      trajTimeVec[iTimeStep] = iTimeStep*dt;
      ros::Time timeNow = rosTimeStartTraj+ros::Duration(trajTimeVec[iTimeStep]);

      // Pose from Trajectory
      geometry_msgs::Vector3 x3Dot, x4Dot;
      GetTrajFromCoeff(tempMap.dimension, robot, timeNow, desCamMsg, desPosMsg, desVelMsg, desAclMsg, x3Dot, x4Dot);
      robot.camPosPath.poses[iTimeStep] = desCamMsg;
      robot.robPosPath.poses[iTimeStep] = desPosMsg;

    }
  }

  if(timersOn){ROS_INFO("Generating a smooth trajectory for %s is %e sec.", robot.ns.c_str(), ros::Time::now().toSec()-tStartSection);}
  return;
}

Eigen::Vector3d Exploration::GetTrajFromCoeff(int dimension, Robot& robot, ros::Time& timeNow,
    geometry_msgs::PoseStamped&  desCamMsg, geometry_msgs::PoseStamped&  desPosMsg,
    geometry_msgs::TwistStamped& desVelMsg, geometry_msgs::AccelStamped& desAclMsg,
    geometry_msgs::Vector3& x3Dot, geometry_msgs::Vector3& x4Dot){

  // Initializations
  double x(0.0), xDot(0.0), xDDot(0.0), xDDDot(0.0), xDDDDot(0.0), aX(0.0),
         y(0.0), yDot(0.0), yDDot(0.0), yDDDot(0.0), yDDDDot(0.0), aY(0.0),
         z(0.0), zDot(0.0), zDDot(0.0), zDDDot(0.0), zDDDDot(0.0), aZ(0.0);

  // Time Along Trajectory (starting at 0) & Total Duration
  double trajTimeNow = timeNow.toSec()-robot.trajInfo.header.stamp.toSec();
  double trajTimeMax = robot.trajInfo.timeStartSeg.back();
  if(trajTimeNow > trajTimeMax)
    trajTimeNow = trajTimeMax;

  // Obtain Segment of Coefficients
  int numSegments = robot.trajInfo.numSegments;
  int currentSeg = DetermineSegmentFromPoint(trajTimeNow, numSegments, robot.trajInfo.timeStartSeg);

  // Time Along Current Segment (each segment starting at 0)
  double correctedTime = trajTimeNow-robot.trajInfo.timeStartSeg[currentSeg];

  // Position, Velocity, & Acceleration
  for(int iCoeff(0); iCoeff < robot.trajInfo.numCoeff; iCoeff++){
    aX = robot.trajInfo.xCoeff[currentSeg*robot.trajInfo.numCoeff+iCoeff];
    x += aX*pow(correctedTime, iCoeff);
    aY = robot.trajInfo.yCoeff[currentSeg*robot.trajInfo.numCoeff+iCoeff];
    y += aY*pow(correctedTime, iCoeff);
    if(dimension == 3){
      aZ = robot.trajInfo.zCoeff[currentSeg*robot.trajInfo.numCoeff+iCoeff];
      z += aZ*pow(correctedTime, iCoeff);
    }

    // 1st-4th Time Derivatives
    if(iCoeff > 0){
      xDot += aX*iCoeff
          *pow(correctedTime, iCoeff-1);
      yDot += aY*iCoeff
          *pow(correctedTime, iCoeff-1);
      if(dimension == 3)
        zDot += aZ*iCoeff
            *pow(correctedTime, iCoeff-1);
      if(iCoeff > 1){
        xDDot += aX*iCoeff*(iCoeff-1)
            *pow(correctedTime, iCoeff-2);
        yDDot += aY*iCoeff*(iCoeff-1)
            *pow(correctedTime, iCoeff-2);
        if(dimension == 3)
          zDDot += aZ*iCoeff*(iCoeff-1)
              *pow(correctedTime, iCoeff-2);
        if(iCoeff > 3){
          xDDDot += aX*iCoeff*(iCoeff-1)*(iCoeff-2)
              *pow(correctedTime, iCoeff-3);
          yDDDot += aY*iCoeff*(iCoeff-1)*(iCoeff-2)
              *pow(correctedTime, iCoeff-3);
          if(dimension == 3)
            zDDDot += aZ*iCoeff*(iCoeff-1)*(iCoeff-2)
                *pow(correctedTime, iCoeff-3);
          if(iCoeff > 4){
            xDDDDot += aX*iCoeff*(iCoeff-1)*(iCoeff-2)*(iCoeff-3)
                *pow(correctedTime, iCoeff-4);
            yDDDDot += aY*iCoeff*(iCoeff-1)*(iCoeff-2)*(iCoeff-3)
                *pow(correctedTime, iCoeff-4);
            if(dimension == 3)
              zDDDDot += aZ*iCoeff*(iCoeff-1)*(iCoeff-2)*(iCoeff-4)
                  *pow(correctedTime, iCoeff-4);
          }
        }
      }
    }
  }


  // __ Rotational Kinematics __ \\

  // Starting & Ending Angle
  double thetaChange = robot.trajInfo.thetaEnd-robot.trajInfo.thetaStart;

  // Move Angle Difference from Domain [0,2*pi) to (-pi,pi]
  if(thetaChange >   M_PI)
    thetaChange -= 2*M_PI;
  if(thetaChange <= -M_PI)
    thetaChange += 2*M_PI;

  // Angular Velocity
  double omegaZ = thetaChange/trajTimeMax;

  // Camera Direction about 3rd Body-Fixed Axis
  double desTheta = robot.trajInfo.thetaStart+omegaZ*trajTimeNow;


  // __ Output __ \\

  // Header
  std_msgs::Header header;
  header.frame_id = robot.trajInfo.header.frame_id;
  header.stamp = robot.trajInfo.header.stamp+ros::Duration(trajTimeNow);
  desCamMsg.header = desPosMsg.header = desVelMsg.header = desAclMsg.header = header;

  // Camera Pose
  desCamMsg.pose.position.x = x;
  desCamMsg.pose.position.y = y;
  if(dimension == 3){desCamMsg.pose.position.z = z;}
  else{desCamMsg.pose.position.z = robot.trajInfo.fixedZVal;}
  Eigen::Matrix3d R = Euler321ToMatrix(desTheta, radiansCamPitch, 0.0);
  RotMatrixToQuat(desCamMsg.pose.orientation, R);

  // Robot Pose
  desPosMsg.pose.position.x = desCamMsg.pose.position.x-robot.vecRobot2Camera_RobotFrame(0);
  desPosMsg.pose.position.y = desCamMsg.pose.position.y-robot.vecRobot2Camera_RobotFrame(1);
  desPosMsg.pose.position.z = desCamMsg.pose.position.z-robot.vecRobot2Camera_RobotFrame(2);
  desPosMsg.pose.orientation = FixedQuatTransformation(desCamMsg.pose.orientation, robot.RotMatrixCameraRobot);
  // Note: pose offset uses 'vecRobot2Camera_RobotFrame' instead of 'vecRobot2Camera_CameraFrame' because camera may be flipped

  // Camera & Robot Velocity, Robot Higher Derivatives
  geometry_msgs::Vector3 zeroVec; zeroVec.x = 0.0; zeroVec.y = 0.0; zeroVec.z = 0.0;// convenience

  if(trajTimeNow < trajTimeMax){// nonzero speed along trajectory

    // Translational Velocity
    desVelMsg.twist.linear.x = xDot;
    desVelMsg.twist.linear.y = yDot;
    if(dimension == 3){desVelMsg.twist.linear.z = zDot;}
    else{desVelMsg.twist.linear.z = 0.0;}

    // Angular Velocity
    desVelMsg.twist.angular = zeroVec;
    desVelMsg.twist.angular.z = omegaZ;

    // Translational Acceleration
    desAclMsg.accel.linear.x = xDDot;
    desAclMsg.accel.linear.y = yDDot;
    if(dimension == 3){desAclMsg.accel.linear.z = zDDot;}
    else{desAclMsg.accel.linear.z = 0.0;}

    // 3rd & 4th Translational Derivatives
    x3Dot.x = xDDDot ; x3Dot.y = yDDDot ; x3Dot.z = zDDDot ;
    x4Dot.x = xDDDDot; x4Dot.y = yDDDDot; x4Dot.z = zDDDDot;
  }
  else{// stop at the end: all 0
    desVelMsg.twist.linear  = zeroVec;
    desVelMsg.twist.angular = zeroVec;
    desAclMsg.accel.linear  = zeroVec;
    x3Dot                   = zeroVec;
    x4Dot                   = zeroVec;
  }

  // Always No Rotational Acceleration
  desAclMsg.accel.angular = zeroVec;

  // Return Desired Direction of the 1st Axis of the Body-Fixed Frame
  Eigen::Matrix3d robotRd;
  QuatToRotMatrix(desPosMsg.pose.orientation, robotRd);
  return robotRd*e1;
}

int Exploration::DetermineSegmentFromPoint(double& timeNow, int& numSegments, vector<double>& timeStartSeg){
  int currentSeg = numSegments-1;
  while(true){
    if(timeNow < timeStartSeg[currentSeg]){
      currentSeg--;
      if(currentSeg <= 0){
        currentSeg = 0;
        break;
      }
    }
    else
      break;
  }

  return currentSeg;
}


// Visualization

void Exploration::VisualizeCandidates(
    double& bestCandInfoGain, vector<double>& vals, ros::Publisher& publisher, int& bestInd, double& minVal){

  // Message Time Stamp
  arrow.header.stamp = ros::Time::now();

  // Cycle Through Viable Candidates, Generating Marker Array Message
  for(int iCand = 0; iCand < numCandidatesTotal; iCand++){
    int candIndex = candidateIndices[iCand];
    if(!candidateDone[iCand] && freeCells[candIndex]){// passes entropy threshold & reachable

      // Arrow ID
      arrow.id = iCand;

      // Arrow Color
      if(vals[iCand] >= minVal){
        arrow.color.b = vals[iCand]/bestCandInfoGain;
        arrow.color.r = 1.0-arrow.color.b;
        if(iCand == bestInd)// full opacity only for optimal candidate
          arrow.color.a = 1.0;
        else
          arrow.color.a = nonOptimalOpacity;
      }
      else
        arrow.color.a = 0.0;

      // Arrow Locations (start & tip)
      arrowLoc.x = candidateLocations[iCand][0];//-0.5*arrowLength*optimalUVecs[iCand][0];
      arrowLoc.y = candidateLocations[iCand][1];//-0.5*arrowLength*optimalUVecs[iCand][1];
      arrowLoc.z = candidateLocations[iCand][2];//-0.5*arrowLength*optimalUVecs[iCand][1];
      arrow.points[0] = arrowLoc;
      arrowLoc.x += arrowLength*optimalUVecs[iCand][0];
      arrowLoc.y += arrowLength*optimalUVecs[iCand][1];
      arrowLoc.z += arrowLength*optimalUVecs[iCand][2];
      arrow.points[1] = arrowLoc;
      entropyArrowArray.markers[iCand] = arrow;

    }
    else// Make Transparent (delete action is unreliable)
      entropyArrowArray.markers[iCand].color.a = 0.0;
  }

  // Publish Candidate Marker Message
  publisher.publish(entropyArrowArray);


  return;
}

void Exploration::VisualizeCandidatesMultiRobot(vector<Robot>& robotVec, int& numRobots){

  // Initializations
  vector<double> bestCandInfoGains(numRobots, 0.0), infoGainsBuff(numRobots, 0.0);
  vector<int> bestCandIdices(numRobots,0);
  int bestInd;

  // Message Time Stamp
  arrow.header.stamp = ros::Time::now();

  for(int iRobot = 0; iRobot < numRobots; iRobot++)
    bestCandInfoGains[iRobot] = MaxOfVector(robotVec[iRobot].candidateInfoGainWithBump, bestCandIdices[iRobot]);
  double bestCandInfoGain = MaxOfVector(bestCandInfoGains, bestInd);

  // Cycle Through Viable Candidates, Generating Marker Array Message
  for(int iCand = 0; iCand < numCandidatesTotal; iCand++){
    int candIndex = candidateIndices[iCand];
    if(!candidateDone[iCand]
    && freeCells[candIndex]
    && candidateExpectedInfoGain[iCand] > 0.0){// passes entropy threshold & reachable

      // Arrow ID
      arrow.id = iCand;

      // Arrow Color (opacity: candidate reward, color: robot identifier)
      for(int iRobot = 0; iRobot < numRobots; iRobot++)
        infoGainsBuff[iRobot] = robotVec[iRobot].candidateInfoGainWithBump[iCand];
      MaxOfVector(infoGainsBuff, bestInd);
      for(int iRobot = 0; iRobot < numRobots; iRobot++){
        if(robotVec[iRobot].candidateInfoGainWithBump[iCand]
           == bestCandInfoGains[iRobot])
          bestInd = iRobot;
      }
      for(int iRobot = 0; iRobot < numRobots; iRobot++){
        if(bestInd == iRobot){
          arrow.color = robotVec[iRobot].color;
          arrow.color.a = infoGainsBuff[iRobot]/bestCandInfoGain;
          break;
        }
      }

      // Arrow Locations (start & tip)
      arrowLoc.x = candidateLocations[iCand][0];//-0.5*arrowLength*optimalUVecs[iCand][0];
      arrowLoc.y = candidateLocations[iCand][1];//-0.5*arrowLength*optimalUVecs[iCand][1];
      arrowLoc.z = candidateLocations[iCand][2];//-0.5*arrowLength*optimalUVecs[iCand][1];
      arrow.points[0] = arrowLoc;

      arrowLoc.x += arrowLength*magnitudeXY*optimalUVecs[iCand][0];
      arrowLoc.y += arrowLength*magnitudeXY*optimalUVecs[iCand][1];
      arrowLoc.z += arrowLength*optimalUVecZComp;
      arrow.points[1] = arrowLoc;
      entropyArrowArray.markers[iCand] = arrow;
    }
    else// Make Transparent (delete action is unreliable)
      entropyArrowArray.markers[iCand].color.a = 0.0;
  }

  // Publish Candidate Marker Message
  candidatesDiscountedPub.publish(entropyArrowArray);


  return;
}

void Exploration::GenerateOccupancyGridCostMap(
    Mapping& mapInput, vector<double>& costMap, double& maxVal, nav_msgs::OccupancyGrid& ogm, Robot& robot){

  if(mapInput.dimension == 2){
    for(int ind2D(0); ind2D < mapInput.numCellsTotal; ind2D++){
      if(costMap[ind2D] < maxVal)
        ogm.data[ind2D] = floor(costMap[ind2D]/maxVal*100+0.5);
      else
        ogm.data[ind2D] = 100;
    }
  }
  else{
    int i(0);
    double costMapValTemp;
    vector<double> testLoc(3);
    testLoc[2] = ogm.info.origin.position.z = robot.sensorLoc[2];
    for(int iY(0); iY < mapInput.numCellsEachDimension[1]; iY++){
      testLoc[1] = mapInput.minimumLocations[1]+iY*mapInput.alpha;
      for(int iX(0); iX < mapInput.numCellsEachDimension[0]; iX++){
        testLoc[0] = mapInput.minimumLocations[0]+iX*mapInput.alpha;
        costMapValTemp = costMap[mapInput.IndFromMapLocation(testLoc)];
        if(costMapValTemp < maxVal)
          ogm.data[i] = floor(costMapValTemp/maxVal*100+0.5);
        else
          ogm.data[i] = 100;
        i++;
      }
    }
  }

  return;
}

void Exploration::PublishPaths(Robot& robot){

  // Camera Position
  robot.cameraPositionPub
      .publish(robot.camPosPath);

  // Robot Position
  robot.desiredTrajPositionPub
      .publish(robot.robPosPath);

  return;
}


// Misc.

void Exploration::GetSensorPose(int& robotNum){

  // Obtain Sensor TF
  tf::StampedTransform transform
      = LoopUntilTransformAcquired(
        tfListener, rdcdMap.frame,
        robotVec[robotNum].exploreFrame);

  // Sensor Position
  robotVec[robotNum].sensorLoc
      = {transform.getOrigin().x(),
         transform.getOrigin().y(),
         transform.getOrigin().z()};

  // Sensor Attitude
  tf::Quaternion quatTF = transform.getRotation();
  robotVec[robotNum].sensorAtt.x = quatTF.x();
  robotVec[robotNum].sensorAtt.y = quatTF.y();
  robotVec[robotNum].sensorAtt.z = quatTF.z();
  robotVec[robotNum].sensorAtt.w = quatTF.w();

  return;

}

void Exploration::GetRobotCameraTFs(Robot& robot){

  // TF from Robot to Camera
  transformR2C = LoopUntilTransformAcquired(tfListener, robot.robotFrame, robot.exploreFrame);

  // Camera Position Offset from Robot (robot frame)
  robot.vecRobot2Camera_RobotFrame <<
      transformR2C.getOrigin().x(),
      transformR2C.getOrigin().y(),
      transformR2C.getOrigin().z();

  // Attitude Offset (quaternion)
  geometry_msgs::Quaternion quatR2C;
  quatR2C.x = transformR2C.getRotation().x();
  quatR2C.y = transformR2C.getRotation().y();
  quatR2C.z = transformR2C.getRotation().z();
  quatR2C.w = transformR2C.getRotation().w();

  // Rotation Matrices
  QuatToRotMatrix(quatR2C, RotMatrixRobotCamera) ;
  robot.RotMatrixCameraRobot = RotMatrixRobotCamera.transpose();

  // Camera Position Offset from Robot (camera frame)
  robot.vecRobot2Camera_CameraFrame = robot.RotMatrixCameraRobot*robot.vecRobot2Camera_RobotFrame;

  return;
}

int Exploration::MakeCyclicIndexInt(int& signedInt, int& size){

  if(signedInt < 0)
    return size+signedInt;
  else if(signedInt >= size)
    return signedInt-size;
  else
    return signedInt;

}









