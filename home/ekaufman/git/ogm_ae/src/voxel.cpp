#include "ogm_ae/voxel.h"

Voxel::Voxel(){}


void Voxel::mapInfo::GetMapParameters(string& objective){
  string paramNameStart = "/"+objective+"/";
  ros::param::get(paramNameStart+"occ_topic",  occTopic  );
  ros::param::get(paramNameStart+"H_topic",    HTopic    );
  ros::param::get(paramNameStart+"frame",      frame     );
  ros::param::get(paramNameStart+"dimension",  dimension );
  ros::param::get(paramNameStart+"alpha",      alpha     );
  ros::param::get(paramNameStart+"P_init",     initP     );
  ros::param::get(paramNameStart+"occ_min",    occProbMin);
  ros::param::get(paramNameStart+"occ_max",    occProbMax);
  ros::param::get(paramNameStart+"occ_thresh", occThresh );
  ros::param::get(paramNameStart+"track_diff", trackDiff );
  minimumLocations.resize(dimension);
  maximumLocations.resize(dimension);
  dimNames = {"x","y","z"};
  for(int i(0); i < dimension; i++){
    ros::param::get(paramNameStart+dimNames[i]+"_map_min", minimumLocations[i]);
    ros::param::get(paramNameStart+dimNames[i]+"_map_max", maximumLocations[i]);
  }
  if(objective == "mapping"){
    ros::param::get(paramNameStart+"rgb_topic",  rgbTopic    );
    ros::param::get(paramNameStart+"diff_topic", changesTopic);
  }

  return;
}


void Voxel::GetExplorationParameters(){
  string paramNameStart("/exploration/"), nsParamStart, vizParamStart;
  vizParamStart = paramNameStart+"visualization/";

  // Robot-Nonspecific
  ros::param::get(paramNameStart+"height",               explore.height                 );
  ros::param::get(paramNameStart+"acceptable_coll_prob", explore.acceptableCollisionProb);
  ros::param::get(paramNameStart+"candidate_separation", explore.candidateSeparation    );
  ros::param::get(paramNameStart+"min_info_gain_thresh", explore.minInfoGainThresh      );
  ros::param::get(paramNameStart+"max_speed",            explore.maxSpeed               );
  ros::param::get(paramNameStart+"poly_order",           explore.polyOrder              );
  ros::param::get(paramNameStart+"points_per_segment",   explore.pointsPerSegment       );
  ros::param::get(paramNameStart+"traj_time_step",       explore.trajTimeStep           );
  ros::param::get(paramNameStart+"timers_on",            explore.timersOn               );
  ros::param::get(paramNameStart+"simulate_gazebo",      explore.simulateGazebo         );
  ros::param::get(paramNameStart+"gazebo_topic",         explore.gazeboTopic            );
  ros::param::get(paramNameStart+"free_nbrs_coll_map",   explore.reqNumFreeNeighbors    );
  ros::param::get(paramNameStart+"num_rays_per_cand",    explore.numRaysPerCandidate    );
  ros::param::get(paramNameStart+"FOV_angle",            explore.angleFOV               );
  ros::param::get(paramNameStart+"bump_dist",            explore.bumpDist           );
  ros::param::get(paramNameStart+"bump_val_at_dist",     explore.bumpValAtDist          );
  ros::param::get(paramNameStart+"discount_radius",      explore.discountRadius         );
  ros::param::get(paramNameStart+"max_computation_time", explore.maxComputationTime     );
  ros::param::get(paramNameStart+"max_angular_vel",      explore.maxRadPerSec           );

  // Robot-Nonspecific Exploration Visualization
  ros::param::get(vizParamStart+"viz_cost_map",           explore.visualizeCostMap     );
  ros::param::get(vizParamStart+"viz_cand_entropies",     explore.vizCandEntropies     );
  ros::param::get(vizParamStart+"arrow_scale",            explore.arrowScale           );
  ros::param::get(vizParamStart+"non_optimal_opacity",    explore.nonOptimalOpacity    );
  ros::param::get(vizParamStart+"viz_cand_entr_dists",    explore.vizCandEntropiesDists);
  ros::param::get(vizParamStart+"entropy_marker_topic",   explore.entropyMarkerTopic   );
  ros::param::get(vizParamStart+"dist_info_marker_topic", explore.distInfoMarkerTopic  );

  // Robot-Specific Topics with General Parameters: Update with Namespaced Topics
  ros::param::get(paramNameStart+"camera_path_topic",   robotInformation.back().cameraPathTopic  );
  ros::param::get(paramNameStart+"position_path_topic", robotInformation.back().positionPathTopic);
  ros::param::get(paramNameStart+"traj_info_topic",     robotInformation.back().trajInfoTopic    );
  ros::param::get(paramNameStart+"des_pose_topic",      robotInformation.back().desiredPoseTopic );
  ros::param::get(paramNameStart+"des_cam_topic",       robotInformation.back().desiredCamTopic  );
  ros::param::get(paramNameStart+"des_twist_topic",     robotInformation.back().desiredTwistTopic);
  ros::param::get(paramNameStart+"des_accel_topic",     robotInformation.back().desiredAccelTopic);
  ros::param::get(vizParamStart +"djk_topic",           robotInformation.back().djkTopic         );
  ros::param::get(vizParamStart +"cost_map_topic",      robotInformation.back().costMapTopic     );
  for(int i(0); i < numRobots; i++){
    nsParamStart = robotInformation[i].ns+"/";
    robotInformation[i].cameraPathTopic     = nsParamStart+robotInformation.back().cameraPathTopic  ;
    robotInformation[i].positionPathTopic   = nsParamStart+robotInformation.back().positionPathTopic;
    robotInformation[i].trajInfoTopic       = nsParamStart+robotInformation.back().trajInfoTopic    ;
    robotInformation[i].desiredPoseTopic    = nsParamStart+robotInformation.back().desiredPoseTopic ;
    robotInformation[i].desiredCamTopic     = nsParamStart+robotInformation.back().desiredCamTopic  ;
    robotInformation[i].desiredTwistTopic   = nsParamStart+robotInformation.back().desiredTwistTopic;
    robotInformation[i].desiredAccelTopic   = nsParamStart+robotInformation.back().desiredAccelTopic;
    robotInformation[i].djkTopic            = nsParamStart+robotInformation.back().djkTopic         ;
    robotInformation[i].costMapTopic        = nsParamStart+robotInformation.back().costMapTopic     ;
  }

  // Initialize Standard Unit-Vectors
  e1 << 1.0, 0.0, 0.0;
  e2 << 0.0, 1.0, 0.0;
  e3 << 0.0, 0.0, 1.0;

  return;
}

void Voxel::GetRobotParameters(string& objective, int& numSensorsTotal){

  // Get Robot Information
  ros::param::get("num_robots", numRobots);
  robotInformation = vector<robotInfo>(numRobots);

  // Count the Total Number of Sensors & Get Namespaces
  numSensorsTotal = 0;
  for(int iRobot(0); iRobot < numRobots; iRobot++){
    ros::param::get("robot_"+to_string(iRobot)+"/ns"         ,
                    robotInformation[iRobot].ns          );
    ros::param::get("robot_"+to_string(iRobot)+"/num_sensors",
                    robotInformation[iRobot].numSensors  );
    ros::param::get("robot_"+to_string(iRobot)+"/coll_rad"   ,
                    robotInformation[iRobot].collisionRad);
    ros::param::get("robot_"+to_string(iRobot)+"/frame"      ,
                    robotInformation[iRobot].robotFrame  );
    ros::param::get("robot_"+to_string(iRobot)+"/color_r"    ,
                    robotInformation[iRobot].color.r     );
    ros::param::get("robot_"+to_string(iRobot)+"/color_g"    ,
                    robotInformation[iRobot].color.g     );
    ros::param::get("robot_"+to_string(iRobot)+"/color_b"    ,
                    robotInformation[iRobot].color.b     );

    // Robot Frame is Namespaced
    robotInformation[iRobot].robotFrame
        = robotInformation[iRobot].ns+"/"+robotInformation[iRobot].robotFrame;

    if(objective == "exploration"){
      int exploreFrameNum;
      ros::param::get("robot_"+to_string(iRobot)+"/sensor_explore"        ,
                      exploreFrameNum                      );
      ros::param::get("sensor_"+to_string(exploreFrameNum)+"/naming/frame_explore",
                      robotInformation[iRobot].exploreFrame);
      robotInformation[iRobot].exploreFrame
          = robotInformation[iRobot].ns+"/"+robotInformation[iRobot].exploreFrame;

      // For Exploration, Only 1 Sensor per Robot
      robotInformation[iRobot].numSensors = 1;
      numSensorsTotal++;
    }
    else{
      numSensorsTotal += robotInformation[iRobot].numSensors;
    }
  }

  return;
}

void Voxel::mapInfo::GetSensorParameters(
    string& objective, int& numRobotsTotal, vector<robotInfo>& robotInformation){

  // Allocated Memory
  sensorInformation = vector<mapInfo::sensorInfo>(numSensors);

  int sensorNumber(0), sensorIndex;
  string sensorIdentifier, paramNameStart, sensorParamStart;
  for(int iRobot(0); iRobot < numRobotsTotal; iRobot++){
    for(int iSensor(0); iSensor < robotInformation[iRobot].numSensors; iSensor++){

      // Sensor-Robot Association
      sensorInformation[sensorNumber].robotIndex = iRobot;

      // Determine Sensor for this Robot
      if(     objective == "mapping")
        sensorIdentifier = to_string(iSensor);
      else if(objective == "exploration")
        sensorIdentifier = "explore";
      ros::param::get("robot_"+to_string(iRobot)+"/sensor_"+sensorIdentifier, sensorIndex);
      paramNameStart = "sensor_"+to_string(sensorIndex)+"/";

      // Naming
      sensorParamStart = paramNameStart+"naming/";
      ros::param::get(sensorParamStart+"name",
                      sensorInformation[sensorNumber].name );
      sensorInformation[sensorNumber].name
          = "Robot "+to_string(iRobot)+" "+sensorInformation[sensorNumber].name  ;
      ros::param::get(sensorParamStart+"topic",
                      sensorInformation[sensorNumber].topic);
      sensorInformation[sensorNumber].topic
          = robotInformation[iRobot].ns+"/"+sensorInformation[sensorNumber].topic;
      ros::param::get(sensorParamStart+"frame",
                      sensorInformation[sensorNumber].frame);
      sensorInformation[sensorNumber].frame
          = robotInformation[iRobot].ns+"/"+sensorInformation[sensorNumber].frame;

      // Properties
      sensorParamStart = paramNameStart+"properties/";
      ros::param::get(sensorParamStart+"min_range",     sensorInformation[sensorNumber].minRange   );
      ros::param::get(sensorParamStart+"max_range",     sensorInformation[sensorNumber].maxRange   );
      ros::param::get(sensorParamStart+"sigma",         sensorInformation[sensorNumber].sigma      );
      ros::param::get(sensorParamStart+"prob_hit",      sensorInformation[sensorNumber].probHit    );
      ros::param::get(sensorParamStart+"prob_rand",     sensorInformation[sensorNumber].probRand   );
      ros::param::get(sensorParamStart+"prob_max_free", sensorInformation[sensorNumber].probMaxFree);
      ros::param::get(sensorParamStart+"prob_max_occ",  sensorInformation[sensorNumber].probMaxOcc );
      ros::param::get(sensorParamStart+"prob_det_sat",  sensorInformation[sensorNumber].probDetSat );
      ros::param::get(sensorParamStart+"consider_max",  sensorInformation[sensorNumber].considerMax);
      ros::param::get(sensorParamStart+"num_max_skip",  sensorInformation[sensorNumber].numMaxSkips);
      ros::param::get(sensorParamStart+"has_colors",    sensorInformation[sensorNumber].hasColors  );

      // Timers
      ros::param::get(paramNameStart+"print_timers", sensorInformation[sensorNumber].printTimers);

      sensorNumber++;
    }
  }

  return;
}

void Voxel::GetParameters(string& objective){

  // Robot Parameters
  GetRobotParameters(objective, map.numSensors);

  // Map Parameters (required for all nodes)
  map.GetMapParameters(objective);

  // Sensor Parameters
  map.GetSensorParameters(objective, numRobots, robotInformation);

  return;
}

void Voxel::mapInfo::BasicMapInfoInit(string& objective){

  // Map Information Initialization
  numCellsEachDimension.resize(dimension);
  numCellsTotal = 1;
  maxNumCellsAlongRay = 0;
  for(int i(0); i < dimension; i++){
    numCellsEachDimension[i] = floor((maximumLocations[i]-minimumLocations[i])/alpha+0.5)+1;
    numCellsTotal*= numCellsEachDimension[i];
    maxNumCellsAlongRay += numCellsEachDimension[i];
  }
  stride.resize(dimension);
  stride[0] = 1;
  for(int i = 1; i < dimension; i++)
    stride[i] = stride[i-1]*numCellsEachDimension[i-1];

  // Print Basic Map Information
  cout << "For " << objective << ", total " << numCellsTotal << " grid cells\nwith edge length " << alpha << "\nare dimensioned ";
  for(int i(0); i < dimension; i++){
    cout << numCellsEachDimension[i];
    if(i != dimension-1)
      cout << "x";
    else
      cout << "." << endl;
  }

  // Occupancy
  int numElementsPerCell = 1;
  string addedLabelInfo = "";
  MapInfoToMultiArray(
      numElementsPerCell, addedLabelInfo, occData, initP);
  totalEntropy = numCellsTotal*EntropySingleCell(initP);

  return;
}

void Voxel::mapInfo::SensorUpdateInit(){

  // Sensor Information & Memory Allocation
  for(int i(0); i < numSensors; i++){

    // Preallocated Memory for Unit-Vectors Toward Other Robots
    sensorInformation[i].uvecToOtherRobots.resize(sensorInformation.back().robotIndex, vector<double>(dimension));

    // Preallocated Memory for Inverse Sensor Model
    sensorInformation[i].mapEstRay.   resize(maxNumCellsAlongRay);
    sensorInformation[i].sensorRanges.resize(maxNumCellsAlongRay);
    sensorInformation[i].cellIndices. resize(maxNumCellsAlongRay);

    // Preallocated Memory for Ray Casting
    sensorInformation[i].rayCastCellIndices.resize(dimension, vector<int>(maxNumCellsAlongRay));
    sensorInformation[i].rayCastCellDepths .resize(dimension, vector<double>(maxNumCellsAlongRay));

    // Maximum Reading Skips
    sensorInformation[i].numCurSkips = 0;

    // Timers
    sensorInformation[i].numSamplesTaken = 0;

    // Normalizing
    double probDenom(sensorInformation[i].probHit+sensorInformation[i].probRand);
    sensorInformation[i].probHit  /= probDenom;
    sensorInformation[i].probRand /= probDenom;
  }

  // Make Uncolored Occupied Cells Grey (e.g. PointCloud2 generated from Laser Scan)
  defaultColor.resize(dimension);
  for(int i(0); i < dimension; i++)
    defaultColor[i] = 128;

  return;
}

template<typename mapType, typename mapDataType>
void Voxel::mapInfo::MapInfoToMultiArray(int& numElementsPerCell, string& addedLabelInfo, mapType& mapData, mapDataType& initVal){

  // Initialize Map Data & Layout
  mapData.data.resize(numElementsPerCell*numCellsTotal);
  fill(mapData.data.begin(), mapData.data.end(), initVal);
  mapData.layout.data_offset = 0;
  mapData.layout.dim.resize(dimension);
  for(int i(0); i < dimension; i++){
    mapData.layout.dim[i].label  = dimNames[i]+addedLabelInfo;
    mapData.layout.dim[i].size   = numCellsEachDimension[i];
    mapData.layout.dim[i].stride = stride[i];
  }
  return;
}

// TOMORROW! Output: Eigen::Matrix3d& RotMatrixCameraRobot, Eigen::Vector3d& vecRobot2Camera_RobotFrame
void Voxel::GetRobotCameraTFs(string& sensorFrame, string& robotFrame, int& robotNum){

  // TF from Robot to Camera
  explore.transformR2C = LoopUntilTransformAcquired(explore.tfListener, robotFrame, sensorFrame);

  // Camera Position Offset from Robot (robot frame)
  robotInformation[robotNum].vecRobot2Camera_RobotFrame <<
      explore.transformR2C.getOrigin().x(),
      explore.transformR2C.getOrigin().y(),
      explore.transformR2C.getOrigin().z();

  // Attitude Offset (quaternion)
  geometry_msgs::Quaternion quatR2C;
  quatR2C.x = explore.transformR2C.getRotation().x();
  quatR2C.y = explore.transformR2C.getRotation().y();
  quatR2C.z = explore.transformR2C.getRotation().z();
  quatR2C.w = explore.transformR2C.getRotation().w();

  // Rotation Matrices
  QuatToRotMatrix(quatR2C, explore.RotMatrixRobotCamera) ;
  robotInformation[robotNum].RotMatrixCameraRobot = explore.RotMatrixRobotCamera.transpose();

  // Camera Position Offset from Robot (camera frame)
  robotInformation[robotNum].vecRobot2Camera_CameraFrame = robotInformation[robotNum].RotMatrixCameraRobot*robotInformation[robotNum].vecRobot2Camera_RobotFrame;

  // Desired Sensor Alignment
  if(robotInformation[robotNum].RotMatrixCameraRobot(2,2) >= 0.0)
    e3Signed << 0.0, 0.0,  1.0;
  else
    e3Signed << 0.0, 0.0, -1.0;

  return;
}

void Voxel::ExplorationInit(){
  explore.sensorNumber = 0;
  explore.tStartTraj   = 0.0;
  explore.initTraj = true;
  explore.freeCells.resize(collisionMap.numCellsTotal);
  explore.numVisitedCandidates = 0;
  explore.numLowHCandidates = 0;
  explore.numReachCandidates = 0;

  // Ensure candidate selection occurs on first loop pass
  explore.tNow = 0.0;
  explore.tStartTraj = 1.0;

  // Use TFs for Offset Robot/Camera Offset
  for(int i(0); i < numRobots; i++){
    GetRobotCameraTFs(robotInformation[i].exploreFrame,
                      robotInformation[i].robotFrame, i);
  }

  // Candidate Indices
  int numCellsBtwnCandidates(ceil(explore.candidateSeparation/collisionMap.alpha));
  cout << "Candidates are separated " << explore.candidateSeparation
       << "m apart with\ncells with edge length " << collisionMap.alpha
      << "m\nsuch that pose candidates are "
       << numCellsBtwnCandidates
       << " cells apart." << endl;
  vector<int> numCandidatesEachDirection(collisionMap.dimension);
  vector<double> firstCandidateLocation(collisionMap.dimension), evalCandidateLocation(collisionMap.dimension);
  explore.numCandidatesTotal = 1;
  for(int i(0); i < collisionMap.dimension; i++){
    numCandidatesEachDirection[i] = floor(1.0*(collisionMap.numCellsEachDimension[i]-2)/(numCellsBtwnCandidates));
    if(numCandidatesEachDirection[i] > 0)
      ROS_INFO ("Exploration: number of candidates in dimension %i is %i", i, numCandidatesEachDirection[i]);
    else
      ROS_ERROR("Exploration: number of candidates in dimension %i is %i", i, numCandidatesEachDirection[i]);
    firstCandidateLocation[i] = collisionMap.minimumLocations[i]+collisionMap.alpha;
    explore.numCandidatesTotal *= numCandidatesEachDirection[i];
  }
  explore.candidateLocations.resize(explore.numCandidatesTotal, vector<double>(collisionMap.dimension));
  explore.optimalUVecs.resize(      explore.numCandidatesTotal, vector<double>(collisionMap.dimension));
  explore.candidateIndices.resize(explore.numCandidatesTotal);
  explore.candidateDone.resize(explore.numCandidatesTotal, false);
  explore.candidateExpectedInfoGain.resize(explore.numCandidatesTotal, 0.0);

  int indCandidate(0);
  for(int iX = 0; iX < numCandidatesEachDirection[0]; iX++){
    evalCandidateLocation[0] = firstCandidateLocation[0]+iX*numCellsBtwnCandidates*collisionMap.alpha;
    for(int iY = 0; iY < numCandidatesEachDirection[1]; iY++){
      evalCandidateLocation[1] = firstCandidateLocation[1]+iY*numCellsBtwnCandidates*collisionMap.alpha;
      explore.candidateLocations[indCandidate] = evalCandidateLocation;
      explore.candidateIndices[indCandidate] = collisionMap.IndFromMapLocation(evalCandidateLocation);
      indCandidate++;
    }
  }

  // Sensor Unit Vectors
  explore.rayUVecs.resize(explore.numRaysPerCandidate, vector<double>(collisionMap.dimension));
  for(int i(0); i < explore.numRaysPerCandidate; i++){
    double angle(2.0*M_PI*i/explore.numRaysPerCandidate);
    explore.rayUVecs[i] = {cos(angle), sin(angle)};
  }
  explore.numRaysEachSide = floor(0.5*explore.numRaysPerCandidate*explore.angleFOV/(2*M_PI));
  explore.infoGainEachRay.resize(explore.numRaysPerCandidate);
  explore.infoGainEachDirection.resize(explore.numRaysPerCandidate);

  // Path
  explore.bumpSigSqrd = -0.5*pow(explore.bumpDist, 2)/log(explore.bumpValAtDist);
  explore.costMapInitValue = (double)(collisionMap.numCellsTotal*collisionMap.alpha);
  explore.visited.resize(collisionMap.numCellsTotal, false);
  explore.costHorizVert = collisionMap.alpha;
  explore.costDiagonal  = sqrt(2.0)*collisionMap.alpha;
  explore.pathwayBuff.resize(collisionMap.dimension, vector<int>(collisionMap.numCellsTotal));
  explore.djkDistBuff.resize(collisionMap.numCellsTotal);
  explore.fixedPoseStampedParams.pose.position.z = explore.height;
  explore.fixedPoseStampedParams.pose.orientation.x = 0.0;
  explore.fixedPoseStampedParams.pose.orientation.y = 0.0;
  explore.fixedPoseStampedParams.pose.orientation.z = 0.0;
  explore.fixedPoseStampedParams.pose.orientation.w = 1.0;
  explore.fixedPoseStampedParams.header.frame_id = collisionMap.frame;

  // Robot-Specific
  for(int iRobot(0); iRobot < numRobots; iRobot++){
    robotInformation[iRobot].bestCandInd = -1;
    robotInformation[iRobot].sensorLoc    .resize(3);
    robotInformation[iRobot].nextSensorLoc.resize(3);
    robotInformation[iRobot].costMap.resize(collisionMap.numCellsTotal, explore.costMapInitValue);
    robotInformation[iRobot].candidateInfoGainWithBump.resize(explore.numCandidatesTotal, 0.0);
  }
  entropyBumpsAllRobots.resize(numRobots, 0.0);

  // Visualization
  if(explore.vizCandEntropies
  || explore.vizCandEntropiesDists){

    // Future Pose Candidates (Rviz marker arrays)
    explore.arrow.scale.x = explore.arrowScale/2;
    explore.arrow.scale.y = explore.arrowScale;
    explore.arrow.scale.z = explore.arrowScale;
    explore.arrow.header.frame_id = collisionMap.frame;
    explore.arrow.ns = "basic_shapes";
    explore.arrow.action = visualization_msgs::Marker::ADD;
    explore.arrow.type   = visualization_msgs::Marker::ARROW;
    explore.arrow.color.g = 0.0;
    explore.arrow.color.a = 1.0;
    explore.arrow.lifetime = ros::Duration();
    explore.arrow.points.resize(2);
    explore.arrowLength = 0.5*numCellsBtwnCandidates*collisionMap.alpha;
    explore.arrowLoc.z = explore.height;
    explore.entropyArrowArray.markers.resize(explore.numCandidatesTotal, explore.arrow);
    explore.entropyDistArrowArray = explore.entropyArrowArray;

    // Cost Map
    collisionMap.OccupancyGridMsgInit(explore.costMapViz, explore.height);

  }

  return;
}

int Voxel::explorationInfo::MakeCyclicIndexint(int& signedInt, int& size){

  if(signedInt < 0)
    return size+signedInt;
  else if(signedInt >= size)
    return signedInt-size;
  else
    return signedInt;

}

vector<double> Voxel::explorationInfo::SumScalarQuantityOfRays(vector<double>& input, int& raysEachSide){
  int size(input.size());
  vector<double> output(size, 0.0);
  int rayInd;

  // Get First Index
  for(rayInd = -raysEachSide; rayInd <= raysEachSide; rayInd++){
    output[0] += input[MakeCyclicIndexint(rayInd, size)];
  }

  // Modify from Last Index
  for(int i = 1; i < size; i++){

    // Add a ray in the positive direction
    rayInd = i+raysEachSide;
    output[i] = output[i-1]+input[MakeCyclicIndexint(rayInd, size)];

    // Remove a ray in the negative direction
    rayInd = i-raysEachSide-1;
    output[i] -= input[MakeCyclicIndexint(rayInd, size)];

  }

  return output;
}

void Voxel::Initializations(string& objective){

  // Constant Parameters
  map.sqrt2pi = sqrt(2*M_PI);
  e1 << 1.0, 0.0, 0.0;
  e2 << 0.0, 1.0, 0.0;
  e3 << 0.0, 0.0, 1.0;

  // Basic Map Information
  map.BasicMapInfoInit(objective);

  // Sensor Update
  map.SensorUpdateInit();

  // Map Color
  if(objective == "mapping"){
    int numElementsPerCell = map.dimension;
    string addedLabelInfo = " RGB ("+to_string(map.dimension)+" elements each)";
    int initColor(-1);// uncolored identifier
    map.MapInfoToMultiArray(
          numElementsPerCell, addedLabelInfo, map.rgbColors, initColor);
  }

  for(int i(0); i < map.numSensors; i++){

    // Avoiding Measuring Other Robots as the Map
    map.sensorInformation[i].otherRobotFrames       .resize(numRobots-1);
    map.sensorInformation[i].otherRobotRadii        .resize(numRobots-1);
    map.sensorInformation[i].maxDotProdToOtherRobots.resize(numRobots-1);

    int otherRobotInd(0);
    for(int iRobot(0); iRobot < numRobots; iRobot++){
      if(iRobot != map.sensorInformation[i].robotIndex){
        map.sensorInformation[i].otherRobotFrames[otherRobotInd]
            = robotInformation[iRobot].robotFrame  ;
        map.sensorInformation[i].otherRobotRadii [otherRobotInd]
            = robotInformation[iRobot].collisionRad;
        otherRobotInd++;
      }
    }
    map.numRobots = numRobots;

    // Remove?  // Origin at Sensors
    map.sensorInformation[i].cameraLocLocalFrame.point.x = 0.0;
    map.sensorInformation[i].cameraLocLocalFrame.point.y = 0.0;
    map.sensorInformation[i].cameraLocLocalFrame.point.z = 0.0;
  }

  return;
}

void Voxel::FindFreeCells(int& numFreeCellNeighbors, mapInfo& mapInput){

  // Intialize Cells to Free/Unvisted
  fill(explore.freeCells.begin(), explore.freeCells.end(), true );
  fill(explore.visited.  begin(), explore.visited.  end(), false);

  // Check Cells & their Neighbors
  int i, indTest, iX, iY;
  bool testCell;
  for(i = 0; i < mapInput.numCellsTotal; i++){
    testCell = true;
    for(iX = -numFreeCellNeighbors; iX <= numFreeCellNeighbors && testCell; iX++){
      for(iY = -numFreeCellNeighbors; iY <= numFreeCellNeighbors && testCell; iY++){

        // Index in Question
        indTest = i+iX*mapInput.stride[0]+iY*mapInput.stride[1];

        // Mark as occupied if: neighbor is off map or current cell/neighbor is occupied
        if(indTest < 0 || indTest >= mapInput.numCellsTotal)// avoid possible segmentation fault
          testCell = false;
        else if(mapInput.occData.data[indTest] > explore.acceptableCollisionProb)
          testCell = false;
        if(!testCell){
          explore.freeCells[i] = false;
          explore.visited[i] = true;
          break;
        }
      }
    }
  }

  return;
}

void Voxel::mapInfo::MapLocToDimensionalIndex(vector<double>& mapLoc, vector<int>& dimInd){

  for(int i(0); i < dimension; i++)
    dimInd[i] = floor((mapLoc[i]-minimumLocations[i])/alpha+0.5);

  return;
}

void Voxel::mapInfo::UpdateCellProbabilitiesAndColors(
    bool& ISMSuccess, vector<int>& color, int& numCellsAlongRay, int& sensorNumber){
  bool colorFirstCell = true;
  int mapInd, colorInd;
  if(ISMSuccess){
    for(int i(0); i < numCellsAlongRay; i++){
      mapInd = sensorInformation[sensorNumber].cellIndices[i];
      if(mapInd != -1){// (check)
        occData.data[mapInd] = sensorInformation[sensorNumber].mapEstRay[i];
        colorInd = 3*mapInd;// 3: R,G,B
        if(sensorInformation[sensorNumber].mapEstRay[i] >= occThresh){
          if(colorFirstCell || rgbColors.data[colorInd] == -1){// first occupied cell or occupied & uncolored
            if(sensorInformation[sensorNumber].hasColors){
              for(int k = 0; k < 3; k++)
                rgbColors.data[colorInd+k] = color[k];
              colorFirstCell = false;
            }
            else{// no color provided, but cell is likely occupied
              if(rgbColors.data[colorInd] == -1){// only apply defaultColor if previously unassigned
                for(int k = 0; k < 3; k++)
                  rgbColors.data[colorInd+k] = defaultColor[k];
              }
              return;
            }
          }
        }
      }
      else// cells outside map limit, & any remaining cells will be, too
        return;
    }
  }
  return;
}

vector<double> Voxel::mapInfo::SensorLocRelToOtherRobots(int& sensorNumber){

  // Initializations
  tf::StampedTransform stampedTF;

  // Sensor Location
  stampedTF = LoopUntilTransformAcquired(
        sensorInformation[sensorNumber].tfListener,
        frame, sensorInformation[sensorNumber].frame);
  vector<double> sensorLoc = {
    stampedTF.getOrigin().x(),
    stampedTF.getOrigin().y(),
    stampedTF.getOrigin().z()};

  // Determine Directions Toward Other Robots
  int otherRobotInd(0);
  for(int iRobot(0); iRobot < numRobots; iRobot++){
    if(iRobot != sensorInformation[sensorNumber].robotIndex){

      // Other Robot Location
      stampedTF
          = LoopUntilTransformAcquired(
            sensorInformation[sensorNumber].tfListener,
            frame, sensorInformation[sensorNumber].otherRobotFrames[otherRobotInd]);
      vector<double> otherRobotLoc ={
         stampedTF.getOrigin().x(),
         stampedTF.getOrigin().y(),
         stampedTF.getOrigin().z()};
      otherRobotLoc.resize(dimension);

      // Unit Vector from Sensor to Other Robot
      double otherRobotDistAway;
      sensorInformation[sensorNumber].uvecToOtherRobots[otherRobotInd]
          = GetUVec(sensorLoc, otherRobotLoc, otherRobotDistAway);

      // Max Dot Product Between This & Measurement Ray Direction
      if(otherRobotDistAway > sensorInformation[sensorNumber].otherRobotRadii[otherRobotInd])
        sensorInformation[sensorNumber].maxDotProdToOtherRobots[otherRobotInd] = 1.0
            -pow(sensorInformation[sensorNumber].otherRobotRadii[otherRobotInd], 2)
            /pow(otherRobotDistAway, 2);
      else// inside other robot's collision zone: neglect all rays
        sensorInformation[sensorNumber].maxDotProdToOtherRobots[otherRobotInd] = -1.0;
      otherRobotInd++;
    }
  }


  return sensorLoc;
}

bool Voxel::mapInfo::MeasRayOK(vector<double>& uVec, double& depth, int& sensorNumber){

  if(isnan(depth))
      return false;
  else if(depth >= sensorInformation[sensorNumber].maxRange){
    if(sensorInformation[sensorNumber].numCurSkips < sensorInformation[sensorNumber].numMaxSkips){
      sensorInformation[sensorNumber].numCurSkips++;// skipping this maximum reading
      return false;
    }
    else
      sensorInformation[sensorNumber].numCurSkips = 0;// acceptable maximum reading, continue...
  }
  else{
    for(int otherRobotInd(0); otherRobotInd < numRobots-1; otherRobotInd++){
      if(DotProd(uVec, sensorInformation[sensorNumber].uvecToOtherRobots[otherRobotInd])
         > sensorInformation[sensorNumber].maxDotProdToOtherRobots[otherRobotInd])
        return false;
    }
  }

  return true;
}


void Voxel::mapInfo::FindCellsComposingBlock(vector<double>& testLoc,
                                             vector<double>& minCorner, vector<int>& numCellsThisDim,
                                             vector<int>& indsToUpdate, int& blockIndex, int iDim){

  // Reset Index
  if(iDim == 0)
    blockIndex = 0;

  // Recursive Call
  for(int i(0); i < numCellsThisDim[iDim]; i++){
    testLoc[iDim] = minCorner[iDim]+i*alpha;
    if(iDim == dimension-1){
      indsToUpdate[blockIndex] = IndFromMapLocation(testLoc);
      blockIndex++;
    }
    else
      FindCellsComposingBlock(testLoc, minCorner, numCellsThisDim, indsToUpdate, blockIndex, iDim+1);
  }

  return;
}

void Voxel::mapInfo::UpdateMap(
    int& sensorNumber, int& numRays, vector< vector<double> >& meas, vector< vector<int> >& color){

  // Initializations
  double depth, probRayReachesFirstCell, entropyRayStart(0.0), entropyRayEnd(0.0), entropyChange;
  vector<double> minCorner = maximumLocations, maxCorner = minimumLocations, testLoc(dimension);
  bool cellsUpdated(false);
  double timeRayCast, durRayCast(0.0);
  int ind;
  double reachesCellMinThresh = 0.1;// TODO: object & param

  // Start Time
  if(sensorInformation[sensorNumber].printTimers){sensorInformation[sensorNumber].tScanUpdateStart = ros::Time::now().toSec();}

  // Sensor Location & Directions Toward Other Robots
  vector<double> sensorLoc = SensorLocRelToOtherRobots(sensorNumber);

  // Update Map from Scan Ray-by-Ray
  for(int i(0); i < numRays; i++){

    // Measurement Direction
    vector<double> uVec = GetUVec(sensorLoc, meas[i], depth);

    // Check if Measurement is NaN, Maximum, and/or Toward Another Robot
    if(MeasRayOK(uVec, depth, sensorNumber)){

      // Ray Casting
      timeRayCast = ros::Time::now().toSec();
      int numCellsAlongRay = RayCasting(
            sensorLoc, uVec, sensorNumber, probRayReachesFirstCell);
      durRayCast += ros::Time::now().toSec()-timeRayCast;

      // Sensors Only Provide Accurate Measurements Beyond Some Minimum Threshold
      if(probRayReachesFirstCell >= reachesCellMinThresh){
        cellsUpdated = true;

        // Cell Probabilities & Entropies Along Casted Ray
        for(int i(0); i < numCellsAlongRay; i++){
          sensorInformation[sensorNumber].mapEstRay[i] = occData.data[sensorInformation[sensorNumber].cellIndices[i]];
          entropyRayStart += EntropySingleCell(sensorInformation[sensorNumber].mapEstRay[i]);
        }

        // Inverse Sensor Model
        bool ISMSuccess = RayInverseSensorModel(
              depth, numCellsAlongRay, sensorNumber, probRayReachesFirstCell);

        // Update Map Probabilities & Colors
        UpdateCellProbabilitiesAndColors(
              ISMSuccess, color[i], numCellsAlongRay, sensorNumber);

        // Final Ray Entropies
        for(int i(0); i < numCellsAlongRay; i++)
          entropyRayEnd += EntropySingleCell(sensorInformation[sensorNumber].mapEstRay[i]);

        // Update Scan Min & Max Box
        ind = 0;                  ChangedCellLimits(ind, minCorner, maxCorner, sensorNumber);
        ind = numCellsAlongRay-1; ChangedCellLimits(ind, minCorner, maxCorner, sensorNumber);
        testLoc = MapLocationFromInd(sensorInformation[sensorNumber].cellIndices[ind]);
      }
    }
  }

  // Only Publish & Analyze if Measurements Updated the Map TODO: clean/caption below
  if(cellsUpdated){

    // Entropy Update
    entropyChange = entropyRayEnd-entropyRayStart;
    totalEntropy += entropyChange;

    // Only Publish Map Changes
    if(trackDiff){

      // Generate Low-Memory Message
      ComposeChangedMapCellMsg(minCorner, maxCorner);

      // Publish the Message
      mapChangesPub.publish(changedCells);
    }

    // Publish the Entire Map
    else{

      // Publish the Entire Map
      mapProbsPub.publish(occData  );
      mapRGBPub  .publish(rgbColors);
    }

    // End Time & Analysis
    if(sensorInformation[sensorNumber].printTimers){
      sensorInformation[sensorNumber].tScanUpdateEnd = ros::Time::now().toSec();
      if(sensorInformation[sensorNumber].tScanUpdateEnd > sensorInformation[sensorNumber].tScanUpdateStart){// neglect loop-backs from bag files
        sensorInformation[sensorNumber].timeCurrent = sensorInformation[sensorNumber].tScanUpdateEnd-sensorInformation[sensorNumber].tScanUpdateStart;
        sensorInformation[sensorNumber].numSamplesTaken++;
        sensorInformation[sensorNumber].timeAvg *= ((double)(sensorInformation[sensorNumber].numSamplesTaken-1)/sensorInformation[sensorNumber].numSamplesTaken);
        sensorInformation[sensorNumber].timeAvg += sensorInformation[sensorNumber].timeCurrent/sensorInformation[sensorNumber].numSamplesTaken;
        cout << "\n" << sensorInformation[sensorNumber].name << " sensor (number " << sensorNumber << "):"
             << "\nRay Count: " << numRays
             << "\nTimes:"
             << "\n  RayCast: " << durRayCast << " (" << durRayCast/sensorInformation[sensorNumber].timeCurrent*100 << "%)"
             << "\n  Total:   " << sensorInformation[sensorNumber].timeCurrent
             << "\n  Average: " << sensorInformation[sensorNumber].timeAvg
             << "\nEntropy: "
             << "\n  Change:  " << entropyChange
             << "\n  Total:   " << totalEntropy
             << endl;
      }
    }
  }

  return;
}

void Voxel::mapInfo::ComposeChangedMapCellMsg(
    vector<double>& minCorner, vector<double>& maxCorner, bool hasColors){

  // Initializations
  int numCellsToUpdate = 1;
  vector<int> numCellsThisDim(dimension);
  for(int iDim(0); iDim < dimension; iDim++){
    numCellsThisDim[iDim] = floor((maxCorner[iDim]-minCorner[iDim])/alpha)+1;
    numCellsToUpdate *= numCellsThisDim[iDim];
  }
  vector<int> indsToUpdate(numCellsToUpdate);
  int blockIndex(0);
  vector<double> testLoc(dimension);

  // Cells with Potential Updates
  FindCellsComposingBlock(testLoc, minCorner, numCellsThisDim, indsToUpdate, blockIndex);

  // Compose Custom Message of (Possibly) Changing Cells
  changedCells.inds.resize(numCellsToUpdate);
  changedCells.probs.resize(numCellsToUpdate);
  if(hasColors){changedCells.colors.resize(3*numCellsToUpdate);}
  int iMapColor, iMsgColor, iRGB;
  for(int i(0); i < numCellsToUpdate; i++){

    // Probabilities
    changedCells.inds [i] = indsToUpdate[i]              ;// cell indices
    changedCells.probs[i] = occData.data[indsToUpdate[i]];// cell probabilities

    // Colors
    if(hasColors){
      iMapColor = 3*indsToUpdate[i];
      iMsgColor = 3*i;
      for(iRGB = 0; iRGB < 3; iRGB++)// 3 RGB Values
        changedCells.colors[iMsgColor+iRGB] = rgbColors.data[iMapColor+iRGB];
    }
  }

  // Header
  changedCells.header.stamp = ros::Time::now();
  changedCells.header.frame_id = frame;

  return;
}

void Voxel::mapInfo::ChangedCellLimits(int& ind, vector<double>& minCorner, vector<double>& maxCorner, int& sensorNumber){

  // Location of First/Last Cell Along Ray
  vector<double> testLoc = MapLocationFromInd(sensorInformation[sensorNumber].cellIndices[ind]);

  for(int iDim(0); iDim < dimension; iDim++){
    if(testLoc[iDim] < minCorner[iDim]){
      if(testLoc[iDim] < minimumLocations[iDim])
        testLoc[iDim] = minimumLocations[iDim];
      else
        minCorner[iDim] = testLoc[iDim];
    }
    if(testLoc[iDim] > maxCorner[iDim]){
      if(testLoc[iDim] > maximumLocations[iDim])
        testLoc[iDim] = maximumLocations[iDim];
      else
        maxCorner[iDim] = testLoc[iDim];
    }
  }

  return;
}

double Voxel::mapInfo::EnsureRayCastingInsideMapLimits(
    int& sensorNumber, vector<double>& origin, vector<double>& rayMaxLoc, vector<double>& uVec){

  // Maximum ray location without considering map limits
  for(int i(0); i < dimension; i++)
    rayMaxLoc[i] = origin[i]+uVec[i]*sensorInformation[sensorNumber].maxRange;

  // Graphic showing where ray should be considered
  /*
   *             (max case)
   *   Outside     X
   *   Map        X
   * ____________X____
   *            /     |
   *   Inside  /      |  Outside
   *   Map    /       |  Map
   *         /        |
   *     (Robot)      |
   *                  |
   *
   *  /: Ray considered
   *  X: Ray neglected
   *
   */

  // Initialize variables for obtaining the ray distance ratio of maximum case
  double correctionRatio(1.0);
  double checkCorrectionRatio;
  double compOutsideMapLimits;

  // Determine the minimum ratio (0 < ratio <= 1)
  for(int i(0); i < dimension; i++){
    if(     rayMaxLoc[i] < minimumLocations[i])
      compOutsideMapLimits = minimumLocations[i];
    else if(rayMaxLoc[i] > maximumLocations[i])
      compOutsideMapLimits = maximumLocations[i];
    else// component inside map limits
      compOutsideMapLimits = rayMaxLoc[i];

    checkCorrectionRatio = (compOutsideMapLimits-origin[i])
                              /(rayMaxLoc[i]-origin[i]);
    if(checkCorrectionRatio < correctionRatio)
      correctionRatio = checkCorrectionRatio;
  }

  // Apply ratio so rayMaxLoc is inside map limits
  sensorInformation[sensorNumber].maxRangeMap = correctionRatio*sensorInformation[sensorNumber].maxRange;
  for(int i(0); i < dimension; i++)
    rayMaxLoc[i] = origin[i]+sensorInformation[sensorNumber].maxRangeMap*uVec[i];

  return correctionRatio*sensorInformation[sensorNumber].maxRange;
}

vector<double> Voxel::mapInfo::SnapLocationToMap(vector<double>& mapLoc){

  // Closest Index of mapLoc
  int ind = IndFromMapLocation(mapLoc);

  // True Index Location
  return MapLocationFromInd(ind);

}

vector<int> Voxel::mapInfo::FindCellEdges(
    int& sensorNumber, vector<double>& origin, vector<double>& rayMaxLoc, vector<double>& uVec, double& maxRangeRay, double& probRayReachesFirstCell){

  // Starting & Ending Locations Snapped to Map Grid
  vector<double> snappedStartingLoc    = SnapLocationToMap(origin   );
  vector<double> snappedEndingLocation = SnapLocationToMap(rayMaxLoc);

  // Number of Cells Along the Ray in Each Dimension
  vector<int> numCellsAlongRayXYZ(dimension);
  for(int i(0); i < dimension; i++)
    numCellsAlongRayXYZ[i] = floor(abs((snappedEndingLocation[i]-snappedStartingLoc[i])/alpha)+0.5);

  // Initializations
  probRayReachesFirstCell = 1.0;
  vector<int> rayIndices(dimension, 0);
  vector<double> edgeLoc(dimension), mapLoc(dimension), slopes(dimension), vecToCell(dimension);// temporary vectors

  // Cycle Through Ray Components in Each Map Dimension
  for(int iEdges = 0; iEdges < dimension; iEdges++){// evaluate {x,y,z}-edges

    // Get component in the iEdges direction & consider if nonzero
    if(abs(uVec[iEdges]) > 0.0){
      double increment = sgn(uVec[iEdges])*alpha;// increment size & direction
      edgeLoc[iEdges] = snappedStartingLoc[iEdges]+0.5*increment;// first cell edge

      // Slope in each dimension with respect to iEdges direction
      for(int i(0); i < dimension; i++){
        slopes[i]  = uVec[i]/uVec[iEdges];// 0 <= slopes[i] < inf, slopes[i] = 1 if i == iEdges
        edgeLoc[i] = slopes[i]*(edgeLoc[iEdges]-origin[iEdges])+origin[i];// when i == iEdges, edgeLoc[i] is unchanged
      }

      // Cycle through all possible intersections between the ray & cells
      for(int k = 0; k < numCellsAlongRayXYZ[iEdges]; k++){

        // Intersection & distance to this point
        for(int i(0); i < dimension; i++){
          edgeLoc[i]  += slopes[i]*increment ;
          vecToCell[i] = edgeLoc[i]-origin[i];
        }
        double distToCellEdge = NormVec(vecToCell);

        // Treat readings outside map limits as maximum readings
        if(distToCellEdge > maxRangeRay){
          distToCellEdge = sensorInformation[sensorNumber].maxRange;
          break;
        }
        else{// Cell is within map limits & will be considered

          // Map Location of Intersection
          mapLoc = edgeLoc;
          mapLoc[iEdges] += 0.5*increment;// move from edge to center for intersecting edge for correct map index

          // Cell Edge is Too Close to Robot
          if(distToCellEdge < sensorInformation[sensorNumber].minRange){
            probRayReachesFirstCell *= (1.0-occData.data[IndFromMapLocation(mapLoc)]);
          }

          // Cell Edge in FOV & Map
          else{

            // Save this cell information for further analysis
            sensorInformation[sensorNumber].rayCastCellIndices[iEdges][rayIndices[iEdges]] = IndFromMapLocation(mapLoc);
            sensorInformation[sensorNumber].rayCastCellDepths [iEdges][rayIndices[iEdges]] = distToCellEdge;

            if(sensorInformation[sensorNumber].rayCastCellIndices[iEdges][rayIndices[iEdges]] < 0)
              ROS_WARN_ONCE("TF still initializing...");

            // Increase index for saving & referencing the number of saved components in each dimension
            rayIndices[iEdges] += 1;

          }
        }
      }
    }
  }

  // Update (possibly decrease) the number of cells intersected by the ray in each dimension
  for(int i(0); i < dimension; i++)
    numCellsAlongRayXYZ[i] = rayIndices[i];

  return numCellsAlongRayXYZ;
}

int Voxel::mapInfo::OrderRayCastCellsByIncreasingDistance(
    int& sensorNumber, vector<int>& numCellsAlongRayXYZ){

  // Initializations
  int numCellsAlongRay(0);
  for(int i(0); i < dimension; i++)
    numCellsAlongRay += numCellsAlongRayXYZ[i];

  vector<int> iXYZ = {0,0,0};// index of saved variables

  for(int k = 0; k < numCellsAlongRay; k++){

    int closestEdge(dimension);// edge identifier, map.dimension is 1 more than possible
    double dist(sensorInformation[sensorNumber].maxRange);// distance to cell, initialized at maximum

    // Identify Closest Edge
    for(int i(0); i < dimension; i++){// cycle through each dimension
      if(iXYZ[i] < numCellsAlongRayXYZ[i]){
        if(dist > sensorInformation[sensorNumber].rayCastCellDepths[i][iXYZ[i]]){
          dist = sensorInformation[sensorNumber].rayCastCellDepths[i][iXYZ[i]];
          closestEdge = i;
        }
      }
    }

    // Insert Closest Edge as Next Element
    for(int i(0); i < dimension; i++){
      if(closestEdge == i){
        sensorInformation[sensorNumber].cellIndices [k] = sensorInformation[sensorNumber].rayCastCellIndices[i][iXYZ[i]];
        sensorInformation[sensorNumber].sensorRanges[k] = sensorInformation[sensorNumber].rayCastCellDepths [i][iXYZ[i]];
        iXYZ[i]++;
        break;
      }
    }
  }

  return numCellsAlongRay;
}

int Voxel::mapInfo::RayCasting(
    vector<double>& origin, vector<double>& uVec, int& sensorNumber, double& probRayReachesFirstCell){

  // Ensure Final Ray Location (rayMaxLoc) Inside Map Limits at Distance maxRangeRay
  vector<double> rayMaxLoc(dimension);
  double maxRangeRay = EnsureRayCastingInsideMapLimits(sensorNumber, origin, rayMaxLoc, uVec);

  // Determine Cell Edges
  vector<int> numCellsAlongRayXYZ = FindCellEdges(sensorNumber, origin, rayMaxLoc, uVec, maxRangeRay, probRayReachesFirstCell);

  // Order Cells Intersected by Ray by Increasing Distance
  return OrderRayCastCellsByIncreasingDistance(sensorNumber, numCellsAlongRayXYZ);
}

void Voxel::mapInfo::FindDetectionProbabilities(
    vector<double>& detectionProbability, int& numCellsAlongRay, int& sensorNumber, double probabilityRayReachesCell){
  bool satLimitReached(false);
  for(int k = 0; k < numCellsAlongRay; k++){
    detectionProbability[k] = probabilityRayReachesCell*sensorInformation[sensorNumber].mapEstRay[k];
    if(!satLimitReached){
      probabilityRayReachesCell *= (1.0-sensorInformation[sensorNumber].mapEstRay[k]);
      if(probabilityRayReachesCell < sensorInformation[sensorNumber].probDetSat){
        probabilityRayReachesCell = sensorInformation[sensorNumber].probDetSat;
        satLimitReached = true;
      }
    }
  }
  detectionProbability[numCellsAlongRay] = probabilityRayReachesCell;

  return;
}

double Voxel::mapInfo::GetUnnormalizedProbabilities(vector<double>& detectionProbability,
      vector<double>& unnormalizedOccupancyProbabilities, double& z, int& numCellsAlongRay, int& sensorNumber){

  double addedTermBuff, invNormalizer(0.0);
  for(int k = 0; k < numCellsAlongRay; k++){
    addedTermBuff = detectionProbability[k]
            *ForwardSensorModel(z, sensorInformation[sensorNumber].sensorRanges[k], sensorNumber);
    unnormalizedOccupancyProbabilities[k] = sensorInformation[sensorNumber].mapEstRay[k]*invNormalizer+addedTermBuff;
    invNormalizer += addedTermBuff;
  }
  if(sensorInformation[sensorNumber].considerMax){
    invNormalizer += detectionProbability[numCellsAlongRay]
            *ForwardSensorModel(z, sensorInformation[sensorNumber].maxRange, sensorNumber);
  }

  return invNormalizer;
}

bool Voxel::mapInfo::NormalizeProbabilities(
    double& invNormalizer, vector<double>& unnormalizedOccupancyProbabilities, vector<double>& normalizedOccupancyProbabilities, int& numCellsAlongRay, int& sensorNumber){

  if(invNormalizer > 0.0){
    for(int k = 0; k < numCellsAlongRay; k++){
      double occupancyProbabilityBeforeCheck = unnormalizedOccupancyProbabilities[k]
                                                /invNormalizer;
      // Truncate probabilities close to 0 & 1
      if(     occupancyProbabilityBeforeCheck < occProbMin)
        normalizedOccupancyProbabilities[k] = occProbMin;
      else if(occupancyProbabilityBeforeCheck > occProbMax)
        normalizedOccupancyProbabilities[k] = occProbMax;
      else
        normalizedOccupancyProbabilities[k] = occupancyProbabilityBeforeCheck;
    }
  }
  else{
    ROS_WARN("Inverse sensor model normalizer is below machine accuracy.");
    return false;
  }

  return true;
}

bool Voxel::mapInfo::CheckInsideFOV(double& z, int& sensorNumber){

  // Ray Check if Min Reading
  if(z < sensorInformation[sensorNumber].minRange)
    return false;

  // Ray Check if Beyond FOV
  if(z >= sensorInformation[sensorNumber].maxRangeMap || isnan(z) || isinf(z)){
    if(sensorInformation[sensorNumber].considerMax){
      z = sensorInformation[sensorNumber].maxRange;
      return true;
    }
    else
      return false;
  }
  return true;
}

bool Voxel::mapInfo::RayInverseSensorModel(// P(cell occupancies along ray | pose, measurement)
    double& z, int& numCellsAlongRay, int& sensorNumber, double& probRayReachesFirstCell){

  // Ray Check if Outside FOV
  if(!CheckInsideFOV(z, sensorNumber)){return false;}

  // Initializations
  vector<double> detectionProbability(numCellsAlongRay+1), unnormalizedOccupancyProbabilities(numCellsAlongRay);

  // Detection Probabilities
  FindDetectionProbabilities(detectionProbability, numCellsAlongRay, sensorNumber, probRayReachesFirstCell);

  // Obtain unnormalized probabilities & the normalizer
  double invNormalizer = GetUnnormalizedProbabilities(detectionProbability, unnormalizedOccupancyProbabilities, z, numCellsAlongRay, sensorNumber);

  // Normalized unnormalized probabilities
  return NormalizeProbabilities(invNormalizer, unnormalizedOccupancyProbabilities, sensorInformation[sensorNumber].mapEstRay, numCellsAlongRay, sensorNumber);
}

double Voxel::mapInfo::ForwardSensorModel(// p(measurement | pose, first occupied cell)
    double& z, double& zExpectedValue, int& sensorNumber){
  if(z < sensorInformation[sensorNumber].maxRange){// sensor reading inside FOV -> occupied cell or phantom reading
    if(zExpectedValue < sensorInformation[sensorNumber].maxRange)// considering an occupied voxel inside FOV
      return sensorInformation[sensorNumber].probHit*exp(-pow(z-zExpectedValue, 2)/(2*sensorInformation[sensorNumber].sigma*sensorInformation[sensorNumber].sigma))/(sensorInformation[sensorNumber].sigma*sqrt2pi)// hit cell (Gaussian)
            +sensorInformation[sensorNumber].probRand/(sensorInformation[sensorNumber].maxRange-sensorInformation[sensorNumber].minRange)// phantom reading (uniform)
            ;
    else// empty map case: impossible with sensor reading in FOV, COMMENTED OUT: is a phantom reading
      return 0.0;//sensorInformation[sensorNumber].probRand/(sensorInformation[sensorNumber].maxRange-sensorInformation[sensorNumber].minRange);
  }
  else{// sensor returns a max reading
    if(zExpectedValue < sensorInformation[sensorNumber].maxRange)// P(z = sensorInformation[sensorNumber].maxRange | occupied space in FOV)
      return sensorInformation[sensorNumber].probMaxFree;
    else// P(z = sensorInformation[sensorNumber].maxRange | no occupied space in FOV)
      return sensorInformation[sensorNumber].probMaxOcc;
//    return sensorInformation[sensorNumber].probHit/(sensorInformation[sensorNumber].sigma*sqrt2pi)+sensorInformation[sensorNumber].probRand/(sensorInformation[sensorNumber].maxRange-sensorInformation[sensorNumber].minRange);
  }
}

int Voxel::mapInfo::IndFromMapLocation(vector<double>& mapLoc){

  // Return map index if one exists, otherwise return -1
  int ind(0);
  for(int i(0); i < dimension; i++){
    if(mapLoc[i] > minimumLocations[i]-alpha/2
       && mapLoc[i] < maximumLocations[i]+alpha/2)
      ind += stride[i]*floor((mapLoc[i]-minimumLocations[i])/alpha+0.5);
    else
      return -1;
  }
  if(ind > -1 && ind < numCellsTotal)
    return ind;
  else
    return -1;

}

vector<double> Voxel::mapInfo::MapLocationFromInd(int ind){

  // Return Map Location from Index
  vector<double> mapLoc(dimension, 0.0);
  if(ind > -1 && ind < numCellsTotal){
    int remainderTemp;
    for(int i(0); i < dimension-1; i++){//int i = dimension-1; i > 0; i--){// 2 1 0
      remainderTemp = floor(fmod(ind, numCellsEachDimension[i])+0.5);
      mapLoc[i] = remainderTemp*alpha+minimumLocations[i];
      ind = floor((ind-remainderTemp)/numCellsEachDimension[i]+0.5);
    }
    mapLoc[dimension-1] = ind*alpha+minimumLocations[dimension-1];// last dimension does not have remainders
  }

  return mapLoc;
}

vector<int> Voxel::mapInfo::DimensionalIndicesFromMapIndex(int ind){

  // Return Map Location from Index
  vector<int> mapLoc(dimension, 0);
  if(ind > -1 && ind < numCellsTotal){
    for(int i(0); i < dimension-1; i++){//int i = dimension-1; i > 0; i--){// 2 1 0
      mapLoc[i] = fmod(ind, numCellsEachDimension[i]);
      ind = (ind-mapLoc[i])/numCellsEachDimension[i];
    }
    mapLoc[dimension-1] = ind;// last dimension does not have remainders
  }

  return mapLoc;
}

int Voxel::explorationInfo::SumExpectedInfoGainsEachLocation(mapInfo& mapInput){

  // Initializations
  int numCandConsidered(0); int uvecInd;
  numDoneCandidates = 0;// TODO: update and printout these timers
  numCollCandidates = 0;

  // Cycle Through Viable Candidates
  for(int iCand = 0; iCand < numCandidatesTotal; iCand++){

    // Unusable Candidates
    if(candidateDone[iCand])
      numDoneCandidates++;
    else if(!freeCells[candidateIndices[iCand]]){
      numDoneCandidates++;
      numCollCandidates++;
    }

    // Check Among Viable Candidates
    else{

      // Obtain Each Ray Expected Info Gain & Take Summation
      for(int iRay = 0; iRay < numRaysPerCandidate; iRay++)
        infoGainEachRay[iRay]// negative entropy change
            = -1.0*mapInput.SingleRayExpectedEntropyChange(candidateLocations[iCand], rayUVecs[iRay]);

      // Entropy Sum of Rays
      infoGainEachDirection = SumScalarQuantityOfRays(infoGainEachRay, numRaysEachSide);

      // Maximum Info Gain Magnitude & Direction
      candidateExpectedInfoGain[iCand] = MaxOfVector(infoGainEachDirection, uvecInd);
      optimalUVecs[iCand] = rayUVecs[uvecInd];

      // Check Threshold
      if(candidateExpectedInfoGain[iCand] < minInfoGainThresh){// comeback
        candidateExpectedInfoGain[iCand] = 0.0;
//        candidateDone[iCand] = true;
        numLowHCandidates++;
      }
      else
        numCandConsidered++;

    }
  }

  return numCandConsidered;

}

void Voxel::explorationInfo::VisualizeCandidates(
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
      arrow.points[0] = arrowLoc;
      arrowLoc.x = candidateLocations[iCand][0]+arrowLength*optimalUVecs[iCand][0];
      arrowLoc.y = candidateLocations[iCand][1]+arrowLength*optimalUVecs[iCand][1];
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

void Voxel::explorationInfo::VisualizeCandidatesMultiRobot(vector<robotInfo>& robotInformation, int& numRobots){

  // Initializations
  vector<double> bestCandInfoGains(numRobots, 0.0), infoGainsBuff(numRobots, 0.0);
  vector<int> bestCandIdices(numRobots,0);
  int bestInd;

  // Message Time Stamp
  arrow.header.stamp = ros::Time::now();

  for(int iRobot = 0; iRobot < numRobots; iRobot++)
    bestCandInfoGains[iRobot] = MaxOfVector(robotInformation[iRobot].candidateInfoGainWithBump, bestCandIdices[iRobot]);
  double bestCandInfoGain = MaxOfVector(bestCandInfoGains, bestInd);

  // Cycle Through Viable Candidates, Generating Marker Array Message
  for(int iCand = 0; iCand < numCandidatesTotal; iCand++){
    int candIndex = candidateIndices[iCand];
    if(!candidateDone[iCand] && freeCells[candIndex]){// passes entropy threshold & reachable

      // Arrow ID
      arrow.id = iCand;

      // Arrow Color (opacity: candidate reward, color: robot identifier)
      for(int iRobot = 0; iRobot < numRobots; iRobot++)
        infoGainsBuff[iRobot] = robotInformation[iRobot].candidateInfoGainWithBump[iCand];
      MaxOfVector(infoGainsBuff, bestInd);
      for(int iRobot = 0; iRobot < numRobots; iRobot++){
        if(robotInformation[iRobot].candidateInfoGainWithBump[iCand]
           == bestCandInfoGains[iRobot])
          bestInd = iRobot;
      }
      for(int iRobot = 0; iRobot < numRobots; iRobot++){
        if(bestInd == iRobot){
          arrow.color = robotInformation[iRobot].color;
          arrow.color.a = infoGainsBuff[iRobot]/bestCandInfoGain;
          break;
        }
      }

      // Arrow Locations (start & tip)
      arrowLoc.x = candidateLocations[iCand][0];//-0.5*arrowLength*optimalUVecs[iCand][0];
      arrowLoc.y = candidateLocations[iCand][1];//-0.5*arrowLength*optimalUVecs[iCand][1];
      arrow.points[0] = arrowLoc;
      arrowLoc.x = candidateLocations[iCand][0]+arrowLength*optimalUVecs[iCand][0];
      arrowLoc.y = candidateLocations[iCand][1]+arrowLength*optimalUVecs[iCand][1];
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


void Voxel::explorationInfo::FindViableCandidateInfoGains(mapInfo& mapInput){

  // Start Timer
  if(timersOn){tStartSection = ros::Time::now().toSec();}

  // Expected Information Gain for Each Candidate Location at the Best Direction
  int numCandConsidered = SumExpectedInfoGainsEachLocation(mapInput);

//  cout << "Candidate Pose Availability Information:"
//       << "\nTotal from Beginning: " << numCandidatesTotal
//       << "\nSafe & Above Thresh:  " << numCandConsidered
//       << "\nAmong the " << numDoneCandidates << " Neglected:"
//       << "\n  Below Thresh:         " << numLowHCandidates
//       << "\n  Risks Collision:      " << numCollCandidates
//       << "\n  Already Visited:      " << numVisitedCandidates
//       << endl;

  // Terminate for No Viable Candidates
  if(numCandConsidered == 0){
    ROS_INFO("No candidates are considered. Terminating exploration...");
    ros::shutdown();
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
    // Uncomment for printing the timer
//    ROS_INFO("Exploration optimal pose discovery time: %e", tEndSection-tStartSection);
  }

  return;
}

double Voxel::explorationInfo::MaximizeCandidateObjective(robotInfo& robot){

  while(true){

    // Maximum Information Gain Location
    double award = MaxOfVector(robot.candidateInfoGainWithBump, robot.bestCandInd);
    int candIndex = candidateIndices[robot.bestCandInd];

    // Verify the Robot Not Here Already
    if(robot.costMap[candIndex] == 0.0){
      robot.candidateInfoGainWithBump[robot.bestCandInd] = 0.0;
      candidateDone[robot.bestCandInd] = true;
      ROS_WARN("Robot %s is already located at maximum candidate info gain location with candidate index %d of %d total...\nRemoving candidate & finding new maximum...", robot.ns.c_str(), candIndex, numCandidatesTotal);
    }

    // Optimal Pose Requires Translational Motion
    else
      return award;

  }
}

bool Voxel::explorationInfo::FindOptimalReachablePose(robotInfo& robot){

  // Clear Old Information & Bump Information
  fill(robot.candidateInfoGainWithBump.begin(),
       robot.candidateInfoGainWithBump.end()  ,
       0.0);
  double minFunVal = 0.1;// TODO: param

  // Penalize Candidates by a Travel Distance Cost
  int candIndex; double distAway;
  for(int i(0); i < numCandidatesTotal; i++){
    candIndex = candidateIndices[i];
    if(!candidateDone[i]// not done
     && robot.costMap[candIndex] < costMapInitValue){// reachable

      // Gaussian-Like Bump Function to Prioritize Shorter Actions
      distAway = robot.costMap[candIndex];
      robot.candidateInfoGainWithBump[i]
          = GaussianLikeBump(
            robot.costMap[candIndex], bumpSigSqrd, minFunVal)
            *candidateExpectedInfoGain[i];
      numReachCandidates++;

    }
    else
      robot.candidateInfoGainWithBump[i] = 0.0;
  }

  // Select the Location Index with Maximum Information Gain
  robot.entropyBumpMax = MaximizeCandidateObjective(robot);

  // Checks & Returns
  if(robot.entropyBumpMax <= 0.0){
    robot.bestCandInd = -1;
    for(int i(0); i < numCandidatesTotal; i++){
      if(freeCells[candidateIndices[i]] == true){
        if(robot.costMap[i] == 0.0)
          ROS_ERROR("robot.costMap[%d] == 0.0", i);
        if(candidateExpectedInfoGain[i] <= 0.0)
          ROS_WARN("candidateExpectedInfoGain[%d] = %e", i, candidateExpectedInfoGain[i]);
        else
          ROS_INFO("candidateExpectedInfoGain[%d] = %e", i, candidateExpectedInfoGain[i]);
      }
    }
    return false;
  }
  else{
    numVisitedCandidates++;
    return true;
  }
}

double Voxel::mapInfo::SingleRayExpectedEntropyChange(vector<double>& sensorLoc, vector<double>& rayUVec){

  // Initializations
  double rayEntropyChange(0.0), probRayReachesFirstCell;
  int sensorNumber(0);

  // Ray Casting (NOTE: this can be removed with initial ray cast that is shifted to any cell)
  int numCellsAlongRay = RayCasting(
        sensorLoc, rayUVec, sensorNumber, probRayReachesFirstCell);//, measOutsideMapLimits);

  // TODO: verify max case with exploration (neglected now)



  // Cell Probabilities Along Casted Ray
  for(int i(0); i < numCellsAlongRay; i++)
    sensorInformation[sensorNumber].mapEstRay[i] = occData.data[sensorInformation[sensorNumber].cellIndices[i]];


  // Uncomment Below for Frontiers-Only Exploration

//  bool isFrontier(false);
//  for(int i(0); i < numCellsAlongRay; i++){
//    if(sensorInformation[sensorNumber].mapEstRay[i] > 0.05){
//      if(sensorInformation[sensorNumber].mapEstRay[i] < 0.95){
////        cout << "YES Frontier: [][][][?]" << endl;
////        return -0.5;
//        isFrontier = true;
//      }
//      else{
////        cout << "NO Frontier:  [][][][x]" << endl;
////        return -0.0;
//        break;
//      }
//    }
//  }
////  cout << "NO Frontier:  [][][][0]" << endl;
////  return -0.0;


  // Detection Probabilities
  int numMeasOutcomes(numCellsAlongRay+1);
  vector<double> detectionProbability(numMeasOutcomes);
  FindDetectionProbabilities(detectionProbability, numCellsAlongRay, sensorNumber, probRayReachesFirstCell);
  double probMeasHitsAnyCells = probRayReachesFirstCell-detectionProbability.back();// reaches first & not a max reading

  // Unnormalized Probabilities & Normalizer
  vector< vector<double> > cellProbabilitiesMatrix = vector< vector<double> >(numMeasOutcomes-1, vector<double>(numMeasOutcomes-1));//asdf
  vector< vector<double> > normalizedCellProbabilitiesMatrix = cellProbabilitiesMatrix;
  vector<double> invNormalizers(numMeasOutcomes-1, 0.0), measProbs(numMeasOutcomes-1);
  for(int iMeas = 0; iMeas < numMeasOutcomes-1; iMeas++){
    double zMeas = sensorInformation[sensorNumber].sensorRanges[iMeas];
    invNormalizers[iMeas] = GetUnnormalizedProbabilities(
          detectionProbability, cellProbabilitiesMatrix[iMeas], zMeas, numCellsAlongRay, sensorNumber);

    if(!NormalizeProbabilities(invNormalizers[iMeas], cellProbabilitiesMatrix[iMeas], normalizedCellProbabilitiesMatrix[iMeas], numCellsAlongRay, sensorNumber))
      return 0.0;
  }

  // Normalized Probabilities
  double invNormalizerSum = accumulate(invNormalizers.begin(), invNormalizers.end(), 0.0);
  if(invNormalizerSum > 0.0){
    for(int i(0); i < numCellsAlongRay; i++)
      measProbs[i] = invNormalizers[i]/invNormalizerSum;
  }
  else
    return 0.0;

  // Expected Entropy Changes (all cells along ray)
  for(int iCell = 0; iCell < numCellsAlongRay; iCell++){

    // Cell's Entropy With All Possible Measurements
    for(int iMeas = 0; iMeas < numCellsAlongRay; iMeas++)
      rayEntropyChange += measProbs[iMeas]*EntropySingleCell(normalizedCellProbabilitiesMatrix[iMeas] [iCell]);

    // Cell's Entropy Without Any New Measurements
    rayEntropyChange -= EntropySingleCell(sensorInformation[sensorNumber].mapEstRay[iCell]);
  }

  // Eliminate Possible Max Case
  rayEntropyChange *= probMeasHitsAnyCells;

  if(isnan(rayEntropyChange)){
    ROS_ERROR("Expected entropy change is NaN.");
    return 0.0;
  }
  else{
    if(rayEntropyChange < 0.0)// normal expected information increase
      return rayEntropyChange;
    else// well-known cells at probability saturation limits
      return 0.0;
  }
}

void Voxel::MapReductionGetParameters(){

  // Reduced Map
  string exploreString("/exploration/");
  ros::param::get(exploreString+"height"         ,    explore.height             );
  ros::param::get(exploreString+"alpha_xy_factor",    explore.alphaXY            );
  ros::param::get(exploreString+"alpha_z_factor" ,    explore.alphaZ             );
  ros::param::get(exploreString+"min_max_thresh" ,    reduction.minMaxThresh     );
  ros::param::get(exploreString+"free_nbrs_coll_map", explore.reqNumFreeNeighbors);
  ros::param::get(exploreString+"cell_comb_process",  reduction.cellCombProcess  );
  ros::param::get(exploreString+"prob_H_map_thresh",  reduction.probHMapThresh   );
  reduction.entropyMapThresh = EntropySingleCell(reduction.probHMapThresh);
  explore.reqNumFreeNeighbors;

  ros::param::get("/mapping/occ_topic", reduction.map3DTopic);
  ros::param::get("/mapping/frame"    , reducedMap.frame    );

  // Parameters Specific to Map Reduction Type
  if(     reduction.cellCombProcess == "free_entropy"){
    if(     reduction.goal == "collision"){
      ros::param::get("/collision/occ_topic"   , reduction.map2DTopic);
      ros::param::get("/collision/viz_topic"   , reduction.ogmTopic  );

    }
    else if(reduction.goal == "entropy"  ){
      ros::param::get("/entropy/occ_topic", reduction.map2DTopic);
      ros::param::get("/entropy/viz_topic", reduction.ogmTopic  );
    }
    else
      ROS_ERROR("Reduction goal is improperly set at %s.", reduction.goal.c_str());
  }
  else if(reduction.cellCombProcess == "min"
       || reduction.cellCombProcess == "max"
       || reduction.cellCombProcess == "min_max"
       || reduction.cellCombProcess == "mean"){
    ros::param::get("/collision/occ_topic", reduction.map2DTopic);
    ros::param::get("/collision/viz_topic", reduction.ogmTopic  );
  }
  else
    ROS_ERROR("Reduction cell combination process is\nimproperly set at %s.", reduction.cellCombProcess.c_str());

  return;
}

void Voxel::mapInfo::OccupancyGridMsgInit(nav_msgs::OccupancyGrid& ogm, double& mapHeight){

  // Assign map parameters to occupancy grid message
  ogm.header.frame_id = frame;
  ogm.info.resolution = alpha;
  ogm.info.width      = numCellsEachDimension[0];
  ogm.info.height     = numCellsEachDimension[1];
  ogm.info.origin.position.x = minimumLocations[0]-0.5*alpha;
  ogm.info.origin.position.y = minimumLocations[1]-0.5*alpha;
  ogm.info.origin.position.z = mapHeight;
  ogm.data.resize(numCellsTotal);

  return;
}

void Voxel::ReduceMapParamsInit(){

  string objective;

  // Complete Map Parameters & Initializations
  objective = "mapping";
  map.GetMapParameters(objective);
  map.BasicMapInfoInit(objective);

  // Auxillary Parameters for Reduced Map
  MapReductionGetParameters();

  // Camera Frame & Origin
  ros::param::get("/exploration/sensor_0/naming/frame", reduction.cameraOrigin.header.frame_id);
  reduction.cameraOrigin.point.x = 0.0;
  reduction.cameraOrigin.point.y = 0.0;
  reduction.cameraOrigin.point.z = 0.0;

  // Initialize 2D Map
  int numDecimals(4);
  reducedMap.dimension = 2;
  reducedMap.numCellsEachDimension.resize(2);
  reducedMap.minimumLocations.resize(2);
  reducedMap.maximumLocations.resize(2);

  if(reduction.cellCombProcess == "free_entropy" && reduction.goal == "entropy")
    explore.alphaXY = 1;
  reducedMap.alpha = explore.alphaXY*map.alpha;
  reducedMap.numCellsTotal = 1;
  for(int i(0); i < 2; i++){
    if(reduction.goal == "entropy")
      reducedMap.numCellsEachDimension[i] = map.numCellsEachDimension[i];
    else
    reducedMap.numCellsEachDimension[i] = floor(1.0*map.numCellsEachDimension[i]/explore.alphaXY);

    reducedMap.minimumLocations[i] = map.minimumLocations[i]
        +0.5*map.alpha*(explore.alphaXY-1);
    reducedMap.maximumLocations[i] = reducedMap.minimumLocations[i]
        +reducedMap.alpha*(reducedMap.numCellsEachDimension[i]-1);
    reducedMap.numCellsTotal *= reducedMap.numCellsEachDimension[i];

  }

  reducedMap.occData.data.resize(reducedMap.numCellsTotal, map.initP);
  reducedMap.stride.resize(2);
  reducedMap.stride[0] = 1;
  reducedMap.stride[1] = reducedMap.numCellsEachDimension[0];

  // Occupancy Grid Message
  reducedMap.OccupancyGridMsgInit(reduction.OGM, explore.height);

  // Vector Offsets (small cell centers to the centers of big cells covering them)
  // X & Y:
  reduction.offset.resize(explore.alphaXY);
  reduction.offset[0] = 0.5*(1-explore.alphaXY)*map.alpha;
  for(int i = 1; i < explore.alphaXY; i++){
    reduction.offset[i] = reduction.offset[i-1]+map.alpha;
  }
  // Z:
  if(reduction.cellCombProcess == "free_entropy"
  && reduction.goal                   == "entropy"){
    int numCellsSkip = 2;
    reduction.numZCells = map.numCellsEachDimension[2];
    reduction.locZStart = map.minimumLocations[2];
    if(reduction.numZCells-numCellsSkip < 1)
      ROS_ERROR("Requested skipping %d cells but only %d cells are available. Neglecting skip...", numCellsSkip, reduction.numZCells);
    else
      reduction.numZCells -= numCellsSkip;
    reduction.offsetZ.resize(reduction.numZCells);
    reduction.offsetZ[0] = numCellsSkip*map.alpha;
  }
  else{
    reduction.numZCells = explore.alphaZ;
    reduction.locZStart = explore.height;
    reduction.offsetZ.resize(reduction.numZCells);
    reduction.offsetZ[0] = 0.5*(1-explore.alphaZ)*map.alpha;
  }
  for(int i = 1; i < reduction.numZCells; i++){
    reduction.offsetZ[i] = reduction.offsetZ[i-1]+map.alpha;
  }

  // Robot Vicinity
  reduction.robotPt     = {0.0, 0.0};
  reduction.robotPtLast = {0.0, 0.0};

  // Temporary Variables
  reduction.numCellsThisDim = {explore.alphaXY, explore.alphaXY, reduction.numZCells};
  reduction.numCellsInside = explore.alphaXY*explore.alphaXY*reduction.numZCells;
  reduction.IndsInside .resize(reduction.numCellsInside);
  reduction.probsInside.resize(reduction.numCellsInside);
  reduction.loc2D.resize(2);
  reduction.loc3D.resize(map.dimension);

  // Set Variables from Reduction
  reducedMap.alpha = Round2DecimalLevel(reducedMap.alpha, numDecimals);
  reducedMap.minimumLocations[0] = Round2DecimalLevel(reducedMap.minimumLocations[0], numDecimals);
  reducedMap.maximumLocations[0] = Round2DecimalLevel(reducedMap.maximumLocations[0], numDecimals);
  reducedMap.minimumLocations[1] = Round2DecimalLevel(reducedMap.minimumLocations[1], numDecimals);
  reducedMap.maximumLocations[1] = Round2DecimalLevel(reducedMap.maximumLocations[1], numDecimals);

  // Set Parameters for Exploration
  if(reduction.goal == "collision"){
    string paramSetStart("/"+reduction.goal+"/");
    ros::param::set(paramSetStart+"alpha"    ,   reducedMap.alpha              );
    ros::param::set(paramSetStart+"x_map_min",   reducedMap.minimumLocations[0]);
    ros::param::set(paramSetStart+"x_map_max",   reducedMap.maximumLocations[0]);
    ros::param::set(paramSetStart+"y_map_min",   reducedMap.minimumLocations[1]);
    ros::param::set(paramSetStart+"y_map_max",   reducedMap.maximumLocations[1]);
  }

  ROS_INFO("Map reduction initialization for %s goal is complete.", reduction.goal.c_str());

  return;
}

void Voxel::mapReduction::ProbsInsideLargerVoxel(double& startLocZ, mapInfo& mapToReduce){

  // Dimensions X,Y
  vector<double> minCorner(mapToReduce.dimension);
  for(int i(0); i < 2; i++)
    minCorner[i] = loc2D[i]+offset.front();

  // Dimension Z
  minCorner[2] = startLocZ+offsetZ.front();

  // Extract Indices Composing Reduced Map Cell
  int ind3D(0);
  mapToReduce.FindCellsComposingBlock(loc3D, minCorner, numCellsThisDim, IndsInside, ind3D);

  // Get Probabilities for Each Index
  for(int i(0); i < numCellsInside; i++)
    probsInside[i] = mapToReduce.occData.data[IndsInside[i]];

  return;
}

double Voxel::mapReduction::ProbReducedMapCell(mapInfo& mapToReduce, mapInfo& reducedMap, int& indReducedMap){

  // Initialize Probabilty of Reduced Cell
  double output;

  // 2D Location
  loc2D = reducedMap.MapLocationFromInd(indReducedMap);

  // 3D Cells Projected to 2D Location
  ProbsInsideLargerVoxel(locZStart, mapToReduce);

  // Find Projected Cell Value
  int ind;
  if(     cellCombProcess == "max"
       || (cellCombProcess == "free_entropy"
           && goal                == "collision"   ))
    output = MaxOfVector(probsInside, ind)      ;

  else if(cellCombProcess == "min")
    output = MinOfVector(probsInside, ind)      ;

  else if(cellCombProcess == "min_max"){
    output = MaxOfVector(probsInside, ind)      ;
    if(output < minMaxThresh)
      output = MinOfVector(probsInside, ind)    ;
  }
  else if(cellCombProcess == "mean")
    output = accumulate(probsInside.begin(),
                        probsInside.end()  , 0.0)
                        /probsInside.size()     ;
  else if(cellCombProcess == "free_entropy"
       && goal                   == "entropy"     ){
    vector<double> entropiesInside = probsInside;
    for(int i(0); i < probsInside.size(); i++){
      entropiesInside[i] = EntropySingleCell(probsInside[i]);
    }
    output = MaxOfVector(entropiesInside, ind);

    if(output > entropyMapThresh)// information may be reasonably gained viewing these cells
      output = probsInside[ind];
    else{// information may not be reasonably gained viewing these cells: set to max/min
      if(accumulate(probsInside.begin(),
                    probsInside.end()  , 0.0)
                    /probsInside.size() > 0.5)
        output = mapToReduce.occProbMax;
      else
        output = mapToReduce.occProbMin;
    }
  }
  else
    ROS_ERROR("Cell combination process %s is not recognized.", cellCombProcess.c_str());

  return output;
}

template<typename valType>
void Voxel::mapInfo::SetValToSpaceAroundCellLoc(vector<double>& loc, int& numCellsEachDirection, valType inputVal, vector<valType>& saveVals){

  int mapInd = IndFromMapLocation(loc), indTest;
  for(int iX = -numCellsEachDirection; iX <= numCellsEachDirection; iX++){
    for(int iY = -numCellsEachDirection; iY <= numCellsEachDirection; iY++){
      indTest = mapInd+iX*stride[0]+iY*stride[1];
      if(indTest > -1 && indTest < numCellsTotal)
        saveVals[indTest] = inputVal;
    }
  }

  return;
}

void Voxel::mapReduction::ClearRobotLoc(int& reqNumFreeNeighbors, mapInfo& mapToReduce, mapInfo& reducedMap, int& numRobots, vector<robotInfo>& robotInformation){

  tf::TransformListener tfListener;

  // Clear Robot Immediate Vicinity
  if(goal != "entropy"){

    cameraOrigin.header.stamp = ros::Time::now();

    for(int iTF(0); iTF < numRobots; iTF++){
      cameraOrigin.header.frame_id = robotInformation[iTF].robotFrame;
      tf::StampedTransform robotTF = LoopUntilTransformAcquired(tfListener, mapToReduce.frame, cameraOrigin.header.frame_id);
      robotPt = {robotTF.getOrigin().x(), robotTF.getOrigin().y()};
      reducedMap.SetValToSpaceAroundCellLoc(
            robotPt, reqNumFreeNeighbors, mapToReduce.occProbMin, reducedMap.occData.data);
    }
  }

  return;
}

// Complete Map Transfer Version
void Voxel::MapProbsReductionCallback(
    const std_msgs::Float64MultiArray::ConstPtr& msg){

  // 3D Map Message (full map)
  std_msgs::Float64MultiArray mapMsg = *msg;
  reduction.OGM.header.stamp = ros::Time::now();
  map.occData.data = mapMsg.data;

  // Combine Full Map Into Reduced Map
  for(int indReducedMap(0) ; indReducedMap < reducedMap.numCellsTotal; indReducedMap++)
    reducedMap.occData.data[indReducedMap] = reduction.ProbReducedMapCell(map, reducedMap, indReducedMap);

  // Clear Robot Location & Publish the Reduced Map
  reduction.ClearRobotLoc(explore.reqNumFreeNeighbors, map, reducedMap, numRobots, robotInformation);

  // Update Occupancy Grid Message
  for(int i(0); i < reducedMap.numCellsTotal; i++)
    reduction.OGM.data[i] = floor(reducedMap.occData.data[i]*100+0.5);
  reduction.OGM.header.stamp = ros::Time::now();
  reduction.OGM.header.frame_id = map.frame;

  // Publish Reduced Map
  map.mapProbsPub.publish(reducedMap.occData);
  ogm2DPub.   publish(reduction.OGM          );

  return;
}

void Voxel::mapInfo::UpdateChangedMapCells(){

  int iMapProb;

  // Update Potentially Changed Cells of Full Map Only
  for(int i(0); i < changedCells.inds.size(); i++){
    iMapProb = changedCells.inds[i];
    occData.data[iMapProb] = changedCells.probs[i];
  }

  return;
}

//// Map Changes Version
//void Voxel::MapProbChangesReductionCallback(
//    const ogm_ae::UpdatedMapCells::ConstPtr& msg){

//  // RESUME HERE IN 2018
////  reduction.OGM.header.stamp = ros::Time::now();

//  // 3D Map Message (only potential changes)
//  map.changedCells = *msg;
////  map.UpdateChangedMapCells();

////  // Min/Max Locations of Block to Update
////  vector<double> minCorner(reducedMap.dimension), maxCorner(reducedMap.dimension);
////  reduction.loc3D = map.MapLocationFromInd(map.changedCells.inds.front());
////  for(int i(0); i < reducedMap.dimension; i++){
////    minCorner[i] = reduction.loc3D[i];
////    if(minCorner[i] > reducedMap.minimumLocations[i])
////      minCorner[i] = reducedMap.minimumLocations[i];
////  }
////  minCorner = reducedMap.SnapLocationToMap(minCorner);
////  reduction.loc3D = map.MapLocationFromInd(map.changedCells.inds.back ());
////  for(int i(0); i < reducedMap.dimension; i++){
////    maxCorner[i] = reduction.loc3D[i];
////    if(maxCorner[i] > reducedMap.maximumLocations[i])
////      maxCorner[i] = reducedMap.maximumLocations[i];
////  }
////  maxCorner = reducedMap.SnapLocationToMap(maxCorner);

//////  reducedMap.ComposeChangedMapCellMsg(minCorner, maxCorner, false);


////  // Number of Cells to Update
////  vector<int> numUpdatedReducedCellsThisDim(reducedMap.dimension);
////  int numReducedCellsToUpdate(1);
////  for(int i(0); i < reducedMap.dimension; i++){
////    numUpdatedReducedCellsThisDim[i] = floor((maxCorner[i]-minCorner[i])/reducedMap.alpha+0.5)+1;
////    numReducedCellsToUpdate *= numUpdatedReducedCellsThisDim[i];
////  }

////  // Determine Which Reduced Cells Might Change
////  int ind(0);
////  vector<int> indsUpdatedReducedCellsThisDim(numReducedCellsToUpdate);
////  reducedMap.FindCellsComposingBlock(reduction.loc2D, minCorner, numUpdatedReducedCellsThisDim, indsUpdatedReducedCellsThisDim, ind);

////  // Combine Full Map Into Reduced Map
////  for(int i(0) ; i < numReducedCellsToUpdate; i++){
////    ind = indsUpdatedReducedCellsThisDim[i];
////    reducedMap.occData.data[ind] = reduction.ProbReducedMapCell(map, reducedMap, ind);
////  }

////  // Clear Robot Location & Publish the Reduced Map
////  reduction.ClearRobotLoc(explore.reqNumFreeNeighbors, map, reducedMap, numRobots, robotInformation);

////  // Update Occupancy Grid Message
////  for(int i(0); i < reducedMap.numCellsTotal; i++)
////    reduction.OGM.data[i] = floor(reducedMap.occData.data[i]*100+0.5);
////  reduction.OGM.header.stamp = ros::Time::now();
////  reduction.OGM.header.frame_id = map.frame;

////  // Publish Reduced Map
////  map.mapProbsPub.publish(reducedMap.occData);
////  ogm2DPub.   publish(reduction.OGM          );

//  return;
//}


// Map Changes Version
void Voxel::MapProbChangesReductionCallback(
    const ogm_ae::UpdatedMapCells::ConstPtr& msg){

  reduction.OGM.header.stamp = ros::Time::now();

  // 3D Map Message (only potential changes)
  map.changedCells = *msg;
  map.UpdateChangedMapCells();

  // Min/Max Locations of Block to Update
  vector<double> minCorner(reducedMap.dimension), maxCorner(reducedMap.dimension);
  reduction.loc3D = map.MapLocationFromInd(map.changedCells.inds.front());
  for(int i(0); i < reducedMap.dimension; i++){
    minCorner[i] = reduction.loc3D[i];
    if(minCorner[i] > reducedMap.minimumLocations[i])
      minCorner[i] = reducedMap.minimumLocations[i];
  }
  minCorner = reducedMap.SnapLocationToMap(minCorner);
  reduction.loc3D = map.MapLocationFromInd(map.changedCells.inds.back ());
  for(int i(0); i < reducedMap.dimension; i++){
    maxCorner[i] = reduction.loc3D[i];
    if(maxCorner[i] > reducedMap.maximumLocations[i])
      maxCorner[i] = reducedMap.maximumLocations[i];
  }
  maxCorner = reducedMap.SnapLocationToMap(maxCorner);

//  reducedMap.ComposeChangedMapCellMsg(minCorner, maxCorner, false);


  // Number of Cells to Update
  vector<int> numUpdatedReducedCellsThisDim(reducedMap.dimension);
  int numReducedCellsToUpdate(1);
  for(int i(0); i < reducedMap.dimension; i++){
    numUpdatedReducedCellsThisDim[i] = floor((maxCorner[i]-minCorner[i])/reducedMap.alpha+0.5)+1;
    numReducedCellsToUpdate *= numUpdatedReducedCellsThisDim[i];
  }

  // Determine Which Reduced Cells Might Change
  int ind(0);
  vector<int> indsUpdatedReducedCellsThisDim(numReducedCellsToUpdate);
  reducedMap.FindCellsComposingBlock(reduction.loc2D, minCorner, numUpdatedReducedCellsThisDim, indsUpdatedReducedCellsThisDim, ind);

  // Combine Full Map Into Reduced Map
  for(int i(0) ; i < numReducedCellsToUpdate; i++){
    ind = indsUpdatedReducedCellsThisDim[i];
    reducedMap.occData.data[ind] = reduction.ProbReducedMapCell(map, reducedMap, ind);
  }

  // Clear Robot Location & Publish the Reduced Map
  reduction.ClearRobotLoc(explore.reqNumFreeNeighbors, map, reducedMap, numRobots, robotInformation);

  // Update Occupancy Grid Message
  for(int i(0); i < reducedMap.numCellsTotal; i++)
    reduction.OGM.data[i] = floor(reducedMap.occData.data[i]*100+0.5);
  reduction.OGM.header.stamp = ros::Time::now();
  reduction.OGM.header.frame_id = map.frame;

  // Publish Reduced Map
  map.mapProbsPub.publish(reducedMap.occData);
  ogm2DPub.   publish(reduction.OGM          );

  return;
}

void Voxel::explorationInfo::GenerateOccupancyGridCostMap(
    mapInfo& mapInput, vector<double>& costMap, double& maxVal, nav_msgs::OccupancyGrid& ogm){

  for(int ind2D(0); ind2D < mapInput.numCellsTotal; ind2D++){
    if(costMap[ind2D] < maxVal)
      ogm.data[ind2D] = floor(costMap[ind2D]/maxVal*100+0.5);
    else
      ogm.data[ind2D] = 100;
  }

  return;
}

bool Voxel::explorationInfo::SetUpCostMap(mapInfo& mapInput, vector<robotInfo>& robotInformation, int& robotNum, int& indCurrent){

  // Mark Occupied Cells & Other Robots as Visited Cells
  robotInformation[robotNum].visited = visited;
  for(int iRobot(0); iRobot < robotInformation.size(); iRobot++){
    if(iRobot != robotNum)
      mapInput.SetValToSpaceAroundCellLoc(
            robotInformation[iRobot].sensorLoc, reqNumFreeNeighbors, true, robotInformation[robotNum].visited);
  }

  // Mark Initial Cell Cost as 0
  indCurrent = mapInput.IndFromMapLocation(robotInformation[robotNum].sensorLoc);
  ROS_INFO("Starting location of Robot %d: (%e, %e, %e)", robotNum,
       robotInformation[robotNum].sensorLoc[0],
       robotInformation[robotNum].sensorLoc[1],
       robotInformation[robotNum].sensorLoc[2]);
  if(!freeCells[indCurrent]){
    if(mapInput.occData.data[indCurrent] > acceptableCollisionProb)
      ROS_ERROR("Starting cell of robot %d is inside a collision zone.", robotNum);
    else{
      vector<int> dimInd(mapInput.dimension); bool existsAWayOut(false);
      dimInd = mapInput.DimensionalIndicesFromMapIndex(indCurrent);
      for(int iX = -1; iX <= 1; iX++){
        for(int iY = -1; iY <= 1; iY++){
          if(!(iX == 0 && iY == 0)
          && dimInd[0]+iX > -1
          && dimInd[0]+iX < mapInput.numCellsEachDimension[0]
          && dimInd[1]+iY > -1
          && dimInd[1]+iY < mapInput.numCellsEachDimension[1]){
            int indCheck = indCurrent+iX*mapInput.stride[0]+iY*mapInput.stride[1];
            if(mapInput.occData.data[indCheck] <= acceptableCollisionProb){
              existsAWayOut = true;
              freeCells[indCheck] = true;
              robotInformation[robotNum].visited[indCheck] = false;
            }
          }
        }
      }
      if(existsAWayOut){
        ROS_WARN("Starting neighboring cell(s) of robot %d inside a collision zone, but it has a path out.", robotNum);
        return true;
      }
      else
        ROS_ERROR("Starting neighboring cell(s) of robot %d inside a collision zone with no path out.", robotNum);
    }
    return false;
  }
  else
    return true;
}

bool Voxel::explorationInfo::GenerateCostMap(
    mapInfo& mapInput, vector<robotInfo>& robotInformation, int& robotNum, int& indCurrent){

  // Start Timer
  if(timersOn){tStartSection = ros::Time::now().toSec();}

  cout << "sterling1" << endl;

  // Initializations
  fill(robotInformation[robotNum].costMap.begin(), robotInformation[robotNum].costMap.end(), costMapInitValue);
  vector<int> dimInd(mapInput.dimension);
  vector<double> tempCosts; vector<int> tempInds;
  bool replaceInd(false); int indToReplace;
  double nodeValBuff;
  int numIter(0);
  double currentCost(0.0);
  robotInformation[robotNum].costMap[indCurrent] = currentCost;
  cout << "sterling2" << endl;

  // Check All Reachable Cells
  while(true){

    // Current Cell Dimensional Index
    dimInd = mapInput.DimensionalIndicesFromMapIndex(indCurrent);

    // Cycle Through All Neighboring Cells
    for(int iX = -1; iX <= 1; iX++){
      for(int iY = -1; iY <= 1; iY++){

        // Verify the Neighbor on the Map is Not the Cell in Question
        if(!(iX == 0 && iY == 0)
          && iX+dimInd[0] > -1 && iX+dimInd[0] < mapInput.numCellsEachDimension[0]
          && iY+dimInd[1] > -1 && iY+dimInd[1] < mapInput.numCellsEachDimension[1]){

          // Map Index of this Neighbor// possible TODO: recursion
          int indPt((dimInd[0]+iX)*mapInput.stride[0]+(dimInd[1]+iY)*mapInput.stride[1]);

          // Verify the Cell is Unvisted
          if(!robotInformation[robotNum].visited[indPt]){

            // Diagonal Case
            if(iX != 0 && iY != 0)
              nodeValBuff = robotInformation[robotNum].costMap[indCurrent]+costDiagonal;

            // Horizontal/Vertical case
            else
              nodeValBuff = robotInformation[robotNum].costMap[indCurrent]+costHorizVert;

            // Assign New Distance Cost to the Node if Possible
            if(nodeValBuff < robotInformation[robotNum].costMap[indPt]){
              robotInformation[robotNum].costMap[indPt] = nodeValBuff;

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
    if(numIter == 0){
      cout << "sterling3" << endl;
      cout << "indCurrent = " << indCurrent << endl;
      cout << "robotInformation[robotNum].visited size = " << robotInformation[robotNum].visited.size() << endl;
      cout << "robotInformation[robotNum].visited capacity = " << robotInformation[robotNum].visited.capacity() << endl;
      cout << "tempCosts size = " << tempCosts.size() << endl;
    }

    // Mark Cell as Visited & Ensure it Never Minimizes 'tempCosts'
    robotInformation[robotNum].visited[indCurrent] = true;
    currentCost = MinOfVector(tempCosts, indToReplace);
    if(numIter == 0){cout << "indToReplace = " << indToReplace << endl;}
    indCurrent = tempInds[indToReplace];
    if(numIter == 0){cout << "from tempInds, indCurrent = " << indCurrent << ", tempCosts size = " << tempCosts.size() << endl;}
    tempCosts[indToReplace] = costMapInitValue;// reset to max value
    replaceInd = true;

    // Repeat if Possible
    numIter++;
    if(currentCost < costMapInitValue && numIter < mapInput.numCellsTotal)
      maxCostMapAsgndVal = currentCost;
    else{
      if(numIter == 1){
        ROS_ERROR("Robot %d: Moving to any neighboring cells risks collision.", robotNum);
        return false;
      }
      break;// cost map complete
    }
  }
  cout << "sterling4" << endl;

  // Visualize (candidates showing entropies only)
  if(visualizeCostMap){
    costMapViz.header.stamp = ros::Time::now();
    GenerateOccupancyGridCostMap(mapInput, robotInformation[robotNum].costMap, maxCostMapAsgndVal, costMapViz);
    robotInformation[robotNum].costMapPub.publish(costMapViz);
  }
  cout << "sterling5" << endl;

  // End Timer
  if(timersOn){
    tEndSection = ros::Time::now().toSec();
    ROS_INFO("Costmap generation time for Robot %d: %e", robotNum, tEndSection-tStartSection);
  }

  return true;
}

bool Voxel::explorationInfo::FindDijkstraTrajectory(mapInfo& mapInput, vector<robotInfo>& robotInformation, int& robotNum){

  // Convert Candidate Index to Map Index
  int desMapInd = candidateIndices[robotInformation[robotNum].bestCandInd];

  // Desired (final) & Starting (sensor location) Indices
  vector<int> dimIndDes(mapInput.dimension), dimStart(mapInput.dimension);
  dimIndDes = mapInput.DimensionalIndicesFromMapIndex(desMapInd);
  vector<double> locStart = robotInformation[robotNum].sensorLoc;
  mapInput.MapLocToDimensionalIndex(locStart, dimStart);

  // Dijkstra's Waypoints Inserted in Reverse with 'pathwayBuff'
  int wpInd(0);
  for(int i(0); i < mapInput.dimension; i++)
    pathwayBuff[i][wpInd] = dimIndDes[i];// redflag

  // Map Index of Ending Point
  int indPt(0);
  for(int i(0); i < mapInput.dimension; i++)
    indPt += pathwayBuff[i][wpInd]*mapInput.stride[i];
  djkDistBuff[wpInd] = robotInformation[robotNum].costMap[indPt];// redflag

  // Cycle Through Neighbors, Saving Minimum Cost Neighbor for Next Loop
  int iXSave, iYSave, mapIndSave;
  while(true){
    double minVal(costMapInitValue);
    for(int iX = -1; iX <= 1; iX++){// possible TODO: recursion
      for(int iY = -1; iY <= 1; iY++){

        // Verify Cell is a Neighbor on the Map
        if(!(iX == 0 && iY == 0)// possible TODO: recursion
          && iX+pathwayBuff[0][wpInd] > -1
          && iX+pathwayBuff[0][wpInd] < mapInput.numCellsEachDimension[0]
          && iY+pathwayBuff[1][wpInd] > -1
          && iY+pathwayBuff[1][wpInd] < mapInput.numCellsEachDimension[1]){

          // Extract Neighbor Cost & Save
          indPt =
              (pathwayBuff[0][wpInd]+iX)*mapInput.stride[0]
             +(pathwayBuff[1][wpInd]+iY)*mapInput.stride[1];
          if(robotInformation[robotNum].costMap[indPt] < minVal){
            minVal = robotInformation[robotNum].costMap[indPt];
            iXSave = iX; iYSave = iY;
            mapIndSave = indPt;
          }
        }
      }
    }

    // Ensure that the Pose is Reachable
    if(minVal == costMapInitValue){
      ROS_ERROR("Dijkstra's algorithm cannot be completed: no viable paths along cost map.");
      return false;
    }

    // Save Pathway Point to Buffers
    wpInd++;
    pathwayBuff[0][wpInd] = pathwayBuff[0][wpInd-1]+iXSave;
    pathwayBuff[1][wpInd] = pathwayBuff[1][wpInd-1]+iYSave;
    djkDistBuff[wpInd] = robotInformation[robotNum].costMap[mapIndSave];

    // Dijkstra's Waypoints Successfully Reached Current Robot Pose
    if(pathwayBuff[0][wpInd] == dimStart[0]
    && pathwayBuff[1][wpInd] == dimStart[1]){// this never happens on failure

      // Initialize Variables to Insert Data from Buffers
      wayPoints.header.stamp = ros::Time::now();// redflag
      wayPoints.header.frame_id = mapInput.frame;
      numDjkPts = wpInd+1;// redflag
      wayPoints.poses.resize(numDjkPts);// 3 private // redflag
      djkLocations.resize(numDjkPts);// redflag
      djkTimes.resize(numDjkPts);// redflag
      fixedPoseStampedParams.header.stamp = ros::Time::now();// redflag
      fill(wayPoints.poses.begin(), wayPoints.poses.end(), fixedPoseStampedParams);

      // Unsnap Initial Location from Grid to Measured Value
      djkLocations[0] << locStart[0], locStart[1];
      wayPoints.poses[0].pose.position.x = djkLocations[0](0);
      wayPoints.poses[0].pose.position.y = djkLocations[0](1);
      djkTimes[0] = 0.0;// redflag
      wayPoints.poses[0].header.stamp
          = fixedPoseStampedParams.header.stamp;

      // Offsets/Max Speeds to Avoid Quick Rotations
      double theta; QuatToYawRot(theta, robotInformation[robotNum].sensorAtt);
      vector<double> uvecNow = {cos(theta), sin(theta)};
      theta = acos(DotProd(optimalUVecs[robotInformation[robotNum].bestCandInd], uvecNow));
      double minTrajTime = theta/maxRadPerSec;
      double distOffset;// distance offset for unsnapped initial location

      // Insert Buffer Data for Remaining Waypoints
      for(int i = 1; i <= wpInd; i++){

        // Waypoint Locations
        djkLocations[i](0) = mapInput.minimumLocations[0]+mapInput.alpha*pathwayBuff[0][wpInd-i];
        wayPoints.poses[i].pose.position.x = djkLocations[i](0);
        djkLocations[i](1) = mapInput.minimumLocations[1]+mapInput.alpha*pathwayBuff[1][wpInd-i];
        wayPoints.poses[i].pose.position.y = djkLocations[i](1);

        // Distance Offset & Robot Speed with Upper Rotational Threshold
        if(i == 1){
          distOffset = (djkLocations[1]-djkLocations[0]).norm()
                             -djkDistBuff[wpInd-i];
          double totTrajDist = distOffset+djkDistBuff[0];
          if(totTrajDist/robotInformation[robotNum].speed < minTrajTime)
            robotInformation[robotNum].speed = totTrajDist/minTrajTime;
          else
            robotInformation[robotNum].speed = maxSpeed;

        }

        // Waypoint Times
        djkTimes[i] = (distOffset+djkDistBuff[wpInd-i])/robotInformation[robotNum].speed;
        wayPoints.poses[i].header.stamp
            = fixedPoseStampedParams.header.stamp
            +ros::Duration(djkTimes[i]);// redflag

      }

      // redflag
      // Uncomment for Publishing (publishes when computed, prior to execution)
//      robotInformation[robotNum].waypointsPub.publish(wayPoints);

      return true;
    }
    else if(minVal <= 0){
      ROS_ERROR("Costmap for robot %d is minimized at an incorrect local minimum", robotNum);
      ros::shutdown();
      return false;
    }
  }

  return false;
}

void Voxel::PublishPaths(int& robotNum){

  // Camera Position
  robotInformation[robotNum].cameraPositionPub
      .publish(robotInformation[robotNum].camPosPath);

  // Robot Position
  robotInformation[robotNum].desiredTrajPositionPub
      .publish(robotInformation[robotNum].robPosPath);

  return;
}

bool Voxel::GenerateSegmentsForPatching(int& robotNum){

  // At Least 2 Points Required
  if(explore.numDjkPts < 2){
      ROS_ERROR("At least 2 waypoints are required for trajectory generation.");
      return false;
  }

  // The Minimum Required Points for Constrained Least Squares
  int reqPts = explore.pointsPerSegment+2;// pts/(1 seg) + (start & terminal points)

  // If More Points are Needed
  if(explore.numDjkPts < reqPts){

    // Insert 'numPtsAdd' >= 0 by Interpolation if Necessary
    int numPtsAdd(reqPts-explore.numDjkPts);

    // Obtain the Time & Distance Between the Terminal Point & Point Preceeding for a Unit Vector
    double delT = explore.djkTimes[explore.numDjkPts-1]-explore.djkTimes[explore.numDjkPts-2];
    double dist = robotInformation[robotNum].speed*delT;
    Eigen::Vector2d uVec = explore.djkLocations[explore.numDjkPts-1]-explore.djkLocations[explore.numDjkPts-2];
    uVec /= uVec.norm();

    // Insert Interpolated Points
    explore.djkLocations.resize(reqPts);
    explore.djkTimes    .resize(reqPts);
    for(int i = explore.numDjkPts-1; i < reqPts; i++){
      explore.djkLocations[i] = explore.djkLocations[i-1]+(dist/(numPtsAdd+1))*uVec;
      explore.djkTimes[i] = explore.djkTimes[i-1]+delT/(numPtsAdd+1);
    }
    explore.numDjkPts = reqPts;

    // This Case Corresponds to a Single Segment
    explore.numSegments = 1;
    explore.remainingPts = 0;
  }

  // Dijkstra's Waypoints Alone are Enough for Constrained Least Squares
  else{

    // -1: for shared waypoints between segments (num & den), -2: for endpoints (num) (-3 together)
    explore.numSegments = floor((explore.numDjkPts-3)/(explore.pointsPerSegment-1));

    // -1: shared waypoint, +1: last unshared waypoint, -2: endpoints
    explore.remainingPts = explore.numDjkPts-(explore.numSegments*(explore.pointsPerSegment-1)+1)-2;

  }

  return true;
}

void Voxel::GenerateSmoothTrajectory(mapInfo& mapInput, int& robotNum, ros::Time& rosTimeStartTraj){

  // State Matrix 'AMatrix' Relates Polynomial Coefficients of 'tVec' to 'xVec'/'yVec'
  int numRows = explore.numDjkPts-2+(explore.numSegments-1);// -2: endpoints (constraints), numSegments-1: shared waypoints
  int numCoeff = explore.polyOrder+1;
  Eigen::MatrixXd AMatrix = Eigen::MatrixXd::Zero(numRows, explore.numSegments*(numCoeff));
  Eigen::VectorXd xVec =    Eigen::VectorXd::Zero(numRows);
  Eigen::VectorXd yVec =    Eigen::VectorXd::Zero(numRows);
  Eigen::VectorXd tVec =    Eigen::VectorXd::Zero(numRows);

  // Constraints
  int numConstraints(2*explore.numSegments);
  Eigen::MatrixXd CMatrix = Eigen::MatrixXd::Zero(numConstraints, explore.numSegments*(numCoeff));

  // Polulate Matrices & Vectors
  int numPts, iStartRow, iRow, iStartCol, CMatRow(0), numSgn(1), tmp;
  vector<double> timeStartSeg(explore.numSegments+1, 0.0);// start time, shared times among segments, terminal time

  for(int iSegs = 0; iSegs < explore.numSegments; iSegs++){

    // Row & Column Indices Shared Among Time, State, & Constraint Matrices/Vectors
    iStartRow = iSegs*(explore.pointsPerSegment);
    iStartCol = iSegs*(numCoeff);

    // Time Vector
    if(iSegs > 0)
      timeStartSeg[iSegs] = explore.djkTimes[iStartRow-iSegs+1];

    // Determine the Number of Points on this Segment
    if(iSegs < explore.numSegments-1)
      numPts = explore.pointsPerSegment;
    else
      numPts = explore.pointsPerSegment+explore.remainingPts;


    // __ State Matrix __ \\

    for(int iPts = 0; iPts < numPts; iPts++){
      iRow = iStartRow+iPts;
      tVec(iRow) = explore.djkTimes[iRow+1-iSegs]-timeStartSeg[iSegs];
      xVec(iRow) = explore.djkLocations[iRow+1-iSegs](0);
      yVec(iRow) = explore.djkLocations[iRow+1-iSegs](1);
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
    timeStartSeg[explore.numSegments] = explore.djkTimes.back();
    double t = timeStartSeg[explore.numSegments]-timeStartSeg[explore.numSegments-1];
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

  // X Version of Solution Vector
  locAndConstraintsVec.block(0,0,AMatrix.cols(),1) = 2*AMatrix.transpose()*xVec;
  locAndConstraintsVec(AMatrix.cols()) = explore.djkLocations[0](0);
  locAndConstraintsVec(AMatrix.cols()+numConstraints-1) = explore.djkLocations.back()(0);
  VectorXd xCoeff(dimCCM);
  xCoeff = ConstrainedCostsMatrix.colPivHouseholderQr().solve(locAndConstraintsVec);

  // Y Version of Solution Vector
  locAndConstraintsVec.block(0,0,AMatrix.cols(),1) = 2*AMatrix.transpose()*yVec;
  locAndConstraintsVec(AMatrix.cols()) = explore.djkLocations[0](1);
  locAndConstraintsVec(AMatrix.cols()+numConstraints-1) = explore.djkLocations.back()(1);
  VectorXd yCoeff(dimCCM);
  yCoeff = ConstrainedCostsMatrix.colPivHouseholderQr().solve(locAndConstraintsVec);

  // Initialize Path Message
  std_msgs::Header header;
  header.frame_id = mapInput.frame;
  header.stamp = rosTimeStartTraj;
  int numTimeSteps = ceil(timeStartSeg[explore.numSegments]/explore.trajTimeStep);
  double dt = timeStartSeg[explore.numSegments]/numTimeSteps;
  numTimeSteps += 1;// include time step for 0

  // Set Initial Attitudes using Projection of b1 onto Plane Normal to e3  Eigen::Matrix3d sensorRotMatrix;
  Eigen::Matrix3d sensorRotMatrix;
  QuatToRotMatrix(robotInformation[robotNum].sensorAtt, sensorRotMatrix);
  Eigen::Vector3d b1Sensor;
  QuatToRotMatrix(robotInformation[robotNum].sensorAtt, sensorRotMatrix);
  Eigen::Vector3d sensorUVecWorldFrame = sensorRotMatrix*e1;
  Eigen::Vector3d sensorVecE1E2Plane = sensorUVecWorldFrame-(sensorUVecWorldFrame.dot(e3))*e1;
  double normSensorVecE1E2Plane = sensorVecE1E2Plane.norm();
  if(normSensorVecE1E2Plane > 0.0)
    b1Sensor = sensorVecE1E2Plane/normSensorVecE1E2Plane;
  else{
    ROS_ERROR("While making smooth trajectory, initial b1 aligned with e3:\nSetting b1 = e1.");
    b1Sensor = e1;
  }

  // Populate 'trajInfo' Object
  robotInformation[robotNum].trajInfo.header = header;
  robotInformation[robotNum].trajInfo.numSegments = explore.numSegments;
  robotInformation[robotNum].trajInfo.numCoeff = numCoeff;
  robotInformation[robotNum].trajInfo.timeStartSeg = timeStartSeg;
  unsigned numUsefulCoeff = explore.numSegments*numCoeff;
  robotInformation[robotNum].trajInfo.xCoeff.resize(numUsefulCoeff);
  robotInformation[robotNum].trajInfo.yCoeff.resize(numUsefulCoeff);
  if(!(EigenToStack(xCoeff, robotInformation[robotNum].trajInfo.xCoeff, numUsefulCoeff)
    && EigenToStack(yCoeff, robotInformation[robotNum].trajInfo.yCoeff, numUsefulCoeff))){
    ROS_ERROR("Bad conversion of Eigen to vector for 'trajInfo' object.");
    return;
  }
  robotInformation[robotNum].trajInfo.fixedZVal  = explore.height;
  robotInformation[robotNum].trajInfo.thetaStart = atan2(b1Sensor(1), b1Sensor(0));
  robotInformation[robotNum].trajInfo.thetaEnd   = atan2(explore.optimalUVecs[robotInformation[robotNum].bestCandInd][1],
                              explore.optimalUVecs[robotInformation[robotNum].bestCandInd][0]);

  // Publish Trajectory Information
  robotInformation[robotNum].trajCoeffsPub.publish(robotInformation[robotNum].trajInfo);

  // Publish Paths if Requested
  if(explore.publishPaths){

    // Initializations
    robotInformation[robotNum].camPosPath.header = header;
    robotInformation[robotNum].robPosPath.header = header;
    robotInformation[robotNum].camPosPath.poses.resize(numTimeSteps);
    robotInformation[robotNum].robPosPath.poses.resize(numTimeSteps);
    explore.trajTimeVec.resize(numTimeSteps);

    // Temporary Stamped ROS Message
    geometry_msgs::PoseStamped  desCamMsg;
    geometry_msgs::PoseStamped  desPosMsg;
    geometry_msgs::TwistStamped desVelMsg;
    geometry_msgs::AccelStamped desAclMsg;

    // Cycle Through All Time Steps: Obtain Poses, Velocities, & Accelerations
    if(explore.visualizeCostMap){
      for(int iTimeStep(0); iTimeStep < numTimeSteps; iTimeStep++){

        // Time under consideration
        explore.trajTimeVec[iTimeStep] = iTimeStep*dt;
        ros::Time timeNow = rosTimeStartTraj+ros::Duration(explore.trajTimeVec[iTimeStep]);

        // Pose from Trajectory
        GetXYTrajFromCoeff(robotNum, robotInformation[robotNum].trajInfo, timeNow, desCamMsg, desPosMsg, desVelMsg, desAclMsg);
        robotInformation[robotNum].camPosPath.poses[iTimeStep] = desCamMsg;
        robotInformation[robotNum].robPosPath.poses[iTimeStep] = desPosMsg;

      }
    }
  }
  return;
}

void Voxel::GetXYTrajFromCoeff(int& robotNum,
    ogm_ae::PolyLeastSquaresTraj& trajInfo, ros::Time& timeNow,
    geometry_msgs::PoseStamped&  desCamMsg, geometry_msgs::PoseStamped&  desPosMsg,
    geometry_msgs::TwistStamped& desVelMsg, geometry_msgs::AccelStamped& desAclMsg){

  // Initializations
  double x, xDot, xDDot, aX, y, yDot, yDDot, aY;
  x = xDot = xDDot = y = yDot = yDDot = 0.0;
  geometry_msgs::Vector3 vec;

  // Time Along Trajectory (starting at 0) & Total Duration
  double trajTimeNow = timeNow.toSec()-trajInfo.header.stamp.toSec();
  double trajTimeMax = trajInfo.timeStartSeg.back();
  if(trajTimeNow > trajTimeMax)
    trajTimeNow = trajTimeMax;

  // Obtain Segment of Coefficients
  int numSegments = trajInfo.numSegments;
  int currentSeg = DetermineSegmentFromPoint(trajTimeNow, numSegments, trajInfo.timeStartSeg);

  // Time Along Current Segment (each segment starting at 0)
  double correctedTime = trajTimeNow-trajInfo.timeStartSeg[currentSeg];

  // X,Y Position, Velocity, & Acceleration
  for(int iCoeff(0); iCoeff < trajInfo.numCoeff; iCoeff++){
    aX = trajInfo.xCoeff[currentSeg*trajInfo.numCoeff+iCoeff];
    aY = trajInfo.yCoeff[currentSeg*trajInfo.numCoeff+iCoeff];
    x += aX*pow(correctedTime, iCoeff);
    y += aY*pow(correctedTime, iCoeff);
    if(iCoeff > 0){
      xDot += aX*iCoeff*pow(correctedTime, iCoeff-1);
      yDot += aY*iCoeff*pow(correctedTime, iCoeff-1);
      if(iCoeff > 1){
        xDDot += aX*iCoeff*(iCoeff-1)*pow(correctedTime, iCoeff-2);
        yDDot += aY*iCoeff*(iCoeff-1)*pow(correctedTime, iCoeff-2);
      }
    }
  }


  // __ Rotational Kinematics __ \\

  // Starting & Ending Angle
  double thetaChange = trajInfo.thetaEnd-trajInfo.thetaStart;

  // Move Angle Difference from Domain [0,2*pi) to (-pi,pi]
  if(thetaChange >   M_PI)
    thetaChange -= 2*M_PI;
  if(thetaChange <= -M_PI)
    thetaChange += 2*M_PI;

  // Angular Velocity
  double omegaZ = thetaChange/trajTimeMax;

  // Camera Direction about 3rd Body-Fixed Axis
  double desTheta = trajInfo.thetaStart+omegaZ*trajTimeNow;


  // __ Output __ \\

  // Header
  std_msgs::Header header;
  header.frame_id = trajInfo.header.frame_id;
  header.stamp = trajInfo.header.stamp+ros::Duration(trajTimeNow);
  desCamMsg.header = desPosMsg.header = desVelMsg.header = desAclMsg.header = header;

  // Camera Pose
  desCamMsg.pose.position.x = x; desCamMsg.pose.position.y = y; desCamMsg.pose.position.z = trajInfo.fixedZVal;
  YawRotToQuat(desTheta, desCamMsg.pose.orientation);

  // Robot Pose
  desPosMsg.pose.position.x = desCamMsg.pose.position.x-robotInformation[robotNum].vecRobot2Camera_RobotFrame(0);
  desPosMsg.pose.position.y = desCamMsg.pose.position.y-robotInformation[robotNum].vecRobot2Camera_RobotFrame(1);
  desPosMsg.pose.position.z = desCamMsg.pose.position.z-robotInformation[robotNum].vecRobot2Camera_RobotFrame(2);
  desPosMsg.pose.orientation = FixedQuatTransformation(desCamMsg.pose.orientation, robotInformation[robotNum].RotMatrixCameraRobot);
  // Note: pose offset uses 'vecRobot2Camera_RobotFrame' instead of 'vecRobot2Camera_CameraFrame' because camera may be flipped

  // Camera & Robot Velocity
  vec.x = xDot; vec.y = yDot; vec.z = 0.0;
  desVelMsg.twist.linear = vec;
  vec.x = 0.0; vec.y = 0.0; vec.z = omegaZ;
  desVelMsg.twist.angular = vec;

  // Camera & Robot Acceleration
  vec.x = xDDot; vec.y = yDDot; vec.z = 0.0;
  desAclMsg.accel.linear = vec;
  vec.x = 0.0; vec.y = 0.0; vec.z = 0.0;
  desAclMsg.accel.angular = vec;

  return;
}

int Voxel::DetermineSegmentFromPoint(double& timeNow, int& numSegments, vector<double>& timeStartSeg){
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

bool Voxel::RobotExplorationBidding(){

  double bestBid;
  int numOptimalPosesFound(0), optimalRobotInd;
  int candIndex;
  vector<bool> optimalPosesFound(numRobots, false);
  vector<double> distsToEachPose(numRobots), mapLocOptimal(collisionMap.dimension), mapLocNeighbor(collisionMap.dimension);
  while(numOptimalPosesFound < numRobots){

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
//      ros::shutdown();
      return false;
    }

    // Update Information for Hightest Bid
    candIndex = explore.candidateIndices[robotInformation[optimalRobotInd].bestCandInd];
    distsToEachPose[optimalRobotInd] = robotInformation[optimalRobotInd].costMap[candIndex];
    mapLocOptimal = collisionMap.MapLocationFromInd(candIndex);
//        cout << "Robot " << robotInformation[optimalRobotInd].ns
//             << " is assigned " << entropyBumpsAllRobots[optimalRobotInd]
//             << " at candidate " << robotInformation[optimalRobotInd].bestCandInd
//             << ", totaling " << numOptimalPosesFound+1 << " future pose(s) found."
//             << endl;

    // Remove Highest Bid from Future Consideration
    optimalPosesFound[optimalRobotInd] = true;
    entropyBumpsAllRobots[optimalRobotInd] = -1.0;
    numOptimalPosesFound++;

    // Compute Best Candidate Indices For Remaining Robots
    for(int iRobot(0); iRobot < numRobots; iRobot++){
      if(!optimalPosesFound[iRobot]){

        double proximityDiscount;
        for(int iCand = 0; iCand < explore.numCandidatesTotal; iCand++){

          // Only Consider if Not Done & Reachable
          candIndex = explore.candidateIndices[iCand];
          if(!explore.candidateDone[iCand]// not done
             && robotInformation[iRobot].costMap[candIndex] < explore.costMapInitValue// reachable
             ){

            // Map Location of Candidate
            mapLocNeighbor = collisionMap.MapLocationFromInd(candIndex);

            if(abs(mapLocOptimal[0]-mapLocNeighbor[0]) <= explore.discountRadius
            && abs(mapLocOptimal[1]-mapLocNeighbor[1]) <= explore.discountRadius){

              // Neighbor Robot Discounting Factor
              double distToOptimalPose = NormBtwn2Vecs(mapLocOptimal, mapLocNeighbor);
              if(distToOptimalPose < explore.discountRadius)
                proximityDiscount = pow(distToOptimalPose/explore.discountRadius, 2);
              else
                proximityDiscount = 1.0;

              // Apply Discount
              robotInformation[iRobot].candidateInfoGainWithBump[iCand] *= proximityDiscount;

            }
          }
        }

        // Update Desired Pose Accounting for Discounts
//            entropyBumpsAllRobots[iRobot]
//                = MaxOfVector(robotInformation[iRobot].candidateInfoGainWithBump,
//                              robotInformation[iRobot].bestCandInd);
        entropyBumpsAllRobots[iRobot] = explore.MaximizeCandidateObjective(robotInformation[iRobot]);
      }
    }
  }

  return true;
}

void Voxel::mapInfo::GenerateRvizMapMarker(
    visualization_msgs::Marker& marker, int& mapInd, vector<double>& mapLoc,
    double& probRange, double& occupancyVizMin, double& prob, vector<int>& color){

  // Extract Data
  marker.id = mapInd;
  mapLoc = MapLocationFromInd(mapInd);
  marker.pose.position.x = mapLoc[0];
  marker.pose.position.y = mapLoc[1];
  marker.pose.position.z = mapLoc[2];
  if(prob > occupancyVizMin)// currently this cell considered occupied
    marker.color.a = (prob-occupancyVizMin)/probRange;
  else// last time considered occupied, not anymore
    marker.color.a = 0.0;
  if(color[0] < 0){
    marker.color.r = 0.5f;
    marker.color.g = 0.5f;
    marker.color.b = 0.5f;
  }
  else{
    marker.color.r = 1.0f/255*color[0];
    marker.color.g = 1.0f/255*color[1];
    marker.color.b = 1.0f/255*color[2];
  }

  return;
}
