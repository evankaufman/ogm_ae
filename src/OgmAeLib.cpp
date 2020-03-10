#include "ogm_ae/ogm_ae.hpp"


#define sqrt2pi 2.50662827463

// __ Robot __ \\

Robot::Robot(){}
Robot::Robot(string& objective, int& robotNum){

  // General Parameters
  ros::param::get("robot_"+to_string(robotNum)+"/ns"         , ns          );
  ros::param::get("robot_"+to_string(robotNum)+"/num_sensors", numSensors  );
  ros::param::get("robot_"+to_string(robotNum)+"/coll_rad"   , collisionRad);
  ros::param::get("robot_"+to_string(robotNum)+"/frame"      , robotFrame  );
  ros::param::get("robot_"+to_string(robotNum)+"/color_r"    , color.r     );
  ros::param::get("robot_"+to_string(robotNum)+"/color_g"    , color.g     );
  ros::param::get("robot_"+to_string(robotNum)+"/color_b"    , color.b     );

  // Robot Frame is Namespaced
  robotFrame = ns+"/"+robotFrame;

  // Exploration-Specific Parameters
  if(objective == "exploration"){
    int exploreFrameNum;
    ros::param::get("robot_"+to_string(robotNum)+"/sensor_explore"        ,
                    exploreFrameNum                      );
    ros::param::get("sensor_"+to_string(exploreFrameNum)+"/naming/frame_explore",
                    exploreFrame);
    exploreFrame
        = ns+"/"+exploreFrame;

    // Exploration: Only 1 Sensor per Robot
    numSensors = 1;
  }

}


// __ Mapping __ \\

// Class Initialization

void Mapping::PubSubInit(){

  // ROS Nodehandle
  ros::NodeHandle nh;

  // Initializers Specific to the Node
  GeneralInit(numSensorsTotal, numRobots, robotVec);

  // Publishers
    // Full Occupancy Grid Map
    mapProbsPub = nh.advertise<std_msgs::Float64MultiArray>
        (occTopic, 1, true);

    // Map Updates Only
    mapChangesPub = nh.advertise<ogm_ae::UpdatedMapCells>
        (changesTopic, 1);

    // RGB Map Colors
    mapRGBPub = nh.advertise<std_msgs::Int16MultiArray>
        (rgbTopic , 1, true);

    // Map Entropy
    mapEntropyPub = nh.advertise<std_msgs::Float64>
      (HTopic, 1, true);

  // Subscribers (note: TF is required for mapping, used here for its fast update rate)

    if(numThreads < 1){
      numThreads = 1;
      ROS_WARN("Number of threads is incorrectly set. Now set to 1.");
    }

    // TF for Multithreaded Degrading Full Map Publishing
    if(degrade){
      TFMapDegradation = nh.subscribe
        ("/tf", 1, &Mapping::TFCallbackMapDegradation, this);

      // Increase Thread Number if Necessary
      if(numThreads == 1){
        numThreads = 2;
        ROS_WARN("Adding a thread (total: 2) for full map publishing with degradation.");
      }
    }

    // TF for Multithreaded Full Map Entropy Publishing
    if(numThreads > 1 && trackH)
      TFMapEntropy     = nh.subscribe
        ("/tf", 1, &Mapping::TFCallbackMapEntropy    , this);

    // Sensor Scans
    for(int sensorNumber = 0; sensorNumber < numSensorsTotal; sensorNumber++){
      sensors[sensorNumber].sub.subscribe
          (nh, sensors[sensorNumber].topic, 1);
      sensors[sensorNumber].tfFilter = new tf::MessageFilter<sensor_msgs::PointCloud2>
          (sensors[sensorNumber].sub, sensors[sensorNumber].tfListenerMsgFilter, frame, 1);
      sensors[sensorNumber].tfFilter->registerCallback
          (boost::bind(&Mapping::PointCloud2TFFilterCallback, this, _1, sensorNumber));
  }

  return;
}

void Mapping::GeneralInit(int& numSensorsTotal, int& numRobots, vector<Robot>& robotVec){

  lockMapChangesMsg = false;
  lockFullMapThread = false;
  lockEntropyThread = false;
  tDegradedLast = ros::Time::now().toSec();
  lockSensor.resize(numSensorsTotal, false);

  // Note: this is for a complete map (not reduced/projected)
  string objective = "mapping";

  // Map Parameters (required for all nodes)
  GetMapParameters(objective);

  // Sensor Parameters
  GetSensorParameters(objective, numRobots, robotVec, numSensorsTotal);

  // Basic Map Information
  BasicMappingInit(objective);

  // Sensor Update
  SensorUpdateInit(numSensorsTotal);

  // Map Color
  int numElementsPerCell = dimension;
  string addedLabelInfo = " RGB ("+to_string(dimension)+" elements each)";
  int initColor(-1);// uncolored identifier
  MappingToMultiArray(
        numElementsPerCell, addedLabelInfo, rgbColors, initColor);

  // Avoiding Measuring Other Robots as the Map
  for(int i(0); i < numSensorsTotal; i++){
      sensors[i].otherRobotFrames       .resize(numRobots-1);
      sensors[i].otherRobotRadii        .resize(numRobots-1);
      sensors[i].maxDotProdToOtherRobots.resize(numRobots-1);
      int otherRobotInd(0);
      for(int iRobot(0); iRobot < numRobots; iRobot++){
        if(iRobot != sensors[i].robotIndex){
          sensors[i].otherRobotFrames[otherRobotInd]
              = robotVec[iRobot].robotFrame  ;
          sensors[i].otherRobotRadii [otherRobotInd]
              = robotVec[iRobot].collisionRad;
          otherRobotInd++;
        }
      }
    }

  return;
}

void Mapping::GetMapParameters(string& objective){
  string paramNameStart = "/"+objective+"/";
  ros::param::get(paramNameStart+"occ_topic"  , occTopic  );
  ros::param::get(paramNameStart+"track_H"    , trackH    );
  ros::param::get(paramNameStart+"H_topic"    , HTopic    );
  ros::param::get(paramNameStart+"frame"      , frame     );
  ros::param::get(paramNameStart+"dimension"  , dimension );
  ros::param::get(paramNameStart+"alpha"      , alpha     );
  ros::param::get(paramNameStart+"P_init"     , initP     );
  ros::param::get(paramNameStart+"occ_min"    , occProbMin);
  ros::param::get(paramNameStart+"occ_max"    , occProbMax);
  ros::param::get(paramNameStart+"occ_thresh" , occThresh );
  ros::param::get(paramNameStart+"track_diff" , trackDiff );
  ros::param::get(paramNameStart+"dgrd_map"   , degrade   );
  ros::param::get(paramNameStart+"dgrd_rate"  , dgrdLamda );
  ros::param::get(paramNameStart+"dgrd_freq"  , dgrdHz    );
  ros::param::get(paramNameStart+"num_threads", numThreads);
  ros::param::get(paramNameStart+"ray_num_max", rayNumMax );
  ros::param::get(paramNameStart+"min_P_ISM"  , minProbISM);




  minimumLocations.resize(dimension);
  maximumLocations.resize(dimension);
  dimNames = {"x","y","z"};
  for(int i(0); i < dimension; i++){
    ros::param::get(paramNameStart+dimNames[i]+"_map_min", minimumLocations[i]);
    ros::param::get(paramNameStart+dimNames[i]+"_map_max", maximumLocations[i]);
  }

  // Mapping-Specific
  ros::param::get("/mapping/rgb_topic" , rgbTopic    );
  ros::param::get("/mapping/diff_topic", changesTopic);

  return;
}

void Mapping::GetSensorParameters(
    string& objective, int& numRobotsTotal, vector<Robot>& robotVec, int& numSensorsTotal){

  // or here

  // Allocated Memory
  sensors = vector<Mapping::Sensor>(numSensorsTotal);

  int sensorNumber(0), sensorIndex;
  string sensorIdentifier, paramNameStart, sensorParamStart;
  for(int iRobot(0); iRobot < numRobotsTotal; iRobot++){
    for(int iSensor(0); iSensor < robotVec[iRobot].numSensors; iSensor++){

      // Sensor-Robot Association
      sensors[sensorNumber].robotIndex = iRobot;

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
                      sensors[sensorNumber].name );
      sensors[sensorNumber].name
          = "Robot "+to_string(iRobot)+" "+sensors[sensorNumber].name  ;
      ros::param::get(sensorParamStart+"topic",
                      sensors[sensorNumber].topic);
      sensors[sensorNumber].topic
          = robotVec[iRobot].ns+"/"+sensors[sensorNumber].topic;
      ros::param::get(sensorParamStart+"frame",
                      sensors[sensorNumber].frame);
      sensors[sensorNumber].frame
          = robotVec[iRobot].ns+"/"+sensors[sensorNumber].frame;

      // Properties
      sensorParamStart = paramNameStart+"properties/";
      ros::param::get(sensorParamStart+"min_range"    , sensors[sensorNumber].minRange   );
      ros::param::get(sensorParamStart+"max_range"    , sensors[sensorNumber].maxRange   );
      ros::param::get(sensorParamStart+"sigma"        , sensors[sensorNumber].sigma      );
      ros::param::get(sensorParamStart+"prob_hit"     , sensors[sensorNumber].probHit    );
      ros::param::get(sensorParamStart+"prob_rand"    , sensors[sensorNumber].probRand   );
      ros::param::get(sensorParamStart+"prob_max_free", sensors[sensorNumber].probMaxFree);
      ros::param::get(sensorParamStart+"prob_max_occ" , sensors[sensorNumber].probMaxOcc );
      ros::param::get(sensorParamStart+"consider_max" , sensors[sensorNumber].considerMax);
      ros::param::get(sensorParamStart+"num_max_skip" , sensors[sensorNumber].numMaxSkips);
      ros::param::get(sensorParamStart+"has_colors"   , sensors[sensorNumber].hasColors  );

      // Timers
      ros::param::get(paramNameStart+"print_timers", sensors[sensorNumber].printInfo);

      sensorNumber++;
    }
  }

  return;
}

void Mapping::BasicMappingInit(string& objective){

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
  if(objective == "mapping"){
    int numElementsPerCell = 1;
    string addedLabelInfo = "";
    MappingToMultiArray(
        numElementsPerCell, addedLabelInfo, occData, initP);
    totalEntropy = numCellsTotal*EntropySingleCell(initP);
  }

  return;
}

void Mapping::SensorUpdateInit(int& numSensorsTotal){

  // either here
  // Sensor Information & Memory Allocation
  for(int i(0); i < numSensorsTotal; i++){

    // Zero Location for TFs
    sensors[i].pclZeroPtStamped.point.x = 0.0;
    sensors[i].pclZeroPtStamped.point.y = 0.0;
    sensors[i].pclZeroPtStamped.point.z = 0.0;

    // Preallocated Memory for Unit-Vectors Toward Other Robots
    sensors[i].uvecToOtherRobots.resize(sensors.back().robotIndex, vector<double>(dimension));

    // Preallocated Memory for Inverse Sensor Model
    sensors[i].mapEstRay.   resize(maxNumCellsAlongRay);
    sensors[i].sensorRanges.resize(maxNumCellsAlongRay);
    sensors[i].cellIndices. resize(maxNumCellsAlongRay);

    // Preallocated Memory for Ray Casting
    sensors[i].rayCastCellIndices.resize(dimension, vector<int>(maxNumCellsAlongRay));
    sensors[i].rayCastCellDepths .resize(dimension, vector<double>(maxNumCellsAlongRay));

    // Maximum Reading Skips
    sensors[i].numCurSkips = 0;

    // Timers
    sensors[i].numSamplesTaken = 0;

    // Normalizing
    double probDenom(sensors[i].probHit+sensors[i].probRand);
    sensors[i].probHit  /= probDenom;
    sensors[i].probRand /= probDenom;
  }

  // Make Uncolored Occupied Cells Grey (e.g. PointCloud2 generated from Laser Scan)
  defaultColor.resize(dimension);
  for(int i(0); i < dimension; i++)
    defaultColor[i] = 128;

  return;
}


// ROS Publishing/Subscribing

void Mapping::TFCallbackMapDegradation(const tf2_msgs::TFMessage::ConstPtr& msg){

  if(!lockFullMapThread){
    lockFullMapThread = true;

    double tNow = ros::Time::now().toSec();
    DegradeMap(tDegradedLast, tNow, dgrdLamda);
    tDegradedLast = tNow;
    mapProbsPub.publish(occData);

    lockFullMapThread = false;
  }

  return;
}

void Mapping::TFCallbackMapEntropy(const tf2_msgs::TFMessage::ConstPtr& msg){

  if(!lockEntropyThread){
    lockEntropyThread = true;

    // Sum Entropies Through Full Map
    totalEntropy = 0.0;
    for(int i(0); i < numCellsTotal; i++)
      totalEntropy += EntropySingleCell(occData.data[i]);

    // Publish Full Map Entropy
    totalEntropyMsg.data = totalEntropy;
    mapEntropyPub.publish(totalEntropyMsg);

    lockEntropyThread = false;
  }

  return;
}


void Mapping::PointCloud2TFFilterCallback(
    const boost::shared_ptr<const sensor_msgs::PointCloud2 >& scanInput, int& sensorNumber){

  // Variables Designed for a Maximum of 1 Thread Per Sensor
  if(lockSensor[sensorNumber])
    return;
  else
    lockSensor[sensorNumber] = true;

  // Sensor Header (time from message, header from param)
  std_msgs::Header header = scanInput->header;
  header.frame_id = sensors[sensorNumber].frame;

  // Initialize Measurement Number, Locations, & Colors
  int numRays; bool filterRays; vector<int> filteredRayInds;
  vector< vector<double> > meas ;
  vector< vector<int>    > color;

  // Conversions Required for PointCloud2 (colors or no colors)
  pcl::PCLPointCloud2 scan;
  pcl_conversions::toPCL(*scanInput, scan);
  if(sensors[sensorNumber].hasColors){
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    numRays = ExtractPCL2Locations(scan, cloud, meas, color, filterRays, filteredRayInds);
    for(int i = 0; i < numRays; i++){
      if(!filterRays)
        color[i] = {
          cloud.points[i]                 .r,
          cloud.points[i]                 .g,
          cloud.points[i]                 .b};
      else
        color[i] = {
          cloud.points[filteredRayInds[i]].r,
          cloud.points[filteredRayInds[i]].g,
          cloud.points[filteredRayInds[i]].b};
    }
  }
  else{// no colors
    pcl::PointCloud<pcl::PointXYZ> cloud;
    numRays = ExtractPCL2Locations(scan, cloud, meas, color, filterRays, filteredRayInds);
  }

  // Update the Map with Measurent Scan Ray-by-Ray & Time the Process
  UpdateMap(sensorNumber, numRays, meas, color, numRobots, header);

  // Unlock Sensor for Future Scan Considerations
  lockSensor[sensorNumber] = false;

}


// ROS Conversions

template<typename cloudType>
int Mapping::ExtractPCL2Locations(
    pcl::PCLPointCloud2& scan, cloudType& cloud, vector< vector<double> >& meas, vector< vector<int> >& color, bool& filterRays, vector<int>& filteredRayInds){

  // Extract Location/Color Data
  pcl::fromPCLPointCloud2(scan, cloud);
  int numRaysTotal = cloud.points.size();// cloud.width*cloud.height;

  // Ray Filtering Initialization
  int numRaysToConsider = rayNumMax;
  int percentRaysKeep;
  if(numRaysTotal > numRaysToConsider){
    filterRays = true;
    percentRaysKeep = ceil(100.0*numRaysToConsider/numRaysTotal);
  }
  else{
    filterRays = false;
    percentRaysKeep = 100;
    numRaysToConsider = numRaysTotal;
  }

  if(!filterRays){
    meas .resize(numRaysTotal, vector<double>(dimension));
    color.resize(numRaysTotal, vector<int>(dimension))   ;
    for(int i = 0; i < numRaysTotal; i++){
      meas[i] = {cloud.points[i].x, cloud.points[i].y, cloud.points[i].z};/* TODO: remove this:
      if(!(cloud.points[i].x >= 0.0 && cloud.points[i].x <= 100.0) || !(cloud.points[i].y >= 0.0 && cloud.points[i].y <= 100.0) || !(cloud.points[i].z >= 0.0 && cloud.points[i].z <= 100.0))
        cout << "cloud.points[" << i << "] = (" << cloud.points[i].x << "," << cloud.points[i].y << "," << cloud.points[i].z << ")" << endl;*/
    }
  }
  else{
    int indRayFilter(0), indRayAll(rand() % numRaysTotal);
    meas .resize(numRaysToConsider, vector<double>(dimension));
    color.resize(numRaysToConsider, vector<int>(dimension))   ;
    filteredRayInds.resize(numRaysToConsider);
    while(indRayFilter != numRaysToConsider){

      // Cycle Back
      if(indRayAll == numRaysTotal)
        indRayAll = 0;

      // Use or Neglect Ray
      if(!filterRays || percentRaysKeep > rand() % 100){
        meas [indRayFilter] = {cloud.points[indRayAll].x, cloud.points[indRayAll].y, cloud.points[indRayAll].z};
        filteredRayInds[indRayFilter] = indRayAll;
        indRayFilter++;
      }

      // Increase Ray Index
      indRayAll++;

    }
    numRaysToConsider = indRayFilter;// remove after cyclic above
  }

  return numRaysToConsider;
}

template<typename mapType, typename mapDataType>
void Mapping::MappingToMultiArray(int& numElementsPerCell, string& addedLabelInfo, mapType& mapData, mapDataType& initVal){

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

void Mapping::MapLocToDimensionalIndex(vector<double>& mapLoc, vector<int>& dimInd){

  for(int i(0); i < dimension; i++)
    dimInd[i] = floor((mapLoc[i]-minimumLocations[i])/alpha+0.5);

  return;
}

void Mapping::UpdateCellProbabilitiesAndColors(
    bool& ISMSuccess, vector<int>& color, int& numCellsAlongRay, int& sensorNumber){
  bool colorFirstCell = true;
  int mapInd, colorInd;
//  cout << "Cell color: ("
//       << color[0] << ","
//       << color[1] << ","
//       << color[2] << ")" << endl;
  if(ISMSuccess){
    for(int i(0); i < numCellsAlongRay; i++){
      mapInd = sensors[sensorNumber].cellIndices[i];
      if(mapInd != -1){// (check)
        occData.data[mapInd] = sensors[sensorNumber].mapEstRay[i];
        colorInd = 3*mapInd;// 3: R,G,B
        if(sensors[sensorNumber].mapEstRay[i] >= occThresh){
          if(colorFirstCell || rgbColors.data[colorInd] == -1){// first occupied cell or occupied & uncolored
            if(sensors[sensorNumber].hasColors){
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

void Mapping::UpdateCellsWithExpectedRays(vector<double>& sensorLoc, vector<double>& rayUVec){

  // Initializations
  int sensorNumber(0);
  double probRayReachesFirstCell;

  int numCellsAlongRay = RayCasting(
        sensorLoc, rayUVec, sensorNumber, probRayReachesFirstCell);

  // Cell Probabilities Along Casted Ray
  for(int i(0); i < numCellsAlongRay; i++)
    sensors[sensorNumber].mapEstRay[i] = occData.data[sensors[sensorNumber].cellIndices[i]];

  // Detection Probabilities
  int numMeasOutcomes(numCellsAlongRay+1);
  vector<double> detectionProbability(numMeasOutcomes);
  FindDetectionProbabilities(detectionProbability, numCellsAlongRay, sensorNumber, probRayReachesFirstCell);

  // Expected Value of Ray Depth
  double weightedRayDepth(0.0), weightingProbTotal(0.0);
  for(int i(0); i < numCellsAlongRay; i++){
    weightedRayDepth += detectionProbability[i]*sensors[sensorNumber].sensorRanges[i];
    weightingProbTotal += detectionProbability[i];
  }
  weightedRayDepth /= weightingProbTotal;

  // Inverse Sensor Model
  bool ISMSuccess = RayInverseSensorModel(
        weightedRayDepth, numCellsAlongRay, sensorNumber, probRayReachesFirstCell);

  // Probability Insertion
  if(ISMSuccess){
    for(int i(0); i < numCellsAlongRay; i++)
      occData.data[sensors[sensorNumber].cellIndices[i]];
  }

  return;
}

vector<double> Mapping::Sensor::SensorLocRelToOtherRobots(int& numRobots, std_msgs::Header& header, string& frame){

  // Sensor Location
  pclZeroPtStamped.header = header;
  bool done(false);
  while(!done){
    try{
      tfListenerRobotUvecs.transformPoint(frame, pclZeroPtStamped, sensorPoseStamped);
      done = true;
    }
    catch (tf::TransformException ex){
      ROS_WARN("%s",ex.what());
      ros::Duration(0.01).sleep();
    }
  }
  vector<double> sensorLoc = {sensorPoseStamped.point.x,
                              sensorPoseStamped.point.y,
                              sensorPoseStamped.point.z};

  // Determine Directions Toward Other Robots
  int otherRobotInd(0);
  for(int iRobot(0); iRobot < numRobots; iRobot++){
    if(iRobot != robotIndex){

      // Other Robot Location
      pclZeroPtStamped.header.frame_id = otherRobotFrames[otherRobotInd];
      bool done(false);
      while(!done){
        try{
          tfListenerRobotUvecs.transformPoint(frame, pclZeroPtStamped, sensorPoseStamped);
          done = true;
        }
        catch (tf::TransformException ex){
          ROS_WARN("%s",ex.what());
          ros::Duration(0.01).sleep();
        }
      }
      vector<double> otherRobotLoc = {sensorPoseStamped.point.x,
                                      sensorPoseStamped.point.y,
                                      sensorPoseStamped.point.z};

      // Unit Vector from Sensor to Other Robot
      double otherRobotDistAway;
      uvecToOtherRobots[otherRobotInd]
          = GetUVec(sensorLoc, otherRobotLoc, otherRobotDistAway);

      // Max Dot Product Between This & Measurement Ray Direction
      if(otherRobotDistAway > otherRobotRadii[otherRobotInd])
        maxDotProdToOtherRobots[otherRobotInd] = 1.0
            -pow(otherRobotRadii[otherRobotInd], 2)
            /pow(otherRobotDistAway, 2);
      else// inside other robot's collision zone: neglect all rays
        maxDotProdToOtherRobots[otherRobotInd] = -1.0;
      otherRobotInd++;
    }
  }

  return sensorLoc;
}

bool Mapping::Sensor::MeasRayOK(vector<double>& sensorLoc, vector<double>& meas, vector<double>& uVec, double& depth, int& numRobots){

  // Measurement Range & Direction
  uVec = GetUVec(sensorLoc, meas, depth);

  // Check Range
  if(isnan(depth) || depth >= maxRange){
    if(numCurSkips < numMaxSkips){
      numCurSkips++;// skipping this maximum reading
      return false;
    }
    else
      numCurSkips = 0;// acceptable maximum reading, continue...
    depth = maxRange;
  }
  else{
    for(int otherRobotInd(0); otherRobotInd < numRobots-1; otherRobotInd++){
      if(DotProd(uVec, uvecToOtherRobots[otherRobotInd])
         > maxDotProdToOtherRobots[otherRobotInd])
        return false;
    }
  }

  return true;
}

void Mapping::FindCellsComposingBlock3D(vector<double>& minCorner, vector<int>& numCellsThisDim,vector<int>& indsToUpdate){

  // Note: cheaper option for large blocks in 3D
  int blockIndex(0);
  vector<double> testLoc(3);
  for(int iX(0); iX < numCellsThisDim[0]; iX++){
    testLoc[0] = minCorner[0]+iX*alpha;
    for(int iY(0); iY < numCellsThisDim[1]; iY++){
      testLoc[1] = minCorner[1]+iY*alpha;
      for(int iZ(0); iZ < numCellsThisDim[2]; iZ++){
        testLoc[2] = minCorner[2]+iZ*alpha;
        indsToUpdate[blockIndex] = IndFromMapLocation(testLoc);
        blockIndex++;
      }
    }
  }

  return;
}


void Mapping::FindCellsComposingBlock(vector<double>& testLoc,
                                             vector<double>& minCorner, vector<int>& numCellsThisDim,
                                             vector<int>& indsToUpdate, int& blockIndex, int iDim){

  // Note: general for any dimension, but can be expensive with large block sizes

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

void Mapping::UpdateMap(
    int& sensorNumber, int& numRays, vector< vector<double> >& meas, vector< vector<int> >& color, int& numRobots, std_msgs::Header& header){

  // Initializations
  double depth, probRayReachesFirstCell, entropyRayStart(0.0), entropyRayEnd(0.0), entropyChange;
  vector<double> minCorner = maximumLocations, maxCorner = minimumLocations;
  bool cellsUpdated(false);
  double timeRayCast, durRayCast(0.0);
  int ind;
  vector<double> uVec(dimension);

  // Start Time
  if(sensors[sensorNumber].printInfo){sensors[sensorNumber].tScanUpdateStart = ros::Time::now().toSec();}

  // Sensor Location and Vector Neighborhoods to Avoid for Other Robots
  vector<double> sensorLoc = sensors[sensorNumber].SensorLocRelToOtherRobots(numRobots, header, frame);

  // Update Map from Scan Ray-by-Ray
  for(int i(0); i < numRays; i++){

    // Get Range & Direction, Basic Measurement Checks
    if(sensors[sensorNumber].MeasRayOK(sensorLoc, meas[i], uVec, depth, numRobots)){

      // Ray Casting
      timeRayCast = ros::Time::now().toSec();
      int numCellsAlongRay = RayCasting(
            sensorLoc, uVec, sensorNumber, probRayReachesFirstCell);
      durRayCast += ros::Time::now().toSec()-timeRayCast;

      // Sensors Only Provide Accurate Measurements Beyond Some Minimum Threshold
      if(probRayReachesFirstCell >= minProbISM){
        cellsUpdated = true;

        // Cell Probabilities Along Casted Ray
        for(int i(0); i < numCellsAlongRay; i++)
          sensors[sensorNumber].mapEstRay[i] = occData.data[sensors[sensorNumber].cellIndices[i]];

        // Initial Ray Entropies (1 thread only)
        if(trackH && numThreads == 1){
          for(int i(0); i < numCellsAlongRay; i++)
            entropyRayStart += EntropySingleCell(sensors[sensorNumber].mapEstRay[i]);
        }

        // Inverse Sensor Model
        bool ISMSuccess = RayInverseSensorModel(
              depth, numCellsAlongRay, sensorNumber, probRayReachesFirstCell);

        // Update Map Probabilities & Colors
        UpdateCellProbabilitiesAndColors(
              ISMSuccess, color[i], numCellsAlongRay, sensorNumber);

        // Final Ray Entropies (1 thread only)
        if(trackH && numThreads == 1){
          for(int i(0); i < numCellsAlongRay; i++)
            entropyRayEnd += EntropySingleCell(sensors[sensorNumber].mapEstRay[i]);
        }

        // Update Scan Min & Max Box
        ind = 0;                  ChangedCellLimits(ind, minCorner, maxCorner, sensorNumber);
        ind = numCellsAlongRay-1; ChangedCellLimits(ind, minCorner, maxCorner, sensorNumber);
      }
    }
  }

  // Only Publish & Analyze if Measurements Updated the Map
  if(cellsUpdated){

    // Generate Low-Memory Message of Map Changes
    ogm_ae::UpdatedMapCells updatedCells = ComposeChangedMapCellMsg(minCorner, maxCorner);

    // Publish the Message (Locked)
    while(true){
      if(!lockMapChangesMsg){
        lockMapChangesMsg = true;
        mapChangesPub.publish(updatedCells);
        lockMapChangesMsg = false;
        break;
      }
    }

    // End Time & Analysis
    sensors[sensorNumber].ScanUpdatePrintout(numRays, durRayCast);

    // Total Entropy Update & Message
    if(trackH && numThreads == 1){
      entropyChange = entropyRayEnd-entropyRayStart;
      totalEntropy += entropyChange;
      totalEntropyMsg.data = totalEntropy;
      mapEntropyPub.publish(totalEntropyMsg);
      if(sensors[sensorNumber].printInfo){
        cout
          << "  Entropy: "
          << "\n    Change:  " << entropyChange
          << "\n    Total:   " << totalEntropy
          << endl;
      }
    }

  }

  return;
}

void Mapping::Sensor::ScanUpdatePrintout(int& numRays, double& durRayCast){

  if(printInfo){
    tScanUpdateEnd = ros::Time::now().toSec();
    if(tScanUpdateEnd > tScanUpdateStart){// neglect loop-backs from bag files
      double durCurrent = tScanUpdateEnd-tScanUpdateStart;
      numSamplesTaken++;
      durAvg *= ((double)(numSamplesTaken-1)/numSamplesTaken);
      durAvg += durCurrent/numSamplesTaken;
      cout << "\n" << name << " sensor:"
           << "\n  Ray Count: " << numRays
           << "\n  Times:"
           << "\n    RayCast: " << durRayCast << " (" << durRayCast/durCurrent*100 << "%)"
           << "\n    Total:   " << durCurrent
           << "\n    Average: " << durAvg << endl;

    }
  }

  return;
}

ogm_ae::UpdatedMapCells Mapping::ComposeChangedMapCellMsg(
    vector<double>& minCorner, vector<double>& maxCorner, bool hasColors){

  // Initializations
  ogm_ae::UpdatedMapCells updatedCells;
  updatedCells.header.stamp = ros::Time::now();
  updatedCells.header.frame_id = frame;
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
  updatedCells.inds.resize(numCellsToUpdate);
  updatedCells.probs.resize(numCellsToUpdate);
  if(hasColors){updatedCells.colors.resize(3*numCellsToUpdate);}
  int iMapColor, iMsgColor, iRGB;
  for(int i(0); i < numCellsToUpdate; i++){

    // Probabilities
    updatedCells.inds [i] = indsToUpdate[i]              ;// cell indices
    updatedCells.probs[i] = occData.data[indsToUpdate[i]];// cell probabilities

    // Colors
    if(hasColors){
      iMapColor = 3*indsToUpdate[i];
      iMsgColor = 3*i;
      for(iRGB = 0; iRGB < 3; iRGB++)// 3 RGB Values
        updatedCells.colors[iMsgColor+iRGB] = rgbColors.data[iMapColor+iRGB];
    }
  }

  // Header
  updatedCells.header.stamp = ros::Time::now();
  updatedCells.header.frame_id = frame;

  return updatedCells;
}

void Mapping::ChangedCellLimits(int& ind, vector<double>& minCorner, vector<double>& maxCorner, int& sensorNumber){

  // Location of First/Last Cell Along Ray
  vector<double> testLoc = MapLocationFromInd(sensors[sensorNumber].cellIndices[ind]);

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

double Mapping::EnsureRayCastingInsideMapLimits(
    int& sensorNumber, vector<double>& origin, vector<double>& rayMaxLoc, vector<double>& uVec){

  // Maximum ray location without considering map limits
  for(int i(0); i < dimension; i++)
    rayMaxLoc[i] = origin[i]+uVec[i]*sensors[sensorNumber].maxRange;

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
  sensors[sensorNumber].maxRangeMap = correctionRatio*sensors[sensorNumber].maxRange;
  for(int i(0); i < dimension; i++)
    rayMaxLoc[i] = origin[i]+sensors[sensorNumber].maxRangeMap*uVec[i];

  return correctionRatio*sensors[sensorNumber].maxRange;
}

vector<double> Mapping::SnapLocationToMap(vector<double>& mapLoc){

  // Closest Index of mapLoc
  int ind = IndFromMapLocation(mapLoc);

  // True Index Location
  return MapLocationFromInd(ind);

}

vector<int> Mapping::FindCellEdges(
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
          distToCellEdge = sensors[sensorNumber].maxRange;
          break;
        }
        else{// Cell is within map limits & will be considered

          // Map Location of Intersection
          mapLoc = edgeLoc;
          mapLoc[iEdges] += 0.5*increment;// move from edge to center for intersecting edge for correct map index

          // Cell Edge is Too Close to Robot
          if(distToCellEdge < sensors[sensorNumber].minRange){
            probRayReachesFirstCell *= (1.0-ProbFromMapLocation(mapLoc));
          }

          // Cell Edge in FOV & Map
          else{

            // Save this cell information for further analysis
            sensors[sensorNumber].rayCastCellIndices[iEdges][rayIndices[iEdges]] = IndFromMapLocation(mapLoc);
            sensors[sensorNumber].rayCastCellDepths [iEdges][rayIndices[iEdges]] = distToCellEdge;

            if(sensors[sensorNumber].rayCastCellIndices[iEdges][rayIndices[iEdges]] < 0)
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

int Mapping::OrderRayCastCellsByIncreasingDistance(
    int& sensorNumber, vector<int>& numCellsAlongRayXYZ){

  // Initializations
  int numCellsAlongRay(0);
  for(int i(0); i < dimension; i++)
    numCellsAlongRay += numCellsAlongRayXYZ[i];

  vector<int> iXYZ = {0,0,0};// index of saved variables

  for(int k = 0; k < numCellsAlongRay; k++){

    int closestEdge(dimension);// edge identifier, dimension is 1 more than possible
    double dist(sensors[sensorNumber].maxRange);// distance to cell, initialized at maximum

    // Identify Closest Edge
    for(int i(0); i < dimension; i++){// cycle through each dimension
      if(iXYZ[i] < numCellsAlongRayXYZ[i]){
        if(dist > sensors[sensorNumber].rayCastCellDepths[i][iXYZ[i]]){
          dist = sensors[sensorNumber].rayCastCellDepths[i][iXYZ[i]];
          closestEdge = i;
        }
      }
    }

    // Insert Closest Edge as Next Element
    for(int i(0); i < dimension; i++){
      if(closestEdge == i){
        sensors[sensorNumber].cellIndices [k] = sensors[sensorNumber].rayCastCellIndices[i][iXYZ[i]];
        sensors[sensorNumber].sensorRanges[k] = sensors[sensorNumber].rayCastCellDepths [i][iXYZ[i]];
        iXYZ[i]++;
        break;
      }
    }
  }

  return numCellsAlongRay;
}

int Mapping::RayCasting(
    vector<double>& origin, vector<double>& uVec, int& sensorNumber, double& probRayReachesFirstCell){

  // Ensure Final Ray Location (rayMaxLoc) Inside Map Limits at Distance maxRangeRay
  vector<double> rayMaxLoc(dimension);
  double maxRangeRay = EnsureRayCastingInsideMapLimits(sensorNumber, origin, rayMaxLoc, uVec);

  // Determine Cell Edges
  vector<int> numCellsAlongRayXYZ = FindCellEdges(sensorNumber, origin, rayMaxLoc, uVec, maxRangeRay, probRayReachesFirstCell);

  // Order Cells Intersected by Ray by Increasing Distance
  return OrderRayCastCellsByIncreasingDistance(sensorNumber, numCellsAlongRayXYZ);
}

void Mapping::FindDetectionProbabilities(
    vector<double>& detectionProbability, int& numCellsAlongRay, int& sensorNumber, double probabilityRayReachesCell){
  for(int k = 0; k < numCellsAlongRay; k++){
    detectionProbability[k] = probabilityRayReachesCell*sensors[sensorNumber].mapEstRay[k];
    probabilityRayReachesCell *= (1.0-sensors[sensorNumber].mapEstRay[k]);
  }
  detectionProbability[numCellsAlongRay] = probabilityRayReachesCell;

  return;
}

double Mapping::GetUnnormalizedProbabilities(vector<double>& detectionProbability,
      vector<double>& unnormalizedOccupancyProbabilities, double& z, int& numCellsAlongRay, int& sensorNumber){

  double addedTermBuff, invNormalizer(0.0);
  for(int k = 0; k < numCellsAlongRay; k++){
    addedTermBuff = detectionProbability[k]
            *ForwardSensorModel(z, sensors[sensorNumber].sensorRanges[k], sensorNumber);
    unnormalizedOccupancyProbabilities[k] = sensors[sensorNumber].mapEstRay[k]*invNormalizer+addedTermBuff;
    invNormalizer += addedTermBuff;
  }
  if(sensors[sensorNumber].considerMax){
    invNormalizer += detectionProbability[numCellsAlongRay]
            *ForwardSensorModel(z, sensors[sensorNumber].maxRange, sensorNumber);
  }

  return invNormalizer;
}

bool Mapping::NormalizeProbabilities(
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

bool Mapping::CheckInsideFOV(double& z, int& sensorNumber){

  // Ray Check if Min Reading
  if(z < sensors[sensorNumber].minRange)
    return false;

  // Ray Check if Beyond FOV
  if(z >= sensors[sensorNumber].maxRangeMap || isnan(z) || isinf(z)){
    if(sensors[sensorNumber].considerMax){
      z = sensors[sensorNumber].maxRange;
      return true;
    }
    else
      return false;
  }
  return true;
}

bool Mapping::RayInverseSensorModel(// P(cell occupancies along ray | pose, measurement)
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
  return NormalizeProbabilities(invNormalizer, unnormalizedOccupancyProbabilities, sensors[sensorNumber].mapEstRay, numCellsAlongRay, sensorNumber);
}

double Mapping::ForwardSensorModel(// p(measurement | pose, first occupied cell)
    double& z, double& zExpectedValue, int& sensorNumber){
  if(z < sensors[sensorNumber].maxRange){// sensor reading inside FOV -> occupied cell or phantom reading
    if(zExpectedValue < sensors[sensorNumber].maxRange)// considering an occupied voxel inside FOV
      return sensors[sensorNumber].probHit*exp(-pow(z-zExpectedValue, 2)
             /(2*sensors[sensorNumber].sigma*sensors[sensorNumber].sigma))/(sensors[sensorNumber].sigma*sqrt2pi)// hit cell (Gaussian)
            +sensors[sensorNumber].probRand/(sensors[sensorNumber].maxRange-sensors[sensorNumber].minRange)// phantom reading (uniform)
            ;
    else// empty map case: impossible with sensor reading in FOV, COMMENTED OUT: is a phantom reading
      return 0.0;//sensors[sensorNumber].probRand/(sensors[sensorNumber].maxRange-sensors[sensorNumber].minRange);
  }
  else{// sensor returns a max reading
    if(zExpectedValue < sensors[sensorNumber].maxRange)// P(z = sensors[sensorNumber].maxRange | occupied space in FOV)
      return sensors[sensorNumber].probMaxFree;
    else// P(z = sensors[sensorNumber].maxRange | no occupied space in FOV)
      return sensors[sensorNumber].probMaxOcc;
//    return sensors[sensorNumber].probHit/(sensors[sensorNumber].sigma*sqrt2pi)+sensors[sensorNumber].probRand/(sensors[sensorNumber].maxRange-sensors[sensorNumber].minRange);
  }
}

int Mapping::IndFromMapLocation(vector<double>& mapLoc){

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

double Mapping::ProbFromMapLocation(vector<double>& mapLoc){

  return occData.data[IndFromMapLocation(mapLoc)];

}


vector<double> Mapping::MapLocationFromInd(int ind){

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

vector<int> Mapping::DimensionalIndicesFromMapIndex(int ind){

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

double Mapping::SingleRayExpectedEntropyChange(vector<double>& sensorLoc, vector<double>& rayUVec, int numCellsToConsider){

  // Initializations
  double rayEntropyChange(0.0), probRayReachesFirstCell;
  int sensorNumber(0);
  double probMeasHitsAnyCells;
  int numCasesToConsider;

  // Ray Casting (NOTE: this can be removed with initial ray cast shifted to any cell, not coded yet)
  int numCellsAlongRay = RayCasting(
        sensorLoc, rayUVec, sensorNumber, probRayReachesFirstCell);//, measOutsideMapLimits);


  // Cell Probabilities Along Casted Ray
  for(int i(0); i < numCellsAlongRay; i++)
    sensors[sensorNumber].mapEstRay[i] = occData.data[sensors[sensorNumber].cellIndices[i]];


//  // Uncomment Below for Frontiers-Only Exploration

//  bool isFrontier(false); bool hitAWall(false);
//  for(int i(0); i < numCellsAlongRay; i++){
//    if(sensors[sensorNumber].mapEstRay[i] > 0.05){
//      if(sensors[sensorNumber].mapEstRay[i] < 0.95){
////        cout << "YES Frontier: [][][][?]" << endl;
////        return -0.5;
//        isFrontier = true;
//      }
//      else{
////        cout << "NO Frontier:  [][][][x]" << endl;
//        hitAWall = true;
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

  // Init Stuff
  bool reducedEntropyCalc = true;

  if(reducedEntropyCalc){

    // Only Consider the Ray with Sufficient Cells
    if(numCellsToConsider > numCellsAlongRay)
      return 0.0;

    // Pairs of Ray Indices and Detection Probabilities
    pair<int,double> element;
    vector<pair<int,double> > detectProbPairs(numCellsAlongRay);
    for(int i(0); i < numCellsAlongRay; i++){
      element.first = i;
      element.second = detectionProbability[i];
      detectProbPairs[i] = element;
    }

    // Move the n-th Largest Probability to Index 'indOfNthElement' with Larger Elements to the Left with Amortized O(numCellsAlongRay)
    int indOfNthElement = numCellsToConsider-1;
    nth_element(detectProbPairs.begin(), detectProbPairs.begin()+indOfNthElement, detectProbPairs.end(), PairIntDoubleSecondDoubleGreater);

    // Remove Unused Elements at the Vector End & Sort with Amortized O(numCellsToConsider*log(numCellsToConsider))
    detectProbPairs.resize(numCellsToConsider);
    sort(detectProbPairs.begin(), detectProbPairs.end(), PairIntDoubleFirstIntLess);

//    cout << "Among top " << numCellsToConsider << ":" << endl;
//    for(int i(0); i < detectProbPairs.size(); i++)
//      cout << detectProbPairs[i].first << ", ";

    // Update Vectors with Reduced Data
    numCellsAlongRay   = numCellsToConsider;
    numCasesToConsider = numCellsToConsider;
    probMeasHitsAnyCells = 1.0;
    int rayInd;
    for(int i(0); i < numCellsToConsider; i++){

      // Full Ray Index
      rayInd = detectProbPairs[i].first;

      // Map Index
      sensors[sensorNumber].cellIndices [i] = sensors[sensorNumber].cellIndices [rayInd];

      // Distance
      sensors[sensorNumber].sensorRanges[i] = sensors[sensorNumber].sensorRanges[rayInd];

      // Probability
      sensors[sensorNumber].mapEstRay   [i] = detectProbPairs[i].second;

      // Running Total of Ray Hitting Any Cells
      probMeasHitsAnyCells -= detectProbPairs[i].second;

    }
  }
  else{
    probMeasHitsAnyCells = probRayReachesFirstCell-detectionProbability.back();// reaches first & not a max reading
    numCasesToConsider = numMeasOutcomes-1;
  }

  // Unnormalized Probabilities & Normalizer
  vector< vector<double> > unnormalizedCellProbabilitiesMatrix = vector< vector<double> >(numCasesToConsider, vector<double>(numCasesToConsider));
  vector< vector<double> > normalizedCellProbabilitiesMatrix = unnormalizedCellProbabilitiesMatrix;
  vector<double> invNormalizers(numCasesToConsider, 0.0), measProbs(numCasesToConsider);
  for(int iMeas = 0; iMeas < numCasesToConsider; iMeas++){
    double zMeas = sensors[sensorNumber].sensorRanges[iMeas];
    invNormalizers[iMeas] = GetUnnormalizedProbabilities(
          detectionProbability, unnormalizedCellProbabilitiesMatrix[iMeas], zMeas, numCellsAlongRay, sensorNumber);

    // Normalize Expected Cell Probabilities Synergistically
    if(!NormalizeProbabilities(invNormalizers[iMeas], unnormalizedCellProbabilitiesMatrix[iMeas], normalizedCellProbabilitiesMatrix[iMeas], numCellsAlongRay, sensorNumber))
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
    for(int iMeas = 0; iMeas < numCellsAlongRay; iMeas++){
      rayEntropyChange += measProbs[iMeas]*EntropySingleCell(normalizedCellProbabilitiesMatrix[iMeas] [iCell]);
//      rayEntropyChange += measProbs[iMeas]*KaufmanUncertaintySingleCell(normalizedCellProbabilitiesMatrix[iMeas] [iCell], initP);
    }

    // Cell's Entropy Without Any New Measurements
    rayEntropyChange -= EntropySingleCell(sensors[sensorNumber].mapEstRay[iCell]);
//    rayEntropyChange -= KaufmanUncertaintySingleCell(sensors[sensorNumber].mapEstRay[iCell], initP);


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

void Mapping::OccupancyGridMsgInit(nav_msgs::OccupancyGrid& ogm, double& mapHeight){

  // Assign map parameters to occupancy grid message
  ogm.header.frame_id = frame;
  ogm.info.resolution = alpha;
  ogm.info.width      = numCellsEachDimension[0];
  ogm.info.height     = numCellsEachDimension[1];
  ogm.info.origin.position.x = minimumLocations[0]-0.5*alpha;
  ogm.info.origin.position.y = minimumLocations[1]-0.5*alpha;
  ogm.info.origin.position.z = mapHeight;
  ogm.data.resize(ogm.info.width*ogm.info.height);

  return;
}

void Mapping::UpdateChangedMapCells(){

  int iMapInd;

  // Update Potentially Changed Cells of Full Map Only
  for(int i(0); i < changedCells.inds.size(); i++){
    iMapInd = changedCells.inds[i];
    occData.data[iMapInd] = changedCells.probs[i];
  }

  return;
}

void Mapping::DegradeMap(double& lastTime, double& currTime, double& lam){
  double decay(exp(-lam*(currTime-lastTime)));// modify to not hard coded
  double addedTerm(initP*(1.0-decay));
  for(int i(0); i < occData.data.size(); i++)
    occData.data[i] = occData.data[i]*decay+addedTerm;
  return;
}

void Mapping::RvizMarkerArrayMsgInit(
    visualization_msgs::Marker& marker, visualization_msgs::MarkerArray& markerBuff,
    double& occupancyVizMin, double& probRange){
    double occupancyVizMax;

    // Marker
    marker.header.frame_id = frame;
    marker.ns = "map";
    marker.scale.x = alpha;
    marker.scale.y = alpha;
    marker.scale.z = alpha;
    marker.lifetime = ros::Duration();
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.color.a = 0.0;

    // Marker Array Buffer
    markerBuff.markers.resize(numCellsTotal);

    // Probability Range
    ros::param::get("/visualization/occ_viz_min" , occupancyVizMin);
    ros::param::get("/visualization/occ_viz_max" , occupancyVizMax);
    probRange = occupancyVizMax-occupancyVizMin;

  return;
}

bool Mapping::GenerateRvizMarkerArrayMsg(ogm_ae::UpdatedMapCells& possiblyChangedCells,
    visualization_msgs::Marker& marker, visualization_msgs::MarkerArray& markerArrayMsg, visualization_msgs::MarkerArray& markerBuff,
    double& occupancyVizMin, double& probRange, std_msgs::Float64MultiArray& rdcdMapOccDataBuff, bool checkForNewProb){

  // Initializations
  int iMsgColor, iMapInd, iRGB;
  double prob;
  vector<int> color(3);// RGB
  vector<double> mapLoc(3);

  // Update Potentially-Changed Cells
  int markerArrayInd(0);
  if(possiblyChangedCells.inds.size() > 0){// pass markers from those among message of differences
    marker.header.stamp = ros::Time::now();
    for(int i(0); i < possiblyChangedCells.inds.size(); i++){
      iMapInd = possiblyChangedCells.inds[i];
      prob = possiblyChangedCells.probs[i];
      if(!checkForNewProb || (prob >= occupancyVizMin && prob != rdcdMapOccDataBuff.data[iMapInd])){// visibility threshold
        iMsgColor = 3*i;
        for(iRGB = 0; iRGB < 3; iRGB++)// RGB Colors
          color[iRGB] = possiblyChangedCells.colors[iMsgColor+iRGB];
        GenerateRvizMapMarker(marker, iMapInd, mapLoc, probRange, occupancyVizMin, prob, color);
        markerBuff.markers[markerArrayInd] = marker;
        markerArrayInd++;
        if(checkForNewProb)
          rdcdMapOccDataBuff.data[iMapInd] = prob;
      }
    }
  }

  // Resize & Insert
  markerArrayMsg.markers.resize(markerArrayInd);
  for(unsigned i = 0; i < markerArrayInd; i++)
    markerArrayMsg.markers[i] = markerBuff.markers[i];

  // Return True if Elements Exist
  if(markerArrayInd > 0)
    return true;
  else
    return false;
}

void Mapping::GenerateRvizMapMarker(
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
