#ifndef robot_h
#define robot_h

#include "aux_fun.h"

using namespace std;
using namespace Eigen;

// Robots
class Robot{

public:

  Robot();
  Robot(string& objective, int& robotNum);

  // Basic Information
  string ns, robotFrame, exploreFrame;// exploreFrame specific to sensor
  double collisionRad;
  int numSensors;
  vector<double> sensorLoc, nextSensorLoc;
  geometry_msgs::Quaternion sensorAtt;
  std_msgs::ColorRGBA color;
  int indCurrent;

  // Exploration-Specific
  double speed;
  int bestCandInd;
  double entropyBumpMax;
  Eigen::Vector3d vecRobot2Camera_CameraFrame;
  Eigen::Matrix3d RotMatrixCameraRobot;
  Eigen::Vector3d vecRobot2Camera_RobotFrame;

  vector<bool> visited;
  vector<double> costMap;
  vector<double> candidateInfoGainWithBump;
  vector<double> candidateBumps;
  vector<double> candidateDiscounts;
  nav_msgs::Path camPosPath;
  nav_msgs::Path robPosPath;
  nav_msgs::Path robVelPath;
  nav_msgs::Path robAclPath;
  ogm_ae::PolyLeastSquaresTraj trajInfo;

  // ROS Message Topics
  string cameraPathTopic  ;
  string positionPathTopic;
  string desiredPoseTopic;
  string desiredCamTopic;
  string desiredTwistTopic;
  string desiredAccelTopic;
  string djkTopic;
  string costMapTopic;
  string trajInfoTopic;

  // ROS Handles
  ros::Publisher trajCoeffsPub         ;// Sufficient trajectory information
  ros::Publisher waypointsPub          ;// Dijkstra waypoints
  ros::Publisher cameraPositionPub     ;// camera path
  ros::Publisher desiredTrajPositionPub;// robot path
  ros::Publisher distCostMapPub        ;// candidates considering entropy & distance
  ros::Publisher costMapPub            ;// distance cost map
};

// Mapping
// Complete Map & Sensors
class Mapping{
public:

  Mapping(){}
  Mapping(string objective){
    if(objective == "mapping"){
      ros::param::get("num_robots", numRobots);
      numSensorsTotal = 0; robotVec.resize(0);
      for(int i(0); i < numRobots; i++){
        Robot robot(objective, i);
        numSensorsTotal += robot.numSensors;
        robotVec.push_back(robot);
      }
      robotVec.resize(numRobots);

      PubSubInit();
    }
  }

  virtual ~Mapping() = default;

  void PubSubInit();
  void GeneralInit(int& numSensorsTotal, int& numRobots, vector<Robot>& robotVec);

  // ROS
  ros::Publisher  mapProbsPub;  // publish cell probabilities
  ros::Publisher  mapRGBPub;    // publish cell RGB colors
  ros::Publisher  mapEntropyPub;// publish cell RGB colors
  ros::Publisher  mapChangesPub;// publish changed cells
  ros::Subscriber TFMapDegradation;
  ros::Subscriber TFMapEntropy;


  // Topics & Frames
  string occTopic, rgbTopic, changesTopic, HTopic;// publication topics
  string frame;             // inertial frame fixed occupancy grid maps
  bool trackDiff;// TODO: delete once updated throughtout, publish/cycle through changes only (scalable)

  // Numerical Parameters
  int dimension;// number of map dimensions
  std_msgs::Float64MultiArray occData;// cell probabilities
  std_msgs::Int16MultiArray rgbColors;// cell RGB colors
  ogm_ae::UpdatedMapCells changedCells;// changing cells
  vector<string> dimNames;// name of each dimension in order (e.g. 0->"x", 1->"y", 2->"z")
  int numCellsTotal;// total number of cells
  vector<int> numCellsEachDimension;// number of cells in {x,y,z}-direction
  vector<int> stride;// impact of a cell in each dimension on index
  int maxNumCellsAlongRay;// ray alloc size (over maximum case)
  double alpha;// cell edge length
  vector<double> minimumLocations, maximumLocations;// min/max {x,y,z}-locations
  double initP;// initial cell occupancy probabilities
  double occProbMin, occProbMax;// lower/upper saturation limit on any cell
  double minProbISM ;// minimum threshold probability to consider ISM

  vector<Robot> robotVec;

  int numThreads;
  bool lockFullMapThread, lockEntropyThread;
  double tDegradedLast;
  vector<bool> lockSensor;
  int numRobots, numSensorsTotal;





  // Visualization
  double occThresh;        // min occupancy probability for visualization
  vector<int> defaultColor;// RGB color to visualized cells generated from scans without color data

  // Misc. Params
  double sqrt2pi;// fixed constant in forward sensor model
  double totalEntropy;
  std_msgs::Float64 totalEntropyMsg;
  bool degrade;
  double dgrdLamda, dgrdHz;
  bool lockMapChangesMsg;
  bool trackH;
  int rayNumMax;

  // Sensors
  int numSensors;// total number of depth sensors
  class Sensor{
  public:

    // Naming
    string name ;// sensor name
    string topic;// message topic
    string frame;// reference frame

    // Properties
    double minRange   ;// minimum sensor range
    double maxRange   ;// maximum sensor range
    double sigma      ;// standard deviation
    double probHit    ;// probability reading inside range is a hit
    double probRand   ;// probability reading inside range is phantom
    double probMaxFree;// probability max reading means cell is free
    double probMaxOcc ;// probability max reading means cell is occupied
    bool   considerMax;// whether or not to consider a maximum reading
    bool   hasColors  ;// whether or not sensor provides colors

    // TF
    geometry_msgs::PointStamped cameraLocLocalFrame;
    geometry_msgs::PointStamped cameraLocWorldFrame ;

    // Skipping Maximum Readings (if sensor yields too many)
    int numMaxSkips;
    int numCurSkips;

    // Preallocated Memory for Inverse Sensor Model TODO: SET PRIVATE
    vector<double>   mapEstRay   ;// cell probabilities
    vector<double>   sensorRanges;// distances from sensor to closest cell edges
    vector<int> cellIndices ;// cell indices

    // Preallocated Memory for Ray Casting
    vector< vector<int> >    rayCastCellIndices;// mapDim dimensions of cell indices
    vector< vector<double> > rayCastCellDepths ;// mapDim dimensions of cell depths
    double maxRangeMap;// maximum sensor range within map limits

    // Robot Information
    int robotIndex;
    vector<string> otherRobotFrames;
    vector< vector<double> > uvecToOtherRobots;
    vector<double> otherRobotRadii;
    vector<double> maxDotProdToOtherRobots;

    // Timers
    bool     printInfo     ;// print info (e.g. timers) on/off
    double   tScanUpdateStart;// ROS time before scan update
    double   tScanUpdateEnd  ;// ROS time after scan update
    double   durAvg          ;// duration of average scan update
    int numSamplesTaken ;// number of scans used to update map

    // TF Filtering (unchanged for any sensor with msg type PointCloud2)
    tf::TransformListener tfListenerMsgFilter, tfListenerRobotUvecs;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub;
    tf::MessageFilter<sensor_msgs::PointCloud2> *tfFilter;
    geometry_msgs::PointStamped pclZeroPtStamped, sensorPoseStamped;// robot locations


    // Functions
    vector<double> SensorLocRelToOtherRobots(int& numRobots, std_msgs::Header& header, string& frame);
    bool MeasRayOK(vector<double>& sensorLoc, vector<double>& meas, vector<double>& uVec, double& depth, int& numRobots);
    void ScanUpdatePrintout(int& numRays, double& durRayCast);

  };
  vector<Sensor> sensors;// vector of struct for any number of sensors

  // __ Functions __ \\


    // ROS
    void TFCallbackMapDegradation(const tf2_msgs::TFMessage::ConstPtr& msg);
    void TFCallbackMapEntropy    (const tf2_msgs::TFMessage::ConstPtr& msg);
    void PointCloud2TFFilterCallback(// PointCloud2 & TF message filter & map update
        const boost::shared_ptr<const sensor_msgs::PointCloud2 >& scanInput, int& sensorNumber);

    template<typename cloudType>
    int ExtractPCL2Locations(// use PCL libraries to extract point cloud positions
        pcl::PCLPointCloud2& scan, cloudType& cloud, vector< vector<double> >& meas, vector< vector<int> >& color, bool& filterRays, vector<int>& filteredRayInds);



    // Indexing
    vector<double> MapLocationFromInd(int ind);// gets cell location from map index
    int IndFromMapLocation(vector<double>& mapLoc);// gets map index from cell location
    double ProbFromMapLocation(vector<double>& mapLoc);// gets map probability from cell location
    void MapLocToDimensionalIndex(vector<double>& mapLoc, vector<int>& dimInd);
    vector<int> DimensionalIndicesFromMapIndex(int ind);

    // Ray Casting
    int RayCasting(vector<double>& origin, vector<double>& uVec, int& sensorNumber, double& probRayReachesFirstCell);
    double EnsureRayCastingInsideMapLimits(int& sensorNumber, vector<double>& origin, vector<double>& rayMaxLoc, vector<double>& uVec);
    vector<int> FindCellEdges(int& sensorNumber, vector<double>& origin, vector<double>& rayMaxLoc, vector<double>& uVec, double& maxRangeRay, double& probRayReachesFirstCell);
    vector<double> SnapLocationToMap(vector<double>& mapLoc);
    int OrderRayCastCellsByIncreasingDistance(int& sensorNumber, vector<int>& numCellsAlongRayXYZ);

    // Sensor Modeling
    double ForwardSensorModel(double& z, double& zExpectedValue, int& sensorNumber);
    bool RayInverseSensorModel(double& z, int& numCellsAlongRay, int& sensorNumber, double& probRayReachesFirstCell);
    bool CheckInsideFOV(double& z, int& sensorNumber);
    void FindDetectionProbabilities(vector<double>& detectionProbability, int& numCellsAlongRay, int& sensorNumber, double probabilityRayReachesCell);
    double GetUnnormalizedProbabilities(vector<double>& detectionProbability, vector<double>& unnormalizedOccupancyProbabilities, double& z, int& numCellsAlongRay, int& sensorNumber);
    bool NormalizeProbabilities(double& invNormalizer, vector<double>& unnormalizedOccupancyProbabilities, vector<double>& normalizedOccupancyProbabilities, int& numCellsAlongRay, int& sensorNumber);

    // Map Updating
    void UpdateMap(int& sensorNumber, int& numRays, vector< vector<double> >& meas, vector< vector<int> >& color, int& numRobots, std_msgs::Header& header);
    void UpdateCellProbabilitiesAndColors(bool& ISMSuccess, vector<int>& color, int& numCellsAlongRay, int& sensorNumber);
    void ChangedCellLimits(int& ind, vector<double>& minCorner, vector<double>& maxCorner, int& sensorNumber);
    ogm_ae::UpdatedMapCells ComposeChangedMapCellMsg(vector<double>& minCorner, vector<double>& maxCorner, bool hasColors = true);
    void UpdateChangedMapCells();
    void DegradeMap(double& lastTime, double& currTime, double& lam);

    // Grouping Cells (fully-defined here for templates used outside this library), TODO: recursivity for single function
    template<typename valType>
    void SetValToSpaceAroundCellLoc2D(vector<double>& loc, int& reqNumFreeNeighborsX, int& reqNumFreeNeighborsY, valType inputVal, vector<valType>& saveVals){

      int mapInd = IndFromMapLocation(loc), indTest;
      for(int iX = -reqNumFreeNeighborsX; iX <= reqNumFreeNeighborsX; iX++){
        for(int iY = -reqNumFreeNeighborsY; iY <= reqNumFreeNeighborsY; iY++){
          indTest = mapInd+iX*stride[0]+iY*stride[1];
          if(indTest > -1 && indTest < numCellsTotal)
            saveVals[indTest] = inputVal;
        }
      }

      return;
    }
    template<typename valType>
    void SetValToSpaceAroundCellLoc3D(vector<double>& loc, int& reqNumFreeNeighborsX, int& reqNumFreeNeighborsY, int& reqNumFreeNeighborsZ, valType inputVal, vector<valType>& saveVals){

      int counter(0), negOnes(0), overmaxOnes(0);
      int mapInd = IndFromMapLocation(loc), indTest;
      for(int iX = -reqNumFreeNeighborsX; iX <= reqNumFreeNeighborsX; iX++){
        for(int iY = -reqNumFreeNeighborsY; iY <= reqNumFreeNeighborsY; iY++){
          for(int iZ = -reqNumFreeNeighborsZ; iZ <= reqNumFreeNeighborsZ; iZ++){
            indTest = mapInd+iX*stride[0]+iY*stride[1]+iZ*stride[2];
            if(indTest > -1 && indTest < numCellsTotal){
              counter++;
              saveVals[indTest] = inputVal;
            }
            else{
              if(indTest < 0)
                negOnes++;
              else
                overmaxOnes++;
            }
          }
        }
      }

      return;
    }

    // Recursive
    void FindCellsComposingBlock(vector<double>& testLoc, vector<double>& minCorner, vector<int>& numCellsThisDim, vector<int>& indsToUpdate, int& blockIndex, int iDim = 0);

    // Embedded Looping
    void FindCellsComposingBlock3D(vector<double>& minCorner, vector<int>& numCellsThisDim,vector<int>& indsToUpdate);

    // Parameters & Initializations
    void GetMapParameters(string& objective);
    void GetSensorParameters(string& objective, int& numRobotsTotal, vector<Robot>& robotVec, int& numSensorsTotal);
    void BasicMappingInit(string& objective);
    void SensorUpdateInit(int& numSensorsTotal);
    void OccupancyGridMsgInit(nav_msgs::OccupancyGrid& ogm, double& mapHeight);

    template<typename mapType, typename mapDataType>
    void MappingToMultiArray(int& numElementsPerCell, string& addedLabelInfo, mapType& mapData, mapDataType& initVal);

    // Visualization
    void GenerateRvizMapMarker(visualization_msgs::Marker& marker, int& mapInd, vector<double>& mapLoc, double& probRange, double& occupancyVizMin, double& prob, vector<int>& color);
    void RvizMarkerArrayMsgInit(visualization_msgs::Marker& marker, visualization_msgs::MarkerArray& markerBuff, double& occupancyVizMin, double& probRange);
    bool GenerateRvizMarkerArrayMsg(ogm_ae::UpdatedMapCells& possiblyChangedCells, visualization_msgs::Marker& marker, visualization_msgs::MarkerArray& markerArrayMsg, visualization_msgs::MarkerArray& markerBuff, double& occupancyVizMin, double& probRange, std_msgs::Float64MultiArray& rdcdMapOccDataBuff, bool checkForNewProb);

    // Exploration
    double SingleRayExpectedEntropyChange(vector<double>& sensorLoc, vector<double>& rayUVec, int numCellsToConsider);
    void   UpdateCellsWithExpectedRays   (vector<double>& sensorLoc, vector<double>& rayUVec);


};


#endif
