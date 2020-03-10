#ifndef Exploration_h
#define Exploration_h

using namespace std;
using namespace Eigen;

class Exploration{
//private:// TODO: polulate/organize
public:

  // Map Copies
  Mapping fullMap;
  Mapping rdcdMap;
  Mapping tempMap;
  std_msgs::Float64MultiArray rdcdMapOccDataBuff;


  // Initializations
  Exploration();
  Exploration(vector<Robot>& robotVec, int& numRobots);

  void RdcdMapParamsInit(Mapping& mapInput, vector<Robot>& robotVec, int& numRobots, int dimRdcdMap);
  void GetExplorationParameters(vector<Robot>& robotVec, int& numRobots);
  void ExplorationInit();
  void StartNode();

  // ROS
  ros::NodeHandle nh;
  ros::Publisher ogm2DPub, ogm3DPub;
  ros::Subscriber allMapProbsSub, changedMapProbsSub, TFSubRoboticMotions, TFSubVisualizeRdcdMap;

  void RoboticMotionsThreadCallback(const tf2_msgs::TFMessage::ConstPtr& msg);
  void GetSensorPose(int& robotNum);
  void MapProbFullReductionCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
  void MapProbChangesReductionCallback(const ogm_ae::UpdatedMapCells::ConstPtr& msg);
  void TFCallbackOccGridMsgPublish(const tf2_msgs::TFMessage::ConstPtr& msg);

  // From pubsub library
  int numRobots;
  vector<Robot> robotVec;
  string objective;

  bool explorationStarted;
  ros::Time rosTimeStartNextTraj;
  bool mapChanged;
  bool lockFullMapThread, lockExploreTrajThread, lockOGMPublish;
  int numThreads;
  double fullMapTransferTime;
  vector<double> timeProbsUpdated;

  vector<double> entropyBumpsAllRobots;
  tf::StampedTransform transformW2C, transformW2R, transformR2C;
  tf::TransformListener tfListener;
  Eigen::Matrix3d RotMatrixRobotCamera;
  Eigen::Matrix3d RotMatrixWorldRobot ;
  Eigen::Matrix3d RotMatrixWorldCamera;
  string entropyOccTopic, collOccTopic;
  string entropyMarkerTopic, distInfoMarkerTopic;
  nav_msgs::Path djkTraj, smoothTraj;
  int numRaysPerCandidateTotal, numRaysPerCandidateInZ, numRaysPerCandidateInY;
  int numRaysEachSideAboutZ;
  double angleFOVInZ, angleFOVInY, angleYStart, degreeCamPitch, radiansCamPitch, optimalUVecZComp, magnitudeXY;
  Eigen::Matrix3d RotMatrixMinPitchOptimalPitch;
  double discountRadius;
  double maxComputationTime;
  double optimalTravelDistance;
  double maxRadPerSec;
  int numSensorsTotal;

  // Information Gain
  int numCellsToConsider;
  int dimEntropyMap;
  bool useRdcdMapEntropy;
  double height, sensorHeight;
  int alphaXY;
  int alphaZ ;
  double acceptableCollisionProb;
  double tNow, tEndTraj, tInitTraj, trajDuration;// TODO: check/kill these
  bool initTraj;
  int numPosesDjk, numPosesSmoothTraj;
  vector<bool> freeCells;
  int numCandidatesTotal;
  // changes somewhere in these 3: asdf
  vector< vector<double> > rayUVecs;// same for each candidate location
  vector< vector<double> > optimalUVecs;// specific to each candidate
  vector<int> optimalUVecIndsAboutZ;// specific to each candidate

  double candidateSeparation;
  vector<int> candidateIndices;
  vector< vector<double> > candidateLocations;
  vector<double> candidateExpectedInfoGain;
  vector<bool> candidateDone;
  double minInfoGainThresh;
  vector<double> infoGainEachRayAboutZ;
  vector<double> infoGainEachDirectionAboutZ;// sum with neighboring rays
  double recomputeRadius;

  // Distance Bump
  double bumpValAtDist;// Gaussian value <= 1 at a distance away described below
  double bumpDist;// distance from robot current location
  double bumpSigSqrd;// Gaussian variance corresponding to 'bumpValAtDist' & 'bumpDist'
  double minBumpFunVal;
  double BMax, BFar, beta;

  // Path
  int reqNumFreeNeighborsX, reqNumFreeNeighborsY, reqNumFreeNeighborsZ;
  vector< vector<int> > pathwayBuff;
  vector<bool> visited, visitedUpdating;
  bool edgeCellsVisited;
  double costMapInitValue;
  double maxCostMapAsgndVal;
  geometry_msgs::PoseStamped fixedPoseStampedParams;
  double maxSpeed;
  double costClosestCell;
  double costDiagonal2D ;
  double costDiagonal3D ;
  int pointsPerSegment;

  // Robot-Specific (not sure if I need to break up...)
  nav_msgs::Path wayPoints;
  int polyOrder;
  int numSegments;// Number of Patched Segments
  int numDjkPts;
  int remainingPts;// Remaining Points

  vector< vector<double> > djkLocations;
  vector<double> djkDistBuff;
  vector<double> djkTimes;
  double trajTimeStep;
  vector<double> trajTimeVec;

  // Timers
  bool timersOn;
  double tStartSection, tEndSection;

  // Bidding
  double sqrdRecomputeRadius;

  // Gazebo Simulation
  bool simulateGazebo;
  string gazeboTopic;

  // __ Visualization __ \\

  // Candidates
  bool visualizePaths;
  geometry_msgs::Point arrowLoc;
  visualization_msgs::Marker arrow;
  visualization_msgs::Marker killArrow;
  bool vizCandEntropies;
  bool vizCandEntropiesDists;
  ros::Publisher candidatesPub, candidatesDiscountedPub;
  visualization_msgs::MarkerArray entropyArrowArray;
  visualization_msgs::MarkerArray entropyDistArrowArray;
  double arrowScale;
  vector<int> candidateVizIndices;
  double arrowLength;
  double nonOptimalOpacity;
  deque<ogm_ae::UpdatedMapCells> changedCellsQueue;
  visualization_msgs::MarkerArray markerArrayMsg, markerBuff;
  visualization_msgs::Marker marker;
  double occupancyVizMin, probRange;

  // Cost Map
  bool visualizeCostMap;
  nav_msgs::OccupancyGrid costMapViz;
  bool limitCostMap;
  int maxNumCostMapCells;

  // Reduced Map
  string ogmTopic;
  int numCellsInside;
  vector<int> numCellsThisDim;
  vector< vector<double> > offset;
  vector<int>    IndsInside ;
  double minMaxThresh;
  nav_msgs::OccupancyGrid OGM;

  // Functions
  double MaximizeCandidateObjective(Robot& robot);
  bool FindOptimalReachablePose(Robot& robot);
  void UpdateExpectedInfoGain(int& iCand, Mapping& mapInput);
  int SumExpectedInfoGainsEachLocation(Mapping& mapInput, vector<Robot>& robotVec);
  vector<double> SumScalarQuantityOfRays(vector<double>& input, int& raysEachSide);
  int MakeCyclicIndexInt(int& signedInt, int& size);
  void VisualizeCandidates(double& bestCandInfoGain, vector<double>& vals, ros::Publisher& publisher, int& bestInd, double& minVal);
  void VisualizeCandidatesMultiRobot(vector<Robot>& robotVec, int& numRobots);
  bool ReducedMapCellIsFree2D(int& cellIndex, Mapping& mapInput);
  bool ReducedMapCellIsFree3D(int& cellIndex, Mapping& mapInput);
  bool FindViableCandidateInfoGains(vector<Robot>& robotVec);
  void SetUpCostMap(Mapping& mapInput, vector<Robot>& robotVec);
  bool GenerateCostMap(Mapping& mapInput, Robot& robot, int& maxNumCostMapCells);
  bool FindDijkstraTrajectory(Mapping& mapInput, Robot& robot);
  void GenerateOccupancyGridCostMap(Mapping& mapInput, vector<double>& costMap, double& maxVal, nav_msgs::OccupancyGrid& ogm, Robot& robot);
  int DetermineSegmentFromPoint(double& timeNow, int& numSegments, vector<double>& timeStartSeg);
  void PublishPaths(Robot& robot);
  bool GenerateSegmentsForPatching(Robot& robot);
  void GenerateSmoothTrajectory(Mapping& mapInput, Robot& robot, ros::Time& rosTimeStartTraj);
  Eigen::Vector3d GetTrajFromCoeff(int dimension, Robot& robot, ros::Time& timeNow,
                          geometry_msgs::PoseStamped&  desCamMsg, geometry_msgs::PoseStamped&  desPosMsg,
                          geometry_msgs::TwistStamped& desVelMsg, geometry_msgs::AccelStamped& desAclMsg,
                          geometry_msgs::Vector3& x3Dot, geometry_msgs::Vector3& x4Dot);
  bool RobotExplorationBidding(Mapping&, vector<Robot>& robotVec);
  void ClearRobotLoc(Mapping& mapToReduce, Mapping& reducedMap, int& numRobots, vector<Robot>& robotVec);
  void IndsInsideLargerVoxel(vector<double>& loc, Mapping& mapToReduce, vector<int>& IndsInside);
  double ProbReducedMapCell(vector<double>& probsInside);
  void GetRobotCameraTFs(Robot& robot);
  void HaltTrajectories();
  void UpdateLockedExplorationMap();
  void UpdateChangedCellsOfReducedMap(Mapping& mapInput);

};

#endif
