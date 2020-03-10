#ifndef voxel_h
#define voxel_h

#include <math.h>
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <ctime>
#include <cmath>
#include <algorithm>
#include <memory>
#include <string>
#include "ros/ros.h"
#include "aux_fun.h"

// ROS Headers
#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>

// std_msgs
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float64.h>

// geometry_msgs
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>

// sensor_msgs
#include <sensor_msgs/Range.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

// nav_msgs
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

// visualization_msgs
#include <visualization_msgs/MarkerArray.h>

// custom_msgs
#include <ogm_ae/UpdatedMapCells.h>
#include <ogm_ae/PolyLeastSquaresTraj.h>

// message_filters
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// tf & pcl_ros
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include <pcl_ros/point_cloud.h>


using namespace std;
using namespace Eigen;

class Voxel
{

public:
    Voxel();
    virtual ~Voxel() = default;


    // VARIABLES

    // ROS
    ros::NodeHandle nh; // node handle
    ros::Publisher  ogm3DPub       ;// 3D occupancy grid
    ros::Publisher  ogm3DViz       ;// Visualize 3D occupancy grid
    ros::Publisher  ogm2DPub       ;// 2D occupancy grid (fixed height)
    ros::Publisher  gazeboPub      ;// Publish desired pose model state
    ros::Subscriber mapProbsSub    ;
    ros::Subscriber mapRGBSub      ;
    ros::Subscriber mapChangesSub  ;

    // multi
    ros::Publisher  desPosePub   ;// Publish desired pose
    ros::Publisher  desCamPub    ;// Publish desired pose
    ros::Publisher  desTwistPub  ;// Publish desired velocities
    ros::Publisher  desAccelPub  ;// Publish desired accelerations
    ros::Subscriber trajCameraSub;
    ros::Subscriber trajInfoSub  ;

    // TODO: kill these 3
    ros::Subscriber trajPositionSub;
    ros::Subscriber trajVelocitySub;
    ros::Subscriber trajAcceleraSub;

    // General Math
    Eigen::Vector3d e1, e2, e3, e3Signed;

    // Robots
    int numRobots;
    vector<double> entropyBumpsAllRobots;
    class robotInfo{

    public:

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
    vector<robotInfo> robotInformation;
    void GetRobotParameters(string& objective, int& numSensorsTotal);

    // Complete Map Information & Sensors
    class mapInfo{
    public:

      // ROS
      ros::Publisher  mapProbsPub;  // publish cell probabilities
      ros::Publisher  mapRGBPub;    // publish cell RGB colors
      ros::Publisher  mapEntropyPub;// publish cell RGB colors
      ros::Publisher  mapChangesPub;// publish changed cells

      // Topics & Frames
      string occTopic, rgbTopic, changesTopic, HTopic;// publication topics
      string frame;             // inertial frame fixed occupancy grid maps
      bool trackDiff;// publish/cycle through changes only (scalable)

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

      // Visualization
      double occThresh;        // min occupancy probability for visualization
      vector<int> defaultColor;// RGB color to visualized cells generated from scans without color data

      // Misc. Params
      double sqrt2pi;// fixed constant in forward sensor model
      double totalEntropy;
      std_msgs::Float64 totalEntropyMsg;
      int numRobots;

      // Sensors
      int numSensors;// total number of depth sensors
      struct sensorInfo{

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
        double probDetSat ;// minimum probability of detection (inverse sensor model)
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
        bool     printTimers     ;// print timers on/off
        double   tScanUpdateStart;// ROS time before scan update
        double   tScanUpdateEnd  ;// ROS time after scan update
        double   timeCurrent     ;// duration of current scan update
        double   timeAvg         ;// duration of average scan update
        int numSamplesTaken ;// number of scans used to update map

        // TF Filtering (unchanged for any sensor with msg type PointCloud2)
        tf::TransformListener tfListener;
        message_filters::Subscriber<sensor_msgs::PointCloud2> sub;
        tf::MessageFilter<sensor_msgs::PointCloud2> *tfFilter;

      };
      vector<sensorInfo> sensorInformation;// vector of struct for any number of sensors

      // __ Functions __ \\

        // Indexing
        vector<double> MapLocationFromInd(int ind);// gets cell location from map index
        int IndFromMapLocation(vector<double>& mapLoc);// gets map index from cell location
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
        void UpdateMap(int& sensorNumber, int& numRays, vector< vector<double> >& meas, vector< vector<int> >& color);
        bool MeasRayOK(vector<double>& uVec, double& depth, int& sensorNumber);
        vector<double> SensorLocRelToOtherRobots(int& sensorNumber);
        void UpdateCellProbabilitiesAndColors(bool& ISMSuccess, vector<int>& color, int& numCellsAlongRay, int& sensorNumber);
        void ChangedCellLimits(int& ind, vector<double>& minCorner, vector<double>& maxCorner, int& sensorNumber);
        void ComposeChangedMapCellMsg(vector<double>& minCorner, vector<double>& maxCorner, bool hasColors = true);
        void UpdateChangedMapCells();

        // Grouping Cells
        template<typename valType>
        void SetValToSpaceAroundCellLoc(vector<double>& loc, int& numCellsEachDirection, valType inputVal, vector<valType>& saveVals);
        void FindCellsComposingBlock(vector<double>& testLoc, vector<double>& minCorner, vector<int>& numCellsThisDim, vector<int>& indsToUpdate, int& blockIndex, int iDim = 0);

        // Parameters & Initializations
        void GetMapParameters(string& objective);
        void GetSensorParameters(string& objective, int& numRobotsTotal, vector<robotInfo>& robotInformation);
        void BasicMapInfoInit(string& objective);
        void SensorUpdateInit();
        void OccupancyGridMsgInit(nav_msgs::OccupancyGrid& ogm, double& mapHeight);

        template<typename mapType, typename mapDataType>
        void MapInfoToMultiArray(int& numElementsPerCell, string& addedLabelInfo, mapType& mapData, mapDataType& initVal);

        // Visualization
        void GenerateRvizMapMarker(visualization_msgs::Marker& marker, int& mapInd, vector<double>& mapLoc, double& probRange, double& occupancyVizMin, double& prob, vector<int>& color);


        // Exploration
        double SingleRayExpectedEntropyChange(vector<double>& sensorLoc, vector<double>& rayUVec);


    } map, reducedMap, entropyMap, collisionMap;

    // Exploration
    class explorationInfo{
    public:

      // General

      // multi
      tf::StampedTransform transformW2C, transformW2R, transformR2C;
      tf::TransformListener tfListener;
      Eigen::Matrix3d RotMatrixRobotCamera;
      Eigen::Matrix3d RotMatrixWorldRobot ;
      Eigen::Matrix3d RotMatrixWorldCamera;
      string entropyOccTopic, collOccTopic;
      string entropyMarkerTopic, distInfoMarkerTopic;
      int sensorNumber;
      nav_msgs::Path djkTraj, smoothTraj;
      int numReachCandidates;
      int numRaysPerCandidate;
      int numRaysEachSide;
      double angleFOV;
      double discountRadius;
      double maxComputationTime;
      double maxRadPerSec;

      // Information Gain
      double height, sensorHeight;
      int alphaXY;
      int alphaZ ;
      double acceptableCollisionProb;
      double tNow, tStartTraj, tEndTraj, tInitTraj, trajDuration;
      bool initTraj;
      vector<geometry_msgs::Pose> candidates;
      int numPosesDjk, numPosesSmoothTraj;
      vector<bool> freeCells;
      int numCandidatesTotal;
      int numDoneCandidates;
      int numCollCandidates;
      int numLowHCandidates;
      int numVisitedCandidates;
      vector< vector<double> > rayUVecs;// same for each candidate location
      vector< vector<double> > optimalUVecs;// specific to each candidate

      double candidateSeparation;
      vector<int> candidateIndices;
      vector< vector<double> > candidateLocations;
      vector<double> candidateExpectedInfoGain;
      vector<bool> candidateDone;
      double minInfoGainThresh;
      vector<double> infoGainEachRay;
      vector<double> infoGainEachDirection;// sum with neighboring rays

      // Path
      double bumpValAtDist;// Gaussian value <= 1 at a distance away described below
      double bumpDist;// distance from robot current location
      double bumpSigSqrd;// Gaussian variance corresponding to 'bumpValAtDist' & 'bumpDist'
      int reqNumFreeNeighbors;
      vector< vector<int> > pathwayBuff;
      vector<bool> visited;
      double costMapInitValue;
      double maxCostMapAsgndVal;
      geometry_msgs::PoseStamped fixedPoseStampedParams;
      double maxSpeed;
      double costHorizVert;
      double costDiagonal ;
      int pointsPerSegment;

      // Robot-Specific (not sure if I need to break up...)
      nav_msgs::Path wayPoints;
      int polyOrder;
      int numSegments;// Number of Patched Segments
      int numDjkPts;
      int remainingPts;// Remaining Points

      vector< Eigen::Vector2d > djkLocations;
      vector<double> djkDistBuff;
      vector<double> djkTimes;
      double trajTimeStep;
      vector<double> trajTimeVec;

      // Timers
      bool timersOn;
      double tStartSection, tEndSection;

      // Gazebo Simulation
      bool simulateGazebo;
      string gazeboTopic;

      // __ Visualization __ \\

      // Candidates
      bool publishPaths;
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

      // Cost Map
      bool visualizeCostMap;
      nav_msgs::OccupancyGrid costMapViz;

      // Functions
      double MaximizeCandidateObjective(robotInfo& robot);
      bool FindOptimalReachablePose(robotInfo& robot);
      int SumExpectedInfoGainsEachLocation(mapInfo& mapInput);
      vector<double> SumScalarQuantityOfRays(vector<double>& input, int& raysEachSide);
      int MakeCyclicIndexint(int& signedInt, int& size);
      void VisualizeCandidates(double& bestCandInfoGain, vector<double>& vals, ros::Publisher& publisher, int& bestInd, double& minVal);
      void VisualizeCandidatesMultiRobot(vector<robotInfo>& robotInformation, int& numRobots);
      void FindViableCandidateInfoGains(mapInfo& mapInput);
      bool SetUpCostMap(mapInfo& mapInput, vector<robotInfo>& robotInformation, int& robotNum, int& indCurrent);
      bool GenerateCostMap(mapInfo& mapInput, vector<robotInfo>& robotInformation, int& robotNum, int& indCurrent);
      bool FindDijkstraTrajectory(mapInfo& mapInput, vector<robotInfo>& robotInformation, int& robotNum);
      void GenerateOccupancyGridCostMap(mapInfo& mapInput, vector<double>& costMap, double& maxVal, nav_msgs::OccupancyGrid& ogm);

    } explore;

    // Map Reduction for Exploration
    class mapReduction{

    public:
      string goal;
      string map2DTopic, map3DTopic, ogmTopic;
      string cellCombProcess;
      bool initReq;
      int numCellsInside;
      vector<int> numCellsThisDim;
      vector<double> offset;
      vector<double> offsetZ;
      vector<int>    IndsInside ;
      vector<double> probsInside;
      vector<double> loc2D;
      vector<double> loc3D;
      vector<double> robotPt, robotPtLast;
      double minMaxThresh;
      string robotFrame;
      geometry_msgs::PointStamped cameraOrigin;
      int freeCellsRadius;
      bool coloredCellsOnly;
      int numZCells;
      double locZStart;
      double probHMapThresh;
      double entropyMapThresh;

      nav_msgs::OccupancyGrid OGM;
      tf::TransformListener tfListener;

      // Functions
      void ClearRobotLoc(int& reqNumFreeNeighbors, mapInfo& mapToReduce, mapInfo& reducedMap, int& numRobots, vector<robotInfo>& robotInformation);
      void ProbsInsideLargerVoxel(double& startLocZ, mapInfo& mapToReduce);
      double ProbReducedMapCell(mapInfo& mapToReduce, mapInfo& reducedMap, int& indReducedMap);

    } reduction;


    // Non-MemberFunctions

    // Parameters & Initializations
    // get rid of these...
    void GetParameters(string& objective);
    void Initializations(string& objective);

    // Keep seperate
    void GetExplorationParameters();
    void ExplorationInit();
    void ReduceMapParamsInit();

    // Exploration
    void GenerateSmoothTrajectory(mapInfo& mapInput, int& robotNum, ros::Time& rosTimeStartTraj);
    void FindFreeCells(int& numFreeCellNeighbors, mapInfo& mapInput);
    bool GenerateSegmentsForPatching(int& robotNum);
    int DetermineSegmentFromPoint(double& timeNow, int& numSegments, vector<double>& timeStartSeg);
    bool RobotExplorationBidding();
    void PublishPaths(int& robotNum);
    void GetXYTrajFromCoeff(int& robotNum, ogm_ae::PolyLeastSquaresTraj& trajInfo, ros::Time& timeNow,
                            geometry_msgs::PoseStamped&  desCamMsg, geometry_msgs::PoseStamped&  desPosMsg,
                            geometry_msgs::TwistStamped& desVelMsg, geometry_msgs::AccelStamped& desAclMsg);
    // Misc.
    void GetRobotCameraTFs(string& sensorFrame, string& robotFrame, int& robotNum);

    // Reduction
    void MapReductionGetParameters();
    void MapProbsReductionCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void MapProbChangesReductionCallback(const ogm_ae::UpdatedMapCells::ConstPtr& msg);
    void MapProbChangesCallback(const ogm_ae::UpdatedMapCells::ConstPtr& msg);


};

#endif
