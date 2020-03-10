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

#endif
