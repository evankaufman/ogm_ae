
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <errno.h>
#include <math.h>
#include <string.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <termios.h>
#include <stdint.h>
#include <stdbool.h>
#include <vector>
#include <algorithm>
#include <Eigen/Dense>


// ROS Headers
#include <ros/ros.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/TransformStamped.h>
#include <gazebo/gazebo.hh>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf/transform_listener.h>

#include "ogm_ae/Robot.hpp"
#include "ogm_ae/Mapping.hpp"
#include "ogm_ae/Exploration.hpp"
#include "Mapping.cpp"// no map object in this node, but a required class for exploration class

using namespace std;

void PublishDesiredQuadrotorState(
    ros::Publisher& quadrotorPub, ogm_ae::DesiredState& DesiredStateMsg,
    geometry_msgs::PoseStamped& robotPose, geometry_msgs::TwistStamped& robotVel, geometry_msgs::AccelStamped& robotAccel,
    Eigen::Vector3d& b1D);

void PublishDesiredQuadrotorState(
    ros::Publisher& quadrotorPub, ogm_ae::DesiredState& DesiredStateMsg,
    geometry_msgs::PoseStamped& robotPose, geometry_msgs::TwistStamped& robotVel, geometry_msgs::AccelStamped& robotAccel,
    Eigen::Vector3d& b1D, geometry_msgs::Vector3& x3Dot, geometry_msgs::Vector3& x4Dot);

void DoYawRotation(ros::Publisher& quadrotorPub, ogm_ae::DesiredState& DesiredStateMsg, bool& pubDesiredState, ros::Publisher& desPosePub, ros::Publisher& desTwistPub, ros::Publisher& desAccelPub,
                   ros::Publisher& gazeboPub, geometry_msgs::PoseStamped& robotPose, geometry_msgs::TwistStamped& robotVel, geometry_msgs::AccelStamped& robotAccel, gazebo_msgs::ModelState& ModelStateMsg, ogm_ae::ModelStateStamped& ModelStateStampedMsg, double& yawRotDuration, bool& simulateGazebo);

int main(int argc, char **argv)
{

  // Initializations
  ros::init(argc, argv, "paths_to_cmd");
  string objective = "exploration";

  // Private Parameters: Robot & Sensor Reference Frames
  ros::NodeHandle nh, privateNH("~"); int robotNumber(-1); double mass; bool pubDesiredState(true);
  privateNH.param("robot_number", robotNumber, robotNumber);
  privateNH.param("mass", mass, mass);
  bool runAutonomousExploration(false);


  // Robot
  Robot robot(objective, robotNumber);
  vector<Robot> robotVec(1, robot);
  int numRobots; ros::param::get("num_robots", numRobots);

  // Exploration
  Exploration explore(robotVec, numRobots);
  robot = robotVec[0];
  int trajDim;

  ros::param::get("/exploration/simulate_gazebo", explore.simulateGazebo);

  string mapFrame, modelName, sensorFrame, robotFrame;
  ros::param::get("/mapping/frame", mapFrame);

  modelName   = robot.ns         ;
  robotFrame  = robot.robotFrame ;
  sensorFrame = robot.exploreFrame;

  ROS_INFO("Robot %d (namespace %s): Body Frame = %s, Sensor Frame = %s",
           robotNumber, modelName.c_str(), robotFrame.c_str(), sensorFrame.c_str());

  // Times for Gazebo Loading & Initial Yaw Rotation
  double tSecsToLoad, yawRotDuration;
  if(explore.simulateGazebo)
    ros::param::get("/exploration/gazebo_load_time", tSecsToLoad);
  else
    tSecsToLoad = 1.0;// 1 sec minimum for map reduction parameters
  ros::param::get("/exploration/yaw_rotation_time", yawRotDuration);
  bool doYawRot(false);

  // Cancel Gravity
  ros::ServiceClient WrenchClient = nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
  gazebo_msgs::ApplyBodyWrench wrenchCmdsSRV;

  // Camera Pitch
  explore.radiansCamPitch = explore.degreeCamPitch*M_PI/180;

  // Subscriber
  ros::Subscriber trajInfoSub, ModelStatesSub;
  GenSubscriber<ogm_ae::PolyLeastSquaresTraj> trajInfoSubscription   ;
  GenSubscriber<gazebo_msgs::ModelStates>     modelStatesSubscription;
  trajInfoSub = nh.subscribe
      (robot.trajInfoTopic   , 1, &GenSubscriber<ogm_ae::PolyLeastSquaresTraj>::callback, &trajInfoSubscription   );
  ModelStatesSub = nh.subscribe
      ("/gazebo/model_states", 1, &GenSubscriber<gazebo_msgs::ModelStates>::callback    , &modelStatesSubscription);

  // Publishers
  ros::Publisher desPosePub, desCamPub, desTwistPub, desAccelPub, gazeboPub, quadrotorPub;
  desPosePub  = nh.advertise
      <geometry_msgs::PoseStamped >
      (robot.desiredPoseTopic,    1, true);
  desCamPub   = nh.advertise
      <geometry_msgs::PoseStamped >
      (robot.desiredCamTopic,     1, true);

  // TODO: erase 2 below
  desTwistPub = nh.advertise
      <geometry_msgs::TwistStamped>
      (robot.desiredTwistTopic,   1, true);
  desAccelPub = nh.advertise
      <geometry_msgs::AccelStamped>
      (robot.desiredAccelTopic,   1, true);


  gazeboPub    = nh.advertise
      <ogm_ae::ModelStateStamped  >
      ("gazebo_states_topic_"+to_string(robotNumber), 1, true);

  quadrotorPub = nh.advertise
      <ogm_ae::DesiredState  >
      ("/quadrotor0/desired_traj", 1, true);


  // TODO: follow something like this:
//  if(simulation)
//    moveRobotPub   = nh.advertise
//        <ogm_ae::ModelStateStamped  >
//        ("gazebo_states_topic_"+to_string(robotNumber), 1, true);
//  else

  geometry_msgs::Point zeroPt;
  zeroPt.x = 0.0;
  zeroPt.y = 0.0;
  zeroPt.z = 0.0;
  geometry_msgs::Vector3 zeroVector3;
  zeroVector3.x = 0.0;
  zeroVector3.y = 0.0;
  zeroVector3.z = 0.0;

  geometry_msgs::PoseStamped  robotPose, robotPoseSave, cameraPose;
  robotPose.pose.position = zeroPt;
  ros::param::get("/exploration/height", explore.height);
  robotPose.pose.position.z = explore.height;

  geometry_msgs::TwistStamped robotVel;
  robotVel.twist.linear  = zeroVector3;
  robotVel.twist.angular = zeroVector3;

  geometry_msgs::AccelStamped robotAccel;
  robotAccel.accel.linear  = zeroVector3;
  robotAccel.accel.angular = zeroVector3;

  robotPose .header.frame_id = mapFrame;
  cameraPose.header.frame_id = mapFrame;
  robotVel  .header.frame_id = mapFrame;
  robotAccel.header.frame_id = mapFrame;

  ogm_ae::ModelStateStamped ModelStateStampedMsg;
  gazebo_msgs::ModelState ModelStateMsg;
  ModelStateMsg.model_name = modelName;
  ModelStateMsg.reference_frame = mapFrame;

  ogm_ae::DesiredState DesiredStateMsg;
  DesiredStateMsg.xD     .resize(3, 0.0);
  DesiredStateMsg.xD1Dot .resize(3, 0.0);
  DesiredStateMsg.xD2Dot .resize(3, 0.0);
  DesiredStateMsg.xD3Dot .resize(3, 0.0);
  DesiredStateMsg.xD4Dot .resize(3, 0.0);
  DesiredStateMsg.b1D    .resize(3, 0.0);
  DesiredStateMsg.b1D1Dot.resize(3, 0.0);
  DesiredStateMsg.b1D2Dot.resize(3, 0.0);
  Eigen::Vector3d b1D(1.0, 0.0, 0.0), b1D1Dot, b1D2Dot, omegaD;
  geometry_msgs::Vector3 x3Dot, x4Dot;



  ROS_INFO("Waiting %e seconds for Gazebo to load...", tSecsToLoad);
  usleep(tSecsToLoad*1000000);
  ROS_INFO("Beginning desired trajectory commands.");

  // Get TFs
  explore.GetRobotCameraTFs(robot);

  // Initialized Last Pose as Current
  robotPoseSave = robotPose;

  // Height Difference Between the Robot and Exploration Sensor
  tf::StampedTransform robotTF;
  robotTF = LoopUntilTransformAcquired(explore.tfListener, robotFrame, sensorFrame);
  explore.height -= robotTF.getOrigin().z();// corrected for robot COM


  wrenchCmdsSRV.request.body_name = modelName+"::base_link";
  wrenchCmdsSRV.request.reference_frame = "world";
  wrenchCmdsSRV.request.reference_point.x = 0.0;
  wrenchCmdsSRV.request.reference_point.y = 0.0;
  wrenchCmdsSRV.request.reference_point.z = 0.0;
  wrenchCmdsSRV.request.start_time = ros::Time(0.0);
  wrenchCmdsSRV.request.duration = ros::Duration(-1.0);// apply continuously until new command

  wrenchCmdsSRV.request.wrench.force.x = 0.0;
  wrenchCmdsSRV.request.wrench.force.y = 0.0;
  wrenchCmdsSRV.request.wrench.force.z = mass*9.81;

  wrenchCmdsSRV.request.wrench.torque.x = 0.0;
  wrenchCmdsSRV.request.wrench.torque.y = 0.0;
  wrenchCmdsSRV.request.wrench.torque.z = 0.0;

  WrenchClient.call(wrenchCmdsSRV);
  if(!wrenchCmdsSRV.response.success)
      cout << "Fail! Response message:\n" << wrenchCmdsSRV.response.status_message << endl;

  bool firstPass(true), msgOK(false);

  // Spin with 3 Subscribers (paths of position, velocity, & acceleration)
  while (ros::ok()){

    int modelInd(-1);
    for(int i(0); i < modelStatesSubscription.msg.name.size(); i++){
      if(modelStatesSubscription.msg.name[i] == modelName){
        modelInd = i;
        break;
      }
    }
    if(modelInd == -1)
      ROS_WARN("Model %s not published by Gazebo.", modelName.c_str());
    else{
      robotPose.pose = modelStatesSubscription.msg.pose[modelInd];
      robotVel.twist = modelStatesSubscription.msg.twist[modelInd];


      //    // Current Robot Pose
      //    robotTF = LoopUntilTransformAcquired(explore.tfListener, mapFrame, robotFrame);
      //    robotPose.pose.position.x = robotTF.getOrigin().x();
      //    robotPose.pose.position.y = robotTF.getOrigin().y();
          robotPose.pose.position.z = explore.height   ;
      //    robotPose.pose.orientation.x = robotTF.getRotation().x();
      //    robotPose.pose.orientation.y = robotTF.getRotation().y();
      //    robotPose.pose.orientation.z = robotTF.getRotation().z();
      //    robotPose.pose.orientation.w = robotTF.getRotation().w();

          if(firstPass){
            robotPoseSave = robotPose;
            firstPass = false;
          }

          // Check if Autonomous Exploration Underway (Boolean) & New Messages
          ros::param::get("run_autonomous_exploration", runAutonomousExploration);

          // Yaw Rotation at Beginning or Later on Command
          ros::param::get("do_yaw_rotation", doYawRot);
          if(doYawRot){

            // Do a Yaw Rotation
            DoYawRotation(quadrotorPub, DesiredStateMsg, pubDesiredState, desPosePub, desTwistPub, desAccelPub, gazeboPub, robotPose, robotVel, robotAccel, ModelStateMsg, ModelStateStampedMsg, yawRotDuration, explore.simulateGazebo);
            robotPoseSave = robotPose;

            // Don't Repeat This
            ros::param::set("do_yaw_rotation", false);
            ros::param::set("run_autonomous_exploration", true);

          }

          // Update Message if New
          if(trajInfoSubscription.msg.header.stamp.toSec() <= ros::Time::now().toSec()
          && trajInfoSubscription.msg.timeStartSeg.size() > 0){
            robot.trajInfo = trajInfoSubscription.msg;
            if(robot.trajInfo.zCoeff.size() > 0)
              trajDim = 3;
            else
              trajDim = 2;
            msgOK = true;
          }

          // Pose Data from Function Parameters
          if(msgOK && runAutonomousExploration){

            // Current Time
            ros::Time rosTimeNow = ros::Time::now();

            // Robot Trajectory Function
            b1D = explore.GetTrajFromCoeff(trajDim, robot, rosTimeNow, cameraPose, robotPose, robotVel, robotAccel, x3Dot, x4Dot);

            // Publish Camera
            desCamPub.publish(cameraPose);

            // Save Latest Pose
            robotPoseSave = robotPose;

          }
          else{

            // Fixed Location & Attitude
            robotPose = robotPoseSave;

            // Correct Attitude to e1/e2 Plane if Necessary
            if(robotPose.pose.orientation.x != 0.0 || robotPose.pose.orientation.y != 0.0){
              Eigen::Vector3d b1Sensor;
              robotPose.pose.orientation = MakeRollAndPitch0(robotPose.pose.orientation, b1Sensor, 1);
              robotPoseSave = robotPose;
            }

            // Zero Time Derivatives
            robotVel.twist  .linear  = zeroVector3;
            robotVel.twist  .angular = zeroVector3;
            robotAccel.accel.linear  = zeroVector3;
            robotAccel.accel.angular = zeroVector3;

            // Time Stamps
            robotPose.header.stamp  = ros::Time::now();
            robotVel.header.stamp   = ros::Time::now();
            robotAccel.header.stamp = ros::Time::now();
          }

          // Publish Robot
          desPosePub .publish(robotPose );
          desTwistPub.publish(robotVel  );
          desAccelPub.publish(robotAccel);

          // Gazebo Simulation
          if(explore.simulateGazebo){

            ModelStateMsg.pose  = robotPose.pose ;
            ModelStateMsg.twist = robotVel .twist;
            ModelStateStampedMsg.model_state = ModelStateMsg;
            ModelStateStampedMsg.header.stamp = ros::Time::now();
            gazeboPub.publish(ModelStateStampedMsg);
          }

          if(pubDesiredState)
            PublishDesiredQuadrotorState(quadrotorPub, DesiredStateMsg, robotPose, robotVel, robotAccel, b1D, x3Dot, x4Dot);
    }

    ros::spinOnce();

  }
  return 0;
}

void PublishDesiredQuadrotorState(
    ros::Publisher& quadrotorPub, ogm_ae::DesiredState& DesiredStateMsg,
    geometry_msgs::PoseStamped& robotPose, geometry_msgs::TwistStamped& robotVel, geometry_msgs::AccelStamped& robotAccel,
    Eigen::Vector3d& b1D){

  // Overloaded Case with 0 3rd & 4th Translational Derivatives
   geometry_msgs::Vector3 x3Dot, x4Dot;
   x3Dot.x = 0.0; x3Dot.y = 0.0; x3Dot.z = 0.0;
   x4Dot.x = 0.0; x4Dot.y = 0.0; x4Dot.z = 0.0;

  PublishDesiredQuadrotorState(quadrotorPub, DesiredStateMsg, robotPose, robotVel, robotAccel, b1D, x3Dot, x4Dot);

  return;
}

void PublishDesiredQuadrotorState(
    ros::Publisher& quadrotorPub, ogm_ae::DesiredState& DesiredStateMsg,
    geometry_msgs::PoseStamped& robotPose, geometry_msgs::TwistStamped& robotVel, geometry_msgs::AccelStamped& robotAccel,
    Eigen::Vector3d& b1D, geometry_msgs::Vector3& x3Dot, geometry_msgs::Vector3& x4Dot){

  // Initializations
  Eigen::Vector3d b1D1Dot, b1D2Dot, omegaD;

  // Desired Position & Time Derivatives
  ROSXYZTypeToVec(robotPose.pose.position, DesiredStateMsg.xD    );
  ROSXYZTypeToVec(robotVel.twist.linear  , DesiredStateMsg.xD1Dot);
  ROSXYZTypeToVec(robotAccel.accel.linear, DesiredStateMsg.xD2Dot);
  ROSXYZTypeToVec(x3Dot                  , DesiredStateMsg.xD3Dot);
  ROSXYZTypeToVec(x4Dot                  , DesiredStateMsg.xD4Dot);

  // Desired Direction of 1st Body-Fixed Axis & Time Derivatives
  omegaD << 0.0, 0.0, robotVel.twist.angular.z;
  b1D1Dot = omegaD.cross(b1D    );
  b1D2Dot = omegaD.cross(b1D1Dot);// note: 0 angular acceleration

  EigenVecToStdVec(b1D    , DesiredStateMsg.b1D    );
  EigenVecToStdVec(b1D1Dot, DesiredStateMsg.b1D1Dot);
  EigenVecToStdVec(b1D2Dot, DesiredStateMsg.b1D2Dot);

  quadrotorPub.publish(DesiredStateMsg);

  return;
}

void DoYawRotation(ros::Publisher& quadrotorPub, ogm_ae::DesiredState& DesiredStateMsg, bool& pubDesiredState, ros::Publisher& desPosePub, ros::Publisher& desTwistPub, ros::Publisher& desAccelPub, ros::Publisher& gazeboPub,
                   geometry_msgs::PoseStamped& robotPose, geometry_msgs::TwistStamped& robotVel, geometry_msgs::AccelStamped& robotAccel,
                   gazebo_msgs::ModelState& ModelStateMsg, ogm_ae::ModelStateStamped& ModelStateStampedMsg, double& yawRotDuration, bool& simulateGazebo){

  // Initial Values for Reseting at End
  geometry_msgs::Quaternion quatInit = robotPose.pose.orientation;
  double angVelZInit = robotVel.twist.angular.z, thetaInit;
  QuatToYawRot(thetaInit, quatInit);

  ros::Time yawRotTimeNow = ros::Time::now();
  double yawRotTimeStartSecs = yawRotTimeNow.toSec();
  robotVel.twist.angular.z = 2*M_PI/yawRotDuration;
  while(yawRotTimeNow.toSec() < yawRotTimeStartSecs+yawRotDuration){

    // Pose
    double yawRotTheta
        = 2*M_PI*(yawRotTimeNow.toSec()-yawRotTimeStartSecs)/yawRotDuration
        +thetaInit;
    geometry_msgs::Quaternion quat;
    YawRotToQuat(yawRotTheta, quat);
    Eigen::Vector3d b1D(cos(yawRotTheta), sin(yawRotTheta), 0.0);
    robotPose.pose.orientation = quat;
    robotPose.header.stamp = yawRotTimeNow;
    desPosePub.publish(robotPose);

    // Velocity
    robotVel.header.stamp = yawRotTimeNow;
    desTwistPub.publish(robotVel);

    // Acceleration
    robotAccel.header.stamp = yawRotTimeNow;
    desAccelPub.publish(robotAccel);

    // Gazebo
    if(simulateGazebo){
      ModelStateMsg.pose  = robotPose.pose ;
      ModelStateMsg.twist = robotVel .twist;
      ModelStateStampedMsg.model_state = ModelStateMsg;
      ModelStateStampedMsg.header.stamp = ros::Time::now();
      gazeboPub.publish(ModelStateStampedMsg);
    }

    // Quadrotor Flight Control
    if(pubDesiredState)
      PublishDesiredQuadrotorState(quadrotorPub, DesiredStateMsg, robotPose, robotVel, robotAccel, b1D);

    // Current Time & Spin
    yawRotTimeNow = ros::Time::now();
    ros::spinOnce();
  }

  // Reset Changed Values
  robotPose.pose.orientation = quatInit;
  robotVel.twist.angular.z = angVelZInit;

  return;
}
