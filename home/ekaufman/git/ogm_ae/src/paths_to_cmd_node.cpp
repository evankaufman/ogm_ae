
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
#include <tf/transform_listener.h>


#include "ogm_ae/voxel.h"
#include "ogm_ae/aux_fun.h"

using namespace std;

void DoYawRotation(Voxel& voxel, geometry_msgs::PoseStamped& robotPose, geometry_msgs::TwistStamped& robotVel, geometry_msgs::AccelStamped& robotAccel, gazebo_msgs::ModelState& ModelStateMsg, double& yawRotDuration);

int main(int argc, char **argv)
{

    // Initializations
    ros::init(argc, argv, "paths_to_cmd");
    Voxel voxel;
    ros::param::get("/exploration/simulate_gazebo", voxel.explore.simulateGazebo);
    string objective = "exploration";
    int numExplorationSensorsAndRobots, robotNumber(-1);
    voxel.e1 << 1.0, 0.0, 0.0;
    voxel.e2 << 0.0, 1.0, 0.0;
    voxel.e3 << 0.0, 0.0, 1.0;
    string mapFrame, modelName, sensorFrame, robotFrame;
    ros::param::get("/mapping/frame", mapFrame);
    bool haltExploration(false); double haltStamp = ros::Time::now().toSec();
    double lastMsgTimeStamp = ros::Time::now().toSec();
    ogm_ae::PolyLeastSquaresTraj trajInfo;


    // Private Parameters: Robot & Sensor Reference Frames
    ros::NodeHandle privateNH("~");
    privateNH.param("robot_number", robotNumber, robotNumber);
    bool runAutonomousExploration(false);
//    bool runAutonomousExplorationLastStep(false);
    voxel.GetRobotParameters(objective, numExplorationSensorsAndRobots);
    voxel.GetExplorationParameters();
    modelName   = voxel.robotInformation[robotNumber].ns         ;
    robotFrame  = voxel.robotInformation[robotNumber].robotFrame ;
    sensorFrame = voxel.robotInformation[robotNumber].exploreFrame;
    ROS_INFO("Robot %d (namespace %s): Body Frame = %s, Sensor Frame = %s",
             robotNumber, modelName.c_str(), robotFrame.c_str(), sensorFrame.c_str());

    // Times for Gazebo Loading & Initial Yaw Rotation
    double tSecsToLoad, yawRotDuration;
    if(voxel.explore.simulateGazebo)
      ros::param::get("/exploration/gazebo_load_time", tSecsToLoad);
    else
      tSecsToLoad = 1.0;// 1 sec minimum for map reduction parameters
    ros::param::get("/exploration/yaw_rotation_time", yawRotDuration);
    bool doYawRot(false);

    // Subscribers
    GenSubscriber<ogm_ae::PolyLeastSquaresTraj> trajInfoSubscription;
    voxel.trajInfoSub = voxel.nh.subscribe
        (voxel.robotInformation[robotNumber].trajInfoTopic, 1, &GenSubscriber<ogm_ae::PolyLeastSquaresTraj>::callback, &trajInfoSubscription, ros::TransportHints().unreliable().tcpNoDelay());
//    ros::TransportHints().unreliable();
//    ros::TransportHints().tcpNoDelay();

    // Publishers
    voxel.desPosePub  = voxel.nh.advertise
        <geometry_msgs::PoseStamped >
        (voxel.robotInformation[robotNumber].desiredPoseTopic,  1, true);
    voxel.desCamPub   = voxel.nh.advertise
        <geometry_msgs::PoseStamped >
        (voxel.robotInformation[robotNumber].desiredCamTopic,   1, true);
    voxel.desTwistPub = voxel.nh.advertise
        <geometry_msgs::TwistStamped>
        (voxel.robotInformation[robotNumber].desiredTwistTopic, 1, true);
    voxel.desAccelPub = voxel.nh.advertise
        <geometry_msgs::AccelStamped>
        (voxel.robotInformation[robotNumber].desiredAccelTopic, 1, true);
    voxel.gazeboPub   = voxel.nh.advertise
        <gazebo_msgs::ModelState    >
        (voxel.explore.gazeboTopic,                   1, true);

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
    ros::param::get("/exploration/height", voxel.explore.height);
    robotPose.pose.position.z = voxel.explore.height;

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

    gazebo_msgs::ModelState ModelStateMsg;
    ModelStateMsg.model_name = modelName;
    ModelStateMsg.reference_frame = voxel.map.frame;

    ROS_INFO("Waiting %e seconds for Gazebo to load...", tSecsToLoad);
    usleep(tSecsToLoad*1000000);
    ROS_INFO("Beginning desired trajectory commands.");

    // Get TFs
    voxel.GetRobotCameraTFs(sensorFrame, robotFrame, robotNumber);

    // Initialized Last Pose as Current
    robotPoseSave = robotPose;
    tf::StampedTransform robotTF;

    bool firstPass(true), msgOK(false);

    // Spin with 3 Subscribers (paths of position, velocity, & acceleration)
    while (ros::ok()){

      // Current Robot Pose
      robotTF = LoopUntilTransformAcquired(voxel.explore.tfListener, mapFrame, robotFrame);
      robotPose.pose.position.x = robotTF.getOrigin().x();
      robotPose.pose.position.y = robotTF.getOrigin().y();
      robotPose.pose.position.z = voxel.explore.height   ;
      robotPose.pose.orientation.x = robotTF.getRotation().x();
      robotPose.pose.orientation.y = robotTF.getRotation().y();
      robotPose.pose.orientation.z = robotTF.getRotation().z();
      robotPose.pose.orientation.w = robotTF.getRotation().w();

      if(firstPass){
        robotPoseSave = robotPose;
        firstPass = false;
      }

      // Check if Autonomous Exploration Underway (Boolean) & New Messages
      ros::param::get("run_autonomous_exploration", runAutonomousExploration);

      // Yaw Rotation at Beginning or Later on Command
      ros::param::get("do_yaw_rotation", doYawRot);
      if(doYawRot && runAutonomousExploration){

        // Temoparily Set Exploration Off
        ros::param::set("run_autonomous_exploration", false);

        // Do a Yaw Rotation
        DoYawRotation(voxel, robotPose, robotVel, robotAccel, ModelStateMsg, yawRotDuration);
        robotPoseSave = robotPose;

        // Don't Repeat This
        ros::param::set("do_yaw_rotation", false);
        ros::param::set("run_autonomous_exploration", true);

      }

      // Check Message
      if(trajInfoSubscription.msg.header.stamp.toSec() <= ros::Time::now().toSec()
      && trajInfoSubscription.msg.timeStartSeg.size() > 0){
        trajInfo = trajInfoSubscription.msg;
        msgOK = true;
        if(lastMsgTimeStamp != trajInfo.header.stamp.toSec()){
          haltExploration = false;
          ros::param::set("exploration/halt", haltExploration);
          lastMsgTimeStamp = trajInfo.header.stamp.toSec();
        }
        else
          ros::param::get("exploration/halt", haltExploration);
      }
      else
        ros::param::get("exploration/halt", haltExploration);

      if(haltExploration && haltStamp != trajInfo.header.stamp.toSec()){
        ROS_WARN("Halting exploration...");
        haltStamp = trajInfo.header.stamp.toSec();
      }

      // Pose Data from Function Parameters
      if(msgOK && runAutonomousExploration && !haltExploration){

        // Current Time
        ros::Time rosTimeNow = ros::Time::now();

        // Robot Trajectory Function
        voxel.GetXYTrajFromCoeff(robotNumber,
              trajInfo, rosTimeNow,
              cameraPose, robotPose, robotVel, robotAccel);

        // Publish Camera
        voxel.desCamPub.publish(cameraPose);

        // Save Latest Pose
        robotPoseSave = robotPose;

      }
      else{

        // Fixed Location
        robotPose = robotPoseSave;

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
      voxel.desPosePub .publish(robotPose );
      voxel.desTwistPub.publish(robotVel  );
      voxel.desAccelPub.publish(robotAccel);

      // Gazebo Simulation
      if(voxel.explore.simulateGazebo){
        ModelStateMsg.pose  = robotPose.pose ;
        ModelStateMsg.twist = robotVel .twist;
        voxel.gazeboPub.publish(ModelStateMsg);
      }

      ros::spinOnce();

    }
    return 0;
}

void DoYawRotation(Voxel& voxel,
  geometry_msgs::PoseStamped& robotPose, geometry_msgs::TwistStamped& robotVel, geometry_msgs::AccelStamped& robotAccel,
  gazebo_msgs::ModelState& ModelStateMsg, double& yawRotDuration){

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
    robotPose.pose.orientation = quat;
    robotPose.header.stamp = yawRotTimeNow;
    voxel.desPosePub.publish(robotPose);

    // Velocity
    robotVel.header.stamp = yawRotTimeNow;
    voxel.desTwistPub.publish(robotVel);

    // Acceleration
    robotAccel.header.stamp = yawRotTimeNow;
    voxel.desAccelPub.publish(robotAccel);

    // Gazebo
    if(voxel.explore.simulateGazebo){
      ModelStateMsg.pose  = robotPose.pose ;
      ModelStateMsg.twist = robotVel .twist;
      voxel.gazeboPub.publish(ModelStateMsg);
    }

    // Current Time & Spin
    yawRotTimeNow = ros::Time::now();
    ros::spinOnce();
  }

  // Reset Changed Values
  robotPose.pose.orientation = quatInit;
  robotVel.twist.angular.z = angVelZInit;

  return;
}
