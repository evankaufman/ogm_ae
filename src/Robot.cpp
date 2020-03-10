#include "ogm_ae/Robot.hpp"

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
