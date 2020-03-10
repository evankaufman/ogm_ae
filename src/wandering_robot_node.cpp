#include "ogm_ae/aux_fun.h"

using namespace std;

int main(int argc, char **argv)
{

  // Initializations
  ros::init(argc, argv, "wandering_robot");
  ros::NodeHandle nh;
  int numRobots;
  string mapFrame, modelName;
  ros::Rate loopRate(10);

  // Public Parameters
  ros::param::get("/mapping/frame", mapFrame);
  ros::param::get("/wandering_thing", modelName);
  ros::param::get("/num_robots", numRobots);

  // Publisher
  ros::Publisher gazeboPub = nh.advertise<ogm_ae::ModelStateStamped>("gazebo_states_topic_"+to_string(numRobots), 1, true);

  // Gazebo Message
  ogm_ae::ModelStateStamped ModelStateStampedMsg;
  gazebo_msgs::ModelState ModelStateMsg;
  ModelStateMsg.model_name = modelName;
  ModelStateMsg.reference_frame = mapFrame;

  geometry_msgs::Pose pose; geometry_msgs::Twist twist; double vel, tStart, tNow, durNow, durMax(-1.0);

  vel = 0.5;
  vector<double> xLocs = { 29.0,  29.0,  9.0,  9.0 };
  vector<double> yLocs = {-45.0, -5.0 , -5.0, -45.0};
  vector<double> thetas = {M_PI, 3*M_PI/2, 2*M_PI, M_PI/2};
  vector<double> segLengths(4);// horizontal/vertical translations
  segLengths[0] = abs((xLocs[1]-xLocs[0])+(yLocs[1]-yLocs[0]));
  segLengths[1] = abs((xLocs[2]-xLocs[1])+(yLocs[2]-yLocs[1]));
  segLengths[2] = abs((xLocs[3]-xLocs[2])+(yLocs[3]-yLocs[2]));
  segLengths[3] = abs((xLocs[0]-xLocs[3])+(yLocs[0]-yLocs[3]));

  vector<double> xVels = {0.0, -vel, 0.0 , vel};
  vector<double> yVels = {vel, 0.0 , -vel, 0.0};

  int lastWayPoint, nextWayPoint(0);
  pose.position.z = 0.0;
  twist.linear .z = 0.0;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = 0.0;


  ros::Duration(1.0).sleep();

  tStart = ros::Time::now().toSec();
  while (ros::ok()){

    // Current Time & Duration
    tNow = ros::Time::now().toSec();
    durNow = tNow-tStart;

    // New Segment
    if(durNow > durMax){
      tStart = tNow;
      durNow = 0.0;
      lastWayPoint = nextWayPoint;
      durMax = segLengths[lastWayPoint]/vel;
      if(++nextWayPoint > 3)
        nextWayPoint = 0;
    }

    // Linear Trajectory
    double portionNext = durNow/durMax;
    double portionLast = 1.0-portionNext;
    pose.position.x = portionLast*xLocs[lastWayPoint]+portionNext*xLocs[nextWayPoint];
    pose.position.y = portionLast*yLocs[lastWayPoint]+portionNext*yLocs[nextWayPoint];
    twist.linear.x = xVels[lastWayPoint];
    twist.linear.y = yVels[lastWayPoint];
    YawRotToQuat(thetas[lastWayPoint], pose.orientation);

    // Message & Spin
    ModelStateMsg.pose  = pose ;
    ModelStateMsg.twist = twist;
    ModelStateStampedMsg.model_state = ModelStateMsg;
    ModelStateStampedMsg.header.stamp = ros::Time::now();
    gazeboPub.publish(ModelStateStampedMsg);
    ros::spinOnce();

    loopRate.sleep();

  }

  return 0;
}
