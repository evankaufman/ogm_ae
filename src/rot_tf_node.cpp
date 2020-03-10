
//#include <ros/ros.h>
//#include <tf/transform_broadcaster.h>
//using namespace std;

//int main(int argc, char** argv){
//  ros::init(argc, argv, "robot_tf_publisher");
//  ros::NodeHandle n;

//  int loopHz(100);
//  ros::Rate r(loopHz);

//  tf::TransformBroadcaster broadcaster;

//  double tElapsed, tStep, theta, omega(1.0);
//  ros::spinOnce();

//  while(n.ok()){
//    tStep = 1.0/loopHz;
//    tElapsed += tStep;
//    theta = omega*tElapsed;
//    broadcaster.sendTransform(
//      tf::StampedTransform(
//        tf::Transform(tf::Quaternion(0.0, 0.0, sin(theta/2), cos(theta/2)), tf::Vector3(5.0, -1.0, 1.0)),
//        ros::Time::now(),"world", "spin_view"));
//    cout << "Time: " << tElapsed << ", theta = " << theta << endl;
//    ros::spinOnce();
////    r.sleep();
//    usleep(tStep*1000000);
//  }
//}

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
using namespace std;

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  double tBtwnUpdates = 5.0;
  double loopHz = 1.0/tBtwnUpdates;
  ros::Rate r(loopHz);

  tf::TransformBroadcaster broadcaster;

  double R(30.0), tElapsed, tStep, theta, omega, tLoop(600.0), angleView;
  omega = 2*M_PI/tLoop;



  ros::spinOnce();

  while(n.ok()){
    tStep = 1.0/loopHz;
    tElapsed += tStep;
    theta = omega*tElapsed;
    angleView = theta;//-M_PI/4;// yaw in RViz set to 0 (looking partly tangential, partly toward center)
//    quaternionMsgToTF(quat_msg , quat_tf);
    tf::Quaternion q = tf::createQuaternionFromRPY(theta, 0.0, 0.0);
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0.0, 0.0, sin(angleView/2), cos(angleView/2)), tf::Vector3(R*cos(theta), R*sin(theta), 1.0)),
        ros::Time::now(), "world", "spin_view"));
    cout << "Time: " << tElapsed << ", theta = " << theta << endl;
//    cout << "Quaternion: " << q.z << endl;
    ros::spinOnce();
//    r.sleep();
    usleep(tStep*1000000);
  }
}
