
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
using namespace std;

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  int loopHz(100);
  ros::Rate r(loopHz);

  tf::TransformBroadcaster broadcaster;

  double tElapsed, tStep, theta, omega(0.01);
  ros::spinOnce();

  while(n.ok()){
    tStep = 1.0/loopHz;
    tElapsed += tStep;
    theta = omega*tElapsed;
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0.0, 0.0, sin(theta/2), cos(theta/2)), tf::Vector3(5.0, -1.0, 1.0)),
        ros::Time::now(),"world", "spin_view"));
    cout << "Time: " << tElapsed << ", theta = " << theta << endl;
    ros::spinOnce();
//    r.sleep();
    usleep(tStep*1000000);
  }
}
