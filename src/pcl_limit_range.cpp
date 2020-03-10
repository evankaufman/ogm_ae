#include <ros/ros.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/PointStamped.h>

using namespace std;

class Pcl_conversion{
private:
  ros::NodeHandle nh;
  ros::Publisher pclPub;
  ros::Subscriber pclSub;
  sensor_msgs::PointCloud2 cloudMsg;
  string topicIn;
  string topicOut;
  int sensorNumber;

public:
  bool hasColors;
  double maxRange;
  double maxRangeSqrd;

  void PclCallback(const sensor_msgs::PointCloud2ConstPtr& scanInput){

    // Get Scan Information
    pcl::PCLPointCloud2 scan;
    pcl_conversions::toPCL(*scanInput,scan);

    // Remove Points > 'maxRange'
    if(hasColors){
      pcl::PointCloud<pcl::PointXYZRGB> cloud;
      FindPointCloudRanges(scan, cloud);
    }
    else{// no colors
      pcl::PointCloud<pcl::PointXYZ> cloud;
      FindPointCloudRanges(scan, cloud);
    }

    // Publish
    pclPub.publish(cloudMsg);
  }

  template<typename cloudType>
  void FindPointCloudRanges(
      pcl::PCLPointCloud2& scan, cloudType& cloudIn){

    // Conversion from pcl::PCLPointCloud2 to cloudType
    pcl::fromPCLPointCloud2(scan, cloudIn);

    // Squared Ranges of 'cloudIn' < 'maxRange' Squared
    int numRays = cloudIn.points.size(), indReduced(0);// cloudIn.width*cloudIn.height;
    double rangeSqrd;
    for(int i = 0; i < numRays; i++){
      rangeSqrd =
          cloudIn.points[i].x*cloudIn.points[i].x
         +cloudIn.points[i].y*cloudIn.points[i].y
         +cloudIn.points[i].z*cloudIn.points[i].z;
      if(rangeSqrd < maxRangeSqrd){
        cloudIn.points[indReduced] = cloudIn.points[i];
        indReduced++;
      }
    }
    cloudIn.points.resize(indReduced);
    cloudIn.width = indReduced; cloudIn.height = 1;

    // Conversion from cloudType to pcl::PCLPointCloud2
    pcl::toPCLPointCloud2(cloudIn, scan);

    // Publishable Point Cloud
    pcl_conversions::fromPCL(scan, cloudMsg);

    return;
  }

  Pcl_conversion(ros::NodeHandle* node):nh(*node){


    // Topic Names
    nh.param("topic_in",  topicIn,  topicIn );
    nh.param("topic_out", topicOut, topicOut);
    ROS_INFO("PCL2 (%s) converted to PCL2 (%s)", topicIn.c_str(), topicOut.c_str());

    // Range Limit
    nh.param("sensor_number", sensorNumber, sensorNumber);
    ros::param::get("/mapping/sensor_"+to_string(sensorNumber)+"/properties/max_range", maxRange);
    maxRangeSqrd = maxRange*maxRange;

    // Colors
    nh.param("has_colors", hasColors, hasColors);

    // Subscribe & Publish
    pclPub = nh.advertise<sensor_msgs::PointCloud2>(topicOut, 10);
    pclSub = nh.subscribe<sensor_msgs::PointCloud2>
        (topicIn, 10, &Pcl_conversion::PclCallback, this);
  }
  virtual ~Pcl_conversion() = default;
};

int main(int argc, char **argv){
  ros::init(argc, argv, "pcl_limit_range");
  ros::NodeHandle node("~");

  Pcl_conversion pcl_node(&node);
  ros::spinOnce();

//  ros::Rate rate(10.0);
  while (node.ok()){
    ros::spinOnce();
//    rate.sleep();
  }
  return 0;
}
