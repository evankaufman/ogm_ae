/* Probabilistic Occupancy Grid Mapping
 * by Evan Kaufman, Kuya Takami, and Taeyoung Lee
 * Flight Dynamics and Control Laboratory (FDCL)
 * Mechanical and Aerospace Engineering (MAE)
 * The George Washington University (GWU)
 * Contracted under the US Naval Research Laboratory (NRL)
*/

#include <ogm_ae/ProbabilisticOccupancyGridMapping.hpp>

// Namespaces
using namespace std;

namespace probabilistic_occupancy_grid_mapping{
ProbabilisticOccupancyGridMapping::ProbabilisticOccupancyGridMapping():
  objective("mapping"){

  // Obtain Map Parameters
  voxel.GetParameters(objective);

  // Initialize Variables
  voxel.Initializations(objective);

  // Publish Occupancy Grid
  voxel.map.mapProbsPub = voxel.nh.advertise<std_msgs::Float64MultiArray>
      (voxel.map.occTopic, 1, true);

  // Publish Updates Only
  voxel.map.mapChangesPub = voxel.nh.advertise<ogm_ae::UpdatedMapCells>(voxel.map.changesTopic, 1);

  // Publish RGB Map Colors
  voxel.map.mapRGBPub = voxel.nh.advertise<std_msgs::Int16MultiArray>
      (voxel.map.rgbTopic , 1, true);

  // Publish Map Entropy
  voxel.map.mapEntropyPub = voxel.nh.advertise<std_msgs::Float64>
      (voxel.map.HTopic, 1, true);

  // Message Filter Subscriptions: Pose TF & Point Cloud
  for(int sensorNumber = 0; sensorNumber < voxel.map.numSensors; sensorNumber++){
    voxel.map.sensorInformation[sensorNumber].sub.subscribe
        (voxel.nh, voxel.map.sensorInformation[sensorNumber].topic, 1);
    voxel.map.sensorInformation[sensorNumber].tfFilter = new tf::MessageFilter<sensor_msgs::PointCloud2>
        (voxel.map.sensorInformation[sensorNumber].sub, voxel.map.sensorInformation[sensorNumber].tfListener, voxel.map.frame, 1);
    voxel.map.sensorInformation[sensorNumber].tfFilter->registerCallback
        (boost::bind(&ProbabilisticOccupancyGridMapping::PointCloud2TFFilterCallback, this, _1, sensorNumber));
  }
}

void ProbabilisticOccupancyGridMapping::PointCloud2TFFilterCallback(
    const boost::shared_ptr<const sensor_msgs::PointCloud2 >& scanInput, int& sensorNumber){

  // Sensor Header (time from message, header from param)
  std_msgs::Header header = scanInput->header;
  header.frame_id = voxel.map.sensorInformation[sensorNumber].frame;

  // Initialize Measurement Number, Locations, & Colors
  int numRays;
  vector< vector<double> > meas ;
  vector< vector<int>    > color;

  // Conversions Required for PointCloud2 (colors or no colors)
  pcl::PCLPointCloud2 scan;
  pcl_conversions::toPCL(*scanInput,scan);
  if(voxel.map.sensorInformation[sensorNumber].hasColors){
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    numRays = ExtractPCL2Locations(scan, cloud, meas, color);
    for(int i = 0; i < numRays; i++){
      color[i] = {cloud.points[i].r, cloud.points[i].g, cloud.points[i].b};
    }
  }
  else{// no colors
    pcl::PointCloud<pcl::PointXYZ> cloud;
    numRays = ExtractPCL2Locations(scan, cloud, meas, color);
  }

  // Update the Map with Measurent Scan Ray-by-Ray & Time the Process
  voxel.map.UpdateMap(sensorNumber, numRays, meas, color);


}


template<typename cloudType>
int ProbabilisticOccupancyGridMapping::ExtractPCL2Locations(
    pcl::PCLPointCloud2& scan, cloudType& cloud, vector< vector<double> >& meas, vector< vector<int> >& color){

  // Extract Location/Color Data
  pcl::fromPCLPointCloud2(scan, cloud);

  int numRays = cloud.points.size();// cloud.width*cloud.height;
  meas. resize(numRays, vector<double>(voxel.map.dimension));
  color.resize(numRays, vector<int>(voxel.map.dimension))   ;
  for(int i = 0; i < numRays; i++)
    meas [i] = {cloud.points[i].x, cloud.points[i].y, cloud.points[i].z};

  return numRays;
}

}
