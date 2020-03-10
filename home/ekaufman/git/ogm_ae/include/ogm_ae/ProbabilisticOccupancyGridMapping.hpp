/* Probabilistic Occupancy Grid Mapping
 * by Evan Kaufman, Kuya Takami, and Taeyoung Lee
 * Flight Dynamics and Control Laboratory (FDCL)
 * Mechanical and Aerospace Engineering (MAE)
 * The George Washington University (GWU)
 * Contracted under the US Naval Research Laboratory (NRL)
*/

#ifndef PROBABILISTICOCCUPANCYGRIDMAPPING_HPP
#define PROBABILISTICOCCUPANCYGRIDMAPPING_HPP

// Voxel Shared Among ROS Packages
#include "voxel.h"

// Namespaces
using namespace std;

namespace probabilistic_occupancy_grid_mapping{
class ProbabilisticOccupancyGridMapping{

public:

  ProbabilisticOccupancyGridMapping();
  virtual ~ProbabilisticOccupancyGridMapping() = default;

protected:

  Voxel voxel;
  string objective;

  void PointCloud2TFFilterCallback(// PointCloud2 & TF message filter & map update
      const boost::shared_ptr<const sensor_msgs::PointCloud2 >& scanInput, int& sensorNumber);

  template<typename cloudType>
  int ExtractPCL2Locations(// use PCL libraries to extract point cloud positions
      pcl::PCLPointCloud2& scan, cloudType& cloud, vector< vector<double> >& meas, vector< vector<int> >& color);

};
}

#endif // PROBABILISTICOCCUPANCYGRIDMAPPING_HPP
