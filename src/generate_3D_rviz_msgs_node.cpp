#include "ogm_ae/Robot.hpp"
#include "ogm_ae/Mapping.hpp"

using namespace std;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "generate_3D_rviz_msgs");

  // Initializations
  ros::NodeHandle nh;
  string ogmTopic;
  double occupancyVizMin;
  vector<Robot> robotVec(0); int numSensorsTotal(0), numRobots(0);// map visualization not concerned with robots
  Mapping map;
  map.GeneralInit(numSensorsTotal, numRobots, robotVec);
  ros::param::get("/visualization/ogm_3D_topic", ogmTopic       );

  // Subscriber (probability & color changes)
  ros::Subscriber mapChangesSub; GenSubscriber<ogm_ae::UpdatedMapCells> mapChangesSubscription;
  mapChangesSub = nh.subscribe
      (map.changesTopic, 10, &GenSubscriber<ogm_ae::UpdatedMapCells>::callback, &mapChangesSubscription);

  // Publisher
  ros::Publisher ogm3DViz = nh.advertise<visualization_msgs::MarkerArray>(ogmTopic , 1, true);

  // Marker Array
  visualization_msgs::MarkerArray markerArrayMsg, markerBuff;
  visualization_msgs::Marker marker;
  double probRange;

  // Marker (Array) & Probability Range Initializations
  map.RvizMarkerArrayMsgInit(marker, markerBuff, occupancyVizMin, probRange);

  // Allocate Enough Memory Insize RViz
  marker.id = map.numCellsTotal-1;// max index
  markerArrayMsg.markers.resize(1, marker);
  ogm3DViz.publish(markerArrayMsg);
  ros::spinOnce();

  ROS_INFO("Map visualization ready for mapping node.");

  while(ros::ok()){
    map.changedCells = mapChangesSubscription.msg;

    // Generate & Publish Message
    if(map.GenerateRvizMarkerArrayMsg(map.changedCells, marker, markerArrayMsg, markerBuff, occupancyVizMin, probRange, map.occData, map.cellsColored))
      ogm3DViz.publish(markerArrayMsg);

    ros::spinOnce();
  }

  return 0;
}
