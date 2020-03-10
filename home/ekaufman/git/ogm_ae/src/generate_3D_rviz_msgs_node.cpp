//
#include <sys/time.h>
#include <errno.h>
#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <vector>
#include <algorithm>
#include <iostream>
#include <sstream>

// ROS
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "ogm_ae/aux_fun.h"
#include "ogm_ae/voxel.h"

//#define occupancyVizMax 1.0
//#define occupancyVizMin 0.6


using namespace std;


int main(int argc, char **argv)
{




    ros::init(argc, argv, "generate_3D_rviz_msgs");

    Voxel voxel;

    string objective("mapping"), ogmTopic;
    bool rvizFull3D;
    ros::param::get("/visualization/ogm_3D_topic", ogmTopic);
    ros::param::get("/visualization/rviz_full_3D", rvizFull3D);

    // TODO: make param
    double occupancyVizMax = 1.0;
    double occupancyVizMin = 0.6;


    // Obtain Map Parameters
    voxel.GetParameters(objective);

    // Initialize Variables
    voxel.Initializations(objective);

//    // Subscribers
//    GenSubscriber<ogm_ae::PolyLeastSquaresTraj> trajInfoSubscription;
//    voxel.trajInfoSub = voxel.nh.subscribe
//        (voxel.robotInformation[robotNumber].trajInfoTopic, 1, &GenSubscriber<ogm_ae::PolyLeastSquaresTraj>::callback, &trajInfoSubscription);


    // Subscriptions (probability & color)
    GenSubscriber<ogm_ae::UpdatedMapCells    > mapChangesSubscription;
    GenSubscriber<std_msgs::Float64MultiArray> mapAllProbs           ;
    GenSubscriber<std_msgs::Int16MultiArray  > mapAllColors          ;

    if(voxel.map.trackDiff){
      voxel.mapChangesSub = voxel.nh.subscribe
          (voxel.map.changesTopic, 1, &GenSubscriber<ogm_ae::UpdatedMapCells>::callback, &mapChangesSubscription);
    }
    else{
      voxel.mapProbsSub = voxel.nh.subscribe
          (voxel.map.occTopic, 1, &GenSubscriber<std_msgs::Float64MultiArray>::callback, &mapAllProbs);
      voxel.mapRGBSub = voxel.nh.subscribe
          (voxel.map.rgbTopic, 1, &GenSubscriber<std_msgs::Int16MultiArray  >::callback, &mapAllColors);
    }

    // Publishers
    voxel.ogm3DViz = voxel.nh.advertise<visualization_msgs::MarkerArray>
        (ogmTopic , 1, true);


    // INITIALIZATIONS

    // Marker Array
    visualization_msgs::MarkerArray markerArrayMsg, markerBuff;
    visualization_msgs::Marker marker;
    marker.header.frame_id = voxel.map.frame;
    marker.ns = "map";
    marker.scale.x = voxel.map.alpha;
    marker.scale.y = voxel.map.alpha;
    marker.scale.z = voxel.map.alpha;
    marker.lifetime = ros::Duration();
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::CUBE;

    // Allocate Enough Memory Insize RViz
    marker.id = voxel.map.numCellsTotal-1;// max index
    marker.color.a = 0.0;
    markerArrayMsg.markers.resize(1, marker);
    voxel.ogm3DViz.publish(markerArrayMsg);
    ros::spinOnce();

    double probRange = occupancyVizMax-occupancyVizMin;
    vector<double> mapLoc(3);
    markerBuff.markers.resize(voxel.map.numCellsTotal);
    vector<double> mapLast(voxel.map.numCellsTotal, 0.0);
    double prob;
    int iMsgColor, iMapProb, iRGB;
    vector<int> color(3);// RGB

    ROS_INFO("Map visualization ready for mapping node.");

    while(ros::ok()){
      marker.header.stamp = ros::Time::now();
      int markerArrayInd(0);
      if(voxel.map.trackDiff && mapChangesSubscription.msg.inds.size() > 0){// pass markers from those among message of differences
        voxel.map.changedCells = mapChangesSubscription.msg;
        for(int i(0); i < voxel.map.changedCells.inds.size(); i++){
          iMapProb = voxel.map.changedCells.inds[i];
          prob = voxel.map.changedCells.probs[i];
          if(prob >= occupancyVizMin){// visibility threshold
            if(prob != voxel.map.occData.data[iMapProb]){// new probability
              iMsgColor = 3*i;
              for(iRGB = 0; iRGB < 3; iRGB++)// RGB Colors
                color[iRGB] = voxel.map.changedCells.colors[iMsgColor+iRGB];
              voxel.map.GenerateRvizMapMarker(marker, iMapProb, mapLoc, probRange, occupancyVizMin, prob, color);
              markerBuff.markers[markerArrayInd] = marker;
              markerArrayInd++;
              voxel.map.occData.data[iMapProb] = prob;
            }
          }
        }
      }
      else{// pass markers from those among message of the entire map
        voxel.map.occData = mapAllProbs.msg;
        voxel.map.rgbColors = mapAllColors.msg;
        if(voxel.map.occData.data.size() == voxel.map.numCellsTotal){
          for(int i = 0; i < voxel.map.numCellsTotal; i++){
            if(voxel.map.occData.data[i] > occupancyVizMin || mapLast[i] > occupancyVizMin){

              if(rvizFull3D || voxel.map.occData.data[i] != mapLast[i]){
                if(!rvizFull3D)
                  mapLast[i] = voxel.map.occData.data[i];

                prob = voxel.map.occData.data[i];
                iMsgColor = 3*i;
                for(iRGB = 0; iRGB < 3; iRGB++)
                  color[iRGB] = voxel.map.rgbColors.data[iMsgColor+iRGB];

                voxel.map.GenerateRvizMapMarker(marker, i, mapLoc, probRange, occupancyVizMin, prob, color);
                markerBuff.markers[markerArrayInd] = marker;
                markerArrayInd++;
              }
            }
          }
        }
      }


      // Resize & Insert
      markerArrayMsg.markers.resize(markerArrayInd);
      for(unsigned i = 0; i < markerArrayInd; i++)
        markerArrayMsg.markers[i] = markerBuff.markers[i];

      // Publish & Spin
      if(markerArrayInd > 0)
        voxel.ogm3DViz.publish(markerArrayMsg);
      ros::spinOnce();
    }


    return 0;
}
