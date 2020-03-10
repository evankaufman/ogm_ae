#include "ogm_ae/aux_fun.h"
#include <gazebo_msgs/SetModelState.h>

using namespace std;

int main(int argc, char **argv)
{

  // Initializations
  ros::init(argc, argv, "gazebo_communications");
  ros::NodeHandle nh, privateNH("~");
  ros::Rate loopRate(100);
  bool GazeboClient(false);

  // Private Parameters: All Gazebo Model State Subscribers
  int numModels; privateNH.param("num_models", numModels, numModels);
  vector<ros::Subscriber> subHandles(numModels);
  vector<GenSubscriber<ogm_ae::ModelStateStamped> > subObjectVec(numModels);
  vector<pair<int, double> > msgIndsAndTimeStamps(numModels);
  pair<int, double> pairElement(0, ros::Time::now().toSec());
  for(int i(0); i < numModels; i++){
    subHandles[i] = nh.subscribe
        ("gazebo_states_topic_"+to_string(i), 1, &GenSubscriber<ogm_ae::ModelStateStamped>::callback, &subObjectVec[i]);
    pairElement.first = i;
    msgIndsAndTimeStamps[i] = pairElement;
  }

  // Gazebo Client/Publisher
  ros::ServiceClient clientSetModelState; ros::Publisher gazeboPub;
  gazebo_msgs::SetModelState setModelStateSRV;
  if(GazeboClient)
    clientSetModelState = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  else
    gazeboPub   = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1, true);

  double tMsg, tNow;
  while (ros::ok()){

    // Current Time
    tNow = ros::Time::now().toSec();


    // Check Message Time Stamps from Start of Lineup
    for(int i(0); i < numModels; i++){
      tMsg = subObjectVec[msgIndsAndTimeStamps[i].first].msg.header.stamp.toSec();
      if(tMsg > msgIndsAndTimeStamps[i].second){

        // Update Message Time Stamps
        msgIndsAndTimeStamps[i].second = tMsg;

        // Set Model State Service Call/Publish
        if(GazeboClient){
          setModelStateSRV.request.model_state = subObjectVec[msgIndsAndTimeStamps[i].first].msg.model_state;
          clientSetModelState.call(setModelStateSRV);
          if(!setModelStateSRV.response.success)
              cout << "Set model state failed: response message:\n" << setModelStateSRV.response.status_message << endl;
        }
        else
          gazeboPub.publish(subObjectVec[msgIndsAndTimeStamps[i].first].msg.model_state);


        // Move to End of Line
        pairElement = msgIndsAndTimeStamps[i];
        msgIndsAndTimeStamps.erase(msgIndsAndTimeStamps.begin()+i);
        msgIndsAndTimeStamps.push_back(pairElement);

      }
    }

    // Update Subscriptions
    ros::spinOnce();
    loopRate.sleep();

  }

  return 0;
}
