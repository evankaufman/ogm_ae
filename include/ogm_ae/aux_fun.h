#ifndef AUX_FUN_H
#define AUX_FUN_H

// Basic & Math
#include <iostream>
#include <math.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <ctime>
#include <cmath>
#include <algorithm>
#include <memory>
#include <string>

// ROS Headers
#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>

// std_msgs
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float64.h>

// geometry_msgs
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>

// sensor_msgs
#include <sensor_msgs/Range.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

// nav_msgs
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

// visualization_msgs
#include <visualization_msgs/MarkerArray.h>

// custom_msgs
#include <ogm_ae/UpdatedMapCells.h>
#include <ogm_ae/PolyLeastSquaresTraj.h>
#include <ogm_ae/ModelStateStamped.h>
#include <ogm_ae/DesiredState.h>

// message_filters
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// tf & pcl_ros
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include <pcl_ros/point_cloud.h>

using namespace std;

Eigen::Vector3d e1(1.0, 0.0, 0.0),
                e2(0.0, 1.0, 0.0),
                e3(0.0, 0.0, 1.0);

Eigen::Matrix3d Euler321ToMatrix(double rotZ1, double rotY2, double rotX3){
  double c1, c2, c3, s1, s2, s3;
//  double c1(cos(psi)), s1(sin(psi)), c2(cos(theta)), s2(sin(theta)), c3(cos(phi)), s3(sin(phi));

  c1 = cos(rotZ1);
  s1 = sin(rotZ1);
  c2 = cos(rotY2);
  s2 = sin(rotY2);
  c3 = cos(rotX3);
  s3 = sin(rotX3);
  Eigen::Matrix3d R;
  R << c1*c2, c1*s2*s3-c3*s1, s1*s3+c1*c3*s2,
       c2*s1, c1*c3+s1*s2*s3, c3*s1*s2-c1*s3,
       -s2  , c2*s3         , c2*c3         ;
  return R;
}

template <typename T>
class GenSubscriber
{
public:
    T msg;
    void callback(const typename T::ConstPtr& msgSub){msg = *msgSub;}
};

tf::StampedTransform LoopUntilTransformAcquired(
    tf::TransformListener& listener, string& fromFrame, string& toFrame){
    tf::StampedTransform transform;
    int numLoops(0);
    while(true){
      try{
        listener.waitForTransform(fromFrame, toFrame, ros::Time::now(), ros::Duration(0.01));
        listener.lookupTransform(fromFrame, toFrame, ros::Time(0), transform);
        break;
      }
      catch (tf::TransformException ex){
        numLoops++;
        if(numLoops > 9){
            ROS_WARN("Total of %d failed trials: %s from frame %s to frame %s", numLoops, ex.what(), fromFrame.c_str(), toFrame.c_str());
            ros::Duration(0.1).sleep();
        }
      }
    }
    return transform;
}


double Round2DecimalLevel(double valNeedRounding, int numDecimals){
  int multiplier = pow(10,numDecimals);
  double valRounded = floor(valNeedRounding*multiplier+0.5)/multiplier;
  return valRounded;
}

// STD Vector Operations

template<typename vecType>
vecType NormVec(
    vector<vecType>& vec){
  vecType normSqrd(0);
  for(unsigned i = 0; i < vec.size(); i++)
    normSqrd += vec[i]*vec[i];
  return sqrt(normSqrd);
}

template<typename vecType>
vecType Norm2VecsSqrd(
    vector<vecType>& vec1, vector<vecType>& vec2){
  vecType normSqrd(0);
  if(vec1.size() != vec2.size())
    return 0;
  for(unsigned i = 0; i < vec1.size(); i++)
    normSqrd += pow(vec1[i]-vec2[i],2);
  return normSqrd;
}


template<typename vecType>
vecType DotProd(
    vector<vecType>& vec1, vector<vecType>& vec2){
  vecType dotProd(0);
  for(unsigned i = 0; i < vec1.size(); i++)
    dotProd += vec1[i]*vec2[i];
  return dotProd;
}

template<typename vecType>
vector<vecType> AddVecs(
    vector<vecType>& vec1, vector<vecType>& vec2){
  int numel = vec1.size();
  vector<vecType> addedVec(numel, 0);
  for(unsigned i = 0; i < numel; i++)
    addedVec[i] = vec1[i]+vec2[i];
  return addedVec;
}

template<typename vecType>
vector<vecType> SubtractVecs(
    vector<vecType>& vec1, vector<vecType>& vec2){
  int numel = vec1.size();
  vector<vecType> addedVec(numel, 0);
  for(unsigned i = 0; i < numel; i++)
    addedVec[i] = vec1[i]-vec2[i];
  return addedVec;
}

template<typename vecType>
void MultiplyVec(
    vector<vecType>& vec, vecType& scalar){
  for(unsigned i = 0; i < vec.size(); i++)
    vec[i] *= scalar;
  return;
}

template<typename vecType>
void DivideVec(
    vector<vecType>& vec, vecType& scalar){
  for(unsigned i = 0; i < vec.size(); i++)
    vec[i] /= scalar;
  return;
}

template<typename vecType>
vecType NormBtwn2Vecs(
    vector<vecType>& vec1, vector<vecType>& vec2){
  vecType normSqrd(0);
  for(unsigned i = 0; i < vec1.size(); i++)
    normSqrd += pow(vec1[i]-vec2[i], 2);
  return sqrt(normSqrd);
}

template<typename vecType>
vector<vecType> GetUVec(
    vector<vecType>& fromVec, vector<vecType>& toVec, vecType& dist){
  vector<vecType> uvec = SubtractVecs(toVec, fromVec);
  dist = NormVec(uvec);
  if(dist > 0)// return nonzero vector
    DivideVec(uvec, dist);
  else// return zero vector
    uvec = vector<vecType>(fromVec.size(), 0);
  return uvec;
}

template<typename vecType>
bool NormalizeVec(
    vector<vecType>& vec){
  vecType normInit = NormVec(vec);
  if(normInit == (vecType)0)
    return false;
  else{
    for(unsigned i = 0; i < vec.size(); i++)
      vec[i] /= normInit;
  }
  return true;
}

template <typename NumType>
NumType  sgn(NumType& x){
  return (x > 0) - (x < 0);
}

template <typename EigenVecType>
void EigenVecToStdVec(EigenVecType& eigVec, vector<double>& stdVec){

  if(eigVec.size() != stdVec.size()){
    ROS_WARN("Eigen vector and standard vector sizes are different... correcting this.");
    stdVec.resize(eigVec.size());
  }

  for(int i(0); i < eigVec.size(); i++)
    stdVec[i] = eigVec(i);

  return;
}

template <typename EigenVecType>
void StdVecToEigenVec(vector<double>& stdVec, EigenVecType& eigVec){

  if(eigVec.size() != stdVec.size()){
    ROS_WARN("Eigen vector and standard vector sizes are different... correcting this.");
    stdVec.resize(eigVec.size());
  }

  for(int i(0); i < eigVec.size(); i++)
    eigVec(i) = stdVec[i];

  return;
}


template <typename ROSXYZType>
void ROSXYZTypeToVec(ROSXYZType& rosVector3, vector<double>& vec){

  if(vec.size() != 3){
    int vecSize(vec.size());
    ROS_ERROR("ROS xyz conversion to vector with size %d != 3", vecSize);
  }
  else{
    vec[0] = rosVector3.x;
    vec[1] = rosVector3.y;
    vec[2] = rosVector3.z;
  }

  return;
}

//template <typename NumType>
//NumType EntropySingleCell(NumType P){
//  if(P <= 0 || P >= 1)
//    return (NumType)0;
//  else
//    return -P*log(P)-(1.0-P)*log(1.0-P);
//}

double EntropySingleCell(double& P){

  // Maximized at 0.5
  if(P <= 0 || P >= 1)
    return 0.0;
  else
    return -P*log(P)-(1.0-P)*log(1.0-P);
}

double KaufmanUncertaintySingleCell(double& P, double& P0){

  // Maximized at P0, not 0.5
  double H;
  if(P < P0)
    H = P/(2*P0);
  else
    H = (1.0-P)/(2*(1.0-P0));
  return H*(1.0-H);
}

template <typename NumType>
NumType WeightedEntropySingleCell(NumType& P){
  return pow((1.0-P),1)*EntropySingleCell(P);
}

template <typename NumType>
NumType MaxOfVector(vector<NumType>& vec, int& maxInd){
  auto maxPtr = max_element(std::begin(vec), end(vec));
  maxInd = distance(begin(vec), maxPtr);
  return vec[maxInd];
}

template <typename NumType>
NumType MinOfVector(vector<NumType>& vec, int& minInd){
  auto minPtr = min_element(std::begin(vec), end(vec));
  minInd = distance(begin(vec), minPtr);
  return vec[minInd];
}

#define exp1 2.71828182846
template <typename NumType>
NumType BumpFunction(NumType distAway, NumType& maxDistAway){
  if(distAway >= maxDistAway || distAway <= 0.0)
    return 0;
  else{
    distAway /= maxDistAway;
    return exp1*exp(-1.0/(1.0-distAway*distAway));
  }
}

template <typename NumType>
NumType BumpFunctionNonzeroEverywhere(NumType distAway, NumType& maxDistAway, NumType& nonzeroVal){
  if(distAway >= maxDistAway || distAway <= 0.0)
    return nonzeroVal;
  else{
    distAway /= maxDistAway;
    return nonzeroVal+(1-nonzeroVal)*exp1*exp(-1.0/(1.0-distAway*distAway));
  }
}

template <typename NumType>
NumType GaussianLikeBump(NumType& distAway, NumType& sigSqrd, NumType& minFunVal){
  return exp(-0.5*distAway*distAway/sigSqrd)+minFunVal;
}

template <typename NumType>
NumType DistanceBump(NumType& dist, NumType& distAtMax, NumType& fMax, NumType& fFar, NumType& beta){
  if(dist < distAtMax)
    return 0.5*fMax*(1.0-cos(M_PI*dist/distAtMax));
  else
    return fFar+(fMax-fFar)*exp(-beta*pow(dist-distAtMax, 2));
}

template <typename NumType>
NumType FlatPeakHill(NumType& dist, NumType& distAtMax, NumType& fMax, NumType& fFar, NumType& beta){
  if(dist < distAtMax)
    return fMax;
  else
    return fFar+(fMax-fFar)*exp(-beta*pow(dist-distAtMax, 2));
}


template <typename indType, typename elementType, typename matType>
void PopulateStateCoeffMatrixPosition(
    indType& row, indType& colStart, indType& numCoeff,
    matType& mat, elementType& t, int numSgn){

  for(unsigned iTimes(0); iTimes < numCoeff; iTimes++)
    mat(row,colStart+iTimes) = numSgn*pow(t, iTimes);

  return;
}

template <typename indType, typename elementType, typename matType>
void PopulateStateCoeffMatrixVelocity(
    indType& row, indType& colStart, indType& numCoeff,
    matType& mat, elementType& t, int numSgn){

  for(indType iTimes(0); iTimes < numCoeff; iTimes++)
    mat(row,colStart+iTimes) = numSgn*iTimes*pow(t, iTimes-1);

  return;
}

template <typename quatType>
void YawRotToQuat(double& theta, quatType& quat){

  quat.x = 0.0;
  quat.y = 0.0;
  quat.z = sin(theta/2);
  quat.w = cos(theta/2);

  return;
}

template <typename quatType>
void QuatToYawRot(double& theta, quatType& quat){

  // Angle About 3rd Axis
  theta = 2*atan2(quat.z, quat.w);

  while(theta >  2*M_PI)
    theta -= 2*M_PI;

  while(theta < -2*M_PI)
    theta += 2*M_PI;

  return;
}

template <typename quatType, typename rotMatType>
void QuatToRotMatrix(quatType& quat, rotMatType& R){

  R(0,0) = 1.0-2*quat.y*quat.y-2*quat.z*quat.z; R(0,1) = 2*quat.x*quat.y-2*quat.z*quat.w;     R(0,2) = 2*quat.x*quat.z+2*quat.y*quat.w;
  R(1,0) = 2*quat.x*quat.y+2*quat.z*quat.w;     R(1,1) = 1.0-2*quat.x*quat.x-2*quat.z*quat.z; R(1,2) = 2*quat.y*quat.z-2*quat.x*quat.w;
  R(2,0) = 2*quat.x*quat.z-2*quat.y*quat.w;     R(2,1) = 2*quat.y*quat.z+2*quat.x*quat.w;     R(2,2) = 1.0-2*quat.x*quat.x-2*quat.y*quat.y;

  return;
}

template <typename quatType, typename rotMatType>
void RotMatrixToQuat(quatType& quat, rotMatType& R){

  // Modified Java code from http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/

  double tr = R(0,0) + R(1,1) + R(2,2);

  if (tr > 0) {
    double S = sqrt(tr+1.0) * 2; // S=4*quat.w
    quat.w = 0.25 * S;
    quat.x = (R(2,1) - R(1,2)) / S;
    quat.y = (R(0,2) - R(2,0)) / S;
    quat.z = (R(1,0) - R(0,1)) / S;
  } else if ((R(0,0) > R(1,1)) && (R(0,0) > R(2,2))) {
    double S = sqrt(1.0 + R(0,0) - R(1,1) - R(2,2)) * 2; // S=4*quat.x
    quat.w = (R(2,1) - R(1,2)) / S;
    quat.x = 0.25 * S;
    quat.y = (R(0,1) + R(1,0)) / S;
    quat.z = (R(0,2) + R(2,0)) / S;
  } else if (R(1,1) > R(2,2)) {
    double S = sqrt(1.0 + R(1,1) - R(0,0) - R(2,2)) * 2; // S=4*quat.y
    quat.w = (R(0,2) - R(2,0)) / S;
    quat.x = (R(0,1) + R(1,0)) / S;
    quat.y = 0.25 * S;
    quat.z = (R(1,2) + R(2,1)) / S;
  } else {
    double S = sqrt(1.0 + R(2,2) - R(0,0) - R(1,1)) * 2; // S=4*quat.z
    quat.w = (R(1,0) - R(0,1)) / S;
    quat.x = (R(0,2) + R(2,0)) / S;
    quat.y = (R(1,2) + R(2,1)) / S;
    quat.z = 0.25 * S;
  }

  return;
}

template <typename quatType, typename rotMatType>
quatType FixedQuatTransformation(quatType& quatWorldOther, rotMatType& RotMatrixOtherThis){
  quatType quatWorldThis;
  rotMatType RotMatrixWorldOther, RotMatrixWorldThis;

  // World To Other Rotation Matrix
  QuatToRotMatrix(quatWorldOther, RotMatrixWorldOther);

  // World to This Rotation Matrix
  RotMatrixWorldThis = RotMatrixWorldOther*RotMatrixOtherThis;

  // World to This Quaternion
  RotMatrixToQuat(quatWorldThis, RotMatrixWorldThis);

  return quatWorldThis;

}

template <typename eigenType, typename stackType>
bool EigenToStack(eigenType& eigenVec, stackType& stackObj, unsigned& numel){

  // Transfers first 'numel' elements from vertical 'eigenVec' to 'stackObj' without resizing

  if(numel > eigenVec.rows())
    return false;

  if(numel > stackObj.size())
    return false;

  for(unsigned i(0); i < numel; i++)
    stackObj[i] = eigenVec(i);

  return true;
}


class EigenROS{

public:
  double thetaAboutZ;
  Eigen::Vector3d x, xDot, xDDot, b1d, W, WDot, WDDot;
  Eigen::Matrix3d R;

  geometry_msgs::Pose  pose ;
  geometry_msgs::Twist twist;
  geometry_msgs::Accel accel;

//  geometry_msgs::Quaternion quat;

  void Eigen2Position(){

    pose.position.x = x(0);
    pose.position.y = x(1);
    pose.position.z = x(2);

    return;
  }

  void Theta2Attitude(){

    YawRotToQuat(thetaAboutZ, pose.orientation);

    return;
  }

  void Eigen2Twist(){

    // Translational Velocity
    twist.linear.x = xDot(0);
    twist.linear.y = xDot(1);
    twist.linear.z = xDot(2);

    // Rotational Velocity
    twist.angular.x = W(0);
    twist.angular.y = W(1);
    twist.angular.z = W(2);

    return;
  }

  void Eigen2Accel(){

    // Translational Acceleration
    accel.linear.x = xDDot(0);
    accel.linear.y = xDDot(1);
    accel.linear.z = xDDot(2);

    // Rotational Acceleration
    accel.angular.x = WDot(0);
    accel.angular.y = WDot(1);
    accel.angular.z = WDot(2);

    return;
  }


  // Velocity
//  pose.velo;
};

geometry_msgs::Quaternion MakeRollAndPitch0(geometry_msgs::Quaternion& sensorAtt, Eigen::Vector3d& b1Sensor, int b3Sgn){

  geometry_msgs::Quaternion sensorAtt0Pitch;
  // Set Attitude using Projection of b1 onto Plane Normal to e3  Eigen::Matrix3d sensorRotMatrix;
  Eigen::Matrix3d sensorRotMatrix;
  QuatToRotMatrix(sensorAtt, sensorRotMatrix);
  Eigen::Vector3d sensorUVecWorldFrame = sensorRotMatrix*e1;
  double theta = atan2(sensorUVecWorldFrame(1), sensorUVecWorldFrame(0));
  b1Sensor << cos(theta), sin(theta), 0.0;
//  Eigen::Vector3d sensorVecE1E2Plane = sensorUVecWorldFrame-(sensorUVecWorldFrame.dot(e3))*e1;
//  double normSensorVecE1E2Plane = sensorVecE1E2Plane.norm();
//  if(normSensorVecE1E2Plane > 0.0)
//    b1Sensor = sensorVecE1E2Plane/normSensorVecE1E2Plane;
//  else{
//    ROS_ERROR("While making smooth trajectory, initial b1 aligned with e3:\nSetting b1 = e1.");
//    b1Sensor = e1;
//  }
  if(b3Sgn != -1 && b3Sgn != 1)
    ROS_ERROR("Sign of desired depth sensor direction is not {-1, 1}, but %d.", b3Sgn);
  else{
    Eigen::Vector3d b3Sensor(0.0, 0.0, b3Sgn*1.0);
    Eigen::Vector3d b2Sensor = b3Sensor.cross(b1Sensor);
    sensorRotMatrix.col(0) = b1Sensor;
    sensorRotMatrix.col(1) = b2Sensor;
    sensorRotMatrix.col(2) = b3Sensor;
    RotMatrixToQuat(sensorAtt0Pitch, sensorRotMatrix);
  }

  return sensorAtt0Pitch;
}




// Pair Sorting Functions
bool PairIntDoubleSecondDoubleGreater(const pair<int,double> i, const pair<int,double> j){return (i.second > j.second);}
bool PairIntDoubleFirstIntLess       (const pair<int,double> i, const pair<int,double> j){return (i.first  < j.first );}



#endif // AUX_FUN_H
