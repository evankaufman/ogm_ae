#ifndef AUX_FUN_H
#define AUX_FUN_H

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Accel.h>
#include <eigen3/Eigen/Dense>
#include "tf/transform_listener.h"
#include "tf/message_filter.h"

using namespace std;

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
    while(true){
      try{
        listener.lookupTransform(fromFrame, toFrame, ros::Time(0), transform);
        break;
      }
      catch (tf::TransformException ex){
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
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


template <typename NumType>
NumType  sgn(NumType& x){
  return (x > 0) - (x < 0);
}


//template <typename NumType>
//NumType EntropySingleCell(NumType P){
//  if(P <= 0 || P >= 1)
//    return (NumType)0;
//  else
//    return -P*log(P)-(1.0-P)*log(1.0-P);
//}

double EntropySingleCell(double& P){
  if(P <= 0 || P >= 1)
    return 0.0;
  else
    return -P*log(P)-(1.0-P)*log(1.0-P);
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



#endif // AUX_FUN_H
