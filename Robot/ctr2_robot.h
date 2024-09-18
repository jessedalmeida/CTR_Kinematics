#pragma once

#include <Eigen/Dense>
#include "Kinematics/KinematicLibraryPaths.h"
#include "robotics_math.h"
#include "Types.h"
#include <memory>
#include <utility>

class CTR2Robot
{

public:
  static medlab::Cannula2 createCannula2(medlab::CTR2RobotParams params);

  CTR2Robot(medlab::Cannula2 cannula);
  // CTR2Robot();
  ~CTR2Robot();
  bool init(medlab::CTR2RobotParams params);

  medlab::CTR2RobotParams GetCurRobotParams() {return currCannulaParams_;}
  medlab::CTR2KinematicsInputVector GetCurrKinematicsInputVector() {return currKinematicsInputVector_;}
  RoboticsMath::Vector4d GetCurrQVec() {return currQVec_;}
  void SetCurrQVec(RoboticsMath::Vector4d qVec) {currQVec_ = qVec;}
  RoboticsMath::Vector4d GetQRelative();
  RoboticsMath::Vector4d GetQHome() {return currCannulaParams_.qHome;}
  medlab::InterpRet GetInterpolatedBackbone() {return currInterpolatedBackbone_;}
  int GetNPts() {return nPts_;}
  int GetNInterp() {return nInterp_;}

  void callKinematicsWithDenseOutput(medlab::CTR2KinematicsInputVector newKinematicsInput);

  void interpolateBackbone(Eigen::VectorXd sRef, Eigen::MatrixXd poseDataRef);
  void computeStabilityWeightingMatrix(double sThreshold, double alphaS);
  medlab::KinOut2T currKinematics;  // kinout from kinematics call, transformed into base frame & interpolated

  RoboticsMath::Matrix4d WStability;
  RoboticsMath::Vector4d vS;


private:
  medlab::Cannula2 cannula_;  // cannula tuple object fed to Hunter's kinematics
  medlab::CTR2RobotParams currCannulaParams_; // params that define the cannula3
  medlab::CTR2KinematicsInputVector currKinematicsInputVector_; // full input vector fed to kinematics call
  RoboticsMath::Vector4d currQVec_; // condensed kinematics input vector [psiL, beta]
  medlab::InterpRet currInterpolatedBackbone_; // interpolated cannula [sxn,pxn,qxn]
  int nPts_; // number of points along arclength returned by kinematics
  int nInterp_; // number of points to interpolate on backbone
  Eigen::Matrix<double, 8, Eigen::Dynamic> markersOut_; // Matrix for storing marker output to rviz (used in endonasal_teleop)
  double capStability(double in);

};