#include "ctr2_robot.h"
#include "Kinematics/KinematicLibraryPaths.h"
#include "robotics_math.h"
// #include "Types.h"

#include <vector>
#include <tuple>
#include <memory>
#include <cmath>
#include <utility>
#include <Eigen/Dense>

//#include <ros/console.h>

medlab::Cannula2 CTR2Robot::createCannula2(medlab::CTR2RobotParams params)
{
  // Curvature of each tube
  CTR::Functions::constant_fun< CTR::Vector<2>::type > k_fun1( (params.tube1.k)*Eigen::Vector2d::UnitX() );
  CTR::Functions::constant_fun< CTR::Vector<2>::type > k_fun2( (params.tube2.k)*Eigen::Vector2d::UnitX() );

  // Define tubes
  medlab::TubeType T1 = CTR::make_annular_tube( params.tube1.L, params.tube1.Lt, params.tube1.OD, params.tube1.ID, k_fun1, params.tube1.E, params.tube1.G);
  medlab::TubeType T2 = CTR::make_annular_tube( params.tube2.L, params.tube2.Lt, params.tube2.OD, params.tube2.ID, k_fun2, params.tube2.E, params.tube2.G);

  // medlab::TubeType T1 = CTR::make_annular_tube( params.L1, params.Lt1, params.OD1, params.ID1, k_fun1, params.E1, params.G1 );
  // medlab::TubeType T2 = CTR::make_annular_tube( params.L2, params.Lt2, params.OD2, params.ID2, k_fun2, params.E2, params.G2 );

  // Assemble cannula
  medlab::Cannula2 cannula = std::make_tuple( T1, T2);

  return cannula;
}

CTR2Robot::CTR2Robot(medlab::Cannula2 cannula):
  WStability(RoboticsMath::Matrix4d::Identity()),
//  BaseFrame_WORLD(Eigen::Matrix4d::Identity()),
  cannula_(cannula),
//  qHome_(RoboticsMath::Vector6d::Zero()),
  nInterp_(250)
{
}

CTR2Robot::~CTR2Robot()
{
}

bool CTR2Robot::init(medlab::CTR2RobotParams params)
{
  currCannulaParams_ = params;

  currKinematicsInputVector_.PsiL = params.qHome.head(2);
  currKinematicsInputVector_.Beta = params.qHome.tail(2);
  currKinematicsInputVector_.Ftip = Eigen::Vector3d::Zero();
  currKinematicsInputVector_.Ttip = Eigen::Vector3d::Zero();
  currQVec_ = params.qHome;

  callKinematicsWithDenseOutput(currKinematicsInputVector_); // currInterpolatedBackbone_ set in here

  return true;
}


RoboticsMath::Vector4d CTR2Robot::GetQRelative()
{
  // !! NOTE: THIS IS ONLY VALID FOR THE ENDONASAL ROBOT DUE TO THE BULK TRANSLATION !!

  RoboticsMath::Vector4d qRelative;
  qRelative.topRows(2) = currKinematics.Alpha;
//  qRelative.bottomRows(3) = currQVec_.bottomRows(3) - GetQHome().bottomRows(3);

  // qRelative[5] = currQVec_[5] - GetQHome()[5];
  // qRelative[4] = currQVec_[4] - GetQHome()[4] - qRelative[5];
  qRelative[3] = currQVec_[3] - GetQHome()[3] - qRelative[3];
  qRelative[2] = currQVec_[2] - GetQHome()[2] - qRelative[3];
//  std::cout << "QVec[4] = " << currQVec_[4] << ", QRelative[4] = " << qRelative[4] << std::endl;
//  std::cout << "QVec = " << currQVec_.bottomRows(3).transpose() << std::endl;

  return qRelative;
}

void CTR2Robot::callKinematicsWithDenseOutput(medlab::CTR2KinematicsInputVector newKinematicsInput)
{
  // Set the internal joint values
  currQVec_.topRows(2) = newKinematicsInput.PsiL;
  currQVec_.bottomRows(2) = newKinematicsInput.Beta;

  // Create the Kinematics Solver
  CTR::KinRetDense< CTR::State< std::tuple_size<medlab::Cannula2>::type::value, medlab::OType >> ret1 = CTR::Kinematics_with_dense_output(cannula_, newKinematicsInput, medlab::OType());

  RoboticsMath::Matrix6x4d Jbody;
  Jbody = CTR::GetTipJacobianForTube1(ret1.y_final);    
  // std::cout<<"Jacobian Raw, Jbody: " << CTR::GetTipJacobianForTube1(ret1.y_final)<< std::endl;

  double Stability;
  Stability = CTR::GetStability(ret1.y_final);
//  ROS_INFO_STREAM("Stability: " << Stability);
  ////////////////////
  /// Transform To Base Frame
  ////////////////////
//  poseData = CTR3Robot::transformToBaseFrame(ret1);

    // Pick out arc length points
    nPts_ = ret1.arc_length_points.size();

//    ROS_INFO_STREAM("nPts_ = " << nPts_);

    double* ptr = &ret1.arc_length_points[0];
    Eigen::Map<Eigen::VectorXd> s(ptr, nPts_);

    double zeroIndex;
    Eigen::VectorXd zeroIndexVec;
    Eigen::Vector3d pZero;
    Eigen::Vector4d qZero;
    Eigen::Matrix4d gZero;
    Eigen::Matrix4d gStarZero;
    Eigen::Matrix4d gStarL;

    int count = 0;
    for (int i=0; i < s.size(); ++i)
    {
      if (std::abs(s(i)) < 1.0e-7) {
        zeroIndexVec.resize(count+1);
        zeroIndexVec(count) = (double) i;
        count ++;
      }
    }

    zeroIndex = zeroIndexVec(count-1);

    pZero = ret1.dense_state_output.at( zeroIndex ).p;
    qZero = ret1.dense_state_output.at( zeroIndex ).q;
    gZero = RoboticsMath::assembleTransformation(RoboticsMath::quat2rotm(qZero),pZero);
    gStarZero = RoboticsMath::assembleTransformation(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
    gStarL = gStarZero*RoboticsMath::inverseTransform(gZero);

    Eigen::Matrix<double, 7, Eigen::Dynamic> poseData;
    poseData.resize(7, nPts_);
    for (int i =0; i < nPts_; ++i)
    {
      Eigen::Vector3d pi = ret1.dense_state_output.at( i ).p;
      Eigen::Vector4d qi = ret1.dense_state_output.at( i ).q;
      Eigen::Matrix4d gi = RoboticsMath::assembleTransformation(RoboticsMath::quat2rotm(qi),pi);
      Eigen::Matrix4d gStari = gStarL*gi;
      RoboticsMath::Vector7d xi;
      xi = RoboticsMath::collapseTransform(gStari);
      poseData.col(i) = xi;
    }

  interpolateBackbone(s, poseData);

  /////////////////////
  /////////////////////

  Eigen::Vector3d pTip;
  Eigen::Vector4d qBishop;
  Eigen::Matrix3d RBishop;
  Eigen::Matrix3d Rtip;
  Eigen::Vector4d qTip;

  pTip = ret1.pTip;
  qBishop = ret1.qTip;


  RBishop = RoboticsMath::quat2rotm(qBishop);
  Eigen::Matrix3d rotate_psiL = Eigen::Matrix3d::Identity();
  rotate_psiL(0,0) = cos(newKinematicsInput.PsiL(0));
  rotate_psiL(0,1) = -sin(newKinematicsInput.PsiL(0));
  rotate_psiL(1,0) = sin(newKinematicsInput.PsiL(0));
  rotate_psiL(1,1) = cos(newKinematicsInput.PsiL(0));
  Rtip = RBishop*rotate_psiL;
  qTip = RoboticsMath::rotm2quat(Rtip);

  RoboticsMath::Matrix6x4d Jhybrid = RoboticsMath::Matrix6x4d::Zero();
  RoboticsMath::Matrix6d Rr = RoboticsMath::Matrix6d::Identity();
  Rr.topLeftCorner(3,3) = RBishop;
  Rr.bottomRightCorner(3,3) = RBishop;
  Jhybrid = Rr*Jbody;

  // Parse kinret into currKinematics_
  currKinematics.Ptip[0] = pTip[0];
  currKinematics.Ptip[1] = pTip[1];
  currKinematics.Ptip[2] = pTip[2];
  currKinematics.Qtip[0] = qTip[0];
  currKinematics.Qtip[1] = qTip[1];
  currKinematics.Qtip[2] = qTip[2];
  currKinematics.Qtip[3] = qTip[3];
  currKinematics.Alpha[0] = ret1.y_final.Psi[0];
  currKinematics.Alpha[1] = ret1.y_final.Psi[1];

  // currKinematics.Stability = Stability;
  currKinematics.Jbody = Jbody;
  currKinematics.Jhybrid = Jhybrid;  

  // std::cout<< "currKinematics.Stability: " << currKinematics.Stability<<std::endl;

  // std::cout<<"Done 2 Tube Kin. Jhybrid"<<Jhybrid <<"\n";
  // std::cout<<"Done 2 Tube Kin. Jbody"<<Jbody <<"\n";
}




void CTR2Robot::interpolateBackbone(Eigen::VectorXd sRef, Eigen::MatrixXd poseDataRef)
{
//  Eigen::Matrix<double, 4, Eigen::Dynamic> qRef;
  Eigen::Matrix<double, 4, Eigen::Dynamic, 0, 4, 50> qRef;

  qRef = poseDataRef.bottomRows(4);

//  ROS_INFO_STREAM("qRef is [" << qRef.rows() << " x " << qRef.cols() << "]");
//  ROS_INFO_STREAM("sRef is " << sRef.size());

  // Create a zero to one list for ref arc lengths
  int nRef = sRef.size();
  double totalArcLength = sRef(0) - sRef(nRef - 1);
  Eigen::VectorXd sRef0Vec(nRef);
  sRef0Vec.fill(sRef(nRef-1));
  Eigen::VectorXd zeroToOne = (1 / totalArcLength)*(sRef - sRef0Vec);

  // Create a zero to one vector including ref arc lengths & interp arc lengths (evenly spaced)
  int nPtsTotal = nInterp_ + nRef;

  Eigen::VectorXd xxLinspace(nInterp_);

  xxLinspace.fill(0.0);
  xxLinspace.setLinSpaced(nInterp_, 1.0, 0.0);
  Eigen::VectorXd xxUnsorted(nPtsTotal);
  xxUnsorted << xxLinspace, zeroToOne;

  std::vector<double> vec(xxUnsorted.data(), xxUnsorted.data() + xxUnsorted.size()); // Convert to std::vector
  std::sort(vec.begin(), vec.end()); // Sort the std::vector
  Eigen::VectorXd xxSorted = Eigen::Map<Eigen::VectorXd>(vec.data(), vec.size()); // Map it back to Eigen::VectorXd
  Eigen::VectorXd xx = xxSorted.reverse(); //Rich's interpolation functions call for descending order

  // std::sort(xxUnsorted.data(), xxUnsorted.data() + xxUnsorted.size());
  // Eigen::VectorXd xx = xxUnsorted.reverse(); 

  // List of return arc lengths in the original scaling/offset
  Eigen::VectorXd xxSRef0Vec(nPtsTotal);
  xxSRef0Vec.fill(sRef(nRef-1));
  Eigen::VectorXd sInterp = totalArcLength*xx + xxSRef0Vec;

  // Interpolate to find list of return quaternions
  Eigen::MatrixXd qInterp = RoboticsMath::quatInterp(qRef, zeroToOne, xx);

  // Interpolate to find list of return positions
  Eigen::VectorXd sInterpSpline = sInterp.reverse(); // spline requires ascending order

  std::vector<double> sVec;
  sVec.resize(sRef.size());
  Eigen::VectorXd::Map(&sVec[0], sRef.size()) = sRef.reverse();

  Eigen::VectorXd x = poseDataRef.row(0).reverse(); // interp x
  std::vector<double> xVec;
  xVec.resize(x.size());
  Eigen::VectorXd::Map(&xVec[0], x.size()) = x;
  tk::spline Sx;
  Sx.set_points(sVec, xVec);
  Eigen::VectorXd xInterp(nPtsTotal);

  xInterp.fill(0);
  for (int i = 0; i < nPtsTotal; i++)
  {
    xInterp(i) = Sx(sInterpSpline(i));
  }
  xInterp = xInterp.reverse().eval();

  Eigen::VectorXd y = poseDataRef.row(1).reverse(); // interp y
  std::vector<double> yVec;
  yVec.resize(y.size());
  Eigen::VectorXd::Map(&yVec[0], y.size()) = y;
  tk::spline Sy;
  Sy.set_points(sVec, yVec);
  Eigen::VectorXd yInterp(nPtsTotal);
  yInterp.fill(0);
  for (int i = 0; i < nPtsTotal; i++)
  {
    yInterp(i) = Sy(sInterpSpline(i));
  }
  yInterp = yInterp.reverse().eval();

  Eigen::VectorXd z = poseDataRef.row(2).reverse(); // interp z
  std::vector<double> zVec;
  zVec.resize(z.size());
  Eigen::VectorXd::Map(&zVec[0], z.size()) = z;
  tk::spline Sz;
  Sz.set_points(sVec, zVec);
  Eigen::VectorXd zInterp(nPtsTotal);
  for (int i = 0; i < nPtsTotal; i++)
  {
    zInterp(i) = Sz(sInterpSpline(i));
  }
  zInterp = zInterp.reverse().eval();

  Eigen::MatrixXd pInterp(3, nPtsTotal);
  pInterp.fill(0);
  pInterp.row(0) = xInterp.transpose();
  pInterp.row(1) = yInterp.transpose();
  pInterp.row(2) = zInterp.transpose();

  medlab::InterpRet interpResults;
  interpResults.s = sInterp;
  interpResults.p = pInterp;
  interpResults.q = qInterp;

  currInterpolatedBackbone_ = interpResults;

  // std::cout<<"Backbone Interp: "<<currInterpolatedBackbone_.p<<std::endl;
}




void CTR2Robot::computeStabilityWeightingMatrix(double sThreshold, double alphaS)
{
  // computes WStability and vS
  WStability = (exp(1.0 / (currKinematics.Stability - sThreshold)) - 1.0) * RoboticsMath::Matrix4d::Identity();

  RoboticsMath::Vector4d dSdq = RoboticsMath::Vector4d::Zero();

  double rotationalStep = 0.05*M_PI / 180.0;
  double translationalStep = 1.0E-5;

  RoboticsMath::Vector4d qFD;
  qFD.block<2,1>(0,0) = currQVec_.block<2,1>(0,0);
  qFD.block<2,1>(2,0) = currQVec_.block<2,1>(2,0);

  medlab::CTR2KinematicsInputVector qKinematicsUpper;
  medlab::CTR2KinematicsInputVector qKinematicsLower;

  for (int i=0; i < 4; i++)
  {
    RoboticsMath::Vector4d direction = RoboticsMath::Vector4d::Zero();

    double step = (i<2) ? rotationalStep : translationalStep;
    direction(i) = step;


    RoboticsMath::Vector4d qFDUpper = qFD + direction;
    RoboticsMath::Vector4d qFDLower = qFD - direction;

    qKinematicsUpper.PsiL = qFDUpper.block<2,1>(0,0);
    qKinematicsUpper.Beta = qFDUpper.block<2,1>(2,0);
    qKinematicsUpper.Ftip << 0.0, 0.0, 0.0;
    qKinematicsUpper.Ttip << 0.0, 0.0, 0.0;

    qKinematicsLower.PsiL = qFDLower.block<2,1>(0,0);
    qKinematicsLower.Beta = qFDLower.block<2,1>(2,0);
    qKinematicsLower.Ftip << 0.0, 0.0, 0.0;
    qKinematicsLower.Ttip << 0.0, 0.0, 0.0;

    auto retUpper = CTR::Kinematics_with_dense_output(cannula_, qKinematicsUpper, medlab::OType());
    auto retLower = CTR::Kinematics_with_dense_output(cannula_, qKinematicsLower, medlab::OType());

    double SUpper = CTR::GetStability(retUpper.y_final);
    double SLower = CTR::GetStability(retLower.y_final);

    dSdq(i) = (SUpper - SLower) / (2*step);

  }

  vS = alphaS*dSdq;
}



double CTR2Robot::capStability(double in)
{
  if(in<0.0)  return 0.0;
  if(in>1.0)  return 1.0;
  return in;
}