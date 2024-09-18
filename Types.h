#pragma once


#include "Kinematics/KinematicLibraryPaths.h"

// Here is defined all of the necessary structs


namespace medlab{
    using CurvFun  = CTR::Functions::constant_fun< Eigen::Vector2d >;
    using TubeType = CTR::Tube< CurvFun >;

    using Cannula3 = std::tuple<TubeType, TubeType, TubeType>; // CTR3 Robot architecture
    using Cannula2 = std::tuple<TubeType, TubeType>; // CTR2 Robot architecture

    using OType    = CTR::DeclareOptions < CTR::Option::ComputeJacobian,
                                        CTR::Option::ComputeGeometry >::options;

    using CurvFun  = CTR::Functions::constant_fun< Eigen::Vector2d >;
using TubeType = CTR::Tube< CurvFun >;
using Cannula3 = std::tuple<TubeType, TubeType, TubeType>; // CTR3 Robot architecture
using Cannula2 = std::tuple<TubeType, TubeType>; // CTR2 Robot architecture
// using CannulaN = std::list<TubeType>;
using OType    = CTR::DeclareOptions < CTR::Option::ComputeJacobian,
                                    CTR::Option::ComputeGeometry >::options;
// use this OType if stability and compliance are necessary
// using OType    = CTR::DeclareOptions < CTR::Option::ComputeJacobian,
//                                     CTR::Option::ComputeGeometry,
//                                     CTR::Option::ComputeStability,
//                                     CTR::Option::ComputeCompliance >::options;


// public Cannula3 CannulaNToCannula3(CannulaN c)
// {

// }

// public Cannula2 CannulaNToCannula2(CannulaN c)
// {

// }

// public CannulaN Cannula2ToCannulaN(Cannula2 c)
// {

// }

// ----------------------------------------------------------
// ----------------    General Structs     ------------------
// ----------------------------------------------------------

// struct for physical limits of the robot
//    dynamic means that the limit changes as the robot moves
//    e.g. the inner carriage can't go past the outer carriage
struct Bound{
  double value = 0.0;
  bool isDynamic = false;
};
struct Limits{
  Bound minb; // minimum bound
  Bound maxb; // maximum bound
};



// Base tube struct
// Tube Geometry
struct TubeParams{
  double L;  // [m]   total nitinol tube length
  double Lt; // [m]   straight nitinol tube length
  double Ltorq; // [m] torque tube lenght
  double k;  // [1/m] curvature
  double OD; // [m]   outer diameter
  double ID; // [m]   inner diameter
  double E; // [Pa] Elastic Modulus
  double G; // [Pa] Shear Modulus
};


std::ostream& operator<<(std::ostream& os, const TubeParams& obj){
  os << "L: " << obj.L << ", Lt: " << obj.Lt << ", Ltorq: " << obj.Ltorq << ", k: " << obj.k <<
        ", OD: " << obj.OD << ", ID: " << obj.ID << ", E: " << obj.E << ", G: " << obj.G;
  return os;
}


struct InterpRet {      // Interpolated CTR3 & CTR2 Backbone
  Eigen::VectorXd s;  // Vector containing arc length values (Nx1)  -> Where each tube starts and ends
  Eigen::MatrixXd p;  // Positions stacked in a vector form along the arc length (Nx3)
  Eigen::MatrixXd q;  // Quaternions
};

struct WeightingRet {
  RoboticsMath::Matrix6d W;
  Eigen::VectorXd dh;
};


struct resolvedRatesControllerParams{
  float maxTransSpeed;
  float maxRotSpeed;
  float highpassTrans;
  float highpassRot;
  float saturation_threshold;
  float RAD_to_REV;   // (rev/rad) conversion of radians to motor revolutions
  float M_to_REV;     // (m/rad) conversion of meters to motor revolutions

  Eigen::Vector3d w_tracking_coeffs;
  Eigen::Vector4d w_damping_coeffs;

  // float w_tracking_x;
  // float w_tracking_y;
  // float w_tracking_z;
  
  // float w_damping_alpha0;
  // float w_damping_alpha1;
  // float w_damping_beta0;
  // float w_damping_beta1;
};

struct RobotCarriageParams{
  double travel_span; //[m]
  double L_endoscope; //[m]

  Eigen::Matrix<double,2,2> carriageDim_values; //[m]
};

struct RobotCarriageParams_3Carriages{
  double travel_span; //[m]
  double L_endoscope; //[m]

  Eigen::Matrix<double,2,3> carriageDim_values; //[m]
};

// ----------------------------------------------------------
// ---------------- 3 Tube robot variables ------------------
// ----------------------------------------------------------
struct CTR3RobotParams {

  // Material Properties
  double E;
  double G;

  TubeParams tube1; // inner tube
  TubeParams tube2; // middle tube
  TubeParams tube3; // outer tube
  unsigned int arm_id; // Arm Identifier

  // starting configuration
  RoboticsMath::Vector6d qHome; // { psi_l_home [rad], beta_home [m] }
  Eigen::Matrix4d baseFrame; // TODO: not used anywhere?
};


std::ostream& operator<<(std::ostream& os, const CTR3RobotParams& obj){
  os << "Tube1: " << obj.tube1 << ", Tube2: " << obj.tube2 << ", Tube3: " << obj.tube3 << ", Home: " << obj.qHome.transpose();
  return os;
}

struct CTR3KinematicsInputVector { // format of the input vector fed into Kinematics_with_dense_output()
  Eigen::Vector3d PsiL;
  Eigen::Vector3d Beta;
  Eigen::Vector3d Ftip;
  Eigen::Vector3d Ttip;
};

struct KinOut {
  Eigen::Vector3d Ptip;
  Eigen::Vector4d Qtip;
  Eigen::Vector3d Alpha;
  RoboticsMath::Matrix6d Jbody;
  RoboticsMath::Matrix6d Jhybrid;
  double Stability;
};


// ----------------------------------------------------------
// ----------------- 2 Tube robot variables -----------------
// ----------------------------------------------------------
struct CTR2RobotParams {

  TubeParams tube1; // inner tube
  TubeParams tube2; // outer tube
  unsigned int arm_id; // Arm Identifier

  // starting configuration
  RoboticsMath::Vector4d qHome; // { psi_l_home [rad], beta_home [m] }
  Eigen::Matrix4d baseFrame; // TODO: not used anywhere?
};

std::ostream& operator<<(std::ostream& os, const CTR2RobotParams& obj){
  os << "Tube1: " << obj.tube1 << ", Tube2: " << obj.tube2 << ", Home: " << obj.qHome.transpose();
  return os;
}

struct CTR2KinematicsInputVector { // format of the input vector fed into Kinematics_with_dense_output()
  Eigen::Vector2d PsiL;
  Eigen::Vector2d Beta;
  Eigen::Vector3d Ftip;
  Eigen::Vector3d Ttip;
};

std::ostream& operator<<(std::ostream& os, const CTR2KinematicsInputVector& obj){
  os << "PsiL: " << obj.PsiL.transpose() << ", Beta: " << obj.Beta.transpose() << ", Ftip: " << obj.Ftip.transpose() << ", Ttip: " << obj.Ttip.transpose();
  return os;
}


// 2 Tube KinOut
struct KinOut2T { 
  Eigen::Vector3d Ptip;
  Eigen::Vector4d Qtip;
  Eigen::Vector2d Alpha; 
  RoboticsMath::Matrix6x4d Jbody;
  RoboticsMath::Matrix6x4d Jhybrid;
  double Stability;
};

std::ostream& operator<<(std::ostream& os, const KinOut2T& obj){
  os << "pTip: " << obj.Ptip.transpose() << ", quatTip: " << obj.Qtip.transpose() << 
  ", Alpha: " << obj.Alpha.transpose() << std::endl << "Jbody: " << obj.Jbody <<  std::endl << "Jhybrid: " << obj.Jhybrid;
  return os;
}




}





