#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Dense>

#include <boost/algorithm/string.hpp>
#include <iostream>

#include "Kinematics/KinematicLibraryPaths.h"
#include "robotics_math.h"
#include "Types.h"

#include "Robot/ctr2_robot.h"


int main() {
    medlab::TubeParams tube1{ 238e-3, 128e-3, 404e-3, 0.0, 1.11e-3, 0.91e-3, 60e9, 22.556e9};
    medlab::TubeParams tube2{ 51-3, 21-3, 394-3, 33.82, 1.52e-3, 1.32e-3, 60e9, 22.556e9};

    std::cout << "Tube 1: " << tube1 << std::endl;
    std::cout << "Tube 2: " << tube2 << std::endl;

    Eigen::Vector4d qHome;
    qHome << 0.0, 0.0, 3e-3, 1e-3;

    std::cout << "here" <<std::endl;

    Eigen::Matrix4d baseFrame = Eigen::Matrix4d::Identity();

    medlab::CTR2RobotParams ctr2params{tube1, tube2, 0, qHome, baseFrame};

    medlab::Cannula2 cannula2 = CTR2Robot::createCannula2(ctr2params);

    CTR2Robot ctr2 = CTR2Robot(cannula2);
    ctr2.init(ctr2params);
    

    Eigen::Vector2d alpha_input;
    alpha_input << 0, 0;

    Eigen::Vector2d beta_input;
    beta_input << 20e-3, 10e-3;

    medlab::CTR2KinematicsInputVector ctr2_input{ alpha_input, beta_input, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    std::cout << "Kinematic Input " << ctr2_input << std::endl;

    ctr2.callKinematicsWithDenseOutput(ctr2_input);

    std::cout << "Kinematic Output: "  << ctr2.currKinematics << std::endl;


}
