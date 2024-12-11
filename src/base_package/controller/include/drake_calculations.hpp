#ifndef DRAKE_CALCULATIONS_HPP_
#define DRAKE_CALCULATIONS_HPP_

#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"

namespace drake{

using multibody::MultibodyPlant;
using multibody::Parser;

class DifferentialInverseKinematicsCalculator {

    public:
        DifferentialInverseKinematicsCalculator();

        void init(std::string urdf) {}

        Eigen::VectorXd calculateOneStep(Eigen::VectorXd goalPosition, Eigen::VectorXd currentPose); // returns vector of goal joint velocities (or integrated into positions?).

        Eigen::MatrixXd calculate2DPseudoInverseJacobian(Eigen::VectorXd currentPose); // based on current robot pose

    private:
        MultibodyPlant _plant;
        auto _plantContext
        auto _modelRobot;
        auto _bodyFrame;
        auto _worldFrame;

}

}


#endif