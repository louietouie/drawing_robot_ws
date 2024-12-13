#ifndef DRAKE_CALCULATIONS_HPP_
#define DRAKE_CALCULATIONS_HPP_

#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
// #include "drake/multibody/plant/multibody_plant.h"

namespace base_package{

using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
// using multibody::RigidBody;
using drake::multibody::RigidBodyFrame;
// using multibody::Context;
using drake::systems::Context;

class DifferentialInverseKinematicsCalculator {

    public:
        DifferentialInverseKinematicsCalculator();

        void load_model(std::string urdf);

        Eigen::VectorXd calculateOneStep(Eigen::VectorXd goalPosition, Eigen::VectorXd currentPose); // returns vector of goal joint velocities (or integrated into positions?).

        Eigen::MatrixXd calculate2DPseudoInverseJacobian(); // based on current robot pose

        Eigen::VectorXd calculateCartesianCoordinates(std::string joint_name);

        int getp();

    private:

        MultibodyPlant<double> _plant;
        std::unique_ptr<Context<double>> _plantContextPointer; // initialization can be delayed since it is a pointer, but I might as well do it in the initializer list

};

}

#endif