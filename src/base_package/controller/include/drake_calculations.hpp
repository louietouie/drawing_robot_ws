#ifndef DRAKE_CALCULATIONS_HPP_
#define DRAKE_CALCULATIONS_HPP_

#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"

namespace drake{

using multibody::MultibodyPlant;
using multibody::Parser;
// using multibody::RigidBody;
using multibody::RigidBodyFrame;
// using multibody::Context;
using drake::systems::Context;

class DifferentialInverseKinematicsCalculator {

    public:
        DifferentialInverseKinematicsCalculator();

        void init(std::string urdf);

        Eigen::VectorXd calculateOneStep(Eigen::VectorXd goalPosition, Eigen::VectorXd currentPose); // returns vector of goal joint velocities (or integrated into positions?).

        Eigen::MatrixXd calculate2DPseudoInverseJacobian(Eigen::VectorXd currentPose); // based on current robot pose

    private:
        MultibodyPlant<double> _plant;
        std::unique_ptr<Context<double>> _plantContext;
        // auto _modelRobot;
        RigidBodyFrame<double> _bodyFrame;
        RigidBodyFrame<double> _worldFrame;
        
        // std::unique_ptr<Context<T> >
        // const RigidBodyFrame< T > &
};

}


#endif