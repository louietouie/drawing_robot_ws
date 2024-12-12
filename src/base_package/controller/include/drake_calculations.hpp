#ifndef DRAKE_CALCULATIONS_HPP_
#define DRAKE_CALCULATIONS_HPP_

#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"

namespace base_package{
// namespace drake {

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

    private:

        MultibodyPlant<double> _plant;
        std::unique_ptr<Context<double>> _plantContextPointer; // initialization can be delayed since it is a pointer, but I might as well do it in the initializer list
        // const Context<double>& _plantContext;
        // // auto _modelRobot;
        const RigidBodyFrame<double>& _bodyFrame; // drake doesn't want copies of these frames made (DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RigidBodyFrame) error), so added & to make them references to the frames
        const RigidBodyFrame<double>& _worldFrame;
        
        std::unique_ptr<Context<T> >
        const RigidBodyFrame< T > &
};

}
// }

#endif