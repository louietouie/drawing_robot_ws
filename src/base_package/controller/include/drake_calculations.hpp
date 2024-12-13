#ifndef DRAKE_CALCULATIONS_HPP_
#define DRAKE_CALCULATIONS_HPP_

#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
// #include "drake/multibody/plant/multibody_plant.h"
// #include "drake/common/drake_assert.h"
// #include "drake/geometry/scene_graph.h"
// #include "drake/multibody/benchmarks/acrobot/make_acrobot_plant.h"
// #include "drake/multibody/parsing/parser.h"
// #include "drake/multibody/tree/revolute_joint.h"
// #include "drake/systems/analysis/simulator.h"
// #include "drake/systems/controllers/linear_quadratic_regulator.h"
// #include "drake/systems/framework/diagram_builder.h"
// #include "drake/systems/primitives/affine_system.h"
// #include "drake/visualization/visualization_config_functions.h"

namespace base_package{
// namespace drake {

using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
// using multibody::RigidBody;
using drake::multibody::RigidBodyFrame;
// using multibody::Context;
using drake::systems::Context;

// using drake::geometry::SceneGraph;
// using drake::multibody::benchmarks::acrobot::AcrobotParameters;
// using drake::multibody::benchmarks::acrobot::MakeAcrobotPlant;
// using drake::multibody::AddMultibodyPlantSceneGraph;
// using drake::multibody::MultibodyPlant;
// using drake::multibody::Parser;
// using drake::multibody::JointActuator;
// using drake::multibody::RevoluteJoint;
// using drake::systems::Context;
// using drake::systems::InputPort;
// using Eigen::Vector2d;

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
        // std::unique_ptr<Context<double>> _bodyFramePtr;
        // std::unique_ptr<Context<double>> _worldFramePtr;
        // const RigidBodyFrame<double>& _bodyFrame; // drake doesn't want copies of these frames made (DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RigidBodyFrame) error), so added & to make them references to the frames
        // const RigidBodyFrame<double>& _worldFrame;
        
        // std::unique_ptr<Context<T> >
        // const RigidBodyFrame< T > &
};

}
// }

#endif