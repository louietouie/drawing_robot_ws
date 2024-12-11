#include "include/drake_calculations.hpp"

namespace drake {

    DifferentialInverseKinematicsCalculator::DifferentialInverseKinematicsCalculator(){}

    DifferentialInverseKinematicsCalculator::init(std::string urdf) {

        // const std::string & urdf = get_robot_description();
        // if (!urdf.empty()) {
        //     parser.AddModelFromString(urdf);
        // }

        // CREATE MULTIBODY PLANT FROM URDF STRING
        MultibodyPlant<double> _plant(0.0);
        Parser parser(&_modelRobot);
        parser.AddModelsFromString(urdf);
        _plant.Finalize();

        // CREATE MULTIBODY PLANT CONTEXT
        auto _plantContext = _plant.CreateDefaultContext();

        // Might not be neccessary, is _plant itself already the robot?
        // auto _modelRobot = _plant.GetModelInstanceByName("the_robot_name");

        // CREATE BODY AND WORLD FRAMES
        const auto& _bodyFrame = _plant.getBodyByName("base_link").body_frame();
        // self._G = plant.GetBodyByName("body").body_frame();
        _plant->AddJoint<multibody::WeldJoint>("weld_base", _plant->world_body(), nullopt,
            _bodyFrame, nullopt,
            Isometry3<double>::Identity());

        const auto& _worldFrame = _plant.world_frame();

    }

    Eigen::VectorXd DifferentialInverseKinematicsCalculator::calculateOneStep(VectorX<double> goalPosition, Eigen::VectorXd currentPose) {

        const Eigen::SparseMatrix<double> p_inv = calculate2DPseudoInverseJacobian(currentPose);
        // const currentEndEffectorPosition = forward_kinematics(currentPose)
        // const desiredEndEffectorVelocities = goalPosition - currentEndEffectorPosition
        // const targetJointVelocities = p_inv.dot(desiredEndEffectorVelocities);
        // const targetPositions = integrate(targetJointVelocities)
        Eigen::VectorXd targetPositions;
        return(targetPositions)

        // Notes
        // VectorX vs Eigen::VectorXd?
        // VectorX - (Eigen::Matrix<Scalar, Eigen::Dynamic, 1>) https://drake.mit.edu/doxygen_cxx/namespacedrake.html#a77dd228fb4dd66a2c17dd3f7f38ffd85
        // Eigen::VectorXd - https://eigen.tuxfamily.org/dox/group__matrixtypedefs.html#ga8554c6170729f01c7572574837ecf618
        // Eigen library how to do operations - http://www.eigen.tuxfamily.org/dox/group__TutorialMatrixArithmetic.html

    }


    Eigen::MatrixXd DifferentialInverseKinematicsCalculator::calculate2DPseudoInverseJacobian(Eigen::VectorXd currentPose) {

        _plant->SetPositions(&_plantContext, currentPose);
        // python
        // self._plant.SetPositions(self._plant_context, self._iiwa, q);

        // https://github.com/RobotLocomotion/drake/blob/0944b39967937ac56f497b0a85a88b7f22e5a6ce/examples/planar_gripper/gripper_brick_planning_constraint_helper.cc#L119

        Eigen::Matrix3Xd jacobian(3, _plant.plant().num_positions());

        // Eigen::Ref< const Vector3< T >> & p_BoBp_B(0,0,0);
        Eigen::Vector3 p_BoBp_B(0,0,0);

        jacobian = _plant.CalcJacobianSpatialVelocity(
            _plantContext, multibody::JacobianWrtVariable::kQDot, _bodyFrame, p_BoBp_B, _worldFrame, _worldFrame, &jacobian
        );

        // const Eigen::SparseMatrix<double> p_inv = _plant.MakeActuationMatrixPseudoinverse();
        Eigen::MatrixXd pinv = jacobian.completeOrthogonalDecomposition().pseudoInverse(); // Eigen Library Alternative

        return p_inv;

    }
    
}