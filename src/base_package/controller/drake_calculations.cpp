#include "include/drake_calculations.hpp"

namespace base_package {

    DifferentialInverseKinematicsCalculator::DifferentialInverseKinematicsCalculator()
        :_plant (0.0) // time step 0.0 for continuous system
    {
    }

    void DifferentialInverseKinematicsCalculator::load_model(std::string urdf) {

        Parser parser(&_plant);
        parser.AddModelsFromString(urdf, ".urdf");
        _plant.Finalize();
        auto _plantContextPointer = _plant.CreateDefaultContext(); // this must come after plant Finalize

        // I should add some form of error checking 
        // const std::string & urdf = get_robot_description();
        // if (!urdf.empty()) {
        //     parser.AddModelFromString(urdf);
        // }

        // Might not be neccessary, is _plant itself already the robot?
        // auto _modelRobot = _plant.GetModelInstanceByName("the_robot_name");

        // is this body to world joint neccessary?
        // const auto& body = _plant.GetBodyByName("base_link");
        // _plant.AddJoint<drake::multibody::WeldJoint>("weld_base", _plant.world_body(), std::nullopt,
        //     body, std::nullopt,
        //     drake::Isometry3<double>::Identity());

    }

    Eigen::VectorXd DifferentialInverseKinematicsCalculator::calculateOneStep(Eigen::VectorXd goalPosition, Eigen::VectorXd currentPose) {

        // _plant.SetPositions(_plantContextPointer.get(), currentPose); // this seems wierd. never seen unique_ptr.get()
        
        Eigen::MatrixXd p_inv = calculate2DPseudoInverseJacobian();
        // Eigen::VectorXd currentEndEffectorPosition = calculateCartesianCoordinates("end_effector");
        // // const currentEndEffectorPosition = forward_kinematics(currentPose)
        // Eigen::VectorXd desiredEndEffectorVelocities = goalPosition - currentEndEffectorPosition;
        // Eigen::VectorXd targetJointVelocities = p_inv * desiredEndEffectorVelocities;
        // // const targetPositions = integrate(targetJointVelocities)
        // Eigen::VectorXd targetPositions = currentPose + targetJointVelocities;
        // return(targetPositions);

        Eigen::VectorXd test;
        return test;

        // Notes
        // VectorX vs Eigen::VectorXd?
        // VectorX - (Eigen::Matrix<Scalar, Eigen::Dynamic, 1>) https://drake.mit.edu/doxygen_cxx/namespacedrake.html#a77dd228fb4dd66a2c17dd3f7f38ffd85
        // Eigen::VectorXd - https://eigen.tuxfamily.org/dox/group__matrixtypedefs.html#ga8554c6170729f01c7572574837ecf618
        // Eigen library how to do operations - http://www.eigen.tuxfamily.org/dox/group__TutorialMatrixArithmetic.html

    }

    Eigen::MatrixXd DifferentialInverseKinematicsCalculator::calculate2DPseudoInverseJacobian() {

        // https://github.com/RobotLocomotion/drake/blob/0944b39967937ac56f497b0a85a88b7f22e5a6ce/examples/planar_gripper/gripper_brick_planning_constraint_helper.cc#L119

        Eigen::Matrix3Xd jacobian(3, _plant.num_positions());
        // Eigen::MatrixXd jacobian;

        Eigen::Vector3d p_BoBp_B;
        p_BoBp_B << 0,0,0;

        const RigidBodyFrame<double>& _endFrame = _plant.GetBodyByName("end_effector").body_frame();
        const RigidBodyFrame<double>& _worldFrame = _plant.world_frame();

        _plant.CalcJacobianTranslationalVelocity(
            *_plantContextPointer, drake::multibody::JacobianWrtVariable::kQDot, _endFrame, p_BoBp_B, _worldFrame, _worldFrame, &jacobian);

        // // const Eigen::SparseMatrix<double> p_inv = _plant.MakeActuationMatrixPseudoinverse();
        // Eigen::MatrixXd pinv = jacobian.completeOrthogonalDecomposition().pseudoInverse(); // Eigen Library Alternative

        Eigen::MatrixXd pinv;
        return pinv;

    }

    int DifferentialInverseKinematicsCalculator::getp() {
        return _plant.num_positions();
    }

    Eigen::VectorXd DifferentialInverseKinematicsCalculator::calculateCartesianCoordinates(std::string joint_name) {

        const drake::multibody::RigidBody<double>& body = _plant.GetBodyByName(joint_name);
        drake::math::RigidTransform pose = _plant.EvalBodyPoseInWorld(*_plantContextPointer, body);
        Eigen::VectorXd translation_only = pose.translation();

        return translation_only;

    }
    
}