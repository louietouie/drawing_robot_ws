#include "include/drake_calculations.hpp"

namespace base_package {

    DifferentialInverseKinematicsCalculator::DifferentialInverseKinematicsCalculator()
        :_plant (0.0) // time step 0.0 for continuous system
    {
    }

    void DifferentialInverseKinematicsCalculator::load_model(std::string urdf) {

        Parser parser(&_plant);
        parser.AddModelsFromString(urdf, ".urdf");

        // without this new joint, _plant.num_positions is 9. With it, just 2.
        // sledgehammer_base_link_qw, sledgehammer_base_link_qx, sledgehammer_base_link_qy, sledgehammer_base_link_qz, sledgehammer_base_link_x, sledgehammer_base_link_y, sledgehammer_base_link_z, sledgehammer_shoulder_q, sledgehammer_elbow_q
        // valid names in model instance 'sledgehammer' are: base_link, end_effector, forearm, odrive_base, upperarm
        const auto& basebody = _plant.GetBodyByName("base_link");
        _plant.WeldFrames(_plant.world_frame(), basebody.body_frame(), RigidTransformd());
        _plant.Finalize();

        _plantContextPointer = _plant.CreateDefaultContext(); // this must come after plant Finalize

        // I should add some form of error checking 
        // const std::string & urdf = get_robot_description();
        // if (!urdf.empty()) {
        //     parser.AddModelFromString(urdf);
        // }

        // Might not be neccessary, is _plant itself already the robot?
        // auto _modelRobot = _plant.GetModelInstanceByName("the_robot_name");

    }

    Eigen::VectorXd DifferentialInverseKinematicsCalculator::calculateOneStep(Eigen::VectorXd goalPosition, Eigen::VectorXd currentPose) {

        _plant.SetPositions(_plantContextPointer.get(), currentPose); // this seems wierd. never seen unique_ptr.get()
        Eigen::MatrixXd p_inv = calculate2DPseudoInverseJacobian();
        Eigen::VectorXd currentEndEffectorPosition = calculateCartesianCoordinates("end_effector");
        Eigen::VectorXd desiredEndEffectorVelocities = goalPosition - currentEndEffectorPosition;
        Eigen::VectorXd targetJointVelocities = p_inv * desiredEndEffectorVelocities;
        Eigen::VectorXd targetJointPositions = currentPose + (targetJointVelocities * 0.01); // dt?

        return targetJointPositions;

        // Notes
        // VectorX vs Eigen::VectorXd?
        // VectorX - (Eigen::Matrix<Scalar, Eigen::Dynamic, 1>) https://drake.mit.edu/doxygen_cxx/namespacedrake.html#a77dd228fb4dd66a2c17dd3f7f38ffd85
        // Eigen::VectorXd - https://eigen.tuxfamily.org/dox/group__matrixtypedefs.html#ga8554c6170729f01c7572574837ecf618
        // Eigen library how to do operations - http://www.eigen.tuxfamily.org/dox/group__TutorialMatrixArithmetic.html

    }

    Eigen::MatrixXd DifferentialInverseKinematicsCalculator::calculate2DPseudoInverseJacobian() {

        // https://github.com/RobotLocomotion/drake/blob/0944b39967937ac56f497b0a85a88b7f22e5a6ce/examples/planar_gripper/gripper_brick_planning_constraint_helper.cc#L119

        Eigen::Matrix3Xd jacobian(3, _plant.num_positions()); // if spatial velocity, includes rotation, so 6, not 3.

        Eigen::Vector3d p_BoBp_B; // const?
        p_BoBp_B << 0.0,0.0,0.0;

        const Frame<double>& _endFrame = _plant.GetBodyByName("end_effector").body_frame();
        const Frame<double>& _worldFrame = _plant.world_frame();

        _plant.CalcJacobianTranslationalVelocity(
            *_plantContextPointer, drake::multibody::JacobianWrtVariable::kV, _endFrame, p_BoBp_B, _worldFrame, _worldFrame, &jacobian);


        // const Eigen::SparseMatrix<double> p_inv = _plant.MakeActuationMatrixPseudoinverse();
        Eigen::MatrixXd pinv = jacobian.completeOrthogonalDecomposition().pseudoInverse(); // Eigen Library Alternative

        return pinv;

    }

    Eigen::VectorXd DifferentialInverseKinematicsCalculator::calculateCartesianCoordinates(std::string joint_name) {

        const drake::multibody::RigidBody<double>& body = _plant.GetBodyByName(joint_name);
        drake::math::RigidTransform pose = _plant.EvalBodyPoseInWorld(*_plantContextPointer, body);
        Eigen::VectorXd translation_only = pose.translation();

        return translation_only;

    }
    
}