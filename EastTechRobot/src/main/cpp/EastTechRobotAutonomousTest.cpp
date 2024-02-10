////////////////////////////////////////////////////////////////////////////////
/// @file   EastTechRobotAutonomousTest.cpp
/// @author David Stalter
///
/// @details
/// Implementation of an autonomous test routines for EastTechRobot.
///
/// Copyright (c) 2024 East Technical High School
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
// <none>

// C INCLUDES
#include "frc/geometry/Pose2d.h"                    // for type declaration
#include "frc/geometry/Rotation2d.h"                // for type declaration
#include "frc/geometry/Translation2d.h"             // for type declaration
#include "frc/trajectory/Trajectory.h"              // for working with trajectories
#include "frc/trajectory/TrajectoryConfig.h"        // for creating a trajectory config
#include "frc/trajectory/TrajectoryGenerator.h"     // for generating a trajectory

// C++ INCLUDES
#include "RobotUtils.hpp"                           // for DisplayMessage()
#include "EastTechRobot.hpp"                        // for robot class declaration
#include "EastTechRobotAutonomous.hpp"              // for autonomous declarations


////////////////////////////////////////////////////////////////
/// @method EastTechRobot::AutonomousTestRoutine
///
/// Autonomous test routine.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::AutonomousTestRoutine()
{
    // Returning from here will enter the idle state until autonomous is over
    RobotUtils::DisplayMessage("Auto test routine done.");
}


////////////////////////////////////////////////////////////////
/// @method EastTechRobot::AutonomousTestSwerveRoutine
///
/// Autonomous swerve test routine.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::AutonomousTestSwerveRoutine()
{
    // Simple demonstration of directional movements
    AutonomousSwerveDriveSequence(ROBOT_FORWARD, ROBOT_NO_ROTATE, 0.10, 0.0, 1.0_s, true);
    AutonomousSwerveDriveSequence(ROBOT_LEFT, ROBOT_NO_ROTATE, 0.10, 0.0, 1.0_s, true);
    AutonomousSwerveDriveSequence(ROBOT_REVERSE, ROBOT_NO_ROTATE, 0.10, 0.0, 1.0_s, true);
    AutonomousSwerveDriveSequence(ROBOT_RIGHT, ROBOT_NO_ROTATE, 0.10, 0.0, 1.0_s, true);
    AutonomousSwerveDriveSequence(ROBOT_NO_DIRECTION, ROBOT_CLOCKWISE, 0.0, 0.10, 1.0_s, true);
    AutonomousSwerveDriveSequence(ROBOT_NO_DIRECTION, ROBOT_COUNTER_CLOCKWISE, 0.0, 0.10, 1.0_s, true);
    AutonomousSwerveDriveSequence(ROBOT_FORWARD, ROBOT_COUNTER_CLOCKWISE, 0.10, 0.10, 1.0_s, true);

    // Returning from here will enter the idle state until autonomous is over
    RobotUtils::DisplayMessage("Auto test swerve routine done.");
}


////////////////////////////////////////////////////////////////
/// @method EastTechRobot::AutonomousTestTrajectoryRoutine
///
/// Autonomous swerve test routine.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::AutonomousTestTrajectoryRoutine()
{
    // Swerve trajectory routine, but requires switching to command based robot.

    //TrajectoryConfig config =
    //new TrajectoryConfig(
    //        Constants.AutoConstants.kMaxSpeedMetersPerSecond,
    //        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //    .setKinematics(Constants.Swerve.swerveKinematics);
    TrajectoryConfig trajectoryConfig = {0.5_mps, 1.0_mps_sq};
    trajectoryConfig.SetKinematics(SwerveConfig::Kinematics);

    // An example trajectory to follow.  All units in meters.
    //Trajectory exampleTrajectory =
    //TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        //new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        //List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        //new Pose2d(3, 0, new Rotation2d(0)),
        //config);
    const Pose2d INITIAL_POSE = {0_m, 0_m, 0_deg};
    const Pose2d FINAL_POSE = {2_m, 0_m, 0_deg};
    const std::vector<Translation2d> WAY_POINTS = 
    {
        {0_m, 0_m}
    };
    Trajectory testTrajectory = TrajectoryGenerator::GenerateTrajectory(INITIAL_POSE, WAY_POINTS, FINAL_POSE, trajectoryConfig);
    (void)testTrajectory;

    //var thetaController =
    //new ProfiledPIDController(
    //    Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    //thetaController.enableContinuousInput(-Math.PI, Math.PI);

    //SwerveControllerCommand swerveControllerCommand =
    //new SwerveControllerCommand(
    //    exampleTrajectory,
    //    s_Swerve::getPose,
    //    Constants.Swerve.swerveKinematics,
    //    new PIDController(Constants.AutoConstants.kPXController, 0, 0),
    //    new PIDController(Constants.AutoConstants.kPYController, 0, 0),
    //    thetaController,
    //    s_Swerve::setModuleStates,
    //    s_Swerve);

    // Returning from here will enter the idle state until autonomous is over
    RobotUtils::DisplayMessage("Auto test trajectory routine done.");
}
