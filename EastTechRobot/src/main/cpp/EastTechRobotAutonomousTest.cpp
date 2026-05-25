////////////////////////////////////////////////////////////////////////////////
/// @file   EastTechRobotAutonomousTest.cpp
/// @author David Stalter
///
/// @details
/// Implementation of an autonomous test routines for EastTechRobot.
///
/// Copyright (c) 2026 East Technical High School
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
// <none>

// C INCLUDES
#include "frc/controller/ProfiledPIDController.h"   // for type declaration
#include "frc/geometry/Pose2d.h"                    // for type declaration
#include "frc/geometry/Rotation2d.h"                // for type declaration
#include "frc/geometry/Translation2d.h"             // for type declaration
#include "frc/trajectory/Trajectory.h"              // for working with trajectories
#include "frc/trajectory/TrajectoryConfig.h"        // for creating a trajectory config
#include "frc/trajectory/TrajectoryGenerator.h"     // for generating a trajectory
#include "frc/trajectory/TrapezoidProfile.h"        // for type declaration
#include "frc2/command/Commands.h"                  // for different commands (e.g. Sequence())
#include "frc2/command/SwerveControllerCommand.h"   // for creating a swerve controller command

// C++ INCLUDES
#include "RobotUtils.hpp"                           // for DisplayMessage()
#include "SwerveConfig.hpp"                         // for kinematics and constants
#include "SwerveDrive.hpp"                          // for calling autonomous swerve functions
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
    static SwerveDrive::SwerveDirections swerveDirections;
    swerveDirections.SetSwerveDirections(SwerveDrive::RobotTranslation::ROBOT_TRANSLATION_FORWARD, SwerveDrive::RobotStrafe::ROBOT_STRAFE_NONE, SwerveDrive::RobotRotation::ROBOT_ROTATION_NONE);
    m_pSwerveDrive->AutonomousDrive(swerveDirections, 0.10, 0.0, 0.0, 1.0_s, true);
    swerveDirections.SetSwerveDirections(SwerveDrive::RobotTranslation::ROBOT_TRANSLATION_NONE, SwerveDrive::RobotStrafe::ROBOT_STRAFE_LEFT, SwerveDrive::RobotRotation::ROBOT_ROTATION_NONE);
    m_pSwerveDrive->AutonomousDrive(swerveDirections, 0.10, 0.0, 0.0, 1.0_s, true);
    swerveDirections.SetSwerveDirections(SwerveDrive::RobotTranslation::ROBOT_TRANSLATION_REVERSE, SwerveDrive::RobotStrafe::ROBOT_STRAFE_NONE, SwerveDrive::RobotRotation::ROBOT_ROTATION_NONE);
    m_pSwerveDrive->AutonomousDrive(swerveDirections, 0.10, 0.0, 0.0, 1.0_s, true);
    swerveDirections.SetSwerveDirections(SwerveDrive::RobotTranslation::ROBOT_TRANSLATION_NONE, SwerveDrive::RobotStrafe::ROBOT_STRAFE_RIGHT, SwerveDrive::RobotRotation::ROBOT_ROTATION_NONE);
    m_pSwerveDrive->AutonomousDrive(swerveDirections, 0.10, 0.0, 0.0, 1.0_s, true);
    swerveDirections.SetSwerveDirections(SwerveDrive::RobotTranslation::ROBOT_TRANSLATION_NONE, SwerveDrive::RobotStrafe::ROBOT_STRAFE_NONE, SwerveDrive::RobotRotation::ROBOT_ROTATION_CLOCKWISE);
    m_pSwerveDrive->AutonomousDrive(swerveDirections, 0.0, 0.10, 0.0, 1.0_s, true);
    swerveDirections.SetSwerveDirections(SwerveDrive::RobotTranslation::ROBOT_TRANSLATION_NONE, SwerveDrive::RobotStrafe::ROBOT_STRAFE_NONE, SwerveDrive::RobotRotation::ROBOT_ROTATION_COUNTER_CLOCKWISE);
    m_pSwerveDrive->AutonomousDrive(swerveDirections, 0.0, 0.10, 0.0, 1.0_s, true);
    swerveDirections.SetSwerveDirections(SwerveDrive::RobotTranslation::ROBOT_TRANSLATION_FORWARD, SwerveDrive::RobotStrafe::ROBOT_STRAFE_RIGHT, SwerveDrive::RobotRotation::ROBOT_ROTATION_CLOCKWISE);
    m_pSwerveDrive->AutonomousDrive(swerveDirections, 0.10, 0.10, 0.10, 1.0_s, true);
    swerveDirections.SetSwerveDirections(SwerveDrive::RobotTranslation::ROBOT_TRANSLATION_REVERSE, SwerveDrive::RobotStrafe::ROBOT_STRAFE_LEFT, SwerveDrive::RobotRotation::ROBOT_ROTATION_COUNTER_CLOCKWISE);
    m_pSwerveDrive->AutonomousDrive(swerveDirections, 0.10, 0.10, 0.10, 1.0_s, true);

    // Returning from here will enter the idle state until autonomous is over
    RobotUtils::DisplayMessage("Auto test swerve routine done.");
}


////////////////////////////////////////////////////////////////
/// @method EastTechRobot::AutonomousTestTrajectoryRoutine
///
/// Autonomous swerve test trajectory routine.
///
////////////////////////////////////////////////////////////////
CommandPtr EastTechRobot::AutonomousTestTrajectoryRoutine()
{
    // Swerve trajectory routine.  Still needs some in depth tuning.

    // Constructed with max velocity, max acceleration
    TrajectoryConfig trajectoryConfig = {1.5_mps, 3.0_mps_sq};
    trajectoryConfig.SetKinematics(SwerveConfig::Kinematics);

    // An example trajectory to follow.  All units in meters.
    // 1. Start at the origin facing the +X direction (INITIAL_POSE)
    // 2. Pass through these two interior waypoints, making an 's' curve path (WAY_POINTS)
    // 3. End three meters straight ahead of the start position, facing forward (FINAL_POSE)
    // Note: First pose parameter is field y-axis (positive is forward)
    //       Second pose parameter is field x-axis (positive is left)
    const Pose2d INITIAL_POSE = {0_m, 0_m, 0_deg};
    const Pose2d FINAL_POSE = {3_m, 0_m, 0_deg};
    const std::vector<Translation2d> WAY_POINTS = 
    {
        {1.0_m,  1.0_m},
        {2.0_m, -1.0_m}
    };
    Trajectory testTrajectory = TrajectoryGenerator::GenerateTrajectory(INITIAL_POSE, WAY_POINTS, FINAL_POSE, trajectoryConfig);

    // Translation PID controllers
    PIDController xPidController(1.0, 0.0, 0.0);
    PIDController yPidController(1.0, 0.0, 0.0);

    // Values are max angular velocity, max angular acceleration
    TrapezoidProfile<units::radians>::Constraints thetaPidControllerConstraints(3.1415_rad_per_s, 3.1415_rad_per_s_sq);
    
    // Parameters are P, I, D, TrapezoidProfile<>::Constraints
    ProfiledPIDController<units::radians> thetaPidController(0.5, 0.0, 0.0, thetaPidControllerConstraints);

    thetaPidController.EnableContinuousInput(units::radian_t(-std::numbers::pi), units::radian_t(std::numbers::pi));

    // Construct the swerve controller command
    CommandPtr swerveControllerCommand = 
    SwerveControllerCommand<SwerveConfig::NUM_SWERVE_DRIVE_MODULES>
    (
        testTrajectory,
        [this](){return m_pSwerveDrive->GetPose();},
        SwerveConfig::Kinematics,
        xPidController,
        yPidController,
        thetaPidController,
        [this](auto moduleStates){ m_pSwerveDrive->SetModuleStates(moduleStates); }
        // Last parameter is requirements and (maybe?) can be defaulted
    ).ToPtr();

    // Build the command sequence and return it
    return cmd::Sequence
    (
        InstantCommand([this, initialPose = testTrajectory.InitialPose()]() { m_pSwerveDrive->SetPose(initialPose); }, {}).ToPtr(),
        std::move(swerveControllerCommand),
        InstantCommand([this] { m_pSwerveDrive->SetModuleStates({0.0_m, 0.0_m}, 0.0, false, false); }, {}).ToPtr()
    );
}
