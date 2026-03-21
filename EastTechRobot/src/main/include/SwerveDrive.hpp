////////////////////////////////////////////////////////////////////////////////
/// @file   SwerveDrive.hpp
/// @author David Stalter
///
/// @details
/// Implements functionality for a swerve drive robot base.
///
/// Copyright (c) 2026 East Technical High School
////////////////////////////////////////////////////////////////////////////////

#ifndef SWERVEDRIVE_HPP
#define SWERVEDRIVE_HPP

// SYSTEM INCLUDES
// <none>

// C INCLUDES
#include "frc/geometry/Translation2d.h"                 // for class declaration
#include "frc/kinematics/SwerveDriveOdometry.h"         // for class declaration
#include "frc/kinematics/SwerveModulePosition.h"        // for struct declaration
#include "frc/kinematics/SwerveModuleState.h"           // for struct declaration
#include "units/angle.h"                                // for angle user defined literals
#include "units/angular_velocity.h"                     // for angular velocity user defined literals
#include "units/length.h"                               // for distance user defined literals

// C++ INCLUDES
#include "SwerveConfig.hpp"                             // for swerve configuration and constants
#include "NeoSwerveModule.hpp"                          // for interacting with a Neo swerve module
#include "TalonFxSwerveModule.hpp"                      // for interacting with a TalonFX swerve module
#include "ctre/phoenix6/CANBus.hpp"                     // for CANBus
#include "ctre/phoenix6/Pigeon2.hpp"                    // for PigeonIMU

using namespace ctre::phoenix6;
using namespace frc;


////////////////////////////////////////////////////////////////
/// @class SwerveDrive
///
/// Declarations for a swerve drive object.
///
////////////////////////////////////////////////////////////////
class SwerveDrive
{
public:
    // Constructor
    SwerveDrive(Pigeon2 * pPigeon, const std::function<const CANBus&(std::string_view)>& rGetCanBusReferenceLambda);

    // Gets the current 2D pose from the swerve module states
    Pose2d GetPose();

    // Sets the swerve module states to the passed in 2D pose
    void SetPose(Pose2d pose);

    // Updates each swerve module based on specified inputs (used by autonomous)
    void SetModuleStates(wpi::array<SwerveModuleState, SwerveConfig::NUM_SWERVE_DRIVE_MODULES> swerveModuleStates);

    // Updates each swerve module based on controller inputs (used by teleop)
    void SetModuleStates(Translation2d translation, double rotation, bool bFieldRelative, bool bIsOpenLoop);

    // Puts useful values on the dashboard
    void UpdateSmartDashboard();

    // Update the odometry
    inline void UpdateOdometry()
    {
        m_Odometry.Update(m_pPigeon->GetYaw().GetValue(), GetModulePositions());
    }

    // Sets the gyro yaw back to zero degrees
    inline void ZeroGyroYaw()
    {
        m_pPigeon->SetYaw(0.0_deg);
    }

    // Points all the modules to zero degrees, which should be straight forward
    inline void HomeModules()
    {
        for (uint32_t i = 0U; i < SwerveConfig::NUM_SWERVE_DRIVE_MODULES; i++)
        {
            m_SwerveModules[i].HomeModule();
        }
    }

    // Lock the wheels in an X pattern to prevent movement
    inline void LockWheels()
    {
        for (uint32_t i = 0U; i < SwerveConfig::NUM_SWERVE_DRIVE_MODULES; i++)
        {
            m_SwerveModules[i].LockWheel();
        }
    }

    // Recalibrate the modules based on the absolute encoder
    inline void RecalibrateModules()
    {
        for (uint32_t i = 0U; i < SwerveConfig::NUM_SWERVE_DRIVE_MODULES; i++)
        {
            m_SwerveModules[i].RecalibrateModules();
        }
    }

private:
    wpi::array<SwerveModulePosition, SwerveConfig::NUM_SWERVE_DRIVE_MODULES> GetModulePositions();

    Pigeon2 * m_pPigeon;
    SwerveConfig::SwerveModuleType m_SwerveModules[SwerveConfig::NUM_SWERVE_DRIVE_MODULES];

    // From https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-odometry.html
    // 0 degrees / radians represents the robot angle when the robot is facing directly toward your opponent’s
    // alliance station. As your robot turns to the left, your gyroscope angle should increase. By default, WPILib
    // gyros exhibit the opposite behavior, so you should negate the gyro angle.
    SwerveDriveOdometry<SwerveConfig::NUM_SWERVE_DRIVE_MODULES> m_Odometry;

    SwerveDrive(const SwerveDrive &) = delete;
    SwerveDrive & operator=(const SwerveDrive &) = delete;
};

#endif // SWERVEDRIVE_HPP
