////////////////////////////////////////////////////////////////////////////////
/// @file   NeoSwerveModule.hpp
/// @author David Stalter
///
/// @details
/// Implements functionality for a Neo swerve module on a swerve drive robot.
///
/// Copyright (c) 2025 East Technical High School
////////////////////////////////////////////////////////////////////////////////

#ifndef NEOSWERVEMODULE_HPP
#define NEOSWERVEMODULE_HPP

// SYSTEM INCLUDES
#include <cmath>                                        // for M_PI

// C INCLUDES
#include "ctre/phoenix6/CANcoder.hpp"                   // for CTRE CANcoder API
#include "frc/controller/SimpleMotorFeedForward.h"      // for feedforward control
#include "frc/kinematics/SwerveModulePosition.h"        // for struct declaration
#include "frc/kinematics/SwerveModuleState.h"           // for struct declaration
#include "frc/geometry/Rotation2d.h"                    // for class declaration
#include "rev/SparkMax.h"                               // for interacting with spark max motor controllers
#include "units/angle.h"                                // for degree user defined literal
#include "units/voltage.h"                              // for voltage unit user defined literals

// C++ INCLUDES
#include "SwerveConfig.hpp"                             // for ModuleInformation structure

using namespace ctre::phoenix6::configs;
using namespace ctre::phoenix6::controls;
using namespace ctre::phoenix6::hardware;
using namespace ctre::phoenix6::signals;
using namespace frc;
using namespace rev;
using namespace rev::spark;


////////////////////////////////////////////////////////////////
/// @class NeoSwerveModule
///
/// Declarations for a Neo swerve module object.
///
////////////////////////////////////////////////////////////////
class NeoSwerveModule
{
    friend class SwerveDrive;

private:
    // Constructor
    NeoSwerveModule(SwerveConfig::ModuleInformation moduleInfo);

    // Point the module to zero degrees (forward)
    void HomeModule();

    // Point the module wheel in the correct direciton to form an X to prevent movement
    void LockWheel() { /* Not implemented yet. */ }

    // Align the swerve module to the absolute encoder
    void RecalibrateModules() { /* Not implemented yet. */ }

    // Update a swerve module to the desired state
    void SetDesiredState(SwerveModuleState desiredState, bool bIsOpenLoop);

    // Optimizes the desired swerve module state
    SwerveModuleState Optimize(SwerveModuleState desiredState, Rotation2d currentAngle);

    // Retrieves the swerve module state/position
    SwerveModuleState GetSwerveModuleState();
    SwerveModulePosition GetSwerveModulePosition();

    // Puts useful values on the dashboard
    void UpdateSmartDashboard();

    // Storage space for strings for the smart dashboard
    struct DisplayStrings
    {
        static const unsigned MAX_MODULE_DISPLAY_STRING_LENGTH = 64U;
        char m_CancoderAngleString[MAX_MODULE_DISPLAY_STRING_LENGTH];
        char m_NeoEncoderAngleString[MAX_MODULE_DISPLAY_STRING_LENGTH];
        char m_DriveNeoTemp[MAX_MODULE_DISPLAY_STRING_LENGTH];
        char m_AngleNeoTemp[MAX_MODULE_DISPLAY_STRING_LENGTH];
    };
    DisplayStrings m_DisplayStrings;
    static uint32_t m_DetailedModuleDisplayIndex;

    SwerveConfig::ModulePosition m_MotorGroupPosition;
    SparkMax * m_pDriveSpark;
    SparkMax * m_pAngleSpark;
    SparkRelativeEncoder m_DriveSparkEncoder;
    SparkRelativeEncoder m_AngleSparkEncoder;
    SparkClosedLoopController m_DrivePidController;
    SparkClosedLoopController m_AnglePidController;
    CANcoder * m_pAngleCanCoder;
    Rotation2d m_LastAngle;
    SimpleMotorFeedforward<units::meters> * m_pFeedForward;
    const Rotation2d CANCODER_REFERENCE_ABSOLUTE_OFFSET;

    // Divide by 12 on these constants to convert from volts to percent output for CTRE
    using Distance = units::meters;
    using Velocity = units::compound_unit<Distance, units::inverse<units::seconds>>;
    using Acceleration = units::compound_unit<Velocity, units::inverse<units::seconds>>;
    using kv_unit = units::compound_unit<units::volts, units::inverse<Velocity>>;
    using ka_unit = units::compound_unit<units::volts, units::inverse<Acceleration>>;
    static constexpr units::volt_t KS = (0.667_V / 12.0);
    static constexpr units::unit_t<kv_unit> KV = units::unit_t<kv_unit>(2.44 / 12.0);
    static constexpr units::unit_t<ka_unit> KA = units::unit_t<ka_unit>(0.27 / 12.0);

    // Swerve Profiling Values
    static constexpr double OPEN_LOOP_RAMP = 0.25;
    static constexpr double CLOSED_LOOP_RAMP = 0.0;

    NeoSwerveModule(const NeoSwerveModule &) = delete;
    NeoSwerveModule & operator=(const NeoSwerveModule &) = delete;
};

#endif // NEOSWERVEMODULE_HPP
