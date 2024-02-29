////////////////////////////////////////////////////////////////////////////////
/// @file   TalonFxSwerveModule.hpp
/// @author David Stalter
///
/// @details
/// Implements functionality for a swerve module on a swerve drive robot.
///
/// Copyright (c) 2024 East Technical High School
////////////////////////////////////////////////////////////////////////////////

#ifndef TALONFXSWERVEMODULE_HPP
#define TALONFXSWERVEMODULE_HPP

// SYSTEM INCLUDES
#include <cmath>                                        // for M_PI

// C INCLUDES
#include "frc/controller/SimpleMotorFeedForward.h"      // for feedforward control
#include "frc/kinematics/SwerveModulePosition.h"        // for struct declaration
#include "frc/kinematics/SwerveModuleState.h"           // for struct declaration
#include "frc/geometry/Rotation2d.h"                    // for class declaration
#include "units/angle.h"                                // for degree user defined literal
#include "units/voltage.h"                              // for voltage unit user defined literals

// C++ INCLUDES
#include "ctre/phoenix6/CANcoder.hpp"                   // for CTRE CANcoder API
#include "ctre/phoenix6/TalonFX.hpp"                    // for CTRE TalonFX API

using namespace frc;
using namespace ctre::phoenix6::configs;
using namespace ctre::phoenix6::controls;
using namespace ctre::phoenix6::hardware;
using namespace ctre::phoenix6::signals;


////////////////////////////////////////////////////////////////
/// @class TalonFxSwerveModule
///
/// Declarations for a swerve module object.
///
////////////////////////////////////////////////////////////////
class TalonFxSwerveModule
{
    friend class SwerveDrive;

private:
    enum ModulePosition
    {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT
    };

    struct SwerveModuleConfig
    {
        const char * m_pModuleName;
        ModulePosition m_Position;
        int m_DriveMotorCanId;
        int m_AngleMotorCanId;
        int m_CanCoderId;
        const Rotation2d m_CancoderReferenceAbsoluteOffset;
    };

    // Constructor
    TalonFxSwerveModule(SwerveModuleConfig config);

    // Points the module to zero degrees, which should be straight forward
    inline void HomeModule()
    {
        (void)m_pAngleTalon->SetControl(m_AnglePositionVoltage.WithPosition(0.0_deg));
        m_LastAngle = 0.0_deg;
    }

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
        char m_FxEncoderAngleString[MAX_MODULE_DISPLAY_STRING_LENGTH];
        char m_DriveTalonTemp[MAX_MODULE_DISPLAY_STRING_LENGTH];
        char m_AngleTalonTemp[MAX_MODULE_DISPLAY_STRING_LENGTH];
    };
    DisplayStrings m_DisplayStrings;
    static uint32_t m_DetailedModuleDisplayIndex;

    ModulePosition m_MotorGroupPosition;
    TalonFX * m_pDriveTalon;
    TalonFX * m_pAngleTalon;
    CANcoder * m_pAngleCanCoder;
    Rotation2d m_LastAngle;
    SimpleMotorFeedforward<units::meters> * m_pFeedForward;
    DutyCycleOut m_DriveDutyCycleOut;
    VelocityVoltage m_DriveVelocityVoltage;
    PositionVoltage m_AnglePositionVoltage;
    const Rotation2d CANCODER_REFERENCE_ABSOLUTE_OFFSET;

    // Divide by 12 on these constants to convert from volts to percent output for CTRE
    using Distance = units::meters;
    using Velocity = units::compound_unit<Distance, units::inverse<units::seconds>>;
    using Acceleration = units::compound_unit<Velocity, units::inverse<units::seconds>>;
    using kv_unit = units::compound_unit<units::volts, units::inverse<Velocity>>;
    using ka_unit = units::compound_unit<units::volts, units::inverse<Acceleration>>;
    static constexpr units::volt_t KS = (0.32_V / 12.0);
    static constexpr units::unit_t<kv_unit> KV = units::unit_t<kv_unit>(1.51 / 12.0);
    static constexpr units::unit_t<ka_unit> KA = units::unit_t<ka_unit>(0.27 / 12.0);

    TalonFxSwerveModule(const TalonFxSwerveModule &) = delete;
    TalonFxSwerveModule & operator=(const TalonFxSwerveModule &) = delete;
};

#endif // TALONFXSWERVEMODULE_HPP
