////////////////////////////////////////////////////////////////////////////////
/// @file   SwerveConfig.hpp
/// @author David Stalter
///
/// @details
/// Swerve drive configuration and constants.
///
/// Copyright (c) 2026 East Technical High School
////////////////////////////////////////////////////////////////////////////////

#ifndef SWERVECONFIG_HPP
#define SWERVECONFIG_HPP

// SYSTEM INCLUDES
// <none>

// C INCLUDES
#include "frc/geometry/Translation2d.h"                 // for class declaration
#include "frc/kinematics/SwerveDriveKinematics.h"       // for class declaration
#include "units/length.h"                               // for distance user defined literals

// C++ INCLUDES
#include "ctre/phoenix6/CANcoder.hpp"                   // for CTRE CANcoder API
#include "ctre/phoenix6/TalonFX.hpp"                    // for CTRE TalonFX API

using namespace frc;
using namespace ctre::phoenix6::signals;

class TalonFxSwerveModule;
class NeoSwerveModule;


////////////////////////////////////////////////////////////////
/// @namespace SwerveConfig
///
/// Swerve drive configuration and constants.
///
////////////////////////////////////////////////////////////////
namespace SwerveConfig
{
    enum ModulePosition
    {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT
    };

    struct ModuleInformation
    {
        const char * m_pModuleName;
        ModulePosition m_Position;
        int m_DriveMotorCanId;
        int m_AngleMotorCanId;
        int m_CanCoderId;
        const Rotation2d m_EncoderReferenceAbsoluteOffset;
        std::string_view m_MotorsCanBusName;
        std::string_view m_CanCoderCanBusName;
    };

    // Represents the configurable parameters in a swerve module
    struct ModuleConfiguration
    {
        const double DRIVE_GEAR_RATIO;
        const double ANGLE_GEAR_RATIO;

        // These types are specific to CTRE configurations, but the
        // concepts apply to swerve in general.  Swerve modules using
        // Neo motors will ignore the inverted values here and just
        // directly specify true/false for invert.  If something
        // besides a typical CTRE setup is used, this part of the
        // configuration may need to be reworked.
        const InvertedValue DRIVE_MOTOR_INVERTED_VALUE;
        const InvertedValue ANGLE_MOTOR_INVERTED_VALUE;
        const SensorDirectionValue CANCODER_INVERTED_VALUE;
    };

    // SDS MK4 L3 Very Fast configuration
    static constexpr const ModuleConfiguration SDS_MK4_CONFIG = {(6.12 / 1.0), (12.8 / 1.0), InvertedValue::CounterClockwise_Positive, InvertedValue::Clockwise_Positive, SensorDirectionValue::CounterClockwise_Positive};

    // SDS MK4n L3+ configuration
    static constexpr const ModuleConfiguration SDS_MK4N_CONFIG = {(5.36 / 1.0), (18.75 / 1.0), InvertedValue::CounterClockwise_Positive, InvertedValue::CounterClockwise_Positive, SensorDirectionValue::CounterClockwise_Positive};

    // SDS MK5n configurations
    static constexpr const ModuleConfiguration SDS_MK5N_R1_CONFIG = {(7.03 / 1.0), (287.0 / 11.0), InvertedValue::CounterClockwise_Positive, InvertedValue::CounterClockwise_Positive, SensorDirectionValue::CounterClockwise_Positive};
    static constexpr const ModuleConfiguration SDS_MK5N_R2_CONFIG = {(6.03 / 1.0), (287.0 / 11.0), InvertedValue::CounterClockwise_Positive, InvertedValue::CounterClockwise_Positive, SensorDirectionValue::CounterClockwise_Positive};
    static constexpr const ModuleConfiguration SDS_MK5N_R3_CONFIG = {(5.27 / 1.0), (287.0 / 11.0), InvertedValue::CounterClockwise_Positive, InvertedValue::CounterClockwise_Positive, SensorDirectionValue::CounterClockwise_Positive};

    // The swerve module configuration on the robot (change the typedef based on the motors on the module)
    typedef TalonFxSwerveModule SwerveModuleType;
    static constexpr const ModuleConfiguration & SELECTED_SWERVE_MODULE_CONFIG = SDS_MK5N_R2_CONFIG;

    // Names for the CAN buses that swerve can use
    static constexpr const std::string_view RIO_CAN_BUS_NAME = "rio";
    static constexpr const std::string_view CANIVORE_CAN_BUS_NAME = "canivore-8145";

    static constexpr const size_t NUM_SWERVE_DRIVE_MODULES = 4U;

    // 1 inch = 0.0254 meters
    static constexpr const double METERS_PER_INCH = 0.0254;

    static constexpr double FX_INTEGRATED_SENSOR_UNITS_PER_ROTATION = 2048.0;
    static constexpr double WHEEL_CIRCUMFERENCE = 4.0 * METERS_PER_INCH * M_PI;

    // Distance between front/back wheel centers
    static constexpr const units::meter_t WHEEL_BASE = units::meter_t(0.0 * METERS_PER_INCH);
    // Distance between left/right wheel centers
    static constexpr const units::meter_t TRACK_WIDTH = units::meter_t(0.0 * METERS_PER_INCH);

    // 14.7638 feet per second (conversion *3.28084), 487.0141 degrees per second (conversion *57.2957795131)
    static constexpr units::meters_per_second_t MAX_DRIVE_VELOCITY_MPS = 4.5_mps;
    static constexpr units::radians_per_second_t MAX_ANGULAR_VELOCITY_RAD_PER_SEC = 8.5_rad_per_s;

    static constexpr const Translation2d FRONT_LEFT_MODULE_T2D = {WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0};
    static constexpr const Translation2d FRONT_RIGHT_MODULE_T2D = {WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0};
    static constexpr const Translation2d BACK_LEFT_MODULE_T2D = {-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0};
    static constexpr const Translation2d BACK_RIGHT_MODULE_T2D = {-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0};

    // It would be nice to make this 'const' but some classes (TrajectoryConfig) require non-const objects
    static SwerveDriveKinematics<NUM_SWERVE_DRIVE_MODULES> Kinematics
    {
        FRONT_LEFT_MODULE_T2D,
        FRONT_RIGHT_MODULE_T2D,
        BACK_LEFT_MODULE_T2D,
        BACK_RIGHT_MODULE_T2D
    };

    // Note: If using the RobotTestCode routines (for Neo swerve), these objects have to be disabled (or use different CAN IDs).

    // Config information on each swerve module.
    // Fields are: Name, Position, Drive TalonFX CAN ID, Angle TalonFX CAN ID, CANCoder ID, Angle Offset, Motor CAN Bus Name, Encoder CAN Bus Name
    // 20xx: Angles measured with bevels facing right.
    static constexpr const ModuleInformation FRONT_LEFT_MODULE_INFO = {"Front left", ModulePosition::FRONT_LEFT, 11, 12, 21, 0.0_deg, CANIVORE_CAN_BUS_NAME, CANIVORE_CAN_BUS_NAME};
    static constexpr const ModuleInformation FRONT_RIGHT_MODULE_INFO = {"Front right", ModulePosition::FRONT_RIGHT, 13, 14, 22, 0.0_deg, CANIVORE_CAN_BUS_NAME, CANIVORE_CAN_BUS_NAME};
    static constexpr const ModuleInformation BACK_LEFT_MODULE_INFO = {"Back left", ModulePosition::BACK_LEFT, 15, 16, 23, 0.0_deg, CANIVORE_CAN_BUS_NAME, CANIVORE_CAN_BUS_NAME};
    static constexpr const ModuleInformation BACK_RIGHT_MODULE_INFO = {"Back right", ModulePosition::BACK_RIGHT, 17, 18, 24, 0.0_deg, CANIVORE_CAN_BUS_NAME, CANIVORE_CAN_BUS_NAME};
}

#endif // SWERVECONFIG_HPP
