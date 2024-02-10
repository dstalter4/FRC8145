////////////////////////////////////////////////////////////////////////////////
/// @file   SwerveConfig.hpp
/// @author David Stalter
///
/// @details
/// Swerve drive configuration and constants.
///
/// Copyright (c) 2023 Youth Technology Academy
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
// (none)

using namespace frc;


////////////////////////////////////////////////////////////////
/// @namespace SwerveConfig
///
/// Swerve drive configuration and constants.
///
////////////////////////////////////////////////////////////////
namespace SwerveConfig
{
    static constexpr const size_t NUM_SWERVE_DRIVE_MODULES = 4U;

    // 1 inch = 0.0254 meters
    static constexpr const double METERS_PER_INCH = 0.0254;

    // SDS MK4 L3 Very Fast configuration
    static constexpr double DRIVE_GEAR_RATIO = (6.12 / 1.0);
    static constexpr double ANGLE_GEAR_RATIO = (12.8 / 1.0);
    static constexpr double FX_INTEGRATED_SENSOR_UNITS_PER_ROTATION = 2048.0;
    static constexpr double WHEEL_CIRCUMFERENCE = 4.0 * METERS_PER_INCH * M_PI;

    // Distance between front/back wheel centers 23.5_in (0.5969_m)
    static constexpr const units::meter_t WHEEL_BASE = units::meter_t(23.5 * METERS_PER_INCH);
    // Distance between left/right wheel centers, 21.5_in (0.5461_m)
    static constexpr const units::meter_t TRACK_WIDTH = units::meter_t(21.5 * METERS_PER_INCH);

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
}

#endif // SWERVECONFIG_HPP
