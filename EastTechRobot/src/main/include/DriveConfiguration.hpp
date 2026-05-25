////////////////////////////////////////////////////////////////////////////////
/// @file   DriveConfiguration.hpp
/// @author David Stalter
///
/// @details
/// Declarations describing the drive configuration.
///
/// Copyright (c) 2026 East Technical High School
////////////////////////////////////////////////////////////////////////////////

#ifndef DRIVECONFIGURATION_HPP
#define DRIVECONFIGURATION_HPP

// SYSTEM INCLUDES
// <none>

// C INCLUDES
// (none)

// C++ INCLUDES
// (none)


////////////////////////////////////////////////////////////////
/// @namespace EastTech::Drive::Config
///
/// Provides configuration information about the drive system.
///
////////////////////////////////////////////////////////////////
namespace EastTech
{
namespace Drive
{
namespace Config
{
    enum DriveStyle
    {
        ARCADE_DRIVE,
        TANK_DRIVE,
        GTA_DRIVE
    };

    static const DriveStyle DRIVE_STYLE = ARCADE_DRIVE;

    static const bool   USE_SWERVE_DRIVE                        = true;
    static const bool   SWERVE_SLOW_USE_ROTATION_AXIS           = false;
}
}
}

#endif // DRIVECONFIGURATION_HPP
