////////////////////////////////////////////////////////////////////////////////
/// @file   SwerveConversions.hpp
/// @author David Stalter
///
/// @details
/// Utility routines for swerve drive conversions.
///
/// Copyright (c) 2024 East Technical High School
////////////////////////////////////////////////////////////////////////////////

#ifndef SWERVECONVERSIONS_HPP
#define SWERVECONVERSIONS_HPP

// SYSTEM INCLUDES
// <none>

// C INCLUDES
// (none)

// C++ INCLUDES
// (none)


////////////////////////////////////////////////////////////////
/// @namespace SwerveConversions
///
/// Routines for swerve drive conversions.
///
////////////////////////////////////////////////////////////////
namespace SwerveConversions
{
    // https://docs.ctre-phoenix.com/en/latest/ch14_MCSensor.html#sensor-resolution
    // Units per rotation: 2048 (FX integrated sensor)
    // From FX user guide: kMaxRPM = Free Speed RPM = 6380 RPM
    // Calculate the expect peak sensor velocity (sensor units per 100ms) as:
    // Vsensor_max = (kMaxRPM  / 600) * (kSensorUnitsPerRotation / kGearRatio)
    // Read sensor velocity and solve above equation for kMaxRPM term for any RPM.

    ////////////////////////////////////////////////////////////////
    /// @method SwerveConversions::ConvertCelsiusToFahrenheit
    ///
    /// Adjusts an input angle to be within a new scope.
    ///
    /// @param scopeReference Current Angle
    /// @param newAngle Target Angle
    /// @return Closest angle within scope
    ///
    ////////////////////////////////////////////////////////////////
    inline static double AdjustAngleScope(double scopeReference, double newAngle)
    {
        double lowerBound;
        double upperBound;
        double lowerOffset = static_cast<int>(scopeReference) % 360;

        if (lowerOffset >= 0)
        {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        }
        else
        {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }

        while (newAngle < lowerBound)
        {
            newAngle += 360;
        }
        while (newAngle > upperBound)
        {
            newAngle -= 360;
        }

        if (newAngle - scopeReference > 180)
        {
            newAngle -= 360;
        }
        else if (newAngle - scopeReference < -180)
        {
            newAngle += 360;
        }
        else
        {
        }

        return newAngle;
    }

    ////////////////////////////////////////////////////////////////
    /// @method SwerveConversions::RpsToMps
    ///
    /// Converts rotations per second to meters per second.
    ///
    /// @param wheelRps Wheel rotations per second
    /// @param circumference Circumference of wheel
    /// @return Meters per second
    ///
    ////////////////////////////////////////////////////////////////
    inline static double RpsToMps(double wheelRps, double circumference)
    {
        return wheelRps * circumference;
    }

    ////////////////////////////////////////////////////////////////
    /// @method SwerveConversions::MpsToRps
    ///
    /// Converts meters per second to rotations per second.
    ///
    /// @param wheelMps Wheel meters per second
    /// @param circumference Circumference of wheel
    /// @return Rotations per second
    ///
    ////////////////////////////////////////////////////////////////
    inline static double MpsToRps(double wheelMps, double circumference)
    {
        return wheelMps / circumference;
    }

    ////////////////////////////////////////////////////////////////
    /// @method SwerveConversions::RotationsToMeters
    ///
    /// Converts rotations per second to meters per second.
    ///
    /// @param wheelRotations Wheel rotations
    /// @param circumference Circumference of wheel
    /// @return Meters
    ///
    ////////////////////////////////////////////////////////////////
    inline static double RotationsToMeters(double wheelRotations, double circumference)
    {
        return wheelRotations * circumference;
    }

    ////////////////////////////////////////////////////////////////
    /// @method SwerveConversions::MetersToRotations
    ///
    /// Converts rotations per second to meters per second.
    ///
    /// @param wheelMeters Meters
    /// @param circumference Circumference of wheel
    /// @return Wheel rotations
    ///
    ////////////////////////////////////////////////////////////////
    inline static double MetersToRotations(double wheelMeters, double circumference)
    {
        return wheelMeters / circumference;
    }
}

#endif // SWERVECONVERSIONS_HPP
