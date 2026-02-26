////////////////////////////////////////////////////////////////////////////////
/// @file   SwerveConversions.hpp
/// @author David Stalter
///
/// @details
/// Utility routines for swerve drive conversions.
///
/// Copyright (c) 2026 East Technical High School
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
    // https://app.readthedocs.org/projects/phoenix-documentation/downloads/pdf/latest/ (V5 documentation)
    //
    // Units per rotation: 2048 (Talon FX integrated sensor)
    // Units per rotation: 4096 (CANCoder)
    // Units per rotation: 4096 (CTRE Magnetic Encoder)
    //
    // From the Falcon 500 user guide: kMaxRPM = Free Speed RPM = 6380 RPM
    // From the Kraken X60 WCP documentation: kMaxRPM = Free Speed RPM = 5800 RPM (FOC), 6000 RPM (Trapezoidal)
    // From the Kraken X44 WCP documentation: Free Speed RPM = 7530 RPM (Trapezoidal)
    //
    // V5 documentation (consult offline .pdf file) for Talon FX/SRX sensors bring up:
    // Calculate the expected peak sensor velocity (sensor units per 100ms) as:
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
