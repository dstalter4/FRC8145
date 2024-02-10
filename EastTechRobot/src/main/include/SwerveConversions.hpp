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
    /// @method SwerveConversions::CanCoderToDegrees
    ///
    /// Converts the CANCoder position count to an angle in degrees.
    ///
    /// @param positionCounts CANCoder Position Counts
    /// @param gearRatio Gear Ratio between CANCoder and Mechanism
    /// @return Degrees of Rotation of Mechanism
    ///
    ////////////////////////////////////////////////////////////////
    inline static double CanCoderToDegrees(double positionCounts, double gearRatio)
    {
        return positionCounts * (360.0 / (gearRatio * 4096.0));
    }

    ////////////////////////////////////////////////////////////////
    /// @method SwerveConversions::DegreesToCanCoder
    ///
    /// Converts an angle in degrees to a CANCoder position count.
    ///
    /// @param degrees Degrees of rotation of Mechanism
    /// @param gearRatio Gear Ratio between CANCoder and Mechanism
    /// @return CANCoder Position Counts
    ///
    ////////////////////////////////////////////////////////////////
    inline static double DegreesToCanCoder(double degrees, double gearRatio)
    {
        return degrees / (360.0 / (gearRatio * 4096.0));
    }

    ////////////////////////////////////////////////////////////////
    /// @method SwerveConversions::FalconToDegrees
    ///
    /// Converts the TalonFX built-in encoder position count to an
    /// angle in degrees.
    ///
    /// @param counts Falcon Position Counts
    /// @param gearRatio Gear Ratio between Falcon and Mechanism
    /// @return Degrees of Rotation of Mechanism
    ///
    ////////////////////////////////////////////////////////////////
    inline static double FalconToDegrees(double positionCounts, double gearRatio)
    {
        return positionCounts * (360.0 / (gearRatio * 2048.0));
    }

    ////////////////////////////////////////////////////////////////
    /// @method SwerveConversions::DegreesToFalcon
    ///
    /// Converts an angle in degrees to a TalonFX built-in encoder
    /// position count.
    ///
    /// @param degrees Degrees of rotation of Mechanism
    /// @param gearRatio Gear Ratio between Falcon and Mechanism
    /// @return Falcon Position Counts
    ///
    ////////////////////////////////////////////////////////////////
    inline static double DegreesToFalcon(double degrees, double gearRatio)
    {
        return degrees / (360.0 / (gearRatio * 2048.0));
    }

    ////////////////////////////////////////////////////////////////
    /// @method SwerveConversions::FalconToRpm
    ///
    /// Converts the TalonFX built-in sensor velocity counts to a
    /// RPM value.
    ///
    /// @param velocityCounts Falcon Velocity Counts
    /// @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
    /// @return RPM of Mechanism
    ///
    ////////////////////////////////////////////////////////////////
    inline static double FalconToRpm(double velocityCounts, double gearRatio)
    {
        double motorRPM = velocityCounts * (600.0 / 2048.0);        
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }

    ////////////////////////////////////////////////////////////////
    /// @method SwerveConversions::RpmToFalcon
    ///
    /// Converts a RPM value to TalonFX built-in sensor velocity
    /// counts.
    ///
    /// @param rpm RPM of mechanism
    /// @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
    /// @return RPM of Mechanism
    ///
    ////////////////////////////////////////////////////////////////
    inline static double RpmToFalcon(double rpm, double gearRatio)
    {
        double motorRpm = rpm * gearRatio;
        double sensorCounts = motorRpm * (2048.0 / 600.0);
        return sensorCounts;
    }

    ////////////////////////////////////////////////////////////////
    /// @method SwerveConversions::FalconToMps
    ///
    /// Converts the TalonFX built-in sensor velocity counts to
    /// meters per second.
    ///
    /// @param velocitycounts Falcon Velocity Counts
    /// @param circumference Circumference of Wheel
    /// @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon MPS)
    /// @return Falcon Velocity Counts
    ///
    ////////////////////////////////////////////////////////////////
    inline static double FalconToMps(double velocitycounts, double circumference, double gearRatio)
    {
        double wheelRpm = FalconToRpm(velocitycounts, gearRatio);
        double wheelMps = (wheelRpm * circumference) / 60;
        return wheelMps;
    }

    ////////////////////////////////////////////////////////////////
    /// @method SwerveConversions::MpsToFalcon
    ///
    /// Converts meters per second to TalonFX built-in sensor
    /// velocity counts.
    ///
    /// @param velocity Velocity MPS
    /// @param circumference Circumference of Wheel
    /// @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon MPS)
    /// @return Falcon Velocity Counts
    ///
    ////////////////////////////////////////////////////////////////
    inline static double MpsToFalcon(double velocity, double circumference, double gearRatio)
    {
        double wheelRpm = ((velocity * 60) / circumference);
        double wheelVelocity = RpmToFalcon(wheelRpm, gearRatio);
        return wheelVelocity;
    }

    ////////////////////////////////////////////////////////////////
    /// @method SwerveConversions::FalconToMeters
    ///
    /// Converts the TalonFX built-in encoder position counts to
    /// meters.
    ///
    /// @param positionCounts Falcon Position Counts
    /// @param circumference Circumference of Wheel
    /// @param gearRatio Gear Ratio between Falcon and Wheel
    /// @return Meters
    ///
    ////////////////////////////////////////////////////////////////
    inline static double FalconToMeters(double positionCounts, double circumference, double gearRatio)
    {
        return positionCounts * (circumference / (gearRatio * 2048.0));
    }

    ////////////////////////////////////////////////////////////////
    /// @method SwerveConversions::MetersToFalcon
    ///
    /// Converts meters to TalonFX built-in encoder position counts.
    ///
    /// @param meters Meters
    /// @param circumference Circumference of Wheel
    /// @param gearRatio Gear Ratio between Falcon and Wheel
    /// @return Falcon Position Counts
    ///
    ////////////////////////////////////////////////////////////////
    inline static double MetersToFalcon(double meters, double circumference, double gearRatio)
    {
        return meters / (circumference / (gearRatio * 2048.0));
    }
}

#endif // SWERVECONVERSIONS_HPP
