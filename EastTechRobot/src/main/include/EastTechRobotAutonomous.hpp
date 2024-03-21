////////////////////////////////////////////////////////////////////////////////
/// @file   EastTechRobotAutonomous.hpp
/// @author David Stalter
///
/// @details
/// Contains the declarations for the autonomous portions of code ran in an FRC
/// robot.
///
/// Copyright (c) 2024 East Technical High School
////////////////////////////////////////////////////////////////////////////////

#ifndef EASTTECHROBOTAUTONOMOUS_HPP
#define EASTTECHROBOTAUTONOMOUS_HPP

// SYSTEM INCLUDES
// <none>

// C INCLUDES
// (none)

// C++ INCLUDES
#include "EastTechRobot.hpp"            // for inline autonomous function declarations

using namespace frc;

////////////////////////////////////////////////////////////////
/// @namespace EastTechRobotAutonomous
///
/// Namespace that contains robot autonomous variable and
/// function declarations.
///
////////////////////////////////////////////////////////////////
namespace EastTechRobotAutonomous
{
    // TYPEDEFS
    // (none)
    
    // ENUMS
    // (none)    
    
    // STRUCTS
    // (none)
    
    // VARIABLES
    extern bool bAutonomousExecutionComplete;
    
    // CONSTS
    
    // Autonomous Mode Constants
    // @todo: Convert to class and make a friend in EastTechRobot
    
    // Note: Only enable one autonomous routine!
    // Note: Autonomous routines are currently controlled by
    // the SendableChooser.
    //static const bool       ROUTINE_1                           = true;
    //static const bool       ROUTINE_2                           = false;
    //static const bool       ROUTINE_3                           = false;
    //static const bool       TEST_ENABLED                        = false;

    // Autonomous drive speed constants
    // (none)
    
    // Autonomous angle constants
    static const int        FORTY_FIVE_DEGREES                  = 45;
    static const int        NINETY_DEGREES                      = 90;
    static const int        ONE_HUNDRED_EIGHTY_DEGREES          = 180;
    static const int        THREE_HUNDRED_SIXTY_DEGREES         = 360;
    
    // Autonomous delay constants
    static constexpr units::second_t SWERVE_OP_STEP_TIME_S      =  0.10_s;
    static constexpr units::second_t DELAY_SHORT_S              =  0.50_s;
    static constexpr units::second_t DELAY_MEDIUM_S             =  1.00_s;
    static constexpr units::second_t DELAY_LONG_S               =  2.00_s;
    
} // End namespace



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::AutonomousDelay
///
/// Waits for a specified amount of time in autonomous.  Used
/// while an operation is ongoing but not yet complete, and
/// nothing else needs to occur.
///
////////////////////////////////////////////////////////////////
inline void EastTechRobot::AutonomousDelay(units::second_t time)
{
    Wait(time);
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::AutonomousSwerveDriveSequence
///
/// Drives during autonomous for a specified amount of time
/// using swerve drive modules.
///
////////////////////////////////////////////////////////////////
inline void EastTechRobot::AutonomousSwerveDriveSequence(RobotDirection direction, RobotRotate rotate, double translationSpeed, double strafeSpeed, double rotateSpeed, units::second_t time, bool bFieldRelative)
{
    RobotDirection translationDirection = static_cast<RobotDirection>(direction & ROBOT_TRANSLATION_MASK);
    RobotDirection strafeDirection = static_cast<RobotDirection>(direction & ROBOT_STRAFE_MASK);

    units::meter_t translation = 0.0_m;
    units::meter_t strafe = 0.0_m;

    switch (translationDirection)
    {
        case ROBOT_FORWARD:
        {
            translation = units::meter_t(translationSpeed);
            break;
        }
        case ROBOT_REVERSE:
        {
            translation = units::meter_t(-translationSpeed);
            break;
        }
        default:
        {
            break;
        }
    }

    switch (strafeDirection)
    {
        case ROBOT_LEFT:
        {
            strafe = units::meter_t(strafeSpeed);
            break;
        }
        case ROBOT_RIGHT:
        {
            strafe = units::meter_t(-strafeSpeed);
            break;
        }
        default:
        {
            break;
        }
    }

    switch (rotate)
    {
        case ROBOT_NO_ROTATE:
        {
            // Just in case the user decided to pass a speed anyway
            rotateSpeed = 0.0;
            break;
        }
        case ROBOT_CLOCKWISE:
        {
            rotateSpeed *= -1.0;
            break;
        }
        case ROBOT_COUNTER_CLOCKWISE:
        default:
        {
            break;
        }
    }

    Translation2d translation2d = {translation, strafe};
    units::second_t duration = 0.0_s;
    while (duration < time)
    {
        m_pSwerveDrive->SetModuleStates(translation2d, rotateSpeed, bFieldRelative, true);
        AutonomousDelay(EastTechRobotAutonomous::SWERVE_OP_STEP_TIME_S);
        duration += EastTechRobotAutonomous::SWERVE_OP_STEP_TIME_S;
    }

    // Stop motion
    m_pSwerveDrive->SetModuleStates({0_m, 0_m}, 0.0, true, true);
}

#endif // EASTTECHROBOTAUTONOMOUS_HPP
