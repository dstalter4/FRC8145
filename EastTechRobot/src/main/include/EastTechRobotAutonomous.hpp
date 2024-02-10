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
    static constexpr double DRIVE_SPEED_SLOW                    =  0.30;
    static constexpr double DRIVE_SPEED_FAST                    =  0.50;
    static constexpr double TURN_SPEED                          =  0.25;
    static constexpr double COUNTERACT_COAST_MOTOR_SPEED        =  0.20;
    
    // Autonomous angle constants
    static const int        FORTY_FIVE_DEGREES                  = 45;
    static const int        NINETY_DEGREES                      = 90;
    static const int        ONE_HUNDRED_EIGHTY_DEGREES          = 180;
    static const int        THREE_HUNDRED_SIXTY_DEGREES         = 360;
    
    // Autonomous delay constants
    static constexpr units::second_t SWERVE_OP_STEP_TIME_S      =  0.10_s;
    static constexpr units::second_t COUNTERACT_COAST_TIME_S    =  0.25_s;
    static constexpr units::second_t DELAY_SHORT_S              =  0.50_s;
    static constexpr units::second_t DELAY_MEDIUM_S             =  1.00_s;
    static constexpr units::second_t DELAY_LONG_S               =  2.00_s;
    
    // Autonomous misc constants
    static const unsigned   I2C_THREAD_UPDATE_RATE_MS           = 20U;
    
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
/// @method EastTechRobot::AutonomousDriveSequence
///
/// Drives during autonomous for a specified amount of time
/// using traditional differential drive.
///
////////////////////////////////////////////////////////////////
inline void EastTechRobot::AutonomousDriveSequence(RobotDirection direction, double speed, units::second_t time)
{
    double leftSpeed = 0.0;
    double rightSpeed = 0.0;

    switch (direction)
    {
        case ROBOT_FORWARD:
        {
            leftSpeed = speed * LEFT_DRIVE_FORWARD_SCALAR;
            rightSpeed = speed * RIGHT_DRIVE_FORWARD_SCALAR;
            break;
        }
        case ROBOT_REVERSE:
        {
            leftSpeed = speed * LEFT_DRIVE_REVERSE_SCALAR;
            rightSpeed = speed * RIGHT_DRIVE_REVERSE_SCALAR;
            break;
        }
        case ROBOT_LEFT:
        {
            leftSpeed = speed * LEFT_DRIVE_REVERSE_SCALAR;
            rightSpeed = speed * RIGHT_DRIVE_FORWARD_SCALAR;
            break;
        }
        case ROBOT_RIGHT:
        {
            leftSpeed = speed * LEFT_DRIVE_FORWARD_SCALAR;
            rightSpeed = speed * RIGHT_DRIVE_REVERSE_SCALAR;
            break;
        }
        default:
        {
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            break;
        }
    }

    // First turn the motors on
    m_pLeftDriveMotors->Set(leftSpeed);
    m_pRightDriveMotors->Set(rightSpeed);

    // Time it
    AutonomousDelay(time);

    // Motors back off
    m_pLeftDriveMotors->Set(OFF);
    m_pRightDriveMotors->Set(OFF);
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::AutonomousSwerveDriveSequence
///
/// Drives during autonomous for a specified amount of time
/// using swerve drive modules.
///
////////////////////////////////////////////////////////////////
inline void EastTechRobot::AutonomousSwerveDriveSequence(RobotDirection direction, RobotRotate rotate, double speed, double rotateSpeed, units::second_t time, bool bFieldRelative)
{
    units::meter_t translation = 0.0_m;
    units::meter_t strafe = 0.0_m;

    switch (direction)
    {
        case ROBOT_FORWARD:
        {
            translation = units::meter_t(speed);
            break;
        }
        case ROBOT_REVERSE:
        {
            translation = units::meter_t(-speed);
            break;
        }
        case ROBOT_LEFT:
        {
            strafe = units::meter_t(speed);
            break;
        }
        case ROBOT_RIGHT:
        {
            strafe = units::meter_t(-speed);
            break;
        }
        case ROBOT_NO_DIRECTION:
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



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::AutonomousBackDrive
///
/// Back drives the motors to abruptly stop the robot.
///
////////////////////////////////////////////////////////////////
inline void EastTechRobot::AutonomousBackDrive(RobotDirection currentDirection)
{
    double leftSpeed = EastTechRobotAutonomous::COUNTERACT_COAST_MOTOR_SPEED;
    double rightSpeed = EastTechRobotAutonomous::COUNTERACT_COAST_MOTOR_SPEED;

    switch (currentDirection)
    {
        // If we are currently going forward, back drive is reverse
        case ROBOT_FORWARD:
        {
            leftSpeed *= LEFT_DRIVE_REVERSE_SCALAR;
            rightSpeed *= RIGHT_DRIVE_REVERSE_SCALAR;
            break;
        }
        // If we are currently going reverse, back drive is forward
        case ROBOT_REVERSE:
        {
            leftSpeed *= LEFT_DRIVE_FORWARD_SCALAR;
            rightSpeed *= RIGHT_DRIVE_FORWARD_SCALAR;
            break;
        }
        default:
        {
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            break;
        }
    }
    
    // Counteract coast
    m_pLeftDriveMotors->Set(leftSpeed);
    m_pRightDriveMotors->Set(rightSpeed);
    
    // Delay
    AutonomousDelay(EastTechRobotAutonomous::COUNTERACT_COAST_TIME_S);
    
    // Motors off
    m_pLeftDriveMotors->Set(OFF);
    m_pRightDriveMotors->Set(OFF);
    
    m_pSafetyTimer->Reset();
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::AutonomousBackDriveTurn
///
/// Back drives the motors to abruptly stop the robot during
/// a turn.
///
////////////////////////////////////////////////////////////////
inline void EastTechRobot::AutonomousBackDriveTurn(RobotDirection currentDirection)
{
    double leftSpeed = EastTechRobotAutonomous::COUNTERACT_COAST_MOTOR_SPEED;
    double rightSpeed = EastTechRobotAutonomous::COUNTERACT_COAST_MOTOR_SPEED;

    switch (currentDirection)
    {
        // If the turn is left, counteract is right
        case ROBOT_LEFT:
        {
            leftSpeed *= LEFT_DRIVE_FORWARD_SCALAR;
            rightSpeed *= RIGHT_DRIVE_REVERSE_SCALAR;
            break;
        }
        // If the turn is right, counteract is left
        case ROBOT_RIGHT:
        {
            leftSpeed *= LEFT_DRIVE_REVERSE_SCALAR;
            rightSpeed *= RIGHT_DRIVE_FORWARD_SCALAR;
            break;
        }
        default:
        {
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            break;
        }
    }
    
    // Counteract coast
    m_pLeftDriveMotors->Set(leftSpeed);
    m_pRightDriveMotors->Set(rightSpeed);
    
    // Delay
    AutonomousDelay(EastTechRobotAutonomous::COUNTERACT_COAST_TIME_S);
    
    // Motors off
    m_pLeftDriveMotors->Set(OFF);
    m_pRightDriveMotors->Set(OFF);
    
    m_pSafetyTimer->Reset();
}

#endif // EASTTECHROBOTAUTONOMOUS_HPP
