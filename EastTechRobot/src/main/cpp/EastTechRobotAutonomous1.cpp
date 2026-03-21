////////////////////////////////////////////////////////////////////////////////
/// @file   EastTechRobotAutonomous1.cpp
/// @author David Stalter
///
/// @details
/// Implementation of autonomous routine 1 for EastTechRobot.
///
/// Copyright (c) 2024 East Technical High School
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
// <none>

// C INCLUDES
// (none)

// C++ INCLUDES
#include "RobotUtils.hpp"               // for DisplayMessage()
#include "EastTechRobot.hpp"            // for robot class declaration
#include "EastTechRobotAutonomous.hpp"  // for autonomous declarations


////////////////////////////////////////////////////////////////
/// @method EastTechRobot::AutonomousRoutine1
///
/// Autonomous routine 1.
/// Very rough auto program for shooting the initial amount of fuel from the center
///
////////////////////////////////////////////////////////////////
void EastTechRobot::AutonomousRoutine1()
{
    //drive forward
    m_AutoSwerveDirections.SetSwerveDirections(RobotTranslation::ROBOT_TRANSLATION_REVERSE, RobotStrafe::ROBOT_NO_STRAFE, RobotRotation::ROBOT_NO_ROTATION);
    AutonomousSwerveDriveSequence(m_AutoSwerveDirections, 0.15, 0.0, 0.0, 1.5_s, true);
    //shooter ramp up
    m_pShooterMotors->Set(SHOOTER_MOTOR_SPEED);
    AutonomousDelay(2.0_s);
    //dump
    m_pHopperFeed->SetDutyCycle(HOPPER_FEED_MOTOR_SPEED);
    m_pShooterFeed->SetDutyCycle(SHOOTER_FEED_MOTOR_SPEED);
    AutonomousDelay(5.0_s);
    //everything off
    m_pShooterMotors->Set(0.0);
    m_pHopperFeed->SetDutyCycle(0.0);
    m_pShooterFeed->SetDutyCycle(0.0);

    // Returning from here will enter the idle state until autonomous is over
    RobotUtils::DisplayMessage("Auto routine 1 done.");
}
