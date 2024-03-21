////////////////////////////////////////////////////////////////////////////////
/// @file   EastTechRobotAutonomous2.cpp
/// @author David Stalter
///
/// @details
/// Implementation of autonomous routine 2 for EastTechRobot.
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
/// @method EastTechRobot::AutonomousRoutine2
///
/// Autonomous routine 2.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::AutonomousRoutine2()
{
    // Local objects needed during autonomous
    TalonFX * pPivotLeaderTalon = m_pPivotMotors->GetMotorObject(PIVOT_MOTORS_CAN_START_ID);
    PositionVoltage pivotPositionVoltage(0.0_tr);

    // First start ramping up the shooter motors
    m_pShooterMotors->Set(SHOOTER_MOTOR_SPEAKER_CLOSE_SPEED, SHOOTER_MOTOR_SPEAKER_OFFSET_SPEED);

    // Pivot the mechanism to the desired angle
    (void)pPivotLeaderTalon->SetControl(pivotPositionVoltage.WithPosition(PIVOT_ANGLE_TOUCHING_SPEAKER));

    // Wait a bit for everything to be ready
    AutonomousDelay(1.0_s);

    // Feeder motor to take the shot
    m_pFeederMotor->SetDutyCycle(FEEDER_MOTOR_SPEED);
    AutonomousDelay(0.5_s);

    // Shooter motor off, just back up
    m_pShooterMotors->Set(0.0);
    AutonomousSwerveDriveSequence(RobotDirection::ROBOT_FORWARD, ROBOT_NO_ROTATE, 0.30, 0.0, 3.0_s, true);

    // Returning from here will enter the idle state until autonomous is over
    RobotUtils::DisplayMessage("Auto routine 2 done.");
}
