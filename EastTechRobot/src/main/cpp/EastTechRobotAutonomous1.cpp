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
///
////////////////////////////////////////////////////////////////
void EastTechRobot::AutonomousRoutine1()
{
    // Local objects needed during autonomous
    TalonFX * pPivotLeaderTalon = m_pPivotMotors->GetMotorObject(PIVOT_MOTORS_CAN_START_ID);
    PositionVoltage pivotPositionVoltage(0.0_tr);

    // First start ramping up the shooter motors
    m_pShooterMotors->Set(SHOOTER_MOTOR_SPEAKER_SPEED, SHOOTER_MOTOR_SPEAKER_OFFSET_SPEED);

    // Pivot the mechanism to the desired angle
    (void)pPivotLeaderTalon->SetControl(pivotPositionVoltage.WithPosition(PIVOT_ANGLE_TOUCHING_SPEAKER));

    // Wait a bit for everything to be ready
    AutonomousDelay(1.0_s);

    // Feeder motor to take the shot
    m_pFeederMotor->SetDutyCycle(FEEDER_MOTOR_SPEED);
    AutonomousDelay(0.5_s);

    // Shooter slowly reversing (to slow it down), back up to the next game piece.
    // Start the intake to pick it up, keep feeder on.
    m_pShooterMotors->Set(0.1);
    m_pIntakeMotor->SetDutyCycle(INTAKE_MOTOR_SPEED);
    AutonomousSwerveDriveSequence(RobotDirection::ROBOT_FORWARD, ROBOT_NO_ROTATE, 0.30, 0.0, 1.5_s, true);

    // Stop for a brief period allowing the piece to get picked up
    AutonomousDelay(1.0_s);

    // Intake/feeder motors off, drive back to the speaker
    m_pIntakeMotor->SetDutyCycle(0.0);
    m_pFeederMotor->SetDutyCycle(0.0);
    AutonomousSwerveDriveSequence(RobotDirection::ROBOT_REVERSE, ROBOT_NO_ROTATE, 0.30, 0.0, 1.5_s, true);

    // Should be in position, go through shooting again.  Back feed real quick, ramp up, feed to shoot.
    m_pFeederMotor->SetDutyCycle(-FEEDER_MOTOR_SPEED);
    AutonomousDelay(0.5_s);
    m_pShooterMotors->Set(SHOOTER_MOTOR_SPEAKER_SPEED, SHOOTER_MOTOR_SPEAKER_OFFSET_SPEED);
    AutonomousDelay(1.0_s);
    m_pFeederMotor->SetDutyCycle(FEEDER_MOTOR_SPEED);
    AutonomousDelay(0.5_s);

    // Everything off
    m_pFeederMotor->SetDutyCycle(0.0);
    m_pShooterMotors->Set(0.0);

    // Returning from here will enter the idle state until autonomous is over
    RobotUtils::DisplayMessage("Auto routine 1 done.");
}
