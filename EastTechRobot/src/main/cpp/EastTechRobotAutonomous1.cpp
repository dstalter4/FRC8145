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
/// Autonomous routine 1.  Runs from center.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::AutonomousRoutine1()
{
    // Local objects needed during autonomous
    TalonFX * pPivotLeaderTalon = m_pPivotMotors->GetMotorObject(PIVOT_MOTORS_CAN_START_ID);
    PositionVoltage pivotPositionVoltage(0.0_tr);

    double shooterSpeed = (m_AllianceColor.value() == DriverStation::Alliance::kRed) ? SHOOTER_MOTOR_SPEAKER_CLOSE_CCW_SPEED : SHOOTER_MOTOR_SPEAKER_CLOSE_CW_SPEED;
    double shooterOffsetSpeed = (m_AllianceColor.value() == DriverStation::Alliance::kRed) ? SHOOTER_MOTOR_SPEAKER_CCW_OFFSET_SPEED : SHOOTER_MOTOR_SPEAKER_CW_OFFSET_SPEED;

    // First start ramping up the shooter motors
    m_pShooterMotors->Set(shooterSpeed, shooterOffsetSpeed);

    // Pivot the mechanism to the desired angle
    (void)pPivotLeaderTalon->SetControl(pivotPositionVoltage.WithPosition(PIVOT_ANGLE_TOUCHING_SPEAKER));

    // Wait a bit for everything to be ready
    AutonomousDelay(1.5_s);

    // Feeder motor to take the shot
    m_pFeederMotor->SetDutyCycle(FEEDER_MOTOR_SPEED);
    AutonomousDelay(0.5_s);

    // Shooter slowly reversing (to slow it down), back up to the next game piece.
    // Start the intake to pick it up, keep feeder on.
    m_pShooterMotors->Set(0.1);
    m_pIntakeMotor->SetDutyCycle(INTAKE_MOTOR_SPEED);
    m_AutoSwerveDirections.SetSwerveDirections(RobotTranslation::ROBOT_TRANSLATION_FORWARD, RobotStrafe::ROBOT_NO_STRAFE, RobotRotation::ROBOT_NO_ROTATION);
    AutonomousSwerveDriveSequence(m_AutoSwerveDirections, 0.30, 0.0, 0.0, 1.5_s, true);

    // Stop for a brief period allowing the piece to get picked up
    AutonomousDelay(1.0_s);

    // Intake/feeder motors off, drive back to the speaker
    m_pIntakeMotor->SetDutyCycle(0.0);
    m_pFeederMotor->SetDutyCycle(0.0);
    m_AutoSwerveDirections.SetSwerveDirections(RobotTranslation::ROBOT_TRANSLATION_REVERSE, RobotStrafe::ROBOT_NO_STRAFE, RobotRotation::ROBOT_NO_ROTATION);
    AutonomousSwerveDriveSequence(m_AutoSwerveDirections, 0.30, 0.0, 0.0, 1.5_s, true);

    // Should be in position, go through shooting again.  Back feed real quick, ramp up, feed to shoot.
    m_pFeederMotor->SetDutyCycle(-FEEDER_MOTOR_SPEED);
    AutonomousDelay(0.15_s);
    m_pShooterMotors->Set(shooterSpeed, shooterOffsetSpeed);
    AutonomousDelay(1.0_s);
    m_pFeederMotor->SetDutyCycle(FEEDER_MOTOR_SPEED);
    AutonomousDelay(1.0_s);

    // Everything off and pivot down
    m_pFeederMotor->SetDutyCycle(0.0);
    m_pShooterMotors->Set(0.0);
    (void)pPivotLeaderTalon->SetControl(pivotPositionVoltage.WithPosition(PIVOT_ANGLE_RUNTIME_BASE));

    // Returning from here will enter the idle state until autonomous is over
    RobotUtils::DisplayMessage("Auto routine 1 done.");
}
