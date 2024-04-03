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
/// Autonomous routine 2.  Runs from the source side.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::AutonomousRoutine2()
{
    // Local objects needed during autonomous
    TalonFX * pPivotLeaderTalon = m_pPivotMotors->GetMotorObject(PIVOT_MOTORS_CAN_START_ID);
    PositionVoltage pivotPositionVoltage(0.0_tr);

    // First start ramping up the shooter motors
    double shooterSpeed = (m_AllianceColor.value() == DriverStation::Alliance::kRed) ? SHOOTER_MOTOR_SPEAKER_CLOSE_CCW_SPEED : SHOOTER_MOTOR_SPEAKER_CLOSE_CW_SPEED;
    double shooterOffsetSpeed = (m_AllianceColor.value() == DriverStation::Alliance::kRed) ? SHOOTER_MOTOR_SPEAKER_CCW_OFFSET_SPEED : SHOOTER_MOTOR_SPEAKER_CW_OFFSET_SPEED;
    m_pShooterMotors->Set(shooterSpeed, shooterOffsetSpeed);

    // Pivot the mechanism to the desired angle
    (void)pPivotLeaderTalon->SetControl(pivotPositionVoltage.WithPosition(PIVOT_ANGLE_TOUCHING_SPEAKER));

    // Wait a bit for everything to be ready
    AutonomousDelay(1.5_s);

    // Feeder motor to take the shot
    m_pFeederMotor->SetDutyCycle(FEEDER_MOTOR_SPEED);
    AutonomousDelay(0.5_s);

    // Shooter motor off, feeder motor off, pivot down
    m_pShooterMotors->Set(0.0);
    m_pFeederMotor->SetDutyCycle(0.0);
    (void)pPivotLeaderTalon->SetControl(pivotPositionVoltage.WithPosition(PIVOT_ANGLE_RUNTIME_BASE));

    // Back up to leave past the line
    RobotStrafe autoLeaveBySourceStrafe = (m_AllianceColor.value() == DriverStation::Alliance::kRed) ? RobotStrafe::ROBOT_STRAFE_RIGHT : RobotStrafe::ROBOT_STRAFE_LEFT;
    m_AutoSwerveDirections.SetSwerveDirections(RobotTranslation::ROBOT_TRANSLATION_FORWARD, autoLeaveBySourceStrafe, RobotRotation::ROBOT_NO_ROTATION);
    AutonomousSwerveDriveSequence(m_AutoSwerveDirections, 0.30, 0.0, 0.0, 3.0_s, true);

    // Returning from here will enter the idle state until autonomous is over
    RobotUtils::DisplayMessage("Auto routine 2 done.");
}
