////////////////////////////////////////////////////////////////////////////////
/// @file   EastTechRobotAutonomous3.cpp
/// @author David Stalter
///
/// @details
/// Implementation of autonomous routine 3 for EastTechRobot.
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
/// @method EastTechRobot::AutonomousRoutine3
///
/// Autonomous routine 3.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::AutonomousRoutine3()
{
    // Local objects needed during autonomous
    TalonFX * pPivotLeaderTalon = m_pPivotMotors->GetMotorObject(PIVOT_MOTORS_CAN_START_ID);
    PositionVoltage pivotPositionVoltage(0.0_tr);

    double shooterSpeed = (m_AllianceColor.value() == DriverStation::Alliance::kRed) ? SHOOTER_MOTOR_SPEAKER_CLOSE_CCW_SPEED : SHOOTER_MOTOR_SPEAKER_CLOSE_CW_SPEED;
    double shooterOffsetSpeed = (m_AllianceColor.value() == DriverStation::Alliance::kRed) ? SHOOTER_MOTOR_SPEAKER_CCW_OFFSET_SPEED : SHOOTER_MOTOR_SPEAKER_CW_OFFSET_SPEED;

    // First start ramping up the shooter motors
    m_pShooterMotors->Set(shooterSpeed, shooterOffsetSpeed);

    // Pivot the mechanism to the desired angle
    (void)pPivotLeaderTalon->SetControl(pivotPositionVoltage.WithPosition(PIVOT_ANGLE_TOUCHING_SPEAKER + 2.5_deg));

    // Wait a bit for everything to be ready
    AutonomousDelay(1.5_s);

    // Feeder motor to take the shot
    m_pFeederMotor->SetDutyCycle(FEEDER_MOTOR_SPEED);
    AutonomousDelay(0.5_s);

    // Shooter and feeder motors off
    m_pShooterMotors->Set(0.0);
    m_pFeederMotor->SetDutyCycle(0.0);

    // Intake motor on and reposition pivot before moving to the next note
    m_pIntakeMotor->SetDutyCycle(INTAKE_MOTOR_SPEED);
    (void)pPivotLeaderTalon->SetControl(pivotPositionVoltage.WithPosition(PIVOT_ANGLE_INTAKE_NOTE));

    // Compute the direction of movement based on alliance color since the field is not symmetrical
    RobotDirection autoLeaveByAmpDirection = (m_AllianceColor.value() == DriverStation::Alliance::kRed) ? RobotDirection::ROBOT_LEFT : RobotDirection::ROBOT_RIGHT;
    RobotRotate autoLeaveByAmpRotate = (m_AllianceColor.value() == DriverStation::Alliance::kRed) ? RobotRotate::ROBOT_COUNTER_CLOCKWISE : RobotRotate::ROBOT_CLOCKWISE;
    RobotDirection autoLeaveDirection = static_cast<RobotDirection>(RobotDirection::ROBOT_FORWARD | autoLeaveByAmpDirection);
    AutonomousSwerveDriveSequence(autoLeaveDirection, autoLeaveByAmpRotate, 0.15, 0.18, 0.05, 2.5_s, true);
    AutonomousDelay(1.0_s);

    // Note should be picked up, intake off
    m_pIntakeMotor->SetDutyCycle(0.0);

    // Start the far shot

    // First start ramping up the shooter motors
    shooterSpeed = (m_AllianceColor.value() == DriverStation::Alliance::kRed) ? SHOOTER_MOTOR_SPEAKER_FAR_CCW_SPEED : SHOOTER_MOTOR_SPEAKER_FAR_CW_SPEED;
    shooterOffsetSpeed = (m_AllianceColor.value() == DriverStation::Alliance::kRed) ? SHOOTER_MOTOR_SPEAKER_CCW_OFFSET_SPEED : SHOOTER_MOTOR_SPEAKER_CW_OFFSET_SPEED;
    m_pShooterMotors->Set(shooterSpeed, shooterOffsetSpeed);

    // Pivot the mechanism to the desired angle
    (void)pPivotLeaderTalon->SetControl(pivotPositionVoltage.WithPosition(PIVOT_ANGLE_FROM_PODIUM - 3.5_deg));

    // Adjust robot angle toward speaker
    RobotRotate towardSpeakerRotate = (m_AllianceColor.value() == DriverStation::Alliance::kRed) ? RobotRotate::ROBOT_CLOCKWISE : RobotRotate::ROBOT_COUNTER_CLOCKWISE;
    AutonomousSwerveDriveSequence(RobotDirection::ROBOT_NO_DIRECTION, towardSpeakerRotate, 0.0, 0.0, 0.08, 1.0_s, true);

    // Feeder motor to take the shot
    m_pFeederMotor->SetDutyCycle(FEEDER_MOTOR_SPEED);
    AutonomousDelay(1.5_s);

    // Shooter motor off, feeder off, pivot down
    m_pShooterMotors->Set(0.0);
    m_pFeederMotor->SetDutyCycle(0.0);
    (void)pPivotLeaderTalon->SetControl(pivotPositionVoltage.WithPosition(PIVOT_ANGLE_RUNTIME_BASE));

    // Set the pigeon angle relative to final robot position with zero down field
    units::angle::degree_t gyroYaw = (m_AllianceColor.value() == DriverStation::Alliance::kRed) ? -23.0_deg : 23.0_deg;
    m_pPigeon->SetYaw(gyroYaw);

    // Returning from here will enter the idle state until autonomous is over
    RobotUtils::DisplayMessage("Auto routine 3 done.");
}
