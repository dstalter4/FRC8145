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

    // First start ramping up the shooter motors
    m_pShooterMotors->Set(SHOOTER_MOTOR_SPEAKER_CLOSE_SPEED, SHOOTER_MOTOR_SPEAKER_OFFSET_SPEED);

    // Pivot the mechanism to the desired angle
    (void)pPivotLeaderTalon->SetControl(pivotPositionVoltage.WithPosition(PIVOT_ANGLE_TOUCHING_SPEAKER));

    // Wait a bit for everything to be ready
    AutonomousDelay(1.0_s);

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
    m_pShooterMotors->Set(SHOOTER_MOTOR_SPEAKER_FAR_SPEED, SHOOTER_MOTOR_SPEAKER_OFFSET_SPEED);

    // Pivot the mechanism to the desired angle
    (void)pPivotLeaderTalon->SetControl(pivotPositionVoltage.WithPosition(PIVOT_ANGLE_FROM_PODIUM - 8.0_deg));

    // Adjust robot angle toward speaker
    RobotRotate towardSpeakerRotate = (m_AllianceColor.value() == DriverStation::Alliance::kRed) ? RobotRotate::ROBOT_CLOCKWISE : RobotRotate::ROBOT_COUNTER_CLOCKWISE;
    AutonomousSwerveDriveSequence(RobotDirection::ROBOT_NO_DIRECTION, towardSpeakerRotate, 0.0, 0.0, 0.08, 0.9_s, true);

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
