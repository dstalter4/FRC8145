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
    // The robot faces the driver station, so it is off by 180 degrees
    m_pPigeon->SetYaw(units::angle::degree_t(ANGLE_180_DEGREES));

    // We have to wait for the absolute encoders to stabilize
    while (!m_AbsoluteEncodersInitialized)
    {
        WaitForSensorConfig();
    }

    // Drive towards the reef (remember that the robot is facing the opposite alliance wall)
    m_AutoSwerveDirections.SetSwerveDirections(RobotTranslation::ROBOT_TRANSLATION_REVERSE, RobotStrafe::ROBOT_NO_STRAFE, RobotRotation::ROBOT_NO_ROTATION);
    AutonomousSwerveDriveSequence(m_AutoSwerveDirections, 0.15, 0.0, 0.0, 2.5_s, true);

    if (m_AutonomousScoreCoral.GetSelected())
    {
        // Move the arm and wrist to the L1 position
        m_pArmPivotMotor->SetPositionVoltage(ARM_REEF_L1_TARGET_DEGREES.value());
        m_pWristPivotMotor->SetPositionVoltage(WRIST_REEF_L1_TARGET_DEGREES.value() + (30.0_deg).value());

        AutonomousDelay(1.5_s);

        // Try and score in the trough by jogging for a total of 1.5_s
        m_pGamePieceMotor->SetDutyCycle(-GAME_PIECE_MOTOR_SPEED_SLOW);
        AutonomousDelay(0.25_s);
        m_pGamePieceMotor->SetDutyCycle(0.0);
        AutonomousDelay(0.25_s);
        m_pGamePieceMotor->SetDutyCycle(-GAME_PIECE_MOTOR_SPEED_SLOW);
        AutonomousDelay(0.25_s);
        m_pGamePieceMotor->SetDutyCycle(0.0);
        AutonomousDelay(0.25_s);
        m_pGamePieceMotor->SetDutyCycle(-GAME_PIECE_MOTOR_SPEED_SLOW);
        AutonomousDelay(0.25_s);
        m_pGamePieceMotor->SetDutyCycle(0.0);
        AutonomousDelay(0.25_s);
        m_pGamePieceMotor->SetDutyCycle(-GAME_PIECE_MOTOR_SPEED_SLOW);
        AutonomousDelay(0.25_s);
        m_pGamePieceMotor->SetDutyCycle(0.0);

        AutonomousDelay(1.0_s);

        // Restore the arm and wrist to the neutral position
        m_pArmPivotMotor->SetPositionVoltage(ARM_NEUTRAL_TARGET_DEGREES.value());
        m_pWristPivotMotor->SetPositionVoltage(WRIST_NEUTRAL_TARGET_DEGREES.value());
    }

    // Returning from here will enter the idle state until autonomous is over
    RobotUtils::DisplayMessage("Auto routine 1 done.");
}
