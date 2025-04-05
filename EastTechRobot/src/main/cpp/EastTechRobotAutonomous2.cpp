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
    // The robot faces the driver station, so it is off by 180 degrees
    m_pPigeon->SetYaw(units::angle::degree_t(ANGLE_180_DEGREES));

    // We have to wait for the absolute encoders to stabilize
    while (!m_AbsoluteEncodersInitialized)
    {
        WaitForSensorConfig();
    }

    // Drive towards the reef (remember that the robot is facing the opposite alliance wall)
    m_AutoSwerveDirections.SetSwerveDirections(RobotTranslation::ROBOT_TRANSLATION_REVERSE, RobotStrafe::ROBOT_NO_STRAFE, RobotRotation::ROBOT_NO_ROTATION);
    AutonomousSwerveDriveSequence(m_AutoSwerveDirections, 0.15, 0.0, 0.0, 2.0_s, true);

    // Turn towards the reef (forward is still the other alliance wall, thus clockwise)
    AutonomousRotateByGyroSequence(RobotRotation::ROBOT_CLOCKWISE, 50.0, 0.2, true);

    // Manually (and slowly) move the arm to the L4 position
    m_ArmPosition = ArmPosition::REEF_L4;
    m_ArmTargetDegrees = ARM_REEF_L4_TARGET_DEGREES;
    m_pArmPivotMotor->SetDutyCycle(-0.20);
    while (units::angle::degree_t(m_pArmPivotMotor->m_pTalonFx->GetPosition().GetValue()) > ARM_REEF_L4_TARGET_DEGREES)
    {
        AutonomousDelay(0.02_s);
    }
    m_pArmPivotMotor->SetDutyCycle(0.0);

    // Manually (and slowly) move the wrist to the L4 position
    m_WristTargetDegrees = WRIST_REEF_L4_TARGET_DEGREES;
    m_pWristPivotMotor->SetDutyCycle(-0.10);
    while (units::angle::degree_t(m_pWristPivotMotor->m_pTalonFx->GetPosition().GetValue()) > WRIST_REEF_L4_TARGET_DEGREES)
    {
        AutonomousDelay(0.02_s);
    }
    m_pWristPivotMotor->SetDutyCycle(0.0);

    // Set the lift to L4 (using angle positioning seems to keep the coral in place)
    m_LiftPosition = LiftPosition::LIFT_UP;
    m_LiftTargetDegrees = LIFT_UP_ANGLE;
    m_pLiftMotors->SetAngle(m_LiftTargetDegrees.value());

    // Returning from here will enter the idle state until autonomous is over
    RobotUtils::DisplayMessage("Auto routine 2 done.");
}
