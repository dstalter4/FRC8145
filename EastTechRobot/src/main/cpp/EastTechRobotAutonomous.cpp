////////////////////////////////////////////////////////////////////////////////
/// @file   EastTechRobotAutonomous.cpp
/// @author David Stalter
///
/// @details
/// Implementation of autonomous routines for EastTechRobot.
///
/// Copyright (c) 2024 East Technical High School
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
// <none>

// C INCLUDES
// (none)

// C++ INCLUDES
#include "EastTechRobot.hpp"            // for robot class declaration
#include "EastTechRobotAutonomous.hpp"  // for autonomous declarations
#include "RobotCamera.hpp"              // for interacting with cameras

// NAMESPACE DATA
bool EastTechRobotAutonomous::bAutonomousExecutionComplete;


////////////////////////////////////////////////////////////////
/// @method EastTechRobot::AutonomousInit
///
/// The autonomous init method.  This method is called once each
/// time the robot enters autonomous control.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::AutonomousInit()
{
    RobotUtils::DisplayMessage("AutonomousInit called.");
    
    // Put everything in a stable state
    InitialStateSetup();
    
    // Indicate the autonomous routine has not executed yet
    EastTechRobotAutonomous::bAutonomousExecutionComplete = false;
    
    m_pSafetyTimer->Stop();
    m_pSafetyTimer->Reset();
    
    // Autonomous needs full camera processing
    RobotCamera::SetFullProcessing(true);
    RobotCamera::SetLimelightMode(RobotCamera::LimelightMode::VISION_PROCESSOR);
    RobotCamera::SetLimelightLedMode(RobotCamera::LimelightLedMode::ARRAY_ON);
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::AutonomousPeriodic
///
/// The autonomous control method.  This method is called
/// periodically while the robot is in autonomous control.
/// Even though this method wil be called periodically, it
/// deliberately checks the driver station state controls.
/// This is to give finer control over the autonomous state
/// machine flow.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::AutonomousPeriodic()
{
    // Log a mode change if one occurred
    CheckAndUpdateRobotMode(ROBOT_MODE_AUTONOMOUS);
    
    if (EastTechRobotAutonomous::bAutonomousExecutionComplete)
    {
        return;
    }
    
    // @todo: Figure out how to kick the watchdog from here so it doesn't overrun.
    // @note: Since autonomous is configured as a one shot state machine, there will definitely be watchdog overrun.
    
    // Change values in the header to control having an
    // autonomous routine and which is selected
    
    // Get the selected autonomous routine from the smart dashboard
    std::string selectedAutoRoutineString = m_AutonomousChooser.GetSelected();
    
    // Auto routine 1
    //if ( EastTechRobotAutonomous::ROUTINE_1 )
    if (selectedAutoRoutineString == AUTO_ROUTINE_1_STRING)
    {
        RobotUtils::DisplayMessage("Auto routine 1.");
        AutonomousRoutine1();
    }
    
    // Auto routine 2
    //else if ( EastTechRobotAutonomous::ROUTINE_2 )
    else if (selectedAutoRoutineString == AUTO_ROUTINE_2_STRING)
    {
        RobotUtils::DisplayMessage("Auto routine 2.");
        AutonomousRoutine2();
    }
    
    // Auto routine 3
    //else if ( EastTechRobotAutonomous::ROUTINE_3 )
    else if (selectedAutoRoutineString == AUTO_ROUTINE_3_STRING)
    {
        RobotUtils::DisplayMessage("Auto routine 3.");
        AutonomousRoutine3();
    }

    /* !!! ONLY ENABLE TEST AUTONOMOUS CODE WHEN TESTING
           SELECT A FUNCTIONING ROUTINE FOR ACTUAL MATCHES !!! */
    //else if ( EastTechRobotAutonomous::TEST_ENABLED )
    else if (selectedAutoRoutineString == AUTO_TEST_ROUTINE_STRING)
    {
        RobotUtils::DisplayMessage("Auto test code.");
        AutonomousTestRoutine();
    }

    else
    {
        // No option was selected; ensure known behavior to avoid issues
        RobotUtils::DisplayMessage("No auto selection made, going idle.");
    }
    
    // One shot through autonomous is over, indicate as such.
    EastTechRobotAutonomous::bAutonomousExecutionComplete = true;
    
    /*
    // Idle until auto is terminated
    RobotUtils::DisplayMessage("Auto idle loop.");
    while ( m_pDriverStation->IsAutonomous() && m_pDriverStation->IsEnabled() )
    {
    }
    */
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::AutonomousCommon
///
/// Common autonomous behavior.  It moves away from the alliance
/// wall and to the fuel loading station.  The variance is
/// whether it shoots at the start or at the end.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::AutonomousCommon()
{

    if (m_AllianceColor == DriverStation::Alliance::kRed)
    {
        AutonomousCommonRed();
    }
    else if (m_AllianceColor == DriverStation::Alliance::kBlue)
    {
        AutonomousCommonBlue();
    }
    else
    {
    }
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::AutonomousCommonRed
///
/// Common autonomous behavior when on the red alliance.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::AutonomousCommonRed()
{
}





////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
///                           Red/Blue Separation                            ///
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////





////////////////////////////////////////////////////////////////
// @method EastTechRobot::AutonomousCommonBlue
///
/// Common autonomous behavior when on the blue alliance.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::AutonomousCommonBlue()
{
}
