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
    // Returning from here will enter the idle state until autonomous is over
    RobotUtils::DisplayMessage("Auto routine 3 done.");
}
