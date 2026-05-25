////////////////////////////////////////////////////////////////////////////////
/// @file   EastTechRobotAutonomous.hpp
/// @author David Stalter
///
/// @details
/// Contains the declarations for the autonomous portions of code ran in an FRC
/// robot.
///
/// Copyright (c) 2026 East Technical High School
////////////////////////////////////////////////////////////////////////////////

#ifndef EASTTECHROBOTAUTONOMOUS_HPP
#define EASTTECHROBOTAUTONOMOUS_HPP

// SYSTEM INCLUDES
#include <frc2/command/CommandPtr.h>    // for CommandPtr type

// C INCLUDES
// (none)

// C++ INCLUDES
#include "EastTechRobot.hpp"            // for inline autonomous function declarations

using namespace frc;

////////////////////////////////////////////////////////////////
/// @namespace EastTechRobotAutonomous
///
/// Namespace that contains robot autonomous variable and
/// function declarations.
///
////////////////////////////////////////////////////////////////
namespace EastTechRobotAutonomous
{
    // TYPEDEFS
    // (none)
    
    // ENUMS
    // (none)    
    
    // STRUCTS
    // (none)
    
    // VARIABLES
    extern bool bAutonomousExecutionComplete;
    extern std::optional<CommandPtr> AutonomousCommand;
    
    // CONSTS
    
    // Autonomous Mode Constants
    // @todo: Convert to class and make a friend in EastTechRobot
    
    // Note: Only enable one autonomous routine!
    // Note: Autonomous routines are currently controlled by
    // the SendableChooser.
    //static const bool       ROUTINE_1                           = true;
    //static const bool       ROUTINE_2                           = false;
    //static const bool       ROUTINE_3                           = false;
    //static const bool       TEST_ENABLED                        = false;
    static const bool       USE_COMMAND_BASED_AUTONOMOUS        = false;
} // End namespace

#endif // EASTTECHROBOTAUTONOMOUS_HPP
