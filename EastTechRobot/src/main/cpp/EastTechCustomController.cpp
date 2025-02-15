////////////////////////////////////////////////////////////////////////////////
/// @file   EastTechController.cpp
/// @author David Stalter
///
/// @details
/// A class designed to interface to several controller types (Logitech Gamepad,
/// Xbox GameSir, PS4, etc.) with custom responses.
///
/// Copyright (c) 2024 East Technical High School
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
// <none>

// C INCLUDES
#include "frc/smartdashboard/SmartDashboard.h"  // for interacting with the smart dashboard

// C++ INCLUDES
#include "DriveConfiguration.hpp"               // for DRIVE_STYLE
#include "RobotUtils.hpp"                       // for ASSERT, DEBUG_PRINTS
#include "EastTechController.hpp"               // for class declaration

// STATIC MEMBER DATA
// (none)


////////////////////////////////////////////////////////////////
/// @method EastTechCustomController::EastTechCustomController
///
/// Constructor.
///
////////////////////////////////////////////////////////////////
EastTechCustomController::EastTechCustomController(EastTech::Controller::Config::Models controllerModel, int controllerPort)
: GenericHID(controllerPort)
, CONTROLLER_MODEL(controllerModel)
, CONTROLLER_MAPPINGS(GetControllerMapping(controllerModel))
, m_ThrottleValue(1.0)
{
    ASSERT(CONTROLLER_MAPPINGS != nullptr);
}



////////////////////////////////////////////////////////////////
/// @method EastTechCustomController::GetDriveX
///
/// Returns x-axis drive input.
///
////////////////////////////////////////////////////////////////
double EastTechCustomController::GetDriveX() const
{
    double xAxisValue = GenericHID::GetRawAxis(CONTROLLER_MAPPINGS->AXIS_MAPPINGS.LEFT_X_AXIS);
 
    // x-axis controls are usually very sensitive, so scale them back   
    xAxisValue *= X_AXIS_DRIVE_SENSITIVITY_SCALING;

    return xAxisValue;
}



////////////////////////////////////////////////////////////////
/// @method EastTechCustomController::GetDriveY
///
/// Returns y-axis drive input.
///
////////////////////////////////////////////////////////////////
double EastTechCustomController::GetDriveY() const
{
    double yAxisValue = 0.0;

    switch (EastTech::Drive::Config::DRIVE_STYLE)
    {
        case EastTech::Drive::Config::DriveStyle::ARCADE_DRIVE:
        {
            yAxisValue = GenericHID::GetRawAxis(CONTROLLER_MAPPINGS->AXIS_MAPPINGS.LEFT_Y_AXIS);
            break;
        }
        case EastTech::Drive::Config::DriveStyle::GTA_DRIVE:
        {
            // In order to keep the drive logic the same across
            // all controller models, full forward is represented
            // by -1 and full reverse is represented by +1.
            //   -1
            //    |
            // -1---+1
            //    |
            //   +1
            
            // Left trigger is the 'reverse' value input.
            double leftTriggerValue = GetRawAxis(CONTROLLER_MAPPINGS->AXIS_MAPPINGS.LEFT_TRIGGER);
            
            // Right trigger is the 'forward' value input.
            double rightTriggerValue = GetRawAxis(CONTROLLER_MAPPINGS->AXIS_MAPPINGS.RIGHT_TRIGGER);

            if (RobotUtils::DEBUG_PRINTS)
            {
                SmartDashboard::PutNumber("Raw left trigger", leftTriggerValue);
                SmartDashboard::PutNumber("Raw right trigger", rightTriggerValue);
            }

            // Normalize (controller specific code).
            // After this, left will be 0->+1, right will be -1->0.
            NormalizeTriggers(leftTriggerValue, rightTriggerValue);

            if (RobotUtils::DEBUG_PRINTS)
            {
                SmartDashboard::PutNumber("Normalized left trigger", leftTriggerValue);
                SmartDashboard::PutNumber("Normalized right trigger", rightTriggerValue);
            }
            
            // Hopefully only one trigger is being pushed, but in
            // case both are being pressed, the value will be combined.
            yAxisValue =  leftTriggerValue + rightTriggerValue;
            yAxisValue *= Y_AXIS_DRIVE_SENSITIVITY_SCALING;

            break;
        }
        case EastTech::Drive::Config::DriveStyle::TANK_DRIVE:
        default:
        {
            ASSERT(false);
            break;
        }
    }

    return yAxisValue;
}



////////////////////////////////////////////////////////////////
/// @method EastTechCustomController::GetDriveRotate
///
/// Returns the rotate drive input.
///
////////////////////////////////////////////////////////////////
double EastTechCustomController::GetDriveRotate() const
{
    return GetRawAxis(CONTROLLER_MAPPINGS->AXIS_MAPPINGS.RIGHT_X_AXIS);
}



////////////////////////////////////////////////////////////////
/// @method EastTechCustomController::GetThrottle
///
/// Returns throttle control.  Most controllers do not have an
/// axis that retains its position when not being manipulated by
/// the user.  This requires throttle control to be implemented
/// and remembered in software.
///
////////////////////////////////////////////////////////////////
double EastTechCustomController::GetThrottle() const
{
    // Not implemented yet, just return the default value
    return m_ThrottleValue;

    // Get throttle control for generic Joystick implementation (e.g.
    // Logitech Extreme).  The z axis goes from -1 to 1, so it needs
    // to be normalized.  Subtract one and negate to make it zero
    // based to give a value between zero and two.  Divide by two to
    // get a value between 0 and 1.
    //
    // ((pJoystick->GetThrottle() - 1.0) / -2.0)
}
