////////////////////////////////////////////////////////////////////////////////
/// @file   ControllerTemplateSpecializations.cpp
/// @author David Stalter
///
/// @details
/// Implements the specializations for the EastTechController and
/// EastTechDriveController template classes.
///
/// Copyright (c) 2024 East Technical High School
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
// <none>

// C INCLUDES
// (none)

// C++ INCLUDES
#include "EastTechController.hpp"                   // for class declarations

// STATIC MEMBER DATA
// (none)


////////////////////////////////////////////////////////////////
/// EastTechCustomController template specializations for both
/// EastTechController<> and EastTechDriveController<>.
///
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
/// @method EastTechController<EastTechCustomController>::EastTechController
///
/// Constructor for a template instantiated with a custom East
/// Tech controller type.
///
////////////////////////////////////////////////////////////////
template <>
EastTechController<EastTechCustomController>::EastTechController(EastTech::Controller::Config::Models controllerModel, int controllerPort) :
    m_pController(new EastTechCustomController(controllerModel, controllerPort)),
    m_ControllerModel(controllerModel),
    m_ButtonStateChanges()
{
}

////////////////////////////////////////////////////////////////
/// @method EastTechController<EastTechCustomController>::GetThrottleControl
///
/// Retrieves a throttle value for the controller.  Specialized
/// because some built-in types have an available fixed
/// position axis that can provide throttle (such as a z-axis).
/// This function just needs to pass through for a custom East
/// Tech controller.
///
////////////////////////////////////////////////////////////////
template <>
double EastTechController<EastTechCustomController>::GetThrottleControl()
{
    return m_pController->GetThrottle();
}

////////////////////////////////////////////////////////////////
/// @method EastTechDriveController<EastTechCustomController>::GetDriveXInput
///
/// Retrieves the x-axis drive value from controller inputs.
/// Specialized to provide video game style drive controls
/// instead of using a single axis.
///
////////////////////////////////////////////////////////////////
template <>
double EastTechDriveController<EastTechCustomController>::GetDriveXInput()
{
    return m_pController->GetDriveX();
}

////////////////////////////////////////////////////////////////
/// @method EastTechDriveController<EastTechCustomController>::GetDriveYInput
///
/// Retrieves the y-axis drive value from controller inputs.
/// Specialized to provide video game style drive controls
/// instead of using a single axis.
///
////////////////////////////////////////////////////////////////
template <>
double EastTechDriveController<EastTechCustomController>::GetDriveYInput()
{
    return m_pController->GetDriveY();
}

////////////////////////////////////////////////////////////////
/// @method EastTechDriveController<EastTechCustomController>::GetDriveRotateInput
///
/// Retrieves the drive rotate axis value from controller inputs.
///
////////////////////////////////////////////////////////////////
template <>
double EastTechDriveController<EastTechCustomController>::GetDriveRotateInput()
{
    return m_pController->GetDriveRotate();
}
