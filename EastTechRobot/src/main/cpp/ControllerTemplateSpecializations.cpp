////////////////////////////////////////////////////////////////////////////////
/// @file   ControllerTemplateSpecializations.cpp
/// @author David Stalter
///
/// @details
/// Implements the specializations for the YtaController and YtaDriveController
/// template classes.
///
/// Copyright (c) 2023 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
// <none>

// C INCLUDES
// (none)

// C++ INCLUDES
#include "YtaController.hpp"                    // for class declarations

// STATIC MEMBER DATA
// (none)


////////////////////////////////////////////////////////////////
/// YtaCustomController template specializations for both
/// YtaController<> and YtaDriveController<>.
///
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
/// @method YtaController<YtaCustomController>::YtaController
///
/// Constructor for a template instantiated with a custom YTA
/// controller type.
///
////////////////////////////////////////////////////////////////
template <>
YtaController<YtaCustomController>::YtaController(Yta::Controller::Config::Models controllerModel, int controllerPort) :
    m_pController(new YtaCustomController(controllerModel, controllerPort)),
    m_ControllerModel(controllerModel),
    m_ButtonStateChanges()
{
}

////////////////////////////////////////////////////////////////
/// @method YtaController<YtaCustomController>::GetThrottleControl
///
/// Retrieves a throttle value for the controller.  Specialized
/// because some built-in types have an available fixed
/// position axis that can provide throttle (such as a z-axis).
/// This function just needs to pass through for a custom YTA
/// controller.
///
////////////////////////////////////////////////////////////////
template <>
double YtaController<YtaCustomController>::GetThrottleControl()
{
    return m_pController->GetThrottle();
}

////////////////////////////////////////////////////////////////
/// @method YtaDriveController<YtaCustomController>::GetDriveXInput
///
/// Retrieves the x-axis drive value from controller inputs.
/// Specialized to provide video game style drive controls
/// instead of using a single axis.
///
////////////////////////////////////////////////////////////////
template <>
double YtaDriveController<YtaCustomController>::GetDriveXInput()
{
    return m_pController->GetDriveX();
}

////////////////////////////////////////////////////////////////
/// @method YtaDriveController<YtaCustomController>::GetDriveYInput
///
/// Retrieves the y-axis drive value from controller inputs.
/// Specialized to provide video game style drive controls
/// instead of using a single axis.
///
////////////////////////////////////////////////////////////////
template <>
double YtaDriveController<YtaCustomController>::GetDriveYInput()
{
    return m_pController->GetDriveY();
}

////////////////////////////////////////////////////////////////
/// @method YtaDriveController<YtaCustomController>::GetDriveRotateInput
///
/// Retrieves the drive rotate axis value from controller inputs.
///
////////////////////////////////////////////////////////////////
template <>
double YtaDriveController<YtaCustomController>::GetDriveRotateInput()
{
    return m_pController->GetDriveRotate();
}
