////////////////////////////////////////////////////////////////////////////////
/// @file   YtaController.hpp
/// @author David Stalter
///
/// @details
/// A class designed to interface to several controller types, such as a custom
/// YTA controller implementation or the built-in FRC types.
///
///
/// Copyright (c) 2023 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

#ifndef YTACONTROLLER_HPP
#define YTACONTROLLER_HPP

// SYSTEM INCLUDES
// <none>

// C INCLUDES
#include "frc/GenericHID.h"             // for base class declaration
#include "frc/Joystick.h"               // for interacting with joysticks
#include "frc/PS4Controller.h"          // for creating built-in PS4 controller objects
#include "frc/XboxController.h"         // for creating built-in XBox controller objects

// C++ INCLUDES
#include "ControllerConfiguration.hpp"  // for the controller axis/button mappings
#include "YtaCustomController.hpp"      // for creating custom YTA controllers

using namespace frc;


////////////////////////////////////////////////////////////////
/// @namespace Yta::Controller
///
/// Provides generic declarations for YTA controller related
/// functionality.
///
////////////////////////////////////////////////////////////////
namespace Yta
{
namespace Controller
{
    // For enabling rumble on a controller
    enum RumbleLocation
    {
        RUMBLE_LEFT,
        RUMBLE_RIGHT,
        RUMBLE_BOTH
    };
    
    // These values are deliberately selected to simplify the
    // logic and math in routines dependent on them.
    enum PovDirections
    {
        POV_UP          = 0,
        POV_RIGHT       = 1,
        POV_DOWN        = 2,
        POV_LEFT        = 3,
        POV_NOT_PRESSED = 0xF
    };

    // Used to detect button state changes (such as released to
    // pressed or vice-versa.  Each button is tracked as an
    // individual bit in the uint32_t members.  This works
    // because controllers have fewer than 32 available buttons.
    struct ButtonStateChanges
    {
        enum Transitions
        {
            BUTTON_PRESSED,
            BUTTON_RELEASED
        };

        ButtonStateChanges() :
            m_CurrentValues(0U),
            m_PreviousValues(0U)
        {
        }

        uint32_t m_CurrentValues;
        uint32_t m_PreviousValues;
    };
}
}


////////////////////////////////////////////////////////////////
/// @class YtaController<ControllerType>
///
/// Template class that provides methods for interacting with a
/// generic controller.  The theory behind this class is to
/// enable easy switching of controller models from the primary
/// robot code.  There are many different types of controller
/// objects, most of which derive from GenericHID.  The derived
/// classes may not implement all of the base class methods or
/// can have custom ways of retrieving controller inputs (e.g.
/// GetSquareButton() for a PS4 controller).  To preserve a
/// common interface to robot code, controller objects are
/// created through this template class, picking the correct
/// model to instantiate.  The controller model object is
/// created, stored and maintained through this class.  A
/// common API is provided to access all inputs from the
/// controllers.  These APIs are kept common if possible (by
/// calling GenericHID base methods).  If non-common behavior
/// is needed, the template method can be specialized.  The
/// constructor for this class is an example of that.  Custom
/// YTA controllers have a different constructor signature
/// than the GenericHID derived objects.  It is specialized to
/// provide the necessary custom functionality.
///
/// Examples of valid types to instantiate this class with:
/// Joystick, PS4Controller, XboxController, YtaCustomController
///
////////////////////////////////////////////////////////////////
template <class ControllerType>
class YtaController
{
public:
    // Constructor, which is specialized for some ControllerTypes
    YtaController(Yta::Controller::Config::Models controllerModel, int controllerPort);

    ////////////////////////////////////////////////////////////////
    // Methods to get input from the controller (not currently specialized anywhere)
    ////////////////////////////////////////////////////////////////

    double GetAxisValue(int axis)
    {
        return m_pController->GetRawAxis(axis);
    }

    bool GetButtonState(int button)
    {
        return m_pController->GetRawButton(button);
    }

    int GetPovValue()
    {
        return m_pController->GetPOV();
    }

    ////////////////////////////////////////////////////////////////
    // Methods to compute YTA specific parameters (may be specialized)
    ////////////////////////////////////////////////////////////////

    double GetThrottleControl();

    ////////////////////////////////////////////////////////////////
    /// @method YtaController<ControllerType>::Rumble
    ///
    /// Turns on the controller's rumble feature.
    ///
    ////////////////////////////////////////////////////////////////
    void Rumble(Yta::Controller::RumbleLocation location)
    {
        switch (location)
        {
            case Yta::Controller::RumbleLocation::RUMBLE_LEFT:
            {
                m_pController->SetRumble(GenericHID::RumbleType::kLeftRumble);
                break;
            }
            case Yta::Controller::RumbleLocation::RUMBLE_RIGHT:
            {
                m_pController->SetRumble(GenericHID::RumbleType::kRightRumble);
                break;
            }
            case Yta::Controller::RumbleLocation::RUMBLE_BOTH:
            {
                m_pController->SetRumble(GenericHID::RumbleType::kLeftRumble);
                m_pController->SetRumble(GenericHID::RumbleType::kRightRumble);
                break;
            }
            default:
            {
                break;
            }
        }
    }

    ////////////////////////////////////////////////////////////////
    /// @method YtaController<ControllerType>::GetPovAsDirection
    ///
    /// Retrieves the POV value as a more easily usable enum value
    /// that represents a direction.
    ///
    /// Note: This function is closely coupled to the PovDirections
    /// enum.  Use caution when modifying!
    ///
    ////////////////////////////////////////////////////////////////
    inline Yta::Controller::PovDirections GetPovAsDirection()
    {
        const int POV_NORMALIZATION_ANGLE = 45;
        const int ANGLE_90_DEGREES = 90;
        const int ANGLE_360_DEGREES = 360;

        Yta::Controller::PovDirections povDirection = Yta::Controller::PovDirections::POV_NOT_PRESSED;

        int povValue = GetPovValue();
        
        if (povValue != -1)
        {
            // This gives a value between 45 -> 405
            povValue += POV_NORMALIZATION_ANGLE;
            
            // Normalize between 0 -> 360 (maps controller 0:360 in to 45:360:0:45 out)
            povValue %= ANGLE_360_DEGREES;
            
            // The zero point is now at compass heading 315, where:
            // 0 -> 89 = up
            // 90 -> 179 = right
            // 180 -> 269 = down
            // 270 -> 359 = left
            // Use integer division to get a single value that represents the
            // entire range, which can then be directly converted to the enum type.
            // This cast is risky, but the enum was deliberately crafted to support it.
            povDirection = static_cast<Yta::Controller::PovDirections>(povValue / ANGLE_90_DEGREES);
        }

        return povDirection;
    }

    ////////////////////////////////////////////////////////////////
    /// @method YtaController<ControllerType>::DetectButtonChange
    ///
    /// This method is used to check if a button has undergone a
    /// state change.  The same button can be used to reverse state
    /// of a particular part of the robot (such as a motor or
    /// solenoid).  If the state is reversed inside an 'if'
    /// statement that is part of a loop, the final state will be
    /// whatever transition just occurred, which could be back to
    /// the same state started in.  The intended use case is to
    /// have robot code call this function periodically through a
    /// controller object as the condition of an 'if' statement.
    /// It will read a current value which will then be checked
    /// against the last input value read.  A value of 'true' will
    /// be returned if an appropriate edge change is detected.
    /// is detected (press or release).
    ///
    ////////////////////////////////////////////////////////////////
    inline bool DetectButtonChange(int buttonNumber, Yta::Controller::ButtonStateChanges::Transitions transition = Yta::Controller::ButtonStateChanges::BUTTON_PRESSED)
    {   
        // Create the mask to the bit position for this button
        const uint32_t BUTTON_BIT_POSITION_MASK = 1U << buttonNumber;

        // First read the latest value from the joystick
        if (m_pController->GetRawButton(buttonNumber))
        {
            // If it's a '1', it must be or'ed in
            m_ButtonStateChanges.m_CurrentValues |= BUTTON_BIT_POSITION_MASK;
        }
        else
        {
            // If it's a '0', it must be and'ed in
            m_ButtonStateChanges.m_CurrentValues &= ~BUTTON_BIT_POSITION_MASK;
        }
        
        // Generates some values where only the bit of interest is preserved, in place (not shifted)
        const uint32_t currentMaskedBit =  m_ButtonStateChanges.m_CurrentValues & BUTTON_BIT_POSITION_MASK;
        const uint32_t previousMaskedBit = m_ButtonStateChanges.m_PreviousValues & BUTTON_BIT_POSITION_MASK;

        bool bTriggerChanged = false;

        // Only report a change if the current value is different than the old value
        if ((currentMaskedBit ^ previousMaskedBit) != 0U)
        {
            // Also make sure the transition is to the correct edge
            if ((transition == Yta::Controller::ButtonStateChanges::BUTTON_PRESSED) && (currentMaskedBit != 0U))
            {
                bTriggerChanged = true;
            }
            else if ((transition == Yta::Controller::ButtonStateChanges::BUTTON_RELEASED) && (currentMaskedBit == 0U))
            {
                bTriggerChanged = true;
            }
            else
            {
            }
        }
        
        // Always update the old value
        m_ButtonStateChanges.m_PreviousValues = m_ButtonStateChanges.m_CurrentValues;
        
        return bTriggerChanged;
    }

protected:
    // This is the actual controller object which will retrieve all input
    ControllerType * m_pController;

    // The model of controller being instantiated
    Yta::Controller::Config::Models m_ControllerModel;

private:
    // Tracks the state of the buttons (pressed/released)
    Yta::Controller::ButtonStateChanges m_ButtonStateChanges;

    // Prevent copying/assignment
    YtaController(const YtaController&) = delete;
    YtaController& operator=(const YtaController&) = delete;
};


////////////////////////////////////////////////////////////////
/// @class YtaDriveController<DriveControllerType>
///
/// Template class specifically for a drive controller.  It
/// derives from the previously declared YtaController template
/// class and behaves the same way, other than to provide some
/// additional methods that are specific to drive controllers
/// only (such as getting drive inputs as complex combinations
/// instead of single axes).  The methods here are most likely
/// to require specialization for custom behavior.
///
////////////////////////////////////////////////////////////////
template <class DriveControllerType>
class YtaDriveController : public YtaController<DriveControllerType>
{
public:
    YtaDriveController(Yta::Controller::Config::Models controllerModel, int controllerPort) : YtaController<DriveControllerType>(controllerModel, controllerPort)
    {
    }

    // Get drive x/y axis inputs
    double GetDriveXInput();
    double GetDriveYInput();

    // For swerve drive, gets the rotate input
    double GetDriveRotateInput();

private:
    // Prevent copying/assignment
    YtaDriveController(const YtaDriveController&) = delete;
    YtaDriveController& operator=(const YtaDriveController&) = delete;
};


////////////////////////////////////////////////////////////////
/// Template specialization declarations.
///
/// These specializations indicate which controllers will be
/// implementing custom functionality.  They must be declared
/// before the first time they would be required to be
/// instantiated in the code.  They must also precede the
/// generic template definitions, otherwise those would be used
/// by the compiler instead.  This is why some template bodies
/// appear *after* these declarations.
///
/// See ControllerTemplateSpecializations.cpp for the bodies of
/// these functions.
///
////////////////////////////////////////////////////////////////

// Specialization of the constructor for YtaCustomController type
template <>
YtaController<YtaCustomController>::YtaController(Yta::Controller::Config::Models controllerModel, int controllerPort);

// Specialization of GetThrottleControl() for YtaCustomController type
template <>
double YtaController<YtaCustomController>::GetThrottleControl();

// Specialization of GetDriveXInput() for YtaCustomController type
template <>
double YtaDriveController<YtaCustomController>::GetDriveXInput();

// Specialization of GetDriveYInput() for YtaCustomController type
template <>
double YtaDriveController<YtaCustomController>::GetDriveYInput();

// Specialization of GetDriveRotateInput() for YtaCustomController type
template <>
double YtaDriveController<YtaCustomController>::GetDriveRotateInput();


////////////////////////////////////////////////////////////////
/// Non-specialized template definitions.
///
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
/// @method YtaController<ControllerType>::YtaCustomController
///
/// Constructor.  Instantiates the actual controller object that
/// receives input.
///
////////////////////////////////////////////////////////////////
template <class ControllerType>
YtaController<ControllerType>::YtaController(Yta::Controller::Config::Models controllerModel, int controllerPort) :
    m_pController(new ControllerType(controllerPort)),
    m_ControllerModel(controllerModel),
    m_ButtonStateChanges()
{
}

#endif // YTACONTROLLER_HPP
