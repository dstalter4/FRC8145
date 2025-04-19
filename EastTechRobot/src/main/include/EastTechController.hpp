////////////////////////////////////////////////////////////////////////////////
/// @file   EastTechController.hpp
/// @author David Stalter
///
/// @details
/// A class designed to interface to several controller types, such as a custom
/// East Tech controller implementation or the built-in FRC types.
///
///
/// Copyright (c) 2025 East Technical High School
////////////////////////////////////////////////////////////////////////////////

#ifndef EASTTECHCONTROLLER_HPP
#define EASTTECHCONTROLLER_HPP

// SYSTEM INCLUDES
// <none>

// C INCLUDES
#include "frc/GenericHID.h"             // for base class declaration
#include "frc/Joystick.h"               // for interacting with joysticks
#include "frc/PS4Controller.h"          // for creating built-in PS4 controller objects
#include "frc/XboxController.h"         // for creating built-in XBox controller objects

// C++ INCLUDES
#include "ControllerConfiguration.hpp"  // for the controller axis/button mappings
#include "EastTechCustomController.hpp" // for creating custom East Tech controllers

using namespace frc;


////////////////////////////////////////////////////////////////
/// @namespace EastTech::Controller
///
/// Provides generic declarations for East Tech controller
/// related functionality.
///
////////////////////////////////////////////////////////////////
namespace EastTech
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
/// @class EastTechController<ControllerType>
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
/// East Tech controllers have a different constructor signature
/// than the GenericHID derived objects.  It is specialized to
/// provide the necessary custom functionality.
///
/// Examples of valid types to instantiate this class with:
/// Joystick, PS4Controller, XboxController, EastTechCustomController
///
////////////////////////////////////////////////////////////////
template <class ControllerType>
class EastTechController
{
public:
    // Constructor, which is specialized for some ControllerTypes
    EastTechController(EastTech::Controller::Config::Models controllerModel, int controllerPort);

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
    // Methods to compute East Tech specific parameters (may be specialized)
    ////////////////////////////////////////////////////////////////

    double GetThrottleControl();

    ////////////////////////////////////////////////////////////////
    /// @method EastTechController<ControllerType>::Rumble
    ///
    /// Turns on the controller's rumble feature.
    ///
    ////////////////////////////////////////////////////////////////
    void Rumble(EastTech::Controller::RumbleLocation location)
    {
        switch (location)
        {
            case EastTech::Controller::RumbleLocation::RUMBLE_LEFT:
            {
                m_pController->SetRumble(GenericHID::RumbleType::kLeftRumble);
                break;
            }
            case EastTech::Controller::RumbleLocation::RUMBLE_RIGHT:
            {
                m_pController->SetRumble(GenericHID::RumbleType::kRightRumble);
                break;
            }
            case EastTech::Controller::RumbleLocation::RUMBLE_BOTH:
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
    /// @method EastTechController<ControllerType>::GetPovAsDirection
    ///
    /// Retrieves the POV value as a more easily usable enum value
    /// that represents a direction.
    ///
    /// Note: This function is closely coupled to the PovDirections
    /// enum.  Use caution when modifying!
    ///
    ////////////////////////////////////////////////////////////////
    inline EastTech::Controller::PovDirections GetPovAsDirection()
    {
        const int POV_NORMALIZATION_ANGLE = 45;
        const int ANGLE_90_DEGREES = 90;
        const int ANGLE_360_DEGREES = 360;

        EastTech::Controller::PovDirections povDirection = EastTech::Controller::PovDirections::POV_NOT_PRESSED;

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
            povDirection = static_cast<EastTech::Controller::PovDirections>(povValue / ANGLE_90_DEGREES);
        }

        return povDirection;
    }

    ////////////////////////////////////////////////////////////////
    /// @method YtaController<ControllerType>::DetectPovChange
    ///
    /// This method is used to check if the POV input has undergone
    /// a state change.  Some robot program my logic may only want
    /// to detect POV edge transitions.  If a POV input is held down
    /// this may result in incorrect operation.  This logic will
    /// monitor the POV state and track any changes to it.  If a
    /// state change matches the requested parameter, it will report
    /// the edge transition once (and only once).  This logic
    /// assumes that the robot program only use a POV input for one
    /// purpose (which should always be the case).
    ///
    ////////////////////////////////////////////////////////////////
    inline bool DetectPovChange(EastTech::Controller::PovDirections povDirection)
    {
        bool bPressed = false;
        static bool m_LastPovChangeReported = false;
        EastTech::Controller::PovDirections currentPovDirection = GetPovAsDirection();
        if (currentPovDirection != m_LastPovDirection)
        {
            m_LastPovChangeReported = false;
            m_LastPovDirection = currentPovDirection;
        }
        if ((!m_LastPovChangeReported) && (povDirection == m_LastPovDirection))
        {
            bPressed = true;
            m_LastPovChangeReported = true;
        }
        return bPressed;
    }

    ////////////////////////////////////////////////////////////////
    /// @method EastTechController<ControllerType>::DetectButtonChange
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
    inline bool DetectButtonChange(int buttonNumber, EastTech::Controller::ButtonStateChanges::Transitions transition = EastTech::Controller::ButtonStateChanges::BUTTON_PRESSED)
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
            if ((transition == EastTech::Controller::ButtonStateChanges::BUTTON_PRESSED) && (currentMaskedBit != 0U))
            {
                bTriggerChanged = true;
            }
            else if ((transition == EastTech::Controller::ButtonStateChanges::BUTTON_RELEASED) && (currentMaskedBit == 0U))
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
    EastTech::Controller::Config::Models m_ControllerModel;

private:
    // Tracks the state of the buttons (pressed/released)
    EastTech::Controller::ButtonStateChanges m_ButtonStateChanges;
    EastTech::Controller::PovDirections m_LastPovDirection;
    bool m_LastPovChangeReported;

    // Prevent copying/assignment
    EastTechController(const EastTechController&) = delete;
    EastTechController& operator=(const EastTechController&) = delete;
};


////////////////////////////////////////////////////////////////
/// @class EastTechDriveController<DriveControllerType>
///
/// Template class specifically for a drive controller.  It
/// derives from the previously declared EastTechController template
/// class and behaves the same way, other than to provide some
/// additional methods that are specific to drive controllers
/// only (such as getting drive inputs as complex combinations
/// instead of single axes).  The methods here are most likely
/// to require specialization for custom behavior.
///
////////////////////////////////////////////////////////////////
template <class DriveControllerType>
class EastTechDriveController : public EastTechController<DriveControllerType>
{
public:
    EastTechDriveController(EastTech::Controller::Config::Models controllerModel, int controllerPort) : EastTechController<DriveControllerType>(controllerModel, controllerPort)
    {
    }

    // Get drive x/y axis inputs
    double GetDriveXInput();
    double GetDriveYInput();

    // For swerve drive, gets the rotate input
    double GetDriveRotateInput();

private:
    // Prevent copying/assignment
    EastTechDriveController(const EastTechDriveController&) = delete;
    EastTechDriveController& operator=(const EastTechDriveController&) = delete;
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

// Specialization of the constructor for EastTechCustomController type
template <>
EastTechController<EastTechCustomController>::EastTechController(EastTech::Controller::Config::Models controllerModel, int controllerPort);

// Specialization of GetThrottleControl() for EastTechCustomController type
template <>
double EastTechController<EastTechCustomController>::GetThrottleControl();

// Specialization of GetDriveXInput() for EastTechCustomController type
template <>
double EastTechDriveController<EastTechCustomController>::GetDriveXInput();

// Specialization of GetDriveYInput() for EastTechCustomController type
template <>
double EastTechDriveController<EastTechCustomController>::GetDriveYInput();

// Specialization of GetDriveRotateInput() for EastTechCustomController type
template <>
double EastTechDriveController<EastTechCustomController>::GetDriveRotateInput();


////////////////////////////////////////////////////////////////
/// Non-specialized template definitions.
///
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
/// @method EastTechController<ControllerType>::EastTechCustomController
///
/// Constructor.  Instantiates the actual controller object that
/// receives input.
///
////////////////////////////////////////////////////////////////
template <class ControllerType>
EastTechController<ControllerType>::EastTechController(EastTech::Controller::Config::Models controllerModel, int controllerPort) :
    m_pController(new ControllerType(controllerPort)),
    m_ControllerModel(controllerModel),
    m_ButtonStateChanges(),
    m_LastPovDirection(EastTech::Controller::PovDirections::POV_NOT_PRESSED),
    m_LastPovChangeReported(false)
{
}

#endif // EASTTECHCONTROLLER_HPP
