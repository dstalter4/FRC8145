////////////////////////////////////////////////////////////////////////////////
/// @file   ControllerConfigurations.hpp
/// @author David Stalter
///
/// @details
/// Declarations describing a controller configuration, such as axis and button
/// mappings.
///
///
/// Copyright (c) 2022 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

#ifndef CONTROLLERCONFIGURATION_HPP
#define CONTROLLERCONFIGURATION_HPP

// SYSTEM INCLUDES
// <none>

// C INCLUDES
// (none)

// C++ INCLUDES
// (none)


////////////////////////////////////////////////////////////////
/// @namespace Yta::Controller::Config
///
/// Provides generic declarations for information on YTA
/// controller configuration.
///
////////////////////////////////////////////////////////////////
namespace Yta
{
namespace Controller
{
namespace Config
{
    // Represents the possible controller model options
    enum Models
    {
        BUILT_IN_PS4,
        BUILT_IN_XBOX,
        CUSTOM_LOGITECH,
        CUSTOM_LOGITECH_EXTREME,
        CUSTOM_PLAY_STATION,
        CUSTOM_XBOX
    };

    ////////////////////////////////////////////////////////////////
    /// @struct ControllerMappings
    ///
    /// Represents a mapping of controller inputs to generic names.
    /// This allows robot code to use this structure to map inputs
    /// instead of controller specific terminology.
    ////////////////////////////////////////////////////////////////
    struct Mappings
    {    
        // Structure representing the mapping of axes
        struct AxisMappings
        {
            const unsigned LEFT_X_AXIS;
            const unsigned LEFT_Y_AXIS;
            const unsigned LEFT_TRIGGER;
            const unsigned RIGHT_X_AXIS;
            const unsigned RIGHT_Y_AXIS;
            const unsigned RIGHT_TRIGGER;
        };

        // Structure representing the mapping of buttons
        struct ButtonMappings
        {
            const unsigned NO_BUTTON;
            const unsigned UP_BUTTON;
            const unsigned DOWN_BUTTON;
            const unsigned LEFT_BUTTON;
            const unsigned RIGHT_BUTTON;
            const unsigned SELECT;
            const unsigned START;
            const unsigned LEFT_BUMPER;
            const unsigned RIGHT_BUMPER;
            const unsigned LEFT_STICK_CLICK;
            const unsigned RIGHT_STICK_CLICK;
        };

        // Mappings for a controller (axes and buttons)
        const AxisMappings AXIS_MAPPINGS;
        const ButtonMappings BUTTON_MAPPINGS;
    };


    ////////////////////////////////////////////////////////////////
    /// Logitech controller mappings.
    ////////////////////////////////////////////////////////////////
    namespace Logitech
    {
        enum RawAxes
        {
            LEFT_X_AXIS         = 0,
            LEFT_Y_AXIS         = 1,
            LT                  = 2,
            RT                  = 3,
            RIGHT_X_AXIS        = 4,
            RIGHT_Y_AXIS        = 5
        };
        
        enum RawButtons
        {
            NO_BUTTON           = 0,
            A                   = 1,
            B                   = 2,
            X                   = 3,
            Y                   = 4,
            LB                  = 5,
            RB                  = 6,
            SELECT              = 7,
            START               = 8,
            LEFT_STICK_CLICK    = 9,
            RIGHT_STICK_CLICK   = 10
        };
    }


    ////////////////////////////////////////////////////////////////
    /// PlayStation controller mappings.
    ////////////////////////////////////////////////////////////////
    namespace PlayStation
    {
        enum RawAxes
        {
            LEFT_X_AXIS         = 0,
            LEFT_Y_AXIS         = 1,
            RIGHT_X_AXIS        = 2,
            L2                  = 3,
            R2                  = 4,
            RIGHT_Y_AXIS        = 5
        };
        
        enum RawButtons
        {
            NO_BUTTON           = 0,
            SQUARE              = 1,
            X                   = 2,
            CIRCLE              = 3,
            TRIANGLE            = 4,
            L1                  = 5,
            R1                  = 6,
            SHARE               = 9,
            OPTIONS             = 10,
            LEFT_STICK_CLICK    = 11,
            RIGHT_STICK_CLICK   = 12
        };
    }

        
    ////////////////////////////////////////////////////////////////
    /// Xbox controller mappings.
    ////////////////////////////////////////////////////////////////
    namespace Xbox
    {
        enum RawAxes
        {
            LEFT_X_AXIS         = 0,
            LEFT_Y_AXIS         = 1,
            LT                  = 2,
            RT                  = 3,
            RIGHT_X_AXIS        = 4,
            RIGHT_Y_AXIS        = 5
        };
        
        enum RawButtons
        {
            NO_BUTTON           = 0,
            A                   = 1,
            B                   = 2,
            X                   = 3,
            Y                   = 4,
            LB                  = 5,
            RB                  = 6,
            BACK                = 7,
            START               = 8,
            LEFT_STICK_CLICK    = 9,
            RIGHT_STICK_CLICK   = 10
        };
    }


    ////////////////////////////////////////////////////////////////
    /// Constant expressions mapping the controller styles.
    ////////////////////////////////////////////////////////////////
    static constexpr const Mappings LOGITECH_CONTROLLER_MAPPINGS =
    {
        {
            Logitech::RawAxes::LEFT_X_AXIS,
            Logitech::RawAxes::LEFT_Y_AXIS,
            Logitech::RawAxes::LT,
            Logitech::RawAxes::RIGHT_X_AXIS,
            Logitech::RawAxes::RIGHT_Y_AXIS,
            Logitech::RawAxes::RT
        },
        {
            Logitech::RawButtons::NO_BUTTON,
            Logitech::RawButtons::Y,
            Logitech::RawButtons::A,
            Logitech::RawButtons::X,
            Logitech::RawButtons::B,
            Logitech::RawButtons::SELECT,
            Logitech::RawButtons::START,
            Logitech::RawButtons::LB,
            Logitech::RawButtons::RB,
            Logitech::RawButtons::LEFT_STICK_CLICK,
            Logitech::RawButtons::RIGHT_STICK_CLICK
        }
    };

    // Constant expression mapping the PlayStation controller axes/buttons
    static constexpr const Mappings PLAY_STATION_CONTROLLER_MAPPINGS =
    {
        {
            PlayStation::RawAxes::LEFT_X_AXIS,
            PlayStation::RawAxes::LEFT_Y_AXIS,
            PlayStation::RawAxes::L2,
            PlayStation::RawAxes::RIGHT_X_AXIS,
            PlayStation::RawAxes::RIGHT_Y_AXIS,
            PlayStation::RawAxes::R2
        },
        {
            PlayStation::RawButtons::NO_BUTTON,
            PlayStation::RawButtons::TRIANGLE,
            PlayStation::RawButtons::X,
            PlayStation::RawButtons::SQUARE,
            PlayStation::RawButtons::CIRCLE,
            PlayStation::RawButtons::SHARE,
            PlayStation::RawButtons::OPTIONS,
            PlayStation::RawButtons::L1,
            PlayStation::RawButtons::R1,
            PlayStation::RawButtons::LEFT_STICK_CLICK,
            PlayStation::RawButtons::RIGHT_STICK_CLICK
        }
    };

    // Constant expression mapping the Xbox controller axes/buttons
    static constexpr const Mappings XBOX_CONTROLLER_MAPPINGS =
    {
        {
            Xbox::RawAxes::LEFT_X_AXIS,
            Xbox::RawAxes::LEFT_Y_AXIS,
            Xbox::RawAxes::LT,
            Xbox::RawAxes::RIGHT_X_AXIS,
            Xbox::RawAxes::RIGHT_Y_AXIS,
            Xbox::RawAxes::RT
        },
        {
            Xbox::RawButtons::NO_BUTTON,
            Xbox::RawButtons::Y,
            Xbox::RawButtons::A,
            Xbox::RawButtons::X,
            Xbox::RawButtons::B,
            Xbox::RawButtons::BACK,
            Xbox::RawButtons::START,
            Xbox::RawButtons::LB,
            Xbox::RawButtons::RB,
            Xbox::RawButtons::LEFT_STICK_CLICK,
            Xbox::RawButtons::RIGHT_STICK_CLICK
        }
    };
    
    // Constant expression function to retrieve the mapping for a controller at compile time
    static constexpr const Mappings * GetControllerMapping(Models controllerModel)
    {
        switch (controllerModel)
        {
            case CUSTOM_LOGITECH:
            {
                return &LOGITECH_CONTROLLER_MAPPINGS;
                break;
            }
            case BUILT_IN_PS4:
            case CUSTOM_PLAY_STATION:
            {
                return &PLAY_STATION_CONTROLLER_MAPPINGS;
                break;
            }
            case BUILT_IN_XBOX:
            case CUSTOM_XBOX:
            {
                return &XBOX_CONTROLLER_MAPPINGS;
            }
            default:
            {
                return nullptr;
                break;
            }
        }
    }
}
}
}

#endif // CONTROLLERCONFIGURATION_HPP
