////////////////////////////////////////////////////////////////////////////////
/// @file   EastTechTalon.hpp
/// @author David Stalter
///
/// @details
/// Custom functionality for easier robot programming of CTRE Talon controllers.
///
/// Copyright (c) 2026 East Technical High School
////////////////////////////////////////////////////////////////////////////////

#ifndef EASTTECHTALON_HPP
#define EASTTECHTALON_HPP

// SYSTEM INCLUDES
#include <cstdio>                               // for std::snprintf

// C INCLUDES
#include "frc/smartdashboard/SmartDashboard.h"  // for interacting with the smart dashboard

// C++ INCLUDES
#include "RobotUtils.hpp"                       // for ConvertCelsiusToFahrenheit
#include "ctre/phoenix6/CANBus.hpp"             // for working with CANBus objects
#include "ctre/phoenix6/TalonFX.hpp"            // for CTRE TalonFX API

using namespace frc;
using namespace ctre::phoenix6;
using namespace ctre::phoenix6::configs;
using namespace ctre::phoenix6::controls;
using namespace ctre::phoenix6::hardware;
using namespace ctre::phoenix6::signals;


////////////////////////////////////////////////////////////////
/// @namespace EastTech::Talon
///
/// Namespace that contains declarations for interacting with
/// Talon speed controllers specific to East Tech.
///
////////////////////////////////////////////////////////////////
namespace EastTech::Talon
{
    // Represents how a motor will be controlled
    enum MotorGroupControlMode
    {
        LEADER,                 // First motor in a group
        FOLLOW,                 // Motor follows the leader
        FOLLOW_INVERSE,         // Motor follows the leader, but inverse
        INDEPENDENT,            // Motor needs to be set independently
        INVERSE,                // Motor is the inverse value of the leader
        INDEPENDENT_OFFSET,     // Motor is set independently, but with a different value from leader
        INVERSE_OFFSET,         // Motor is set independently, but with the a different inverse value from leader
        CUSTOM                  // Motor needs to be set later to an option above
    };

    // @todo: This could be templated if other Talon motor controllers are used (e.g. TalonFXS).
    // Represents a combination of objects to use with a TalonFX motor controller
    class TalonFxMotorController
    {
      public:
        // Constructor
        TalonFxMotorController(uint32_t canId, CANBus & rCanBus) :
            m_pTalonFx(new TalonFX(static_cast<int>(canId), rCanBus)),
            m_MotorConfiguration(),
            m_DutyCycleOut(0.0),
            m_PositionVoltage(0.0_tr),
            m_LastOutputValue(0.0)
        {
            m_pTalonFx->GetConfigurator().Apply(m_MotorConfiguration);
            m_pTalonFx->ClearStickyFaults();
        }

        // Retrieve a specific motor object
        TalonFX * GetMotorObject()
        {
            return m_pTalonFx;
        }

        // Retrieve the configuration for a specific motor object
        TalonFXConfiguration * GetMotorConfiguration()
        {
            return &m_MotorConfiguration;
        }

        // Applies the struct-local configuration, which may have been updated externally
        void ApplyConfiguration()
        {
            (void)m_pTalonFx->GetConfigurator().Apply(m_MotorConfiguration);
        }

        // Applies a configuration.  This will not update the struct-local
        // configuration and directly overwrites the configuration on the device.
        // Template type must match CTRE options.
        template <typename ConfigType>
        void ApplyConfiguration(ConfigType config)
        {
            // For now, don't let this be used
            ASSERT(false);
            (void)m_pTalonFx->GetConfigurator().Apply(config);
        }

        // Set the output using duty cycle
        void SetDutyCycle(double dutyCycle)
        {
            // To conserve CAN bus utilization, only send a value if it has changed
            if (dutyCycle != m_LastOutputValue)
            {
                (void)m_pTalonFx->SetControl(m_DutyCycleOut.WithOutput(dutyCycle));
                m_LastOutputValue = dutyCycle;
            }
        }

        // Set the output to hold a specified position
        void SetPositionVoltage(units::angle::degree_t angleToSetDegrees)
        {
            // To conserve CAN bus utilization, only send a value if it has changed
            if (angleToSetDegrees.value() != m_LastOutputValue)
            {
                units::angle::turn_t turns(angleToSetDegrees);
                (void)m_pTalonFx->SetControl(m_PositionVoltage.WithPosition(turns));
                m_LastOutputValue = angleToSetDegrees.value();
            }
        }

        // Set a tone to play on the motor
        void SetTone(MusicTone musicTone)
        {
            (void)m_pTalonFx->SetControl(musicTone);
        }

      private:
        // The Phoenix 6 API requires using different objects with SetControl()
        // function calls.  Create different possible objects so the main robot
        // code doesn't have to worry about it.
        TalonFX * m_pTalonFx;
        TalonFXConfiguration m_MotorConfiguration;
        DutyCycleOut m_DutyCycleOut;
        PositionVoltage m_PositionVoltage;
        double m_LastOutputValue;
    };



    // A structure that doesn't create real motor controller objects.
    // Intended to be used to keep multiple robot configuration
    // options available (i.e. interchange within TalonMotorGroup).
    class EmptyTalon
    {
      public:
        EmptyTalon(const char *, uint32_t, uint32_t, MotorGroupControlMode, NeutralModeValue, const CANBus &) {}

        // TalonMotorGroup stubs
        inline void SetDutyCycle(double) {}
        inline void SetDutyCycle(double, double) {}
        inline void DisplayStatusInformation() {}
        inline EmptyTalon * GetMotorObject(uint32_t) { return &m_EmptyTalonObj; }

        // CTRE stubs
        template <typename Type>
        inline void Apply(Type) {}
        inline void SetPosition(units::angle::turn_t) {}
        inline EmptyTalon & GetPosition() { return m_EmptyTalonObj; }
        inline EmptyTalon & GetConfigurator() { return m_EmptyTalonObj; }
        inline EmptyTalon & GetValue() { return m_EmptyTalonObj; }
        inline double value() { return 0.0; }

      private:
        // Singleton
        static EmptyTalon m_EmptyTalonObj;
    };
}





////////////////////////////////////////////////////////////////
/// Separation of single motor and group motor code.
////////////////////////////////////////////////////////////////





////////////////////////////////////////////////////////////////
/// @class TalonMotorGroup
///
/// Class that provides methods for interacting with a group of
/// Talon speed controllers.
///
/// @todo: Remove template.
///
////////////////////////////////////////////////////////////////
template <class TalonType, class TalonConfigurationType>
class TalonMotorGroup
{
public:

    typedef EastTech::Talon::MotorGroupControlMode MotorGroupControlMode;

    // Constructor
    TalonMotorGroup(
                     const char * pName,
                     uint32_t numMotors,
                     uint32_t leaderCanId,
                     MotorGroupControlMode nonLeaderControlMode,
                     NeutralModeValue neutralMode,
                     CANBus & rCanBus
                   );

    // Retrieve a specific motor object
    TalonType * GetMotorObject(uint32_t canId = GROUP_LEADER_PSEUDO_CAN_ID);

    // Retrieve the configuration for a specific motor object
    TalonConfigurationType * GetMotorConfiguration(uint32_t canId = GROUP_LEADER_PSEUDO_CAN_ID);

    // Applies the struct-local configuration, which may have been updated externally
    void ApplyConfiguration(uint32_t canId = APPLY_TO_ALL_PSEUDO_CAN_ID);

    // Applies a configuration.  This will not update the struct-local
    // configuration and directly overwrites the configuration on the device.
    // Template type must match CTRE options.
    template <typename ConfigType>
    void ApplyConfiguration(ConfigType config, uint32_t canId = APPLY_TO_ALL_PSEUDO_CAN_ID);

    // Function to set the speed of each motor in the group
    void SetDutyCycle(double value, double offset = 0.0);

    // Function to set the motor group output to hold specified angle
    void SetPositionVoltage(units::angle::degree_t angleToSetDegrees);

    // Adds a new motor to a group
    bool AddMotorToGroup(MotorGroupControlMode controlMode);

    // Sets the control mode of a motor in a group (intended for use with the CUSTOM group control mode)
    bool SetMotorInGroupControlMode(uint32_t canId, MotorGroupControlMode controlMode);

    // Change Talon mode between brake/coast
    void SetCoastMode();
    void SetBrakeMode();

    // Return the value of the sensor connected to the Talon
    int GetEncoderValue();
    void TareEncoder();

    // Displays information to the driver station about the motor group
    void DisplayStatusInformation();

private:

    // Represents information about a single motor in a group
    struct MotorInfo
    {
        // Storage space for strings for the smart dashboard
        struct DisplayStrings
        {
            static const uint32_t MAX_MOTOR_DISPLAY_STRING_LENGTH = 64U;
            char m_CurrentTemperatureString[MAX_MOTOR_DISPLAY_STRING_LENGTH];
            char m_HighestTemperatureString[MAX_MOTOR_DISPLAY_STRING_LENGTH];
            char m_ResetOccurredString[MAX_MOTOR_DISPLAY_STRING_LENGTH];
        };

        // Member data
        TalonType * m_pTalon;
        DutyCycleOut m_DutyCycleOut;
        PositionVoltage m_PositionVoltage;
        TalonConfigurationType m_MotorConfiguration;
        const char * m_pName;
        MotorGroupControlMode m_ControlMode;
        uint32_t m_CanId;
        double m_CurrentTemperature;
        double m_HighestTemperature;
        bool m_bResetOccurred;
        DisplayStrings m_DisplayStrings;

        MotorInfo(const char * pName, MotorGroupControlMode controlMode, NeutralModeValue neutralMode, CANBus & rCanBus, uint32_t canId, uint32_t idNumberInGroup) :
            m_pTalon(new TalonType(static_cast<int>(canId), rCanBus)),
            m_DutyCycleOut(0.0),
            m_PositionVoltage(0.0_tr),
            m_MotorConfiguration(),
            m_pName(pName),
            m_ControlMode(controlMode),
            m_CanId(canId),
            m_CurrentTemperature(0.0),
            m_HighestTemperature(0.0),
            m_bResetOccurred(false)
        {
            m_MotorConfiguration.MotorOutput.NeutralMode = neutralMode;

            if (controlMode == MotorGroupControlMode::FOLLOW_INVERSE)
            {
                m_MotorConfiguration.MotorOutput.Inverted = true;
            }

            m_pTalon->ClearStickyFaults();

            // Build the strings to use in the display method
            std::snprintf(&m_DisplayStrings.m_CurrentTemperatureString[0], DisplayStrings::MAX_MOTOR_DISPLAY_STRING_LENGTH, "%s #%u %s", m_pName, idNumberInGroup, "temperature (F)");
            std::snprintf(&m_DisplayStrings.m_HighestTemperatureString[0], DisplayStrings::MAX_MOTOR_DISPLAY_STRING_LENGTH, "%s #%u %s", m_pName, idNumberInGroup, "highest temperature (F)");
            std::snprintf(&m_DisplayStrings.m_ResetOccurredString[0], DisplayStrings::MAX_MOTOR_DISPLAY_STRING_LENGTH, "%s #%u %s", m_pName, idNumberInGroup, "reset occurred");
        }

        // Helper routine for configuring some settings on follower talons
        void SetAsFollower(uint32_t leaderCanId, bool bInvert)
        {
            // Follower will honor invert control, StrictFollower ignores invert control
            Follower follower(leaderCanId, bInvert);
            (void)m_pTalon->SetControl(follower);

            // Phoenix 6 Example: Get the StatusSignal objects and call SetUpdateFrequency() on them.
            //(void)m_pTalon->GetDeviceTemp().SetUpdateFrequency(100_Hz);
        }
    };

    static const uint32_t MAX_NUMBER_OF_MOTORS = 4;
    static const uint32_t GROUP_LEADER_PSEUDO_CAN_ID = 0xFF;
    static const uint32_t APPLY_TO_ALL_PSEUDO_CAN_ID = 0xFF;

    // Member variables
    uint32_t m_NumMotors;                                   // Number of motors in the group
    uint32_t m_LeaderCanId;                                 // Keep track of the CAN ID of the leader Talon in the group
    double m_LastOutputValue;                               // Last value that the output of the group was set to
    MotorInfo * m_pMotorsInfo[MAX_NUMBER_OF_MOTORS];        // The motor objects (@todo: No array, linked list?)
    CANBus & m_rCanBus;                                     // The CANBus that the motors in the group are on

    // Prevent default construction/deletion/copy/move/assignment
    TalonMotorGroup();
    ~TalonMotorGroup();
    TalonMotorGroup( const TalonMotorGroup& ) = delete;
    TalonMotorGroup( const TalonMotorGroup&& ) = delete;
    TalonMotorGroup & operator=( const TalonMotorGroup& ) = delete;
};



////////////////////////////////////////////////////////////////
/// @method TalonMotorGroup::GetMotorObject
///
/// Retrieves a specific Talon motor object from the motor
/// group.  By default it will return the first motor object in
/// the group (the leader Talon).  If a CAN ID is specified, it
/// will retrieve that object instead.  The purpose of this
/// function is to allow robot code to make specific calls on a
/// motor object that may only apply to one motor in a group or
/// a specific motor type since this is a template class.
///
////////////////////////////////////////////////////////////////
template <class TalonType, class TalonConfigurationType>
TalonType * TalonMotorGroup<TalonType, TalonConfigurationType>::GetMotorObject(uint32_t canId)
{
    TalonType * pTalonObject = nullptr;

    // By default, return the first object in the group
    if (canId == GROUP_LEADER_PSEUDO_CAN_ID)
    {
        pTalonObject = m_pMotorsInfo[0]->m_pTalon;
    }
    // If a specific CAN ID was given
    else
    {
        // Loop through the motors
        for (uint32_t i = 0U; i < m_NumMotors; i++)
        {
            // Check if this is the right motor
            if (m_pMotorsInfo[i]->m_CanId == canId)
            {
                pTalonObject = m_pMotorsInfo[i]->m_pTalon;
                break;
            }
        }
    }

    return pTalonObject;
}



////////////////////////////////////////////////////////////////
/// @method TalonMotorGroup::GetMotorConfiguration
///
/// Retrieves a specific Talon motor configuration from the
/// motor group.  By default it will return the configuration of
/// the first motor in the group (the leader Talon).  If a CAN
/// ID is specified, it will retrieve that configuration instead.
/// The purpose of this function is to allow robot code to make
/// specific calls on a motor configuration that may only apply
/// to one motor in a group or for a specific configuration type
/// since this is a template class.
///
////////////////////////////////////////////////////////////////
template <class TalonType, class TalonConfigurationType>
TalonConfigurationType * TalonMotorGroup<TalonType, TalonConfigurationType>::GetMotorConfiguration(uint32_t canId)
{
    TalonConfigurationType * pTalonConfig = nullptr;

    // By default, return the first object in the group
    if (canId == GROUP_LEADER_PSEUDO_CAN_ID)
    {
        pTalonConfig = &(m_pMotorsInfo[0]->m_MotorConfiguration);
    }
    // If a specific CAN ID was given
    else
    {
        // Loop through the motors
        for (uint32_t i = 0U; i < m_NumMotors; i++)
        {
            // Check if this is the right motor
            if (m_pMotorsInfo[i]->m_CanId == canId)
            {
                pTalonConfig = &(m_pMotorsInfo[i]->m_MotorConfiguration);
                break;
            }
        }
    }

    return pTalonConfig;
}



////////////////////////////////////////////////////////////////
/// @method TalonMotorGroup::ApplyConfiguration
///
/// Applies an updated configuration to the talon motors in a
/// group.  Applying a configuration to only a specific motor in
/// the group should use the GetMotorConfiguration() function.
///
////////////////////////////////////////////////////////////////
template <class TalonType, class TalonConfigurationType>
void TalonMotorGroup<TalonType, TalonConfigurationType>::ApplyConfiguration(uint32_t canId)
{
    // Loop through the motors.  This is slightly inefficient because
    // we can't break out of the loop if we find a match since we have
    // to consider the 'apply to all' case.
    for (uint32_t i = 0U; i < m_NumMotors; i++)
    {
        // If the config is applied to all motors in the group, or we found the CAN ID
        if ((canId == APPLY_TO_ALL_PSEUDO_CAN_ID) || (m_pMotorsInfo[i]->m_CanId == canId))
        {
            // Apply the config
            (void)m_pMotorsInfo[i]->m_pTalon->GetConfigurator().Apply(m_pMotorsInfo[i]->m_MotorConfiguration);
        }
    }
}



////////////////////////////////////////////////////////////////
/// @method TalonMotorGroup::ApplyConfiguration
///
/// Applies a specific configuration to the talon motors in a
/// group.  Applying a configuration to only a specific motor in
/// the group should use the GetMotorConfiguration() function.
///
////////////////////////////////////////////////////////////////
template <class TalonType, class TalonConfigurationType>
template <typename ConfigType>
void TalonMotorGroup<TalonType, TalonConfigurationType>::ApplyConfiguration(ConfigType config, uint32_t canId)
{
    // For now, don't let this be used
    ASSERT(false);

    // Loop through the motors.  This is slightly inefficient because
    // we can't break out of the loop if we find a match since we have
    // to consider the 'apply to all' case.
    for (uint32_t i = 0U; i < m_NumMotors; i++)
    {
        // If the config is applied to all motors in the group, or we found the CAN ID
        if ((canId == APPLY_TO_ALL_PSEUDO_CAN_ID) || (m_pMotorsInfo[i]->m_CanId == canId))
        {
            // This doesn't update the local object configuration, which can't be done with the template
            //m_pMotorsInfo[i]->m_MotorConfiguration.<member> = config;
            (void)m_pMotorsInfo[i]->m_pTalon->GetConfigurator().Apply(config);
        }
    }
}



////////////////////////////////////////////////////////////////
/// @method TalonMotorGroup::TalonMotorGroup
///
/// Constructor.  Creates the number of motors specified
/// starting from the CAN ID passed in.
///
////////////////////////////////////////////////////////////////
template <class TalonType, class TalonConfigurationType>
TalonMotorGroup<TalonType, TalonConfigurationType>::TalonMotorGroup(const char * pName, uint32_t numMotors, uint32_t leaderCanId,
                                            MotorGroupControlMode nonLeaderControlMode, NeutralModeValue neutralMode, CANBus & rCanBus) :
    m_NumMotors(numMotors),
    m_LeaderCanId(leaderCanId),
    m_LastOutputValue(0.0),
    m_pMotorsInfo(),
    m_rCanBus(rCanBus)
{
    // Loop for each motor to create
    for (uint32_t i = 0U; (i < numMotors) && (i < MAX_NUMBER_OF_MOTORS); i++)
    {
        // Group ID numbers are used in creating the strings and are not zero based
        uint32_t idNumberInGroup = i + 1U;

        // The leader Talon is unique
        if (i == 0U)
        {
            // Create it
            m_pMotorsInfo[i] = new MotorInfo(pName, MotorGroupControlMode::LEADER, neutralMode, m_rCanBus, leaderCanId, idNumberInGroup);
        }
        // Non-leader Talons
        else
        {
            // Create it
            m_pMotorsInfo[i] = new MotorInfo(pName, nonLeaderControlMode, neutralMode, m_rCanBus, (leaderCanId + i), idNumberInGroup);

            // Only set follow for Talon groups that will be configured as
            // such.  The CTRE Phoenix library now passes the control mode in
            // the Set() method, so we only need to set the followers here.
            if ((nonLeaderControlMode == MotorGroupControlMode::FOLLOW) || (nonLeaderControlMode == MotorGroupControlMode::FOLLOW_INVERSE))
            {
                bool bInvert = (nonLeaderControlMode == MotorGroupControlMode::FOLLOW) ? false : true;
                m_pMotorsInfo[i]->SetAsFollower(leaderCanId, bInvert);
            }
        }
    }
}



////////////////////////////////////////////////////////////////
/// @method TalonMotorGroup::AddMotorToGroup
///
/// Method to add a new motor to a motor group.  Only intended
/// to be used during robot object construction for atypical
/// motor groupings (such as three motors with varied control
/// modes).
///
////////////////////////////////////////////////////////////////
template <class TalonType, class TalonConfigurationType>
bool TalonMotorGroup<TalonType, TalonConfigurationType>::AddMotorToGroup(MotorGroupControlMode controlMode)
{
    bool bResult = false;

    // Make sure there's room for another motor in this group
    if (m_NumMotors < MAX_NUMBER_OF_MOTORS)
    {
        // The new motor CAN ID is the first motor's ID + current number of group motors present
        uint32_t newMotorCanId = m_pMotorsInfo[0]->m_CanId + m_NumMotors;

        // m_NumMotors can be leveraged as the index, as it represents the next unused array element
        // All motors in a group have the same name, so we use the existing one.  Group ID is computed from m_NumMotors.
        m_pMotorsInfo[m_NumMotors] = new MotorInfo(m_pMotorsInfo[0]->m_pName, controlMode, m_rCanBus, newMotorCanId, (m_NumMotors + 1));

        // If this Talon will be a follower, be sure to call Set() to enable it
        if ((controlMode == MotorGroupControlMode::FOLLOW) || (controlMode == MotorGroupControlMode::FOLLOW_INVERSE))
        {
            bool bInvert = (controlMode == MotorGroupControlMode::FOLLOW) ? false : true;
            m_pMotorsInfo[m_NumMotors]->SetAsFollower(m_LeaderCanId, bInvert);
        }

        // Increase the number of motors
        m_NumMotors++;
        
        // Indicate success
        bResult = true;
    }

    return bResult;
}



////////////////////////////////////////////////////////////////
/// @method TalonMotorGroup::SetMotorInGroupControlMode
///
/// Method to set the control mode of a motor in a group.  Only
/// intended to be used during robot object construction for
/// atypical motor groupings (such as three motors with varied
/// control modes).
///
////////////////////////////////////////////////////////////////
template <class TalonType, class TalonConfigurationType>
bool TalonMotorGroup<TalonType, TalonConfigurationType>::SetMotorInGroupControlMode(uint32_t canId, MotorGroupControlMode controlMode)
{
    bool bResult = false;

    // Search for the correct motor in the group
    for (uint32_t i = 0U; i < m_NumMotors; i++)
    {
        // If it matches...
        if (m_pMotorsInfo[i]->m_CanId == canId)
        {
            // ...set the control mode
            m_pMotorsInfo[i]->m_ControlMode = controlMode;

            // If this Talon will be a follower, be sure to call Set() to enable it
            if ((controlMode == MotorGroupControlMode::FOLLOW) || (controlMode == MotorGroupControlMode::FOLLOW_INVERSE))
            {
                bool bInvert = (controlMode == MotorGroupControlMode::FOLLOW) ? false : true;
                m_pMotorsInfo[i]->SetAsFollower(m_LeaderCanId, bInvert);
            }
            else
            {
                // The previous mode might have had follower frame rates, so they need to be reset
                // Phoenix 6 Example: Get the StatusSignal objects and call SetUpdateFrequency() on them.
                //(void)m_pMotorsInfo[i]->m_pTalon->GetDeviceTemp().SetUpdateFrequency(100_Hz);
            }

            // Update the inverted status.  Only FOLLOW_INVERSE uses the built-in invert.
            if (controlMode == MotorGroupControlMode::FOLLOW_INVERSE)
            {
                m_pMotorsInfo[i]->m_MotorConfiguration.MotorOutput.WithInverted(true);
                m_pMotorsInfo[i]->m_pTalon->GetConfigurator().Apply(m_pMotorsInfo[i].m_MotorConfiguration.MotorOutput);
            }
            else
            {
                m_pMotorsInfo[i]->m_MotorConfiguration.MotorOutput.WithInverted(false);
                m_pMotorsInfo[i]->m_pTalon->GetConfigurator().Apply(m_pMotorsInfo[i].m_MotorConfiguration.MotorOutput);
            }

            // Indicate success
            bResult = true;
        }
    }

    return bResult;
}



////////////////////////////////////////////////////////////////
/// @method TalonMotorGroup::SetCoastMode
///
/// Method to change a talon to coast mode.
///
////////////////////////////////////////////////////////////////
template <class TalonType, class TalonConfigurationType>
void TalonMotorGroup<TalonType, TalonConfigurationType>::SetCoastMode()
{
    for (uint32_t i = 0U; i < m_NumMotors; i++)
    {
        m_pMotorsInfo[i]->m_MotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue::Coast;
        m_pMotorsInfo[i]->m_pTalon->GetConfigurator().Apply(m_pMotorsInfo[i]->m_MotorConfiguration.MotorOutput);
    }
}



////////////////////////////////////////////////////////////////
/// @method TalonMotorGroup::SetBrakeMode
///
/// Method to change a talon to brake mode.
///
////////////////////////////////////////////////////////////////
template <class TalonType, class TalonConfigurationType>
void TalonMotorGroup<TalonType, TalonConfigurationType>::SetBrakeMode()
{
    for (uint32_t i = 0U; i < m_NumMotors; i++)
    {
        m_pMotorsInfo[i]->m_MotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue::Brake;
        m_pMotorsInfo[i]->m_pTalon->GetConfigurator().Apply(m_pMotorsInfo[i]->m_MotorConfiguration.MotorOutput);
    }
}



////////////////////////////////////////////////////////////////
/// @method TalonMotorGroup::SetDutyCycle
///
/// Method to set the speed of each motor in the group.  The
/// offset parameter is only valid for motor groups configured
/// as *_OFFSET.
///
////////////////////////////////////////////////////////////////
template <class TalonType, class TalonConfigurationType>
void TalonMotorGroup<TalonType, TalonConfigurationType>::SetDutyCycle(double value, double offset)
{
    for (uint32_t i = 0U; i < m_NumMotors; i++)
    {
        // Most modes wil need to call Set() later, but some won't
        bool bCallSet = true;

        // The value that will be passed to Set()
        double valueToSet = 0.0;

        // Check what the control mode of this motor is.  Most CAN Talons
        // will be set to follow, but some may be independent or inverse (such
        // as if they need to drive in different directions).
        switch (m_pMotorsInfo[i]->m_ControlMode)
        {
            case MotorGroupControlMode::LEADER:
            case MotorGroupControlMode::INDEPENDENT:
            {
                // The leader always gets set via duty cycle, as do motors
                // that are independently controlled (not follow or inverse).
                valueToSet = value;
                break;
            }
            case MotorGroupControlMode::FOLLOW:
            case MotorGroupControlMode::FOLLOW_INVERSE:
            {
                // Nothing to do, motor had SetControl() called during object construction
                bCallSet = false;
                break;
            }
            case MotorGroupControlMode::INVERSE:
            {
                // Motor is attached to drive in opposite direction of leader
                valueToSet = -value;
                break;
            }
            case MotorGroupControlMode::INDEPENDENT_OFFSET:
            {
                // The non-leader motor has a different value in this case
                valueToSet = value + offset;
                break;
            }
            case MotorGroupControlMode::INVERSE_OFFSET:
            {
                // The non-leader motor has a different value in this case
                valueToSet = -(value + offset);
                break;
            }
            default:
            {
                // Can reach here with CUSTOM motors still set.  Calling code should
                // update those motors to a different control mode via class API calls.
                break;
            }
        };

        if (bCallSet)
        {
            // To conserve CAN bus utilization, only send a value if it has changed
            if (valueToSet != m_LastOutputValue)
            {
                // Set the value in the Talon
                (void)m_pMotorsInfo[i]->m_pTalon->SetControl(m_pMotorsInfo[i]->m_DutyCycleOut.WithOutput(valueToSet));
                m_LastOutputValue = valueToSet;
            }
        }
    }
}



////////////////////////////////////////////////////////////////
/// @method TalonMotorGroup::SetPositionVoltage
///
/// Method to set the output of a motor to hold a specified
/// angle.
///
////////////////////////////////////////////////////////////////
template <class TalonType, class TalonConfigurationType>
void TalonMotorGroup<TalonType, TalonConfigurationType>::SetPositionVoltage(units::angle::degree_t angleToSetDegrees)
{
    // The first entry in the motor info array should always
    // be a leader, so that one will be the only one updated.
    // When holding position, we don't want to have to manage
    // the target angles for multiple motors since it would
    // require tracking different set points based on the
    // encoders.  For a motor group to successfully hold
    // position, all but the leader must be a follower (no
    // support for independent control).  This means that a
    // misconfigured group at object instantiation will not
    // send a control signal to all motors in the group.  Make
    // sure this is only called on FOLLOW OR FOLLOW_INVERSE options.

    // To conserve CAN bus utilization, only send a value if it has changed
    if (angleToSetDegrees.value() != m_LastOutputValue)
    {
        // Set the control output
        units::angle::turn_t turns(angleToSetDegrees);
        (void)m_pMotorsInfo[0]->m_pTalon->SetControl(m_pMotorsInfo[0]->m_PositionVoltage.WithPosition(turns));
        m_LastOutputValue = angleToSetDegrees.value();
    }
}



////////////////////////////////////////////////////////////////
/// @method TalonMotorGroup::DisplayStatusInformation
///
/// Sends status information to the smart dashboard.
///
////////////////////////////////////////////////////////////////
template <class TalonType, class TalonConfigurationType>
void TalonMotorGroup<TalonType, TalonConfigurationType>::DisplayStatusInformation()
{
    for (uint32_t i = 0U; i < m_NumMotors; i++)
    {
        m_pMotorsInfo[i]->m_CurrentTemperature = RobotUtils::ConvertCelsiusToFahrenheit(m_pMotorsInfo[i]->m_pTalon->GetDeviceTemp().GetValueAsDouble());
        if (m_pMotorsInfo[i]->m_CurrentTemperature > m_pMotorsInfo[i]->m_HighestTemperature)
        {
            m_pMotorsInfo[i]->m_HighestTemperature = m_pMotorsInfo[i]->m_CurrentTemperature;
        }

        // @todo: Also consider sticky faults?
        m_pMotorsInfo[i]->m_bResetOccurred = m_pMotorsInfo[i]->m_pTalon->HasResetOccurred();

        SmartDashboard::PutNumber(m_pMotorsInfo[i]->m_DisplayStrings.m_CurrentTemperatureString, m_pMotorsInfo[i]->m_CurrentTemperature);
        SmartDashboard::PutNumber(m_pMotorsInfo[i]->m_DisplayStrings.m_HighestTemperatureString, m_pMotorsInfo[i]->m_HighestTemperature);
        SmartDashboard::PutBoolean(m_pMotorsInfo[i]->m_DisplayStrings.m_ResetOccurredString, m_pMotorsInfo[i]->m_bResetOccurred);
    }
}

#endif // EASTTECHTALON_HPP
