////////////////////////////////////////////////////////////////////////////////
/// @file   TalonFxSwerveModule.cpp
/// @author David Stalter
///
/// @details
/// Implements functionality for a swerve module on a swerve drive robot.
///
/// Copyright (c) 2025 East Technical High School
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
// <none>

// C INCLUDES
#include "frc/Timer.h"                              // for timers
#include "frc/smartdashboard/SmartDashboard.h"      // for interacting with the smart dashboard
#include "units/length.h"                           // for units::meters

// C++ INCLUDES
#include "RobotUtils.hpp"                           // for ConvertCelsiusToFahrenheit
#include "SwerveConfig.hpp"                         // for swerve configuration and constants
#include "SwerveConversions.hpp"                    // for conversion functions
#include "TalonFxSwerveModule.hpp"                  // for class declaration

using namespace frc;

uint32_t TalonFxSwerveModule::m_DetailedModuleDisplayIndex = 0U;


////////////////////////////////////////////////////////////////
/// @method TalonFxSwerveModule::TalonFxSwerveModule
///
/// Constructs a swerve module object.  This will configure the
/// settings for each TalonFX (PID values, current limiting,
/// etc.) and the CANCoder.  It also builds the display strings
/// sent to the dashboard.  Note that the CANCoders are placed
/// on the CANivore bus, which requires a 120 ohm terminating
/// resistor.
///
/// 2025: Bevels facing right is 1.0 forward on the Talons.
///
////////////////////////////////////////////////////////////////
TalonFxSwerveModule::TalonFxSwerveModule(SwerveModuleConfig config) :
    m_MotorGroupPosition(config.m_Position),
    m_pDriveTalon(new TalonFX(config.m_DriveMotorCanId)),
    m_pAngleTalon(new TalonFX(config.m_AngleMotorCanId)),
    m_pAngleCanCoder(new CANcoder(config.m_CanCoderId, "canivore-8145")),
    m_LastAngle(),
    m_pFeedForward(new SimpleMotorFeedforward<units::meters>(KS, KV, KA)),
    m_DriveDutyCycleOut(0.0),
    m_DriveVelocityVoltage(0.0_tps),
    m_AnglePositionVoltage(0.0_tr),
    CANCODER_REFERENCE_ABSOLUTE_OFFSET(config.m_CancoderReferenceAbsoluteOffset)
{
    // Build the strings to use in the display method
    std::snprintf(&m_DisplayStrings.m_CancoderAngleString[0], DisplayStrings::MAX_MODULE_DISPLAY_STRING_LENGTH, "%s %s", config.m_pModuleName, "cancoder");
    std::snprintf(&m_DisplayStrings.m_FxEncoderAngleString[0], DisplayStrings::MAX_MODULE_DISPLAY_STRING_LENGTH, "%s %s", config.m_pModuleName, "FX encoder");
    std::snprintf(&m_DisplayStrings.m_DriveTalonTemp[0], DisplayStrings::MAX_MODULE_DISPLAY_STRING_LENGTH, "%s %s", config.m_pModuleName, "drive temp (F)");
    std::snprintf(&m_DisplayStrings.m_AngleTalonTemp[0], DisplayStrings::MAX_MODULE_DISPLAY_STRING_LENGTH, "%s %s", config.m_pModuleName, "angle temp (F)");

    // Configure drive motor controller
    TalonFXConfiguration driveTalonConfig;
    driveTalonConfig.MotorOutput.Inverted = SwerveConfig::SELECTED_SWERVE_MODULE_CONFIG.DRIVE_MOTOR_INVERTED_VALUE;
    driveTalonConfig.MotorOutput.NeutralMode = NeutralModeValue::Brake;
    driveTalonConfig.Feedback.SensorToMechanismRatio = SwerveConfig::SELECTED_SWERVE_MODULE_CONFIG.DRIVE_GEAR_RATIO;

    driveTalonConfig.CurrentLimits.SupplyCurrentLowerLimit = 35.0_A;
    driveTalonConfig.CurrentLimits.SupplyCurrentLimit = 60.0_A;
    driveTalonConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1_s;
    driveTalonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    driveTalonConfig.Slot0.kP = 0.1;
    driveTalonConfig.Slot0.kI = 0.0;
    driveTalonConfig.Slot0.kD = 0.0;

    driveTalonConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25_s;
    driveTalonConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25_s;

    driveTalonConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.0_s;
    driveTalonConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0_s;

    (void)m_pDriveTalon->GetConfigurator().Apply(driveTalonConfig);
    (void)m_pDriveTalon->GetConfigurator().SetPosition(0.0_tr);

    // @todo: Should the talons change default group status rates to preserve CAN bandwidth?
    // @todo_phoenix6: The example calls need to be updated.
    //m_pDriveTalon->SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 100);
    //m_pDriveTalon->SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 100);

    // Configure angle motor controller
    TalonFXConfiguration angleTalonConfig;
    angleTalonConfig.MotorOutput.Inverted = SwerveConfig::SELECTED_SWERVE_MODULE_CONFIG.ANGLE_MOTOR_INVERTED_VALUE;
    angleTalonConfig.MotorOutput.NeutralMode = NeutralModeValue::Coast;
    angleTalonConfig.Feedback.SensorToMechanismRatio = SwerveConfig::SELECTED_SWERVE_MODULE_CONFIG.ANGLE_GEAR_RATIO;
    angleTalonConfig.ClosedLoopGeneral.ContinuousWrap = true;

    angleTalonConfig.CurrentLimits.SupplyCurrentLowerLimit = 25.0_A;
    angleTalonConfig.CurrentLimits.SupplyCurrentLimit = 40.0_A;
    angleTalonConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1_s;
    angleTalonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // @todo_phoenix6: Tune these, they mostly work.
    angleTalonConfig.Slot0.kP = 27.0;
    angleTalonConfig.Slot0.kI = 0.0;
    angleTalonConfig.Slot0.kD = 0.2;

    (void)m_pAngleTalon->GetConfigurator().Apply(angleTalonConfig);

    // Configure CANCoder
    CANcoderConfiguration canCoderConfig;
    // Per the CTRE documentation: Setting this to 1 makes the absolute position unsigned [0, 1)
    canCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0_tr;
    canCoderConfig.MagnetSensor.SensorDirection = SwerveConfig::SELECTED_SWERVE_MODULE_CONFIG.CANCODER_INVERTED_VALUE;
    (void)m_pAngleCanCoder->GetConfigurator().Apply(canCoderConfig);

    RecalibrateModules();
}


////////////////////////////////////////////////////////////////
/// @method TalonFxSwerveModule::RecalibrateModules
///
/// Recalibrates the swerve module by reading the absolute
/// encoder and setting the appropriate motor controller values.
///
////////////////////////////////////////////////////////////////
void TalonFxSwerveModule::RecalibrateModules()
{
    // Reset the swerve module to the absolute angle starting position.
    // This reads the current angle from the CANCoder and figures out how
    // far the module is from the config passed in (the predetermined
    // position from manual measurement/calibration).
    units::angle::turn_t currentCanCoderInTurns = m_pAngleCanCoder->GetAbsolutePosition().GetValue();
    units::angle::degree_t currentCanCoderInDegrees = currentCanCoderInTurns;

    units::angle::degree_t canCoderDeltaDegrees = CANCODER_REFERENCE_ABSOLUTE_OFFSET.Degrees() - currentCanCoderInDegrees;
    if (canCoderDeltaDegrees.value() < 0.0)
    {
        canCoderDeltaDegrees += 360.0_deg;
    }
    units::angle::turn_t fxTargetTurns = canCoderDeltaDegrees;
    units::angle::turn_t fxTargetTurnsStart = fxTargetTurns;

    while (fxTargetTurns > 1.0_tr)
    {
        fxTargetTurns -= 1.0_tr;
    }

    while (fxTargetTurns < -1.0_tr)
    {
        fxTargetTurns += 1.0_tr;
    }

    static bool bPrintedFirstMeasurement = false;
    if (!bPrintedFirstMeasurement)
    {
        std::printf("Swerve mod %d fxTargetTurns (start): %f\n", m_MotorGroupPosition, fxTargetTurnsStart.value());
        std::printf("Swerve mod %d fxPosition: %f\n", m_MotorGroupPosition, m_pAngleTalon->GetPosition().GetValueAsDouble());
        std::printf("Swerve mod %d canCoderDeg: %f\n", m_MotorGroupPosition, currentCanCoderInDegrees.value());
        std::printf("Swerve mod %d canCoderDelta: %f\n", m_MotorGroupPosition, canCoderDeltaDegrees.value());
        std::printf("Swerve mod %d fxTargetTurns (final): %f\n", m_MotorGroupPosition, fxTargetTurns.value());
        bPrintedFirstMeasurement = true;
    }

    m_pAngleTalon->SetPosition(fxTargetTurns);
    m_LastAngle = units::degree_t(m_pAngleTalon->GetPosition().GetValue());
}


////////////////////////////////////////////////////////////////
/// @method TalonFxSwerveModule::Optimize
///
/// Optimizes a swerve module state for use with setting a
/// desired state.  This finds the shortest way to move to a
/// target angle to prevent motion over 180 degrees (reversing
/// the target speed, if necessary).
///
////////////////////////////////////////////////////////////////
SwerveModuleState TalonFxSwerveModule::Optimize(SwerveModuleState desiredState, Rotation2d currentAngle)
{
    // This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not
    double targetAngle = SwerveConversions::AdjustAngleScope(currentAngle.Degrees().value(), desiredState.angle.Degrees().value());
    double targetSpeed = desiredState.speed.value();
    double delta = targetAngle - currentAngle.Degrees().value();

    if (std::abs(delta) > 90)
    {
        targetSpeed = -targetSpeed;
        if (delta > 90)
        {
            targetAngle -= 180;
        }
        else
        {
            targetAngle += 180;
        }
    }

    return {units::velocity::meters_per_second_t(targetSpeed), units::angle::degree_t(targetAngle)};
}


////////////////////////////////////////////////////////////////
/// @method TalonFxSwerveModule::SetDesiredState
///
/// Sets a swerve module to the input state.  It computes the
/// target velocity and angle and updates the motor controllers
/// as appropriate.
///
////////////////////////////////////////////////////////////////
void TalonFxSwerveModule::SetDesiredState(SwerveModuleState desiredState, bool bIsOpenLoop)
{
    // Custom optimize command, since default WPILib optimize assumes continuous controller which CTRE is not
    // @todo_phoenix6: This -1 multiplier is critical.
    desiredState = Optimize(desiredState, -GetSwerveModuleState().angle);

    // Update the drive motor controller
    if (bIsOpenLoop)
    {
        m_DriveDutyCycleOut.Output = desiredState.speed / SwerveConfig::MAX_DRIVE_VELOCITY_MPS;
        (void)m_pDriveTalon->SetControl(m_DriveDutyCycleOut);
    }
    else
    {
        units::angular_velocity::turns_per_second_t driveTalonDesiredVelocityTps = units::angular_velocity::turns_per_second_t(SwerveConversions::MpsToRps(desiredState.speed.value(), SwerveConfig::WHEEL_CIRCUMFERENCE));
        m_DriveVelocityVoltage.Velocity = driveTalonDesiredVelocityTps;
        m_DriveVelocityVoltage.FeedForward = m_pFeedForward->Calculate(desiredState.speed);
        (void)m_pDriveTalon->SetControl(m_DriveVelocityVoltage);
    }

    // Update the angle motor controller
    // Prevent rotating module if speed is less then 1% (prevents jitter).
    // (If the wheels are moving too slow, don't turn them.)
    Rotation2d angle = 0.0_deg;
    if (std::abs(desiredState.speed.value()) <= (SwerveConfig::MAX_ANGULAR_VELOCITY_RAD_PER_SEC.value() * 0.01))
    {
        angle = m_LastAngle;
    }
    else
    {
        angle = desiredState.angle;
    }
    
    units::angle::turn_t targetAngle = angle.Degrees();
    // @todo_phoenix6: This -1 multiplier is critical.
    (void)m_pAngleTalon->SetControl(m_AnglePositionVoltage.WithPosition(-targetAngle));

    // Save off the updated last angle
    m_LastAngle = angle;

/*
    units::angle::degree_t d = m_pAngleTalon->GetPosition().GetValue();
    switch (m_MotorGroupPosition)
    {
        case 0:
        {
            SmartDashboard::PutNumber("Debug A", targetAngle.value());
            SmartDashboard::PutNumber("Debug E", angle.Degrees().value());
            SmartDashboard::PutNumber("Debug I", d.value());
            break;
        }
        case 1:
        {
            SmartDashboard::PutNumber("Debug B", targetAngle.value());
            SmartDashboard::PutNumber("Debug F", angle.Degrees().value());
            SmartDashboard::PutNumber("Debug J", d.value());
            break;
        }
        case 2:
        {
            SmartDashboard::PutNumber("Debug C", targetAngle.value());
            SmartDashboard::PutNumber("Debug G", angle.Degrees().value());
            SmartDashboard::PutNumber("Debug K", d.value());
            break;
        }
        case 3:
        {
            SmartDashboard::PutNumber("Debug D", targetAngle.value());
            SmartDashboard::PutNumber("Debug H", angle.Degrees().value());
            SmartDashboard::PutNumber("Debug L", d.value());
            break;
        }
        default:
        {
            break;
        }
    }
*/
}


////////////////////////////////////////////////////////////////
/// @method TalonFxSwerveModule::GetSwerveModuleState
///
/// Returns a swerve module state based on information from the
/// motor controllers and sensors.
///
////////////////////////////////////////////////////////////////
SwerveModuleState TalonFxSwerveModule::GetSwerveModuleState()
{
    // Get the current velocity
    units::angular_velocity::turns_per_second_t driveTalonVelocityRps = m_pDriveTalon->GetVelocity().GetValue();
    units::velocity::meters_per_second_t velocity(SwerveConversions::RpsToMps(driveTalonVelocityRps.value(), SwerveConfig::WHEEL_CIRCUMFERENCE));

    // Get the current angle
    units::angle::turn_t angleTalonPositionInTurns = m_pAngleTalon->GetPosition().GetValue();
    units::angle::degree_t angle = angleTalonPositionInTurns;

    return {velocity, angle};
}


////////////////////////////////////////////////////////////////
/// @method TalonFxSwerveModule::GetSwerveModulePosition
///
/// Returns a swerve module position based on information from
/// the motor controllers and sensors.
///
////////////////////////////////////////////////////////////////
SwerveModulePosition TalonFxSwerveModule::GetSwerveModulePosition()
{
    // Get the current distance
    units::angle::turn_t driveTalonPositionInTurns = m_pDriveTalon->GetPosition().GetValue();
    units::meter_t distance(SwerveConversions::RotationsToMeters(driveTalonPositionInTurns.value(), SwerveConfig::WHEEL_CIRCUMFERENCE));

    // Get the current angle
    units::angle::turn_t angleTalonPositionInTurns = m_pAngleTalon->GetPosition().GetValue();
    units::angle::degree_t angle = angleTalonPositionInTurns;

    return {distance, angle};
}


////////////////////////////////////////////////////////////////
/// @method TalonFxSwerveModule::UpdateSmartDashboard
///
/// Support routine to put useful information on the dashboard.
///
////////////////////////////////////////////////////////////////
void TalonFxSwerveModule::UpdateSmartDashboard()
{
    // Print the encoder values every time
    SmartDashboard::PutNumber(m_DisplayStrings.m_CancoderAngleString, m_pAngleCanCoder->GetAbsolutePosition().GetValueAsDouble());
    SmartDashboard::PutNumber(m_DisplayStrings.m_FxEncoderAngleString, GetSwerveModulePosition().angle.Degrees().value());

    // Create and start a timer the first time through
    static Timer * pTimer = new Timer();
    static bool bTimerStarted = false;
    if (!bTimerStarted)
    {
        pTimer->Start();
        bTimerStarted = true;
    }
    static units::second_t lastUpdateTime = 0_s;
    units::second_t currentTime = pTimer->Get();

    // If it's time for a detailed update, print more info
    const units::second_t DETAILED_DISPLAY_TIME_S = 0.5_s;
    if ((currentTime - lastUpdateTime) > DETAILED_DISPLAY_TIME_S)
    {
        // Even at the slower update rate, only do one swerve module at a time
        if (m_DetailedModuleDisplayIndex == static_cast<uint32_t>(m_MotorGroupPosition))
        {
            SmartDashboard::PutNumber(m_DisplayStrings.m_DriveTalonTemp, RobotUtils::ConvertCelsiusToFahrenheit(m_pDriveTalon->GetDeviceTemp().GetValueAsDouble()));
            SmartDashboard::PutNumber(m_DisplayStrings.m_AngleTalonTemp, RobotUtils::ConvertCelsiusToFahrenheit(m_pAngleTalon->GetDeviceTemp().GetValueAsDouble()));

            m_DetailedModuleDisplayIndex++;
            if (m_DetailedModuleDisplayIndex == SwerveConfig::NUM_SWERVE_DRIVE_MODULES)
            {
                m_DetailedModuleDisplayIndex = 0U;
            }
        }
        lastUpdateTime = currentTime;
    }
}
