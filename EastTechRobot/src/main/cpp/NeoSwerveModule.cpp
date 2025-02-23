////////////////////////////////////////////////////////////////////////////////
/// @file   NeoSwerveModule.cpp
/// @author David Stalter
///
/// @details
/// Implements functionality for a Neo swerve module on a swerve drive robot.
///
/// Copyright (c) 2024 East Technical High School
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
// <none>

// C INCLUDES
#include "frc/Timer.h"                              // for timers
#include "frc/smartdashboard/SmartDashboard.h"      // for interacting with the smart dashboard
#include "rev/config/SparkMaxConfig.h"              // for creating and applying SparkMax configs
#include "units/length.h"                           // for units::meters

// C++ INCLUDES
#include "NeoSwerveModule.hpp"                      // for class declaration
#include "RobotUtils.hpp"                           // for ConvertCelsiusToFahrenheit
#include "SwerveConfig.hpp"                         // for swerve configuration and constants
#include "SwerveConversions.hpp"                    // for conversion functions

using namespace frc;

uint32_t NeoSwerveModule::m_DetailedModuleDisplayIndex = 0U;


////////////////////////////////////////////////////////////////
/// @method NeoSwerveModule::NeoSwerveModule
///
/// Constructs a swerve module object.  This will configure the
/// settings for each SparkMax (PID values, current limiting,
/// etc.) and the CANCoder.  It also builds the display strings
/// sent to the dashboard.  Note that the CANCoders are placed
/// on the CANivore bus, which requires a 120 ohm terminating
/// resistor.
///
/// 2024: Bevels facing right is 1.0 forward on the Neos.
///
////////////////////////////////////////////////////////////////
NeoSwerveModule::NeoSwerveModule(SwerveModuleConfig config) :
    m_MotorGroupPosition(config.m_Position),
    m_pDriveSpark(new SparkMax(config.m_DriveMotorCanId, SparkMax::MotorType::kBrushless)),
    m_pAngleSpark(new SparkMax(config.m_AngleMotorCanId, SparkMax::MotorType::kBrushless)),
    m_DriveSparkEncoder(m_pDriveSpark->GetEncoder()),
    m_AngleSparkEncoder(m_pAngleSpark->GetEncoder()),
    m_DrivePidController(m_pDriveSpark->GetClosedLoopController()),
    m_AnglePidController(m_pAngleSpark->GetClosedLoopController()),
    m_pAngleCanCoder(new CANcoder(config.m_CanCoderId, "canivore-8145")),
    m_AngleOffset(config.m_AngleOffset),
    m_LastAngle(),
    m_pFeedForward(new SimpleMotorFeedforward<units::meters>(KS, KV, KA))
{
    // Build the strings to use in the display method
    std::snprintf(&m_DisplayStrings.m_CancoderAngleString[0], DisplayStrings::MAX_MODULE_DISPLAY_STRING_LENGTH, "%s %s", config.m_pModuleName, "cancoder");
    std::snprintf(&m_DisplayStrings.m_NeoEncoderAngleString[0], DisplayStrings::MAX_MODULE_DISPLAY_STRING_LENGTH, "%s %s", config.m_pModuleName, "NEO encoder");
    std::snprintf(&m_DisplayStrings.m_DriveNeoTemp[0], DisplayStrings::MAX_MODULE_DISPLAY_STRING_LENGTH, "%s %s", config.m_pModuleName, "drive temp (F)");
    std::snprintf(&m_DisplayStrings.m_AngleNeoTemp[0], DisplayStrings::MAX_MODULE_DISPLAY_STRING_LENGTH, "%s %s", config.m_pModuleName, "angle temp (F)");

    // Configure drive motor controller
    SparkMaxConfig driveConfig;
    driveConfig.SmartCurrentLimit(80);
    driveConfig.Inverted(false);
    driveConfig.SetIdleMode(SparkMaxConfig::IdleMode::kBrake);
    driveConfig.encoder.PositionConversionFactor(SwerveConfig::WHEEL_CIRCUMFERENCE / SwerveConfig::SELECTED_SWERVE_MODULE_CONFIG.DRIVE_GEAR_RATIO);
    driveConfig.encoder.VelocityConversionFactor(SwerveConfig::WHEEL_CIRCUMFERENCE / SwerveConfig::SELECTED_SWERVE_MODULE_CONFIG.DRIVE_GEAR_RATIO / 60.0);
    driveConfig.closedLoop.SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder);
    driveConfig.closedLoop.Pid(0.02, 0.0, 0.0);
    driveConfig.closedLoop.VelocityFF(0.0);
    driveConfig.VoltageCompensation(12.0);
    m_pDriveSpark->Configure(driveConfig, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kPersistParameters);

    m_DriveSparkEncoder.SetPosition(0.0);     // countsPerRev = 42

    // The signals retrieved by each kStatusX periodic frame are listed in SparkLowLevel.h.
    // Set a specific signal frequency using configVariable.signals.Function(value).  The
    // library will then manage the kStatusX as appropriate.
    //driveConfig.signals.PrimaryEncoderPositionPeriodMs(20);


    // Configure angle motor controller
    // Current limiting values: enable, limit, threshold, duration
    SparkMaxConfig angleConfig;
    angleConfig.SmartCurrentLimit(20);
    angleConfig.Inverted(false);
    angleConfig.SetIdleMode(SparkMaxConfig::IdleMode::kCoast);
    angleConfig.encoder.PositionConversionFactor(360.0 / SwerveConfig::SELECTED_SWERVE_MODULE_CONFIG.ANGLE_GEAR_RATIO);
    angleConfig.closedLoop.SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder);
    angleConfig.closedLoop.Pid(0.028, 0.0, 0.0015);
    angleConfig.closedLoop.VelocityFF(0.000);
    angleConfig.VoltageCompensation(12.0);

    m_pAngleSpark->Configure(angleConfig, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kPersistParameters);

    // The signals retrieved by each kStatusX periodic frame are listed in SparkLowLevel.h.
    // Set a specific signal frequency using configVariable.signals.Function(value).  The
    // library will then manage the kStatusX as appropriate.
    //angleConfig.signals.PrimaryEncoderPositionPeriodMs(20);


    // Configure CANCoder
    CANcoderConfiguration canCoderConfig;
    // Per the CTRE documentation: Setting this to 1 makes the absolute position unsigned [0, 1)
    canCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0_tr;
    canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue::CounterClockwise_Positive;
    (void)m_pAngleCanCoder->GetConfigurator().Apply(canCoderConfig);
    //m_pAngleCanCoder->SetStatusFramePeriod(CANCoderStatusFrame_SensorData, 100);
    //m_pAngleCanCoder->SetStatusFramePeriod(CANCoderStatusFrame_VbatAndFaults, 100);

    // Reset the swerve module to the absolute angle starting position.
    // This reads the current angle from the CANCoder and figures out how
    // far the module is from the config passed in (the predetermined
    // position from manual measurement/calibration).

    // With the NEOs/SparkMax controllers, this code doesn't actually seem
    // to have an effect.  The robot startup has to call HomeModules() to
    // get a result.  The suspicion is that the SparkMax won't respond to
    // commands while the robot is not in an enabled, which is the case
    // when constructors run.  Calling SetReference() here won't do anything.

    double absolutePositionDelta = m_pAngleCanCoder->GetAbsolutePosition().GetValueAsDouble() - m_AngleOffset.Degrees().value();
    m_AngleSparkEncoder.SetPosition(absolutePositionDelta);

    // Save off the initial angle
    m_LastAngle = units::degree_t(m_AngleSparkEncoder.GetPosition());
}


////////////////////////////////////////////////////////////////
/// @method NeoSwerveModule::HomeModule
///
/// Points the swerve module straight forward (zero degrees).
///
////////////////////////////////////////////////////////////////
void NeoSwerveModule::HomeModule()
{
    double absolutePositionDelta = m_pAngleCanCoder->GetAbsolutePosition().GetValueAsDouble() - m_AngleOffset.Degrees().value();
    m_AngleSparkEncoder.SetPosition(absolutePositionDelta);
    m_AnglePidController.SetReference(0.0, SparkMax::ControlType::kPosition);
    m_LastAngle = 0.0_deg;
}


////////////////////////////////////////////////////////////////
/// @method NeoSwerveModule::Optimize
///
/// Optimizes a swerve module state for use with setting a
/// desired state.  This finds the shortest way to move to a
/// target angle to prevent motion over 180 degrees (reversing
/// the target speed, if necessary).
///
////////////////////////////////////////////////////////////////
SwerveModuleState NeoSwerveModule::Optimize(SwerveModuleState desiredState, Rotation2d currentAngle)
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
/// @method NeoSwerveModule::SetDesiredState
///
/// Sets a swerve module to the input state.  It computes the
/// target velocity and angle and updates the motor controllers
/// as appropriate.
///
////////////////////////////////////////////////////////////////
void NeoSwerveModule::SetDesiredState(SwerveModuleState desiredState, bool bIsOpenLoop)
{
    // Custom optimize command, since default WPILib optimize assumes continuous controller which CTRE is not
    desiredState = Optimize(desiredState, GetSwerveModuleState().angle);

    // Update the drive motor controller
    if (bIsOpenLoop)
    {
        double percentOutput = desiredState.speed / SwerveConfig::MAX_DRIVE_VELOCITY_MPS;
        m_pDriveSpark->Set(percentOutput);
    }
    else
    {
        m_DrivePidController.SetReference(desiredState.speed.value(), SparkMax::ControlType::kVelocity, ClosedLoopSlot::kSlot0, m_pFeedForward->Calculate(desiredState.speed).value());
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
    m_AnglePidController.SetReference(angle.Degrees().value(), SparkMax::ControlType::kPosition);

    // Save off the updated last angle
    m_LastAngle = angle;
}


////////////////////////////////////////////////////////////////
/// @method NeoSwerveModule::GetSwerveModuleState
///
/// Returns a swerve module state based on information from the
/// motor controllers and sensors.
///
////////////////////////////////////////////////////////////////
SwerveModuleState NeoSwerveModule::GetSwerveModuleState()
{
    // Get the current velocity
    units::velocity::meters_per_second_t velocity(m_DriveSparkEncoder.GetVelocity());

    // Get the current angle
    units::angle::degree_t angle(m_AngleSparkEncoder.GetPosition());

    return {velocity, angle};
}


////////////////////////////////////////////////////////////////
/// @method NeoSwerveModule::GetSwerveModulePosition
///
/// Returns a swerve module position based on information from
/// the motor controllers and sensors.
///
////////////////////////////////////////////////////////////////
SwerveModulePosition NeoSwerveModule::GetSwerveModulePosition()
{
    // Get the current distance
    units::meter_t distance(m_DriveSparkEncoder.GetPosition());

    // Get the current angle
    units::angle::degree_t angle(m_AngleSparkEncoder.GetPosition());

    return {distance, angle};
}


////////////////////////////////////////////////////////////////
/// @method NeoSwerveModule::UpdateSmartDashboard
///
/// Support routine to put useful information on the dashboard.
///
////////////////////////////////////////////////////////////////
void NeoSwerveModule::UpdateSmartDashboard()
{
    // Print the encoder values every time
    SmartDashboard::PutNumber(m_DisplayStrings.m_CancoderAngleString, m_pAngleCanCoder->GetAbsolutePosition().GetValueAsDouble());
    SmartDashboard::PutNumber(m_DisplayStrings.m_NeoEncoderAngleString, m_AngleSparkEncoder.GetPosition());

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
            SmartDashboard::PutNumber(m_DisplayStrings.m_DriveNeoTemp, RobotUtils::ConvertCelsiusToFahrenheit(m_pDriveSpark->GetMotorTemperature()));
            SmartDashboard::PutNumber(m_DisplayStrings.m_AngleNeoTemp, RobotUtils::ConvertCelsiusToFahrenheit(m_pAngleSpark->GetMotorTemperature()));

            m_DetailedModuleDisplayIndex++;
            if (m_DetailedModuleDisplayIndex == SwerveConfig::NUM_SWERVE_DRIVE_MODULES)
            {
                m_DetailedModuleDisplayIndex = 0U;
            }
        }
        lastUpdateTime = currentTime;
    }
}
