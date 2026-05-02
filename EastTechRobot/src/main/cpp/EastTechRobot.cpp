////////////////////////////////////////////////////////////////////////////////
/// @file   EastTechRobot.cpp
/// @author David Stalter
///
/// @details
/// Implementation of the EastTechRobot class.  This file contains the functions
/// for full robot operation in FRC.  It contains the autonomous and operator
/// control routines as well as all necessary support for interacting with all
/// motors, sensors and input/outputs on the robot.
///
/// Copyright (c) 2026 East Technical High School
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
#include <cstddef>                      // for nullptr
#include <cstring>                      // for memset

// C INCLUDES
// (none)

// C++ INCLUDES
#include "EastTechRobot.hpp"            // for class declaration (and other headers)
#include "RobotCamera.hpp"              // for interacting with cameras
#include "RobotUtils.hpp"               // for Trim(), Limit() and DisplayMessage()

// STATIC MEMBER VARIABLES
EastTechRobot * EastTechRobot::m_pThis;


////////////////////////////////////////////////////////////////
/// @method EastTechRobot::EastTechRobot
///
/// Constructor.  Instantiates all robot control objects.
///
////////////////////////////////////////////////////////////////
EastTechRobot::EastTechRobot() :
    m_AutonomousChooser                 (),
    m_AutoSwerveDirections              (),
    m_pDriveController                  (new DriveControllerType(DRIVE_CONTROLLER_MODEL, DRIVE_JOYSTICK_PORT)),
    m_pAuxController                    (new AuxControllerType(AUX_CONTROLLER_MODEL, AUX_JOYSTICK_PORT)),
    m_RioCanBus                         (RIO_CAN_BUS_NAME),
    m_CanivoreBus                       (CANIVORE_CAN_BUS_NAME),
    m_pPigeon                           (new Pigeon2(PIGEON_CAN_ID, m_CanivoreBus)),
    m_pSwerveDrive                      (new SwerveDrive(m_pPigeon, GetCanBusReferenceLambda)),

    //motor initialization          
    m_pShooterHood                      (new TalonFxMotorController(SHOOTER_HOOD_CAN_ID, m_RioCanBus)),
    m_pShooterFeed                      (new TalonFxMotorController(SHOOTER_FEED_CAN_ID, m_RioCanBus)),
    m_pIntake                           (new TalonFxMotorController(INTAKE_CAN_ID, m_RioCanBus)),
    m_pIntakePivot                      (new TalonFxMotorController(INTAKE_PIVOT_CAN_ID, m_RioCanBus)),
    m_pHopperFeed                       (new TalonFxMotorController(HOPPER_FEED_CAN_ID, m_RioCanBus)),

    //motor group for all three shooting motors, this is intentionally set to coast due to the flywheels
    m_pShooterMotors                    (new TalonMotorGroup<TalonFX>("Shooter Motors", THREE_MOTORS, SHOOTER_MOTORS_START_CAN_ID, MotorGroupControlMode::FOLLOW, NeutralModeValue::Coast, m_RioCanBus, false)),
    
    m_pCandle                           (new CANdle(CANDLE_CAN_ID, m_CanivoreBus)),
    m_LedStripSolidColor                (0, (NUMBER_OF_LEDS - 1)),
    m_RainbowAnimation                  (0, (NUMBER_OF_LEDS - 1)),
    m_pDebugOutput                      (new DigitalOutput(DEBUG_OUTPUT_DIO_CHANNEL)),
    m_pCompressor                       (new Compressor(PneumaticsModuleType::CTREPCM)),

    //encoder initialization
    m_pHoodCanCoder                     (new CANcoder(HOOD_CANCODER_CAN_ID)),
    m_pIntakePivotCanCoder              (new CANcoder(INTAKE_PIVOT_CANCODER_CAN_ID)),
   
    m_pMatchModeTimer                   (new Timer()),
    m_pRobotProgramTimer                (new Timer()),
    m_pSafetyTimer                      (new Timer()),
    m_CameraThread                      (RobotCamera::LimelightThread),
    m_RobotMode                         (ROBOT_MODE_NOT_SET),
    m_AllianceColor                     (DriverStation::GetAlliance()),
    m_bRioPinsStable                    (false),
    m_bCameraAlignInProgress            (false),
    m_HoodAngleDegrees                  (0.0_deg),
    m_HeartBeat                         (0U)
{
    RobotUtils::DisplayMessage("Robot constructor.");
    
    // LiveWindow is not used
    LiveWindow::SetEnabled(false);
    
    // Set the autonomous options
    m_AutonomousChooser.SetDefaultOption(AUTO_ROUTINE_1_STRING, AUTO_ROUTINE_1_STRING);
    m_AutonomousChooser.AddOption(AUTO_ROUTINE_2_STRING, AUTO_ROUTINE_2_STRING);
    m_AutonomousChooser.AddOption(AUTO_ROUTINE_3_STRING, AUTO_ROUTINE_3_STRING);
    m_AutonomousChooser.AddOption(AUTO_NO_ROUTINE_STRING, AUTO_NO_ROUTINE_STRING);
    m_AutonomousChooser.AddOption(AUTO_TEST_ROUTINE_STRING, AUTO_TEST_ROUTINE_STRING);
    SmartDashboard::PutData("Autonomous Modes", &m_AutonomousChooser);
    
    RobotUtils::DisplayFormattedMessage("The drive forward axis is: %d\n", EastTech::Controller::Config::GetControllerMapping(DRIVE_CONTROLLER_MODEL)->AXIS_MAPPINGS.RIGHT_TRIGGER);
    RobotUtils::DisplayFormattedMessage("The drive reverse axis is: %d\n", EastTech::Controller::Config::GetControllerMapping(DRIVE_CONTROLLER_MODEL)->AXIS_MAPPINGS.LEFT_TRIGGER);
    RobotUtils::DisplayFormattedMessage("The drive left/right axis is: %d\n", EastTech::Controller::Config::GetControllerMapping(DRIVE_CONTROLLER_MODEL)->AXIS_MAPPINGS.LEFT_X_AXIS);

    CANdleConfiguration candleConfig;
    candleConfig.LED.StripType = StripTypeValue::RGBW;
    m_pCandle->GetConfigurator().Apply(candleConfig);
    m_pCandle->SetControl(m_RainbowAnimation);

    // Spawn the vision thread
    RobotCamera::SetLimelightMode(RobotCamera::LimelightMode::DRIVER_CAMERA);
    RobotCamera::SetLimelightLedMode(RobotCamera::LimelightLedMode::PIPELINE);
    m_CameraThread.detach();

    // Start the free running timer
    m_pRobotProgramTimer->Reset();
    m_pRobotProgramTimer->Start();
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::ResetMemberData
///
/// This method resets relevant member data variables.  Since
/// the robot object is only constructed once, it may be
/// necessary/helpful to return to a state similar to when the
/// constructor first ran (e.g. when enabling/disabling robot
/// states).  Only variables that need to be reset are modified
/// here.  This also works around the issue where non-member
/// static data cannot be easily reinitialized (since clearing
/// the .bss and running static constructors will only happen
/// once on program start up).
///
////////////////////////////////////////////////////////////////
void EastTechRobot::ResetMemberData()
{
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::RobotInit
///
/// This method is run when initializing the robot.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::RobotInit()
{
    RobotUtils::DisplayMessage("RobotInit called.");
    SetStaticThisInstance();
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::RobotPeriodic
///
/// This method is run in all robot states.  It is called each
/// time a new packet is received from the driver station.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::RobotPeriodic()
{
    static bool bRobotPeriodicStarted = false;
    if (!bRobotPeriodicStarted)
    {
        RobotUtils::DisplayMessage("RobotPeriodic called.");
        bRobotPeriodicStarted = true;
    }

    // @todo: Read and display sensor values for calibration when not enabled
    // @note: From testing, smart dashboard prints of sensor values do give real time data.
    CheckIfRioPinsAreStable();
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::CheckIfRioPinsAreStable
///
/// Waits for any sensors on the robot that route to the RIO
/// to stabilize for accurate readings.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::CheckIfRioPinsAreStable()
{
/*
    // This is the logic to wait to take PWM based sensor readings until the RIO is ready.
    // The behavior of the RIO is that it measures how many microseconds the signal is high
    // every second.  This requires waiting to get stable readings.
    // See https://github.com/wpilibsuite/allwpilib/issues/5284 for some related info.
    static units::time::second_t enabledTimeStamp = 0.0_s;
    units::time::second_t currentTimeStamp = m_pRobotProgramTimer->Get();
    if (DriverStation::IsEnabled() && (!m_bRioPinsStable))
    {
        // If the robot was just enabled (in any mode)
        if (enabledTimeStamp == 0.0_s)
        {
            // Set the start time stamp
            enabledTimeStamp = currentTimeStamp;
        }

        // Now check if enough time has passed for the RIO pins to have stabilized
        static constexpr const units::time::second_t RIO_DUTY_CYCLE_ENCODER_STARTUP_DELAY = 2.0_s;
        if ((currentTimeStamp - enabledTimeStamp) > RIO_DUTY_CYCLE_ENCODER_STARTUP_DELAY)
        {
            // Intake pivor encoder configuration algorithm
            double encoderValue = m_pIntakePivotEncoder->Get();
            units::angle::degree_t encoderValueDegrees(encoderValue * ANGLE_360_DEGREES);
          
            // This is the delta between the current mechanism position and the desired starting position (or zero point)
            units::angle::degree_t startingOffsetDegrees = encoderValueDegrees - INTAKE_STARTING_POSITION_DEGREES;
            std::printf("startingOffsetDegrees (start): %f\n", startingOffsetDegrees.value());

            // If the starting offset is negative, we crossed over the absolute encoder boundary
            // We give a tolerance of five degrees in case the mechanism is near where we want to start
            // @todo: Does this need to check for very small readings below zero?
            // @todo: Boundary conditions here will be difficult
            //if (startingOffsetDegrees < ENCODER_BOUNDARY_TOLERANCE_DEGREES)
            //{
                // the 0/1 boundary is 360, so subtract the starting position to see how many degrees were up to that point
                // Add in the absolute value of the overage, which was negative
                //startingOffsetDegrees = (units::angle::degree_t(ANGLE_360_DEGREES) - STARTING_POSITION_ENCODER_VALUE) + encoderValueDegrees;
            //}

            // At this point we have the angle we want relative to zero
            (void)m_pIntakePivot->m_pTalonFx->GetConfigurator().SetPosition(startingOffsetDegrees);
            std::printf("encoderValue: %f\n", encoderValue);
            std::printf("encoderValueDegrees: %f\n", encoderValueDegrees.value());
            std::printf("startingOffsetDegrees (final): %f\n", startingOffsetDegrees.value());

             // Hood encoder configuration algorithm

            encoderValue = m_pHoodEncoder->Get();
            encoderValueDegrees = units::angle::degree_t(encoderValue * ANGLE_360_DEGREES);
          
            // This is the delta between the current mechanism position and the desired starting position (or zero point)
            startingOffsetDegrees = encoderValueDegrees - HOOD_STARTING_POSITION_DEGREES;
            std::printf("startingOffsetDegrees (start): %f\n", startingOffsetDegrees.value());

            // If the starting offset is negative, we crossed over the absolute encoder boundary
            // We give a tolerance of five degrees in case the mechanism is near where we want to start
            // @todo: Does this need to check for very small readings below zero?
            // @todo: Boundary conditions here will be difficult
            //if (startingOffsetDegrees < ENCODER_BOUNDARY_TOLERANCE_DEGREES)
            //{
                // the 0/1 boundary is 360, so subtract the starting position to see how many degrees were up to that point
                // Add in the absolute value of the overage, which was negative
                //startingOffsetDegrees = (units::angle::degree_t(ANGLE_360_DEGREES) - STARTING_POSITION_ENCODER_VALUE) + encoderValueDegrees;
            //}

            // At this point we have the angle we want relative to zero
            (void)m_pShooterHood->m_pTalonFx->GetConfigurator().SetPosition(startingOffsetDegrees);
            std::printf("encoderValue: %f\n", encoderValue);
            std::printf("encoderValueDegrees: %f\n", encoderValueDegrees.value());
            std::printf("startingOffsetDegrees (final): %f\n", startingOffsetDegrees.value());

            m_bRioPinsStable = true;
        }
    }
    else
    {
        // Set the enabled time stamp back to zero until the robot is enabled again
        enabledTimeStamp = 0.0_s;

        // m_bRioPinsStable exists for the life of the program.  Once we have a stable
        // reading acquired, we don't need to do it again until the robot program restarts.
    }
*/
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::ConfigureMotorControllers
///
/// Sets motor controller specific configuration information.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::ConfigureMotorControllers()
{
    // These are the defaults for the configuration (see TalonFX.h)
    //ctre::phoenix::sensors::AbsoluteSensorRange absoluteSensorRange = ctre::phoenix::sensors::AbsoluteSensorRange::Unsigned_0_to_360;
    //double integratedSensorOffsetDegrees = 0;
    //ctre::phoenix::sensors::SensorInitializationStrategy initializationStrategy = ctre::phoenix::sensors::SensorInitializationStrategy::BootToZero;

    // The default constructor for TalonFXConfiguration will call the parent
    // BaseTalonConfiguration constructor with FeedbackDevice::IntegratedSensor.

    /*
    // Example configuration
    TalonFXConfiguration talonConfig;
    talonConfig.slot0.kP = 0.08;
    talonConfig.slot0.kI = 0.0;
    talonConfig.slot0.kD = 0.3;
    talonConfig.slot0.kF = 0.0;
    talonConfig.absoluteSensorRange = AbsoluteSensorRange::Unsigned_0_to_360;
    talonConfig.integratedSensorOffsetDegrees = 0.0;
    talonConfig.initializationStrategy = SensorInitializationStrategy::BootToZero;
    talonConfig.peakOutputForward = 1.0;
    talonConfig.peakOutputReverse = 1.0;
    talonConfig.slot0.closedLoopPeakOutput = 0.10;

    TalonFX * pTalon = new TalonFX(0xFF);
    pTalon->ConfigFactoryDefault();
    pTalon->ConfigAllSettings(talonConfig);
    pTalon->SetSelectedSensorPosition(0);
    const StatorCurrentLimitConfiguration INTAKE_MOTOR_STATOR_CURRENT_LIMIT_CONFIG = {true, 5.0, 50.0, 5.0};
    pTalon->ConfigStatorCurrentLimit(INTAKE_MOTOR_STATOR_CURRENT_LIMIT_CONFIG);
    */

    // Some notes about applying motor configurations:
    // - The classes/structs in YtaTalon.hpp have motor configuration objects in them.
    // - Declaring stack local or class scope configuration objects are *separate and
    //   distinct* from the configuration objects in the YtaTalon.hpp classes/structs.
    // - If a stack local or class scope configuration is applied, it will overwrite
    //   the configuration stored in the device.
    // - Calling the methods provided by YtaTalon.hpp *never* update the configuration
    //   objects in the classes/structs.  To update those objects, retrieve the objects
    //   via things like m_MotorConfiguration (for individual motors) or
    //   GetMotorConfiguration() (for motor groups).
    // - The classes/structs in YtaTalon.hpp provide ApplyConfiguration() routines.
    //   These can be used to directly apply a stack local or class scope configuration,
    //   or to apply an updated configuration when the configuration objects were directly
    //   modified.  Keep the notes above in mind when calling them.
    // - The ApplyConfiguration() method for motor groups will default to applying the
    //   configuration to all motors unless a specific CAN ID is given.  The configuration
    //   applied to each motor in the group is the *saved configuration* for that specific
    //   motor.  It may be different for each motor, depending on the robot code.
    // - Configurations can be applied to the whole configuration object type, or to
    //   sub-types only (e.g. TalonFXConfiguration vs. CurrentLimitsConfigs), as
    //   ApplyConfiguration() is overloaded.  The template version only applies stack
    //   local or class scope configs, so remember the notes above.  Right now the
    //   template version is disabled, so don't call it.
    // - Thank CTRE for all this.  Instead of letting the config be a member of the motor
    //   object class with simple getter/setters, it's separate and overly complex.

    // Example configurations

    // Configure a motor group (only needs to be applied to the lead motor of the group)
    // Brake mode was set when the motor group was constructed
    //(void)m_pMotors->GetMotorConfiguration(MOTORS_CAN_START_ID)->Feedback.WithSensorToMechanismRatio(12.0 / 1.0);
    //(void)m_pMotors->GetMotorConfiguration(MOTORS_CAN_START_ID)->Slot0.WithKP(18.0).WithKI(0.0).WithKD(0.1);
    //m_pMotors->ApplyConfiguration(MOTORS_CAN_START_ID);
    //(void)m_pMotors->GetMotorObject(MOTORS_CAN_START_ID)->GetConfigurator().SetPosition(0.0_tr);

    // Configure a single motor
    //(void)m_pMotor->m_MotorConfiguration.MotorOutput.WithNeutralMode(NeutralModeValue::Brake);
    //(void)m_pMotor->m_MotorConfiguration.Feedback.WithSensorToMechanismRatio(135.0 / 1.0);
    //(void)m_pMotor->m_MotorConfiguration.Slot0.WithKP(18.0).WithKI(0.0).WithKD(0.1);
    //(void)m_pMotor->m_pTalonFx->GetConfigurator().SetPosition(0.0_tr);
    //m_pMotor->ApplyConfiguration();

    //Configuration for Shooter motors x3 (no changes from default right now)
    //(void)m_pShooterMotors->ApplyConfiguration(SHOOTER_MOTORS_START_CAN_ID);

    //Configuration for shooter feed motor (no changes from default right now)
    //m_pShooterFeed->ApplyConfiguration();

    //Configuration for intake motor (no changes from default right now)
    //m_pIntake->ApplyConfiguration();

    //Configuration for hopper feed motor (no changes from default right now)
    //m_pHopperFeed->ApplyConfiguration();



    //Configuration for shooter hood motor
    (void)m_pShooterHood->m_MotorConfiguration.Feedback.WithSensorToMechanismRatio(226.6667 / 1.0);
    (void)m_pShooterHood->m_MotorConfiguration.Slot0.WithKP(36.0).WithKI(0.0).WithKD(0.1);
    (void)m_pShooterHood->m_MotorConfiguration.MotorOutput.WithNeutralMode(NeutralModeValue::Brake);
    (void)m_pShooterHood->m_MotorConfiguration.MotorOutput.WithInverted(InvertedValue::Clockwise_Positive);
    m_pShooterHood->ApplyConfiguration();

    // These haven't been measured for 8145 yet.
    // Full down: 0.602278 (216.82_deg), full up: 0.701889 (252.68_deg)
    // Starting position: 0.0 (0.0_deg)
    constexpr const units::angle::degree_t HOOD_STARTING_ANGLE_CANCODER_REF = 227.0_deg;
    units::angle::degree_t hoodCanCoderDegrees = m_pHoodCanCoder->GetAbsolutePosition().GetValue();
    units::angle::degree_t hoodAngleDelta = hoodCanCoderDegrees - HOOD_STARTING_ANGLE_CANCODER_REF;
    SmartDashboard::PutNumber("Hood delta", hoodAngleDelta.value());

    // If delta is positive, we are above the expected starting point
    //      Upper limit is ~250.0_deg - 227.0_deg = ~23.0_deg
    // If delta is negative, we are below the expected starting point
    //      Lower limit is ~220.0_deg - 227.0_deg = ~-7.0_deg

    units::angle::turn_t hoodSetPositionTurns = hoodAngleDelta;
    (void)m_pShooterHood->m_pTalonFx->GetConfigurator().SetPosition(hoodSetPositionTurns);
    // Return the hood to the middle (in case it wasn't)
    m_HoodAngleDegrees = HOOD_START_OR_TOWER_ANGLE_DEGREES;
    m_pShooterHood->SetPositionVoltage(m_HoodAngleDegrees.value());



    //Configuration for intake pivot motor
    (void)m_pIntakePivot->m_MotorConfiguration.Feedback.WithSensorToMechanismRatio(39.6 / 1.0);
    (void)m_pIntakePivot->m_MotorConfiguration.Slot0.WithKP(18.0).WithKI(0.0).WithKD(0.1);
    (void)m_pIntakePivot->m_MotorConfiguration.MotorOutput.WithNeutralMode(NeutralModeValue::Brake);
    (void)m_pIntakePivot->m_MotorConfiguration.MotorOutput.WithInverted(InvertedValue::Clockwise_Positive);
    m_pIntakePivot->ApplyConfiguration();

    // Full down: 0.531982 (191.51352), 0.522461 (188.08596_deg), full up: 0.248779 (89.56044), 0.230469 (82.96884_deg)
    constexpr const units::angle::degree_t INTAKE_STARTING_ANGLE_CANCODER_REF = 94.0_deg;
    units::angle::degree_t intakeCanCoderDegrees = m_pIntakePivotCanCoder->GetAbsolutePosition().GetValue();
    units::angle::degree_t intakeAngleDelta = intakeCanCoderDegrees - INTAKE_STARTING_ANGLE_CANCODER_REF;
    SmartDashboard::PutNumber("Intake delta", intakeAngleDelta.value());

    // If delta is positive, we are below the expected starting point
    // If delta is negative, we are above the expected starting point

    units::angle::turn_t intakeSetPositionTurns = intakeAngleDelta;
    (void)m_pIntakePivot->m_pTalonFx->GetConfigurator().SetPosition(intakeSetPositionTurns);
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::InitialStateSetup
///
/// This method contains the work flow for putting motors,
/// solenoids, etc. into a known state.  It is intended to be
/// used by both autonomous and user control.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::InitialStateSetup()
{
    // First reset any member data
    ResetMemberData();

    ConfigureMotorControllers();

    // Stop/clear any timers, just in case
    // @todo: Make this a dedicated function.
    m_pMatchModeTimer->Stop();
    m_pMatchModeTimer->Reset();
    m_pSafetyTimer->Stop();
    m_pSafetyTimer->Reset();
    
    // Just in case constructor was called before these were set (likely the case)
    m_AllianceColor = DriverStation::GetAlliance();

    // Set the LEDs to the alliance color
    SetLedsToAllianceColor();

    // Indicate the camera thread can continue
    RobotCamera::ReleaseThread();

    // Clear the debug output pin
    m_pDebugOutput->Set(false);

    // Reset the heartbeat
    m_HeartBeat = 0U;

    // Point the swerve modules straight.  With SparkMax, this (also) addresses
    // an issue where setting position during constructors doesn't take effect.
    m_pSwerveDrive->HomeModules();

    // With CTRE swerve electronics, sometimes the CANcoder appears to not be
    // ready when constructors measure the absolute position.  The issue isn't
    // entirely understood, but recalibrating here seems to provide stability.
    // Note: This won't work if Neo swerve is used.
    m_pSwerveDrive->RecalibrateModules();
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::TeleopInit
///
/// The teleop init method.  This method is called once each
/// time the robot enters teleop control.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::TeleopInit()
{
    RobotUtils::DisplayMessage("TeleopInit called.");
    
    // Autonomous should have left things in a known state, but
    // just in case clear everything.
    InitialStateSetup();

    // Tele-op won't do detailed processing of the images unless instructed to
    RobotCamera::SetFullProcessing(false);
    RobotCamera::SetLimelightMode(RobotCamera::LimelightMode::DRIVER_CAMERA);
    RobotCamera::SetLimelightLedMode(RobotCamera::LimelightLedMode::PIPELINE);

    // Start the mode timer for teleop
    m_pMatchModeTimer->Start();
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::TeleopPeriodic
///
/// The teleop control method.  This method is called
/// periodically while the robot is in teleop control.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::TeleopPeriodic()
{
    // Log a mode change if one occurred
    CheckAndUpdateRobotMode(ROBOT_MODE_TELEOP);

    HeartBeat();

    if (EastTech::Drive::Config::USE_SWERVE_DRIVE)
    {
        SwerveDriveSequence();
    }

    IntakeSequence();
    ShooterSequence();

    //PneumaticSequence();
    
    CameraSequence();

    UpdateSmartDashboard();
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::UpdateSmartDashboard
///
/// Updates values in the smart dashboard.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::UpdateSmartDashboard()
{
    // @todo: Check if RobotPeriodic() is called every 20ms and use static counter.

    units::time::second_t matchTime = 0.0_s;
    double batteryVoltage = DriverStation::GetBatteryVoltage();
    std::string gameData = DriverStation::GetGameSpecificMessage();

    if (DriverStation::IsFMSAttached())
    {
        matchTime = DriverStation::GetMatchTime();
    }
    else
    {
        matchTime = m_pMatchModeTimer->Get();
    }

    struct HubShift
    {
        bool m_Transition;
        bool m_bShift1;
        bool m_bShift2;
        bool m_bShift3;
        bool m_bShift4;
        bool m_EndGame;
    };
    constexpr const HubShift ACTIVE_FIRST = {true, true, false, true, false, true};
    constexpr const HubShift INACTIVE_FIRST = {true, false, true, false, true, true};

    static bool bGotGameData = false;
    static HubShift allianceHubShift;

    // Look for the game data to be ready
    if (!bGotGameData)
    {
        bool bInactiveFirst = false;
        if (!gameData.empty())
        {
            // For some reason the game data is who is *inactive* first (instead of active)
            bInactiveFirst = (((gameData.at(0U) == 'R') && (m_AllianceColor == DriverStation::kRed)) ||
                              ((gameData.at(0U) == 'B') && (m_AllianceColor == DriverStation::kBlue)));
        }

        allianceHubShift = bInactiveFirst ? INACTIVE_FIRST : ACTIVE_FIRST;
        bGotGameData = true;
    }

    // Auto: 20_s, Teleop: 110_s, End Game: 30_s (Driver Control Total: 140_s or 2m20s)
    constexpr const units::time::second_t TRANSITION_END_TIME_S = 130_s;
    constexpr const units::time::second_t SHIFT_1_END_TIME_S = 105_s;
    constexpr const units::time::second_t SHIFT_2_END_TIME_S = 80_s;
    constexpr const units::time::second_t SHIFT_3_END_TIME_S = 55_s;
    constexpr const units::time::second_t SHIFT_4_END_TIME_S = 30_s;

    bool bHubActive = false;
    units::time::second_t shiftTime = 0.0_s;
    if (matchTime > TRANSITION_END_TIME_S)
    {
        bHubActive = allianceHubShift.m_Transition;
        shiftTime = matchTime - TRANSITION_END_TIME_S;
    }
    else if (matchTime > SHIFT_1_END_TIME_S)
    {
        bHubActive = allianceHubShift.m_bShift1;
        shiftTime = matchTime - SHIFT_1_END_TIME_S;
    }
    else if (matchTime > SHIFT_2_END_TIME_S)
    {
        bHubActive = allianceHubShift.m_bShift2;
        shiftTime = matchTime - SHIFT_2_END_TIME_S;
    }
    else if (matchTime > SHIFT_3_END_TIME_S)
    {
        bHubActive = allianceHubShift.m_bShift3;
        shiftTime = matchTime - SHIFT_3_END_TIME_S;
    }
    else if (matchTime > SHIFT_4_END_TIME_S)
    {
        bHubActive = allianceHubShift.m_bShift4;
        shiftTime = matchTime - SHIFT_4_END_TIME_S;
    }
    else
    {
        bHubActive = allianceHubShift.m_EndGame;
        shiftTime = matchTime;
    }

    // Give the drive team some state information
    SmartDashboard::PutBoolean("RIO pins stable", m_bRioPinsStable);
    SmartDashboard::PutNumber("Battery voltage", batteryVoltage);
    SmartDashboard::PutNumber("Match time", matchTime.value());
    SmartDashboard::PutNumber("Shift time", shiftTime.value());
    SmartDashboard::PutBoolean("Hub active", bHubActive);
}



//////////////////////////////////////////////////////////////////
/// @method EastTechRobot::IntakeSequence
///
/// This method handles all intake related behavior. This includes
/// the intake rollers and movement of the physical intake via
/// throughbore encoder.
///
/////////////////////////////////////////////////////////////////
void EastTechRobot::IntakeSequence()
{
    //Turning the motors on and off for intake/outtake
    if (m_pAuxController->GetButtonState(INTAKE_BUTTON)) //intake fuel
    {
        m_pIntake->SetDutyCycle(INTAKE_PIECE_MOTOR_SPEED); 
    }
    else if (m_pAuxController->GetButtonState(OUTTAKE_BUTTON)) //outtake fuel
    {
        m_pIntake->SetDutyCycle(OUTTAKE_PIECE_MOTOR_SPEED);
    }
    else
    {
        m_pIntake->SetDutyCycle(0.0);
    }

    //establishing variable for intake position on dashboard
    static bool bIntakeUp = true;

    //pivoting intake per the throughbore encoder
    if (m_pAuxController->DetectButtonChange(INTAKE_PIVOT_UP_BUTTON))
    {
        bIntakeUp = true;

        //set intake motor to reference angle for up
        (void)m_pIntakePivot->SetPositionVoltage(INTAKE_UP_POSITION_DEGREES.value());
    }
    else if (m_pAuxController->DetectButtonChange(INTAKE_PIVOT_DOWN_BUTTON))
    {
        bIntakeUp = false;

        //set intake motor to reference angle for down
        (void)m_pIntakePivot->SetPositionVoltage(INTAKE_DOWN_POSITION_DEGREES.value());
    }
    else
    {
    }

    //Visual indicator for position of the intake
    units::angle::degree_t intakeCanCoderDegrees = m_pIntakePivotCanCoder->GetAbsolutePosition().GetValue();
    units::angle::degree_t intakeFxDegrees = m_pIntakePivot->m_pTalonFx->GetPosition().GetValue();
    SmartDashboard::PutBoolean("Intake Up?", bIntakeUp);
    SmartDashboard::PutNumber("Intake CANcoder", intakeCanCoderDegrees.value());
    SmartDashboard::PutNumber("Intake FX", intakeFxDegrees.value());
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::ShooterSequence
///
/// This method handles any shooting related behavior. This
/// includes shooter spin up, operation of the hopper and
/// shooter feeder.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::ShooterSequence()
{
    // Check for a request to change the hood position (outside of passing, which is fixed)
    static bool bHoodAtMidRangePosition = false;
    if (m_pAuxController->DetectButtonChange(AUTOMATIC_HOOD_ADJUST))
    {
        bHoodAtMidRangePosition = !bHoodAtMidRangePosition;
    }

    // Check for a request to change passing state
    static bool bPassing = false;
    if (m_pAuxController->DetectButtonChange(PASSING_SHOOTING_CHANGE))
    {
        bPassing = !bPassing;
    }

    // Set the hood angle based on current state
    m_HoodAngleDegrees = bPassing ? HOOD_PASSING_ANGLE_DEGREES : (bHoodAtMidRangePosition ? HOOD_SHOOT_MID_RANGE_ANGLE_DEGREES : HOOD_START_OR_TOWER_ANGLE_DEGREES);
    m_pShooterHood->SetPositionVoltage(m_HoodAngleDegrees.value());
    SmartDashboard::PutNumber("Hood target", m_HoodAngleDegrees.value());
    units::angle::degree_t hoodCanCoderDegrees = m_pHoodCanCoder->GetAbsolutePosition().GetValue();
    SmartDashboard::PutNumber("Hood CANcoder", hoodCanCoderDegrees.value());

    // Shooter motors speed change based on passing
    static double shooterMotorSpeed = 0.0;
    shooterMotorSpeed = bPassing ? SHOOTER_PASSING_MOTOR_SPEED : SHOOTER_MOTOR_SPEED;

    // Allow a manual pre-shot ramp up
    static bool bPreShoot = false;
    static bool bShotInProgress;
    static Timer shootTimer;
    static units::time::second_t shootTimeStamp = 0.0_s;
    if (m_pAuxController->GetAxisValue(PRE_SHOOT_AXIS) > AXIS_INPUT_DEAD_BAND)
    {
        m_pShooterMotors->Set(shooterMotorSpeed);
        bPreShoot = true;
        bShotInProgress = true;
    }
    else
    {
        bPreShoot = false;
    }

    // Control sequence for shooting or passing
    if (m_pAuxController->GetAxisValue(SHOOT_AXIS) > AXIS_INPUT_DEAD_BAND)
    {
        // A shot is requested, check if we are already in progress
        if (!bShotInProgress)
        {
            shootTimer.Reset();
            shootTimer.Start();
            m_pShooterMotors->Set(shooterMotorSpeed);
            shootTimeStamp = shootTimer.Get();
            bShotInProgress = true;
        }
        // If the ramp time is done, or if pre-shoot was requested
        else if (((shootTimer.Get() - shootTimeStamp) > 1.0_s) || bPreShoot)
        {
            // Turn on motors for fuel movement
            m_pHopperFeed->SetDutyCycle(HOPPER_FEED_MOTOR_SPEED);
            m_pShooterFeed->SetDutyCycle(SHOOTER_FEED_MOTOR_SPEED);
        }
        else
        {
        }
    }
    else
    {
        // No request, motors off
        m_pHopperFeed->SetDutyCycle(0.0);
        m_pShooterFeed->SetDutyCycle(0.0);

        // Only disable the shoot motor if pre-shoot is not active
        if (!bPreShoot)
        {
            m_pShooterMotors->Set(0.0);
            bShotInProgress = false;
        }
    }

    SmartDashboard::PutBoolean("Passing?", bPassing);
    SmartDashboard::PutBoolean("Shooting close?", bHoodAtMidRangePosition);
    SmartDashboard::PutNumber("Shooter RPM", m_pShooterMotors->GetMotorObject()->GetVelocity().GetValue().value());
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::PneumaticSequence
///
/// This method contains the main workflow for updating the
/// state of the pnemuatics on the robot.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::PneumaticSequence()
{
    // @todo: Monitor other compressor API data?
    SmartDashboard::PutBoolean("Compressor status", m_pCompressor->IsEnabled());
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::CameraSequence
///
/// This method handles camera related behavior.  See the
/// RobotCamera class for full details.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::CameraSequence()
{
    if (m_pDriveController->GetButtonState(DRIVE_ALIGN_WITH_CAMERA_BUTTON))
    {
        m_bCameraAlignInProgress = true;
        RobotCamera::AutonomousCamera::AlignToTargetSwerve(m_pPigeon->GetYaw().GetValue().value());
    }
    else
    {
        m_bCameraAlignInProgress = false;
    }
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::SwerveDriveSequence
///
/// This method contains the main workflow for swerve drive
/// control.  It will gather input from the drive joystick and
/// then filter those values to ensure they are past a certain
/// threshold (deadband) and generate the information to pass
/// on to the swerve drive system.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::SwerveDriveSequence()
{
    // Check for a switch between field relative and robot centric
    static bool bFieldRelative = true;
    if (m_pDriveController->DetectButtonChange(FIELD_RELATIVE_TOGGLE_BUTTON))
    {
        bFieldRelative = !bFieldRelative;
    }

    if (m_pDriveController->DetectButtonChange(REZERO_SWERVE_BUTTON))
    {
        m_pSwerveDrive->ZeroGyroYaw();
        m_pSwerveDrive->RecalibrateModules();
        m_pSwerveDrive->HomeModules();
    }

    if (m_pDriveController->DetectButtonChange(LOCK_SWERVE_WHEELS_BUTTON))
    {
        m_pSwerveDrive->LockWheels();
    }

    // The GetDriveX() and GetDriveYInput() functions refer to ***controller joystick***
    // x and y axes.  Multiply by -1.0 here to keep the joystick input retrieval code common.
    double translationAxis = RobotUtils::Trim(m_pDriveController->GetDriveYInput() * -1.0, JOYSTICK_TRIM_UPPER_LIMIT, JOYSTICK_TRIM_LOWER_LIMIT);
    double strafeAxis = RobotUtils::Trim(m_pDriveController->GetDriveXInput() * -1.0, JOYSTICK_TRIM_UPPER_LIMIT, JOYSTICK_TRIM_LOWER_LIMIT);
    double rotationAxis = RobotUtils::Trim(m_pDriveController->GetDriveRotateInput() * -1.0, JOYSTICK_TRIM_UPPER_LIMIT, JOYSTICK_TRIM_LOWER_LIMIT);

    // Override normal control if a fine positioning request is made
    switch (m_pDriveController->GetPovAsDirection())
    {
        case EastTech::Controller::PovDirections::POV_UP:
        {
            translationAxis = SWERVE_DRIVE_SLOW_SPEED;
            strafeAxis = 0.0;
            rotationAxis = 0.0;
            break;
        }
        case EastTech::Controller::PovDirections::POV_DOWN:
        {
            translationAxis = -SWERVE_DRIVE_SLOW_SPEED;
            strafeAxis = 0.0;
            rotationAxis = 0.0;
            break;
        }
        case EastTech::Controller::PovDirections::POV_LEFT:
        {
            // Left/right POV control can either toggle strafe or rotation
            translationAxis = 0.0;
            strafeAxis = (EastTech::Drive::Config::SWERVE_SLOW_USE_ROTATION_AXIS) ? (0.0) : (SWERVE_DRIVE_SLOW_SPEED);
            rotationAxis = (EastTech::Drive::Config::SWERVE_SLOW_USE_ROTATION_AXIS) ? (SWERVE_ROTATE_SLOW_SPEED) : (0.0);
            break;
        }
        case EastTech::Controller::PovDirections::POV_RIGHT:
        {
            // Left/right POV control can either toggle strafe or rotation
            translationAxis = 0.0;
            strafeAxis = (EastTech::Drive::Config::SWERVE_SLOW_USE_ROTATION_AXIS) ? (0.0) : (-SWERVE_DRIVE_SLOW_SPEED);
            rotationAxis = (EastTech::Drive::Config::SWERVE_SLOW_USE_ROTATION_AXIS) ? (-SWERVE_ROTATE_SLOW_SPEED) : (0.0);
            break;
        }
        default:
        {
            break;
        }
    }

    SmartDashboard::PutNumber("Strafe", strafeAxis);
    SmartDashboard::PutNumber("Translation", translationAxis);
    SmartDashboard::PutNumber("Rotation", rotationAxis);
    SmartDashboard::PutBoolean("Field Relative", bFieldRelative);

    // Notice that this is sending translation to X and strafe to Y, despite
    // the inputs coming from the opposite of what may be intuitive (strafe as X,
    // translation as Y).  See the comment in Translation2d.h about the robot
    // placed at origin facing the X-axis.  Forward movement increases X and left
    // movement increases Y.
    Translation2d translation = {units::meter_t(translationAxis), units::meter_t(strafeAxis)};

    // Update the swerve module states
    m_pSwerveDrive->SetModuleStates(translation, rotationAxis, bFieldRelative, true);

    // Update the odometry
    m_pSwerveDrive->UpdateOdometry();

    // Display some useful information
    m_pSwerveDrive->UpdateSmartDashboard();
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::DisabledInit
///
/// The disabled init method.  This method is called once each
/// time the robot enters disabled mode.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::DisabledInit()
{
    RobotUtils::DisplayMessage("DisabledInit called.");

    // @todo: Shut off the limelight LEDs?
    RobotCamera::SetLimelightMode(RobotCamera::LimelightMode::DRIVER_CAMERA);
    RobotCamera::SetLimelightLedMode(RobotCamera::LimelightLedMode::PIPELINE);

    // Turn the rainbow animation back on    
    m_pCandle->SetControl(m_RainbowAnimation);
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::DisabledPeriodic
///
/// The disabled control method.  This method is called
/// periodically while the robot is disabled.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::DisabledPeriodic()
{
    // Log a mode change if one occurred
    CheckAndUpdateRobotMode(ROBOT_MODE_DISABLED);
}



////////////////////////////////////////////////////////////////
/// @method main
///
/// Execution start for the robt.
///
////////////////////////////////////////////////////////////////
#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<EastTechRobot>();
}
#endif
