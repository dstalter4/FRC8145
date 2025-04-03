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
/// Copyright (c) 2025 East Technical High School
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
    m_pPigeon                           (new Pigeon2(PIGEON_CAN_ID, "canivore-8145")),
    m_pSwerveDrive                      (new SwerveDrive(m_pPigeon)),
    m_pLiftMotors                       (new TalonMotorGroup<TalonFX>("Lift motors", TWO_MOTORS, LIFT_MOTORS_CAN_START_ID, MotorGroupControlMode::FOLLOW_INVERSE, NeutralModeValue::Brake, false)),
    m_pArmPivotMotor                    (new TalonFxMotorController(ARM_PIVOT_MOTOR_CAN_ID)),
    m_pWristPivotMotor                  (new TalonFxMotorController(WRIST_PIVOT_MOTOR_CAN_ID)),
    m_pGamePieceMotor                   (new TalonFxMotorController(GAME_PIECE_MOTOR_CAN_ID)),
    m_pHangMotor                        (new TalonFxMotorController(HANG_MOTOR_CAN_ID)),
    m_pCandle                           (new CANdle(CANDLE_CAN_ID, "canivore-8145")),
    m_RainbowAnimation                  ({1, 0.5, 308}),
    m_pDebugOutput                      (new DigitalOutput(DEBUG_OUTPUT_DIO_CHANNEL)),
    m_pCompressor                       (new Compressor(PneumaticsModuleType::CTREPCM)),
    m_pArmAbsoluteEncoder               (new DutyCycleEncoder(ARM_ABSOLUTE_ENCODER_DIO_CHANNEL)),
    m_pWristAbsoluteEncoder             (new DutyCycleEncoder(WRIST_ABSOLUTE_ENCODER_DIO_CHANNEL)),
    m_pMatchModeTimer                   (new Timer()),
    m_pRobotProgramTimer                (new Timer()),
    m_pSafetyTimer                      (new Timer()),
    m_CameraThread                      (RobotCamera::LimelightThread),
    m_LiftTargetDegrees                 (LIFT_DOWN_ANGLE),
    m_ArmTargetDegrees                  (ARM_STARTING_POSITION_DEGREES),
    m_ArmLoadingOffsetDegrees           (0.0_deg),
    m_ArmNeutralOffsetDegrees           (0.0_deg),
    m_ArmL1OffsetDegrees                (0.0_deg),
    m_ArmL2L3OffsetDegrees              (0.0_deg),
    m_ArmL4OffsetDegrees                (0.0_deg),
    m_ArmRemoveAlgaeOffsetDegrees       (0.0_deg),
    m_pArmManualOffsetDegrees           (&m_ArmNeutralOffsetDegrees),
    m_WristTargetDegrees                (WRIST_STARTING_POSITION_DEGREES),
    m_WristLoadingOffsetDegrees         (0.0_deg),
    m_WristNeutralOffsetDegrees         (0.0_deg),
    m_WristL1OffsetDegrees              (0.0_deg),
    m_WristL2L3OffsetDegrees            (0.0_deg),
    m_WristL4OffsetDegrees              (0.0_deg),
    m_WristRemoveAlgaeOffsetDegrees     (0.0_deg),
    m_pWristManualOffsetDegrees         (&m_WristNeutralOffsetDegrees),
    m_LiftPosition                      (LiftPosition::LIFT_DOWN),
    m_ArmPosition                       (ArmPosition::NEUTRAL),
    m_RobotMode                         (ROBOT_MODE_NOT_SET),
    m_AllianceColor                     (DriverStation::GetAlliance()),
    m_AbsoluteEncodersInitialized       (false),
    m_bCameraAlignInProgress            (false),
    m_HeartBeat                         (0U)
{
    RobotUtils::DisplayMessage("Robot constructor.");
    
    // LiveWindow is not used
    LiveWindow::SetEnabled(false);
    
    // Set the autonomous options
    m_AutonomousChooser.SetDefaultOption(AUTO_ROUTINE_1_STRING, AUTO_ROUTINE_1_STRING);
    m_AutonomousChooser.AddOption(AUTO_ROUTINE_2_STRING, AUTO_ROUTINE_2_STRING);
    m_AutonomousChooser.AddOption(AUTO_ROUTINE_3_STRING, AUTO_ROUTINE_3_STRING);
    m_AutonomousChooser.AddOption(AUTO_ROUTINE_3_STRING, AUTO_ROUTINE_3_STRING);
    m_AutonomousChooser.AddOption(AUTO_NO_ROUTINE_STRING, AUTO_NO_ROUTINE_STRING);
    m_AutonomousChooser.AddOption(AUTO_TEST_ROUTINE_STRING, AUTO_TEST_ROUTINE_STRING);
    SmartDashboard::PutData("Autonomous Modes", &m_AutonomousChooser);
    
    RobotUtils::DisplayFormattedMessage("The drive forward axis is: %d\n", EastTech::Controller::Config::GetControllerMapping(DRIVE_CONTROLLER_MODEL)->AXIS_MAPPINGS.RIGHT_TRIGGER);
    RobotUtils::DisplayFormattedMessage("The drive reverse axis is: %d\n", EastTech::Controller::Config::GetControllerMapping(DRIVE_CONTROLLER_MODEL)->AXIS_MAPPINGS.LEFT_TRIGGER);
    RobotUtils::DisplayFormattedMessage("The drive left/right axis is: %d\n", EastTech::Controller::Config::GetControllerMapping(DRIVE_CONTROLLER_MODEL)->AXIS_MAPPINGS.LEFT_X_AXIS);

    ConfigureMotorControllers();

    CANdleConfiguration candleConfig;
    candleConfig.stripType = LEDStripType::RGB;
    m_pCandle->ConfigAllSettings(candleConfig);
    m_pCandle->Animate(m_RainbowAnimation);

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

    WaitForSensorConfig();
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::WaitForSensorConfig
///
/// Waits for any sensors on the robot that route to the RIO
/// to stabilize for accurate readings.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::WaitForSensorConfig()
{
    // This is the logic to wait to take the absolute encoder readings until the RIO is ready
    static units::time::second_t enabledTimeStamp = 0.0_s;
    units::time::second_t currentTimeStamp = m_pRobotProgramTimer->Get();
    if (DriverStation::IsEnabled() && (!m_AbsoluteEncodersInitialized))
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
        /// START ARM
            // Increases going down, crosses the zero boundary
            // Decreases going up, crosses the zero boundary
            // Range of motion is ~195.12_deg

            double armEncoderValue = m_pArmAbsoluteEncoder->Get();
            units::angle::degree_t armEncoderValueDegrees(armEncoderValue * ANGLE_360_DEGREES);

            // This is the delta between the current mechanism position and the desired starting position (or zero point)
            units::angle::degree_t armStartingOffsetDegrees = armEncoderValueDegrees - ARM_STARTING_POSITION_ENCODER_VALUE;
            //std::printf("armStartingOffsetDegrees (start): %f\n", armStartingOffsetDegrees.value());

            // If the starting offset is negative, we crossed over the absolute encoder boundary
            // We give a tolerance of five degrees in case the mechanism is near where we want to start
            // @todo: Does this need to check for very small readings below zero?
            // @todo: Boundary conditions here will be difficult
            if (armStartingOffsetDegrees < ENCODER_BOUNDARY_TOLERANCE_DEGREES)
            {
                // the 0/1 boundary is 360, so subtract the starting position to see how many degrees were up to that point
                // Add in the absolute value of the overage, which was negative
                // This used to add units::angle::degree_t(std::abs(armStartingOffsetDegrees.value())), but that's probably wrong
                armStartingOffsetDegrees = (units::angle::degree_t(ANGLE_360_DEGREES) - ARM_STARTING_POSITION_ENCODER_VALUE) + armEncoderValueDegrees;
            }

            // At this point we have the angle we want relative to zero
            (void)m_pArmPivotMotor->m_pTalonFx->GetConfigurator().SetPosition(armStartingOffsetDegrees);
            //std::printf("armEncoderValue: %f\n", armEncoderValue);
            //std::printf("armEncoderValueDegrees: %f\n", armEncoderValueDegrees.value());
            //std::printf("armStartingOffsetDegrees (final): %f\n", armStartingOffsetDegrees.value());
        /// END ARM
        /// START WRIST
            double wristEncoderValue = m_pWristAbsoluteEncoder->Get();
            units::angle::degree_t wristEncoderValueDegrees(wristEncoderValue * ANGLE_360_DEGREES);

            // This is the delta between the current mechanism position and the desired starting position (or zero point)
            units::angle::degree_t wristStartingOffsetDegrees = wristEncoderValueDegrees - WRIST_STARTING_POSITION_ENCODER_VALUE;
            //std::printf("wristStartingOffsetDegrees (start): %f\n", wristStartingOffsetDegrees.value());

            // If the starting offset is negative, we crossed over the absolute encoder boundary
            // We give a tolerance of five degrees in case the mechanism is near where we want to start
            // @todo: Does this need to check for very small readings below zero?
            // @todo: Boundary conditions here will be difficult
            if (wristStartingOffsetDegrees < ENCODER_BOUNDARY_TOLERANCE_DEGREES)
            {
                // the 0/1 boundary is 360, so subtract the starting position to see how many degrees were up to that point
                // Add in the absolute value of the overage, which was negative
                wristStartingOffsetDegrees = (units::angle::degree_t(ANGLE_360_DEGREES) - WRIST_STARTING_POSITION_ENCODER_VALUE) + wristEncoderValueDegrees;//units::angle::degree_t(std::abs(wristStartingOffsetDegrees.value()));
            }

            // At this point we have the angle we want relative to zero
            (void)m_pWristPivotMotor->m_pTalonFx->GetConfigurator().SetPosition(wristStartingOffsetDegrees);
            //std::printf("wristEncoderValue: %f\n", wristEncoderValue);
            //std::printf("wristEncoderValueDegrees: %f\n", wristEncoderValueDegrees.value());
            //std::printf("wristStartingOffsetDegrees (final): %f\n", wristStartingOffsetDegrees.value());
        /// END WRIST
            m_AbsoluteEncodersInitialized = true;
        }
    }
    else
    {
        // Set the enabled time stamp back to zero until the robot is enabled again
        enabledTimeStamp = 0.0_s;

        // m_AbsoluteEncodersInitialized exists for the life of the program.  Once
        // we have a stable reading acquired, we don't need to do it again until the
        // robot program restarts.
    }
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

    // Configure lift motors (only needs to be applied to the lead motor of the group)
    // Brake mode was set when the motor group was constructed
    (void)m_pLiftMotors->GetMotorConfiguration(LIFT_MOTORS_CAN_START_ID)->Feedback.WithSensorToMechanismRatio(12.0 / 1.0);
    (void)m_pLiftMotors->GetMotorConfiguration(LIFT_MOTORS_CAN_START_ID)->Slot0.WithKP(18.0).WithKI(0.0).WithKD(0.1);
    m_pLiftMotors->ApplyConfiguration(LIFT_MOTORS_CAN_START_ID);
    (void)m_pLiftMotors->GetMotorObject(LIFT_MOTORS_CAN_START_ID)->GetConfigurator().SetPosition(0.0_tr);

    // Configure arm pivot motor
    (void)m_pArmPivotMotor->m_MotorConfiguration.MotorOutput.WithNeutralMode(NeutralModeValue::Brake);
    (void)m_pArmPivotMotor->m_MotorConfiguration.Feedback.WithSensorToMechanismRatio(135.0 / 1.0);
    (void)m_pArmPivotMotor->m_MotorConfiguration.Slot0.WithKP(50.0).WithKI(0.0).WithKD(2.0);
    (void)m_pArmPivotMotor->m_pTalonFx->GetConfigurator().SetPosition(0.0_tr);
    m_pArmPivotMotor->ApplyConfiguration();

    // Configure wrist pivot motor
    m_pWristPivotMotor->m_MotorConfiguration.MotorOutput.Inverted = true;
    (void)m_pWristPivotMotor->m_MotorConfiguration.MotorOutput.WithNeutralMode(NeutralModeValue::Brake);
    (void)m_pWristPivotMotor->m_MotorConfiguration.Feedback.WithSensorToMechanismRatio(25.0 / 1.0);
    (void)m_pWristPivotMotor->m_MotorConfiguration.Slot0.WithKP(18.0).WithKI(0.0).WithKD(0.1);
    (void)m_pWristPivotMotor->m_pTalonFx->GetConfigurator().SetPosition(0.0_tr);
    m_pWristPivotMotor->ApplyConfiguration();

    // Configure algae/coral motor
    (void)m_pGamePieceMotor->m_MotorConfiguration.MotorOutput.WithNeutralMode(NeutralModeValue::Coast);
    m_pGamePieceMotor->ApplyConfiguration();

    // Configure hang motor
    (void)m_pHangMotor->m_MotorConfiguration.MotorOutput.WithNeutralMode(NeutralModeValue::Brake);
    m_pHangMotor->ApplyConfiguration();
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

    // Stop/clear any timers, just in case
    // @todo: Make this a dedicated function.
    m_pMatchModeTimer->Stop();
    m_pMatchModeTimer->Reset();
    m_pSafetyTimer->Stop();
    m_pSafetyTimer->Reset();
    
    // Just in case constructor was called before these were set (likely the case)
    m_AllianceColor = DriverStation::GetAlliance();

    // Disable the rainbow animation
    m_pCandle->ClearAnimation(0);

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

    // No superstructure movement can happen until the absolute encoders are ready and stable
    if (m_AbsoluteEncodersInitialized)
    {
        LiftSequence();
        ArmSequence();
        WristSequence();
        GamePieceControlSequence();
        HangSequence();
    }

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
    // Give the drive team some state information
    SmartDashboard::PutBoolean("Absolute encoders ready", m_AbsoluteEncodersInitialized);

    // Build a stack of "lights" to give visual feedback to the
    // drive team on the current overall superstructure position
    bool bLoad = false;
    bool bNeutral = false;
    bool bL1 = false;
    bool bL2 = false;
    bool bL3 = false;
    bool bL4 = false;
    bool bRemoveAlgae = false;
    switch (m_LiftPosition)
    {
        case LiftPosition::LIFT_DOWN:
        {
            switch (m_ArmPosition)
            {
                case ArmPosition::LOADING:
                {
                    bLoad = true;
                    break;
                }
                case ArmPosition::NEUTRAL:
                {
                    bNeutral = true;
                    break;
                }
                case ArmPosition::REEF_L1:
                {
                    bL1 = true;
                    break;
                }
                case ArmPosition::REEF_L2_L3:
                {
                    bL2 = true;
                    break;
                }
                case ArmPosition::REMOVE_ALGAE:
                {
                    bRemoveAlgae = true;
                    break;
                }
                default:
                {
                    break;
                }
            }
            break;
        }
        case LiftPosition::LIFT_MIDDLE:
        {
            switch (m_ArmPosition)
            {
                case ArmPosition::REEF_L2_L3:
                {
                    bL3 = true;
                    break;
                }
                case ArmPosition::REMOVE_ALGAE:
                {
                    bRemoveAlgae = true;
                    break;
                }
                default:
                {
                    break;
                }
            }
            break;
        }
        case LiftPosition::LIFT_UP:
        {
            if (m_ArmPosition == ArmPosition::REEF_L4)
            {
                bL4 = true;
            }
            break;
        }
        default:
        {
            break;
        }
    }
    SmartDashboard::PutBoolean("Load", bLoad);
    SmartDashboard::PutBoolean("Neutral", bNeutral);
    SmartDashboard::PutBoolean("L1", bL1);
    SmartDashboard::PutBoolean("L2", bL2);
    SmartDashboard::PutBoolean("L3", bL3);
    SmartDashboard::PutBoolean("L4", bL4);
    SmartDashboard::PutBoolean("Algae", bRemoveAlgae);
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::LiftSequence
///
/// This method contains the main workflow for controlling
/// the lift on the robot.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::LiftSequence()
{
    // Lift mechanism can do slightly over 5x sprocket rotations from bottom to top
    // Measured 0->1890 degrees = 5.25 turns.  Limit range to five rotations (0->1800 degrees).
    // Give tolerance at top/bottom of 45 degrees.
    static TalonFX * pLiftLeaderTalon = m_pLiftMotors->GetMotorObject(LIFT_MOTORS_CAN_START_ID);

    units::angle::turn_t liftAngleTurns = pLiftLeaderTalon->GetPosition().GetValue();
    units::angle::degree_t liftAngleDegrees = liftAngleTurns;


    static bool bTareInProgress = false;
    static bool bSetNewZero = false;
    // If the tare button is being held
    if (m_pAuxController->GetButtonState(AUX_TARE_LIFT_BUTTON))
    {
        // Allow manual movement, but only down
        if (m_pAuxController->GetPovAsDirection() == AUX_CONTROLS_LIFT_DOWN)
        {
            m_pLiftMotors->Set(-0.1);
            bSetNewZero = true;
        }
        else if (m_pAuxController->GetPovAsDirection() == AUX_CONTROLS_LIFT_UP)
        {
            m_pLiftMotors->Set(0.1);
        }
        else
        {
            m_pLiftMotors->Set(0.0);
        }
        bTareInProgress = true;
    }
    // When the tare button is released, set the new zero
    if (m_pAuxController->DetectButtonChange(AUX_TARE_LIFT_BUTTON, EastTech::Controller::ButtonStateChanges::BUTTON_RELEASED))
    {
        if (bSetNewZero)
        {
            (void)pLiftLeaderTalon->GetConfigurator().SetPosition(0.0_tr);
            m_LiftTargetDegrees = LIFT_DOWN_ANGLE;
            m_LiftPosition = LiftPosition::LIFT_DOWN;
            bSetNewZero = false;
        }
        bTareInProgress = false;
    }
    // Don't continue if a tare is in progress
    if (bTareInProgress)
    {
        return;
    }


    bool bLiftMotionAllowed = false;

    // The arm must be at REEF_L1 or higher to allow motion
    switch (m_ArmPosition)
    {
        case ArmPosition::REEF_L1:
        case ArmPosition::REEF_L2_L3:
        case ArmPosition::REEF_L4:
        case ArmPosition::REMOVE_ALGAE:
        {
            // Allowing motion here regardless of where the lift
            // currently is doesn't matter because the increment
            // and decrement functions do bounds checking.
            bLiftMotionAllowed = true;
            break;
        }
        case ArmPosition::NEUTRAL:
        case ArmPosition::LOADING:
        default:
        {
            // Do nothing, no motion allowed
            break;
        }
    }

    // If motion is allowed, adjust the state
    if (bLiftMotionAllowed)
    {
        // Check if a state change request occurred
        if (m_pAuxController->DetectPovChange(EastTech::Controller::PovDirections::POV_UP))
        {
            // Increase enum value
            IncrementLiftPosition(m_LiftPosition);
        }
        else if (m_pAuxController->DetectPovChange(EastTech::Controller::PovDirections::POV_DOWN))
        {
            // Decrease enum value
            DecrementLiftPosition(m_LiftPosition);
        }
        else
        {
        }
    }

    // Based on the current lift position, set the target angle
    switch (m_LiftPosition)
    {
        case LiftPosition::LIFT_DOWN:
        {
            m_LiftTargetDegrees = LIFT_DOWN_ANGLE;
            break;
        }
        case LiftPosition::LIFT_MIDDLE:
        {
            m_LiftTargetDegrees = LIFT_MIDDLE_ANGLE;
            break;
        }
        case LiftPosition::LIFT_UP:
        {
            m_LiftTargetDegrees = LIFT_UP_ANGLE;
            break;
        }
        default:
        {
            break;
        }
    }


    // Check for a change between brake and coast
    static bool bOnCoast = false;
    if (m_pAuxController->DetectButtonChange(AUX_TOGGLE_LIFT_BRAKE_COAST_BUTTON))
    {
        if (bOnCoast)
        {
            m_pLiftMotors->SetBrakeMode();
        }
        else
        {
            m_pLiftMotors->SetCoastMode();
        }
        bOnCoast = !bOnCoast;
    }


    // Only set the position if motion is enabled
    if (SUPERSTRUCTURE_MOTION_ENABLED)
    {
        m_pLiftMotors->SetAngle(m_LiftTargetDegrees.value());
    }


    // Update smart dashboard
    SmartDashboard::PutNumber("Lift angle", liftAngleDegrees.value());
    SmartDashboard::PutNumber("Lift target angle", m_LiftTargetDegrees.value());
    SmartDashboard::PutNumber("Lift position (enum)", static_cast<uint32_t>(m_LiftPosition));
    SmartDashboard::PutBoolean("Lift motors coast", bOnCoast);
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::ArmSequence
///
/// This method contains the main workflow for controlling
/// the arm pivot mechanisms on the robot.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::ArmSequence()
{
    // If the arm is pointing straight down at the lift, there is ~20 degrees
    // to the hard stop.  180 degrees would point straight up.
    // Moving toward the loading station position is defined as 'lowering', angle is increasing
    // Moving toward the reef placing position is defined as 'raising', angle is decreasing
    units::angle::turn_t armAngleTurns = m_pArmPivotMotor->m_pTalonFx->GetPosition().GetValue();
    units::angle::degree_t armAngleDegrees = armAngleTurns;


    // Check for a request to move the arm    
    if (m_pAuxController->DetectButtonChange(AUX_ARM_TOWARDS_REEF_BUTTON))
    {
        // Increase enum value
        IncrementArmPosition(m_ArmPosition);
    }
    else if (m_pAuxController->DetectButtonChange(AUX_ARM_TOWARDS_LOAD_BUTTON))
    {
        // Decrease enum value
        DecrementArmPosition(m_ArmPosition);
    }
    else
    {
    }


    // Set the arm and wrist positions
    // @todo: Only do this on position changes.  Ideal would be to only set
    //        the position whenever it changes, but that has to be communicated
    //        to the wrist logic and manual movement control.
    switch (m_ArmPosition)
    {
        case ArmPosition::LOADING:
        {
            m_pArmManualOffsetDegrees = &m_ArmLoadingOffsetDegrees;
            m_ArmTargetDegrees = (*m_pArmManualOffsetDegrees) + ARM_LOADING_TARGET_DEGREES;
            m_pWristManualOffsetDegrees = &m_WristLoadingOffsetDegrees;
            m_WristTargetDegrees = (*m_pWristManualOffsetDegrees) + WRIST_LOADING_TARGET_DEGREES;
            break;
        }
        case ArmPosition::NEUTRAL:
        {
            m_pArmManualOffsetDegrees = &m_ArmNeutralOffsetDegrees;
            m_ArmTargetDegrees = (*m_pArmManualOffsetDegrees) + ARM_NEUTRAL_TARGET_DEGREES;
            m_pWristManualOffsetDegrees = &m_WristNeutralOffsetDegrees;
            m_WristTargetDegrees = (*m_pWristManualOffsetDegrees) + WRIST_NEUTRAL_TARGET_DEGREES;
            break;
        }
        case ArmPosition::REEF_L1:
        {
            // Lift is down at this reef level
            m_pArmManualOffsetDegrees = &m_ArmL1OffsetDegrees;
            m_ArmTargetDegrees = (*m_pArmManualOffsetDegrees) + ARM_REEF_L1_TARGET_DEGREES;
            m_pWristManualOffsetDegrees = &m_WristL1OffsetDegrees;
            m_WristTargetDegrees = (*m_pWristManualOffsetDegrees) + WRIST_REEF_L1_TARGET_DEGREES;
            break;
        }
        case ArmPosition::REEF_L2_L3:
        {
            // Lift is either down or mid at this reef level
            m_pArmManualOffsetDegrees = &m_ArmL2L3OffsetDegrees;
            m_ArmTargetDegrees = (*m_pArmManualOffsetDegrees) + ARM_REEF_L2_L3_TARGET_DEGREES;
            m_pWristManualOffsetDegrees = &m_WristL2L3OffsetDegrees;
            m_WristTargetDegrees = (*m_pWristManualOffsetDegrees) + WRIST_REEF_L2_L3_TARGET_DEGREES;
            break;
        }
        case ArmPosition::REEF_L4:
        {
            // Lift is up for this reef level
            m_pArmManualOffsetDegrees = &m_ArmL4OffsetDegrees;
            m_ArmTargetDegrees = (*m_pArmManualOffsetDegrees) + ARM_REEF_L4_TARGET_DEGREES;
            m_pWristManualOffsetDegrees = &m_WristL4OffsetDegrees;
            m_WristTargetDegrees = (*m_pWristManualOffsetDegrees) + WRIST_REEF_L4_TARGET_DEGREES;
            break;
        }
        case ArmPosition::REMOVE_ALGAE:
        {
            // Lift is up for this reef level
            m_pArmManualOffsetDegrees = &m_ArmRemoveAlgaeOffsetDegrees;
            m_ArmTargetDegrees = (*m_pArmManualOffsetDegrees) + ARM_REEF_ALGAE_TARGET_DEGREES;
            m_pWristManualOffsetDegrees = &m_WristRemoveAlgaeOffsetDegrees;
            m_WristTargetDegrees = (*m_pWristManualOffsetDegrees) + WRIST_REEF_ALGAE_TARGET_DEGREES;
            break;
        }
        default:
        {
            break;
        }
    }


    // Only set the position if motion is enabled
    if (SUPERSTRUCTURE_MOTION_ENABLED)
    {
        m_pArmPivotMotor->SetPositionVoltage(m_ArmTargetDegrees.value());
    }


    // Update smart dashboard
    SmartDashboard::PutNumber("Arm angle", armAngleDegrees.value());
    SmartDashboard::PutNumber("Arm target angle", m_ArmTargetDegrees.value());
    SmartDashboard::PutNumber("Arm offset angle", (*m_pArmManualOffsetDegrees).value());
    SmartDashboard::PutNumber("Arm encoder", m_pArmAbsoluteEncoder->Get());
    SmartDashboard::PutNumber("Arm position (enum)", static_cast<uint32_t>(m_ArmPosition));
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::WristSequence
///
/// This method contains the main workflow for controlling
/// the wrist pivot mechanisms on the robot.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::WristSequence()
{
    // Encoder value increases moving toward lift
    // Range = 0.718 (~258.48_deg)
    units::angle::turn_t wristAngleTurns = m_pWristPivotMotor->m_pTalonFx->GetPosition().GetValue();
    units::angle::degree_t wristAngleDegrees = wristAngleTurns;


    // Check for a change to arm/wrist manual adjustment
    // This logic is in the wrist function because it's the more likely one to change
    static bool bAdjustWrist = true;
    if (m_pAuxController->DetectButtonChange(AUX_TOGGLE_ANGLE_ADJUST_BUTTON))
    {
        bAdjustWrist = !bAdjustWrist;
    }

    // Check for a request to manually adjust the wrist or arm angle
    if (m_pAuxController->DetectButtonChange(AUX_INCREASE_ANGLE_OFFSET_BUTTON))
    {
        if (bAdjustWrist)
        {
            (*m_pWristManualOffsetDegrees) += ARM_WRIST_MANUAL_ADJUST_STEP_DEGREES;
        }
        else
        {
            (*m_pArmManualOffsetDegrees) += ARM_WRIST_MANUAL_ADJUST_STEP_DEGREES;
        }
    }
    if (m_pAuxController->DetectButtonChange(AUX_DECREASE_ANGLE_OFFSET_BUTTON))
    {
        if (bAdjustWrist)
        {
            (*m_pWristManualOffsetDegrees) -= ARM_WRIST_MANUAL_ADJUST_STEP_DEGREES;
        }
        else
        {
            (*m_pArmManualOffsetDegrees) -= ARM_WRIST_MANUAL_ADJUST_STEP_DEGREES;
        }
    }


    // Only set the position if motion is enabled
    if (SUPERSTRUCTURE_MOTION_ENABLED)
    {
        m_pWristPivotMotor->SetPositionVoltage(m_WristTargetDegrees.value());
    }


    // Update smart dashboard
    SmartDashboard::PutNumber("Wrist angle", wristAngleDegrees.value());
    SmartDashboard::PutNumber("Wrist target angle", m_WristTargetDegrees.value());
    SmartDashboard::PutNumber("Wrist offset angle", (*m_pWristManualOffsetDegrees).value());
    SmartDashboard::PutNumber("Wrist encoder", m_pWristAbsoluteEncoder->Get());
    SmartDashboard::PutBoolean("Angle adjust wrist", bAdjustWrist);
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::HangSequence
///
/// This method contains the main workflow for hanging.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::HangSequence()
{
    if (m_pDriveController->GetButtonState(DRIVE_HANG_UP_BUTTON))
    {
        m_pHangMotor->SetDutyCycle(1.0);
    }
    else if (m_pDriveController->GetButtonState(DRIVE_HANG_DOWN_BUTTON))
    {
        m_pHangMotor->SetDutyCycle(-1.0);
    }
    else
    {
        m_pHangMotor->SetDutyCycle(0.0);
    }
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::GamePieceControlSequence
///
/// This method contains the main workflow for controlling
/// game pice actions (algae/coral in/out).
///
////////////////////////////////////////////////////////////////
void EastTechRobot::GamePieceControlSequence()
{
    double outputSpeed = GAME_PIECE_MOTOR_SPEED_FAST;
    double outputMultiplier = 1.0;
    switch (m_ArmPosition)
    {
        case ArmPosition::REEF_L1:
        {
            outputSpeed = GAME_PIECE_MOTOR_SPEED_SLOW;
            break;
        }
        case ArmPosition::REEF_L4:
        case ArmPosition::REMOVE_ALGAE:
        {
            outputMultiplier = -1.0;
            break;
        }
        case ArmPosition::LOADING:
        {
            outputMultiplier = -1.0;
        }
        default:
        {
            break;
        }
    }

    if (m_pAuxController->GetButtonState(AUX_GAME_PIECE_IN_BUTTON))
    {
        m_pGamePieceMotor->SetDutyCycle(GAME_PIECE_MOTOR_SPEED_INTAKE * outputMultiplier);
    }
    else if (m_pAuxController->GetButtonState(AUX_GAME_PIECE_OUT_BUTTON))
    {
        m_pGamePieceMotor->SetDutyCycle(-outputSpeed * outputMultiplier);
    }
    else
    {
        m_pGamePieceMotor->SetDutyCycle(0.0);
    }
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
        RobotCamera::SetLimelightPipeline(1);
        RobotCamera::SetLimelightMode(RobotCamera::LimelightMode::VISION_PROCESSOR);
        RobotCamera::AutonomousCamera::AlignToTargetSwerve();
    }
    else
    {
        m_bCameraAlignInProgress = false;
        RobotCamera::SetLimelightPipeline(0);
        RobotCamera::SetLimelightMode(RobotCamera::LimelightMode::DRIVER_CAMERA);
    }
    return;

    // 2024: Go no further

    static bool bFullProcessing = false;
    
    // @note: Use std::chrono if precise time control is needed.
    
    // Check for any change in camera
    if (m_pDriveController->GetButtonState(SELECT_FRONT_CAMERA_BUTTON))
    {
        RobotCamera::SetCamera(RobotCamera::FRONT_USB);
    }
    else if (m_pDriveController->GetButtonState(SELECT_BACK_CAMERA_BUTTON))
    {
        RobotCamera::SetCamera(RobotCamera::BACK_USB);
    }
    else
    {
    }
    
    // Look for full processing to be enabled/disabled
    if (m_pDriveController->DetectButtonChange(CAMERA_TOGGLE_FULL_PROCESSING_BUTTON))
    {
        // Change state first, because the default is set before this code runs
        bFullProcessing = !bFullProcessing;
        RobotCamera::SetFullProcessing(bFullProcessing);
    }
    
    // Look for the displayed processed image to be changed
    if (m_pDriveController->DetectButtonChange(CAMERA_TOGGLE_PROCESSED_IMAGE_BUTTON))
    {
        RobotCamera::ToggleCameraProcessedImage();
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
    m_pCandle->Animate(m_RainbowAnimation);
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
