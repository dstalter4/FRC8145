////////////////////////////////////////////////////////////////////////////////
/// @file   EastTechRobot.hpp
/// @author David Stalter
///
/// @details
/// This is the class declaration for a FRC robot derived from the WPI library
/// base classes.  The TimedRobot class is the base of a robot application that
/// will automatically call appropriate Autonomous and Teleop methods at the
/// right time as controlled by the switches on the driver station or the field
/// controls.
///
/// Copyright (c) 2024 East Technical High School
////////////////////////////////////////////////////////////////////////////////

#ifndef EASTTECHROBOT_HPP
#define EASTTECHROBOT_HPP

// SYSTEM INCLUDES
#include <cmath>                                // for M_PI
#include <thread>                               // for std::thread

// C INCLUDES
#include "frc/Compressor.h"                     // for retrieving info on the compressor
#include "frc/DigitalInput.h"                   // for DigitalInput type
#include "frc/DigitalOutput.h"                  // for DigitalOutput type
#include "frc/DoubleSolenoid.h"                 // for DoubleSolenoid type
#include "frc/DriverStation.h"                  // for interacting with the driver station
#include "frc/Relay.h"                          // for Relay type
#include "frc/Solenoid.h"                       // for Solenoid type
#include "frc/TimedRobot.h"                     // for base class decalartion
#include "frc/livewindow/LiveWindow.h"          // for controlling the LiveWindow
#include "frc/motorcontrol/Spark.h"             // for creating an object to interact with the rev blinkin
#include "frc/smartdashboard/SendableChooser.h" // for using the smart dashboard sendable chooser functionality
#include "frc/smartdashboard/SmartDashboard.h"  // for interacting with the smart dashboard

// C++ INCLUDES
#include "DriveConfiguration.hpp"               // for information on the drive config
#include "EastTechController.hpp"               // for controller interaction
#include "EastTechTalon.hpp"                    // for custom Talon control
#include "RobotUtils.hpp"                       // for ASSERT, DEBUG_PRINTS
#include "SwerveDrive.hpp"                      // for using swerve drive
#include "ctre/phoenix6/Pigeon2.hpp"            // for PigeonIMU

using namespace frc;


////////////////////////////////////////////////////////////////
/// @class EastTechRobot
///
/// Derived class from TimedRobot.  The object that will
/// control all robot functionality.
///
////////////////////////////////////////////////////////////////
class EastTechRobot : public TimedRobot
{
public:
    friend class RobotCamera;
    friend class EastTechRobotTest;

    // MEMBER FUNCTIONS
    
    // Base robot routines
    virtual void RobotInit() override;
    virtual void RobotPeriodic() override;
    
    // Autonomous routines
    virtual void AutonomousInit() override;
    virtual void AutonomousPeriodic() override;
    
    // Teleop routines
    virtual void TeleopInit() override;
    virtual void TeleopPeriodic() override;
    
    // Test mode routines
    virtual void TestInit() override;
    virtual void TestPeriodic() override;
    
    // Robot disabled routines
    virtual void DisabledInit() override;
    virtual void DisabledPeriodic() override;
    
    // Constructor, destructor, copy, assignment
    EastTechRobot();
    virtual ~EastTechRobot() = default;
    EastTechRobot(EastTechRobot&& rhs) = default;
    EastTechRobot& operator=(EastTechRobot&& rhs) = default;
      
private:

    // TYPEDEFS
    typedef EastTech::Talon::MotorGroupControlMode MotorGroupControlMode;
    typedef EastTech::Talon::TalonFxMotorController TalonFxMotorController;
    typedef EastTech::Controller::Config::Models ControllerModels;
    typedef EastTech::Controller::Config::Mappings ControllerMappings;
    typedef EastTechDriveController<EastTechCustomController> DriveControllerType;
    typedef EastTechController<EastTechCustomController> AuxControllerType;
    
    // ENUMS
    enum RobotMode
    {
        ROBOT_MODE_AUTONOMOUS,
        ROBOT_MODE_TELEOP,
        ROBOT_MODE_TEST,
        ROBOT_MODE_DISABLED,
        ROBOT_MODE_NOT_SET
    };
    
    enum RobotDirection : uint32_t
    {
        ROBOT_NO_DIRECTION = 0x0,
        ROBOT_FORWARD = 0x1,
        ROBOT_REVERSE = 0x2,
        ROBOT_LEFT = 0x10,
        ROBOT_RIGHT = 0x20,
        ROBOT_TRANSLATION_MASK = 0xF,
        ROBOT_STRAFE_MASK = 0xF0
    };
    
    enum RobotRotate
    {
        ROBOT_NO_ROTATE,
        ROBOT_CLOCKWISE,
        ROBOT_COUNTER_CLOCKWISE
    };
    
    // STRUCTS
    // (none)
    
    // This is a hacky way of retrieving a pointer to the robot object
    // outside of the robot class.  The robot object itself is a static
    // variable inside the function StartRobot() in the RobotBase class.
    // This makes retrieving the address difficult.  To work around this,
    // we'll allocate some static storage for a pointer to a robot object.
    // When RobotInit() is called, m_pThis will be filled out.  This works
    // because only one EastTechRobot object is ever constructed.
    static EastTechRobot * m_pThis;
    inline void SetStaticThisInstance() { m_pThis = this; }
    inline static EastTechRobot * GetRobotInstance() { return m_pThis; }

    // Increments a variable to indicate the robot code is successfully running
    inline void HeartBeat();
    
    // Checks for a robot state change and logs a message if so
    inline void CheckAndUpdateRobotMode(RobotMode robotMode);

    // Updates information on the smart dashboard for the drive team
    void UpdateSmartDashboard();

    // Autonomous wait for something to complete delay routine
    inline void AutonomousDelay(units::second_t time);

    // Autonomous drive for a specified time
    inline void AutonomousSwerveDriveSequence(RobotDirection direction, RobotRotate rotate, double translationSpeed, double strafeSpeed, double rotateSpeed, units::second_t time, bool bFieldRelative);
    
    // Autonomous routines
    // @todo: Make EastTechRobotAutonomous a friend and move these out (requires accessor to *this)!
    void AutonomousRoutine1();
    void AutonomousRoutine2();
    void AutonomousRoutine3();
    void AutonomousTestRoutine();
    void AutonomousTestSwerveRoutine();
    void AutonomousTestTrajectoryRoutine();
    void AutonomousCommon();
    void AutonomousCommonRed();
    void AutonomousCommonBlue();

    // Resets member variables
    void ResetMemberData();

    // Routine to put things in a known state
    void InitialStateSetup();

    // Configure motor controller parameters
    void ConfigureMotorControllers();

    // Main sequence for drive motor control
    void SwerveDriveSequence();

    // Main sequence for controlling pneumatics
    void PneumaticSequence();
    
    // Main sequence for vision processing
    void CameraSequence();

    // Main sequence for intake control
    void IntakeSequence();

    // Main sequence for superstructure mechanism pivot control
    void PivotSequence();
    void CheckAndUpdateAmpValues();

    // Main sequence for shoot control
    void ShootSequence();

    // Main sequence for lifting the robot
    void LiftSequence();
    
    // MEMBER VARIABLES
    
    // Autonomous
    SendableChooser<std::string>    m_AutonomousChooser;                    // Selects from the dashboard which auto routine to run
    
    // User Controls
    DriveControllerType *           m_pDriveController;                     // Drive controller
    AuxControllerType *             m_pAuxController;                       // Auxillary input controller
    
    // Swerve Drive
    Pigeon2 *                       m_pPigeon;                              // CTRE Pigeon2 IMU
    SwerveDrive *                   m_pSwerveDrive;                         // Swerve drive control
    
    // Motors
    TalonFxMotorController *        m_pIntakeMotor;                         // Intake motor control
    TalonFxMotorController *        m_pFeederMotor;                         // Feeder motor control
    TalonMotorGroup<TalonFX> *      m_pShooterMotors;                       // Shooter motors control
    TalonMotorGroup<TalonFX> *      m_pPivotMotors;                         // Pivot motors control
    EastTech::Talon::EmptyTalonFx * m_pLiftMotors;                          // Lift motors control

    // LEDs
    Spark *                         m_pBlinkin;                             // Pseudo Spark for rev blinkin LED PWM control

    // Digital I/O
    DigitalOutput *                 m_pDebugOutput;                         // Debug assist output
    
    // Analog I/O
    // (none)
    
    // Pneumatics
    Compressor *                    m_pCompressor;                          // Object to get info about the compressor
    
    // Servos
    // (none)
    
    // Encoders
    // (none)
    
    // Timers
    Timer *                         m_pMatchModeTimer;                      // Times how long a particular mode (autonomous, teleop) is running
    Timer *                         m_pSafetyTimer;                         // Fail safe in case critical operations don't complete

    // Camera
    // Note: Only need to have a thread here and tie it to
    // the RobotCamera class, which handles everything else.
    std::thread                     m_CameraThread;
    
    // Misc
    RobotMode                       m_RobotMode;                            // Keep track of the current robot state
    std::optional
    <DriverStation::Alliance>       m_AllianceColor;                        // Color reported by driver station during a match
    bool                            m_bCameraAlignInProgress;               // Indicates if an automatic camera align is in progress
    bool                            m_bShootSpeaker;                        // Differentiates between shooting at the speaker or the amp
    bool                            m_bShootSpeakerClose;                   // Indicates if shooting the speaker from close up or further away
    bool                            m_bShotInProgress;                      // Indicates whether a shot is in progress or not
    bool                            m_bIntakeInProgress;                    // Indicates whether a note is being picked up
    bool                            m_bPivotTareInProgress;                 // Indicates whether or not a tare of the pivot mechanism is in progress
    units::angle::degree_t          m_PivotTargetDegrees;                   // Tracks the desired angle position of the superstructure mechanism
    units::angle::degree_t          m_AmpTargetDegrees;                     // The current target angle for the pivot mechanism when shooting at the amp
    double                          m_AmpTargetSpeed;                       // The current target speed for the shooter motors when shooting at the amp
    uint32_t                        m_HeartBeat;                            // Incremental counter to indicate the robot code is executing
    
    // CONSTS
    
    // Joysticks/Buttons
    // Note: Don't forget to update the controller object typedefs if
    //       necessary when changing these types!
    static const ControllerModels DRIVE_CONTROLLER_MODEL                        = ControllerModels::CUSTOM_XBOX;
    static const ControllerModels AUX_CONTROLLER_MODEL                          = ControllerModels::CUSTOM_XBOX;
    static constexpr const ControllerMappings * const DRIVE_CONTROLLER_MAPPINGS = EastTech::Controller::Config::GetControllerMapping(DRIVE_CONTROLLER_MODEL);
    static constexpr const ControllerMappings * const AUX_CONTROLLER_MAPPINGS   = EastTech::Controller::Config::GetControllerMapping(AUX_CONTROLLER_MODEL);
    
    static const int                DRIVE_JOYSTICK_PORT                     = 0;
    static const int                AUX_JOYSTICK_PORT                       = 1;

    // Driver inputs
    static const int                DRIVE_LIFT_ROBOT_BUTTON                 = DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.RIGHT_BUTTON;
    static const int                DRIVE_ALIGN_WITH_CAMERA_BUTTON          = DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.DOWN_BUTTON;
    static const int                FIELD_RELATIVE_TOGGLE_BUTTON            = DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.LEFT_BUMPER;
    static const int                ZERO_GYRO_YAW_BUTTON                    = DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.RIGHT_BUMPER;
    static const int                CAMERA_TOGGLE_FULL_PROCESSING_BUTTON    = DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.NO_BUTTON;
    static const int                CAMERA_TOGGLE_PROCESSED_IMAGE_BUTTON    = DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.NO_BUTTON;
    static const int                SELECT_FRONT_CAMERA_BUTTON              = DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.NO_BUTTON;
    static const int                SELECT_BACK_CAMERA_BUTTON               = DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.NO_BUTTON;
    
    // Aux inputs
    static const int                ESTOP_BUTTON                            = AUX_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.NO_BUTTON;
    static const int                AUX_SHOOT_AXIS                          = AUX_CONTROLLER_MAPPINGS->AXIS_MAPPINGS.RIGHT_TRIGGER;
    static const int                AUX_INTAKE_AXIS                         = AUX_CONTROLLER_MAPPINGS->AXIS_MAPPINGS.LEFT_TRIGGER;
    static const int                AUX_INTAKE_OUT_BUTTON                   = AUX_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.LEFT_BUMPER;
    static const int                AUX_TOGGLE_SPEAKER_SHOOT_CLOSE          = AUX_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.RIGHT_BUMPER;
    static const int                AUX_TOGGLE_SPEAKER_AMP_SHOOT_BUTTON     = AUX_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.UP_BUTTON;
    static const int                AUX_INTAKE_AT_SOURCE_BUTTON             = AUX_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.DOWN_BUTTON;
    static const int                AUX_TARE_PIVOT_ANGLE                    = AUX_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.START;
    static const int                AUX_MANUAL_PIVOT_AXIS                   = AUX_CONTROLLER_MAPPINGS->AXIS_MAPPINGS.LEFT_Y_AXIS;

    // CAN Signals
    // Note: Remember to check the CAN IDs in use in SwerveDrive.hpp.
    static const unsigned           SHOOTER_MOTORS_CAN_START_ID             = 9;
    static const unsigned           INTAKE_MOTOR_CAN_ID                     = 11;
    static const unsigned           FEEDER_MOTOR_CAN_ID                     = 12;
    static const unsigned           PIVOT_MOTORS_CAN_START_ID               = 13;
    static const unsigned           LIFT_MOTORS_CAN_START_ID                = 15;

    // CANivore Signals
    // Note: IDs 1-4 are used by the CANcoders (see the
    //       SwerveModuleConfigs in SwerveDrive.hpp).
    static const int                PIGEON_CAN_ID                           = 5;

    // PWM Signals
    static const int                BLINKIN_PWM_CHANNEL                     = 0;
    
    // Relays
    // (none)
    
    // Digital I/O Signals
    static const int                DEBUG_OUTPUT_DIO_CHANNEL                = 7;
    
    // Analog I/O Signals
    // (none)
    
    // Solenoid Signals
    // (none)

    // Solenoids
    // (none)

    // Motor speeds
    static constexpr double         INTAKE_MOTOR_SPEED                      = -1.0;
    static constexpr double         FEEDER_MOTOR_SPEED                      =  0.5;
    static constexpr double         SHOOTER_MOTOR_SPEAKER_CLOSE_SPEED       = -0.5;
    static constexpr double         SHOOTER_MOTOR_SPEAKER_FAR_SPEED         = -0.7;
    static constexpr double         SHOOTER_MOTOR_SPEAKER_OFFSET_SPEED      =  0.2;
    static constexpr double         SHOOTER_MOTOR_AMP_SPEED                 = -0.25;
    static constexpr double         SHOOTER_MOTOR_LOAD_AT_SOURCE_SPEED      =  0.30;
    static constexpr double         SHOOTER_STEP_SPEED                      =  0.05;
    static constexpr double         LIFT_MOTOR_SPEED                        =  0.70;
    static constexpr double         LIFT_MOTOR_OFFSET_SPEED                 =  0.15;
    
    // Misc
    const std::string               AUTO_ROUTINE_1_STRING                   = "Speaker center";
    const std::string               AUTO_ROUTINE_2_STRING                   = "Speaker source";
    const std::string               AUTO_ROUTINE_3_STRING                   = "Speaker amp";
    const std::string               AUTO_TEST_ROUTINE_STRING                = "Autonomous Test Routine";
    static constexpr units::angle::degree_t PIVOT_ANGLE_RUNTIME_BASE        =  3.0_deg;
    static constexpr units::angle::degree_t PIVOT_ANGLE_INTAKE_NOTE         = 30.0_deg;
    static constexpr units::angle::degree_t PIVOT_ANGLE_TOUCHING_SPEAKER    = 45.0_deg;
    static constexpr units::angle::degree_t PIVOT_ANGLE_FROM_PODIUM         = 30.0_deg;
    static constexpr units::angle::degree_t PIVOT_ANGLE_TOUCHING_AMP        = 92.0_deg;
    static constexpr units::angle::degree_t SHOOTER_STEP_ANGLE              =  2.0_deg;

    static const int                OFF                                     = 0;
    static const int                ON                                      = 1;
    static const int                SCALE_TO_PERCENT                        = 100;
    static const unsigned           SINGLE_MOTOR                            = 1;
    static const unsigned           TWO_MOTORS                              = 2;
    static const unsigned           NUMBER_OF_LEDS                          = 8;
    static const char               NULL_CHARACTER                          = '\0';

    static const unsigned           CAMERA_RUN_INTERVAL_MS                  = 1000U;
    
    static constexpr double         JOYSTICK_TRIM_UPPER_LIMIT               =  0.05;
    static constexpr double         JOYSTICK_TRIM_LOWER_LIMIT               = -0.05;
    static constexpr double         SWERVE_ROTATE_SLOW_JOYSTICK_THRESHOLD   =  0.10;
    static constexpr double         SWERVE_DRIVE_SLOW_SPEED                 =  0.10;
    static constexpr double         SWERVE_ROTATE_SLOW_SPEED                =  0.10;
    static constexpr double         AXIS_INPUT_DEAD_BAND                    =  0.10;
    static constexpr double         LIFT_MAX_ROLL_DEGREES                   =  5.00;
    static constexpr double         LIFT_OFFSET_STOP_POINT_DEGREES          =  0.50;

    static constexpr units::second_t    SAFETY_TIMER_MAX_VALUE_S            =  5.00_s;


    // The below code/equations are for arcade drive, not swerve drive.

    // These indicate which motor value (+1/-1) represent
    // forward/reverse in the robot.  They are used to keep
    // autonomous movement code common without yearly updates.

    static constexpr double         LEFT_DRIVE_FORWARD_SCALAR               = -1.00;
    static constexpr double         LEFT_DRIVE_REVERSE_SCALAR               = +1.00;
    static constexpr double         RIGHT_DRIVE_FORWARD_SCALAR              = +1.00;
    static constexpr double         RIGHT_DRIVE_REVERSE_SCALAR              = -1.00;

    ////////////////////////////////////////////////////////////////
    // Inputs from joystick:
    //
    // Forward:     (0, -1)
    // Reverse:     (0, +1)
    // Left:        (-1, 0)
    // Right:       (+1, 0)
    //
    // Equations:
    //
    //     x+y   x-y   -x+y   -x-y
    // F:   -1    +1     -1     +1
    // B:   +1    -1     +1     -1
    // L:   -1    -1     +1     +1
    // R:   +1    +1     -1     -1
    //
    // Output to motors:
    //
    // Left forward/right = +1, Right forward/left  = +1:
    // Left reverse/left  = -1, Right reverse/right = -1:
    // x-y, -x-y
    //
    // Left forward/right = -1, Right forward/left  = -1:
    // Left reverse/left  = +1, Right reverse/right = +1:
    // -x+y, x+y
    //
    // Left forward/right = +1, Right forward/left  = -1:
    // Left reverse/left  = -1, Right reverse/right = +1:
    // x-y, x+y
    //
    // Left forward/right = -1, Right forward/left  = +1:
    // Left reverse/left  = +1, Right reverse/right = -1:
    // -x+y, -x-y
    ////////////////////////////////////////////////////////////////

    inline static constexpr double LeftDriveEquation(double xInput, double yInput)
    {
        double leftValue = 0.0;

        if (static_cast<int>(LEFT_DRIVE_FORWARD_SCALAR) == 1)
        {
            leftValue = xInput - yInput;
        }
        else
        {
            leftValue = -xInput + yInput;
        }
        
        return leftValue;
    }

    inline static constexpr double RightDriveEquation(double xInput, double yInput)
    {
        double rightValue = 0.0;

        if (static_cast<int>(RIGHT_DRIVE_FORWARD_SCALAR) == 1)
        {
            rightValue = -xInput - yInput;
        }
        else
        {
            rightValue = xInput + yInput;
        }
        
        return rightValue;
    }

};  // End class



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::HeartBeat
///
/// Increments the heartbeat counter.
///
////////////////////////////////////////////////////////////////
inline void EastTechRobot::HeartBeat()
{
    m_HeartBeat++;
    SmartDashboard::PutNumber("Heartbeat", m_HeartBeat);
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::CheckAndUpdateRobotMode
///
/// Checks the current robot mode for a state change and updates
/// accordingly, including displaying a message.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::CheckAndUpdateRobotMode(RobotMode robotMode)
{
    // These array messages match the order of the RobotMode enum
    const char * MODE_CHANGE_ENTER_MESSAGES[] = 
                {
                    "Autonomous entered.",
                    "Teleop entered.",
                    "Test entered.",
                    "Disabled entered."
                };

    const char * MODE_CHANGE_EXIT_MESSAGES[] = 
                {
                    "Autonomous exited.",
                    "Teleop exited.",
                    "Test exited.",
                    "Disabled exited."
                };
    
    // Check for the mode to have changed
    if (m_RobotMode != robotMode)
    {
        // First display the exit message for the old mode
        RobotUtils::DisplayMessage(MODE_CHANGE_EXIT_MESSAGES[m_RobotMode]);

        // Enter the new mode and display an enter message
        m_RobotMode = robotMode;
        RobotUtils::DisplayMessage(MODE_CHANGE_ENTER_MESSAGES[m_RobotMode]);
    }
}

#endif // EASTTECHROBOT_HPP
