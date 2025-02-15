////////////////////////////////////////////////////////////////////////////////
/// @file   RobotTestCode.cpp
/// @author David Stalter
///
/// @details
/// Implementation of the EastTechRobot test functions.  This keeps official
/// stable robot code isolated.
///
/// Copyright (c) 2024 East Technical High School
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
// <none>

// C INCLUDES
#include "frc/BuiltInAccelerometer.h"   // for the built-in accelerometer
#include "rev/CANSparkMax.h"            // for interacting with spark max motor controllers

// C++ INCLUDES
#include "RobotUtils.hpp"               // for DisplayMessage(), DisplayFormattedMessage()
#include "EastTechRobot.hpp"            // for robot class declaration


// Helper macro to get the robot object, only for use in test class code
#define EASTTECH_ROBOT_OBJ() EastTechRobot::GetRobotInstance()

using namespace rev;


////////////////////////////////////////////////////////////////
/// @class EastTechRobotTest
///
/// A class used to test robot functionality.  The intention of
/// this class is to enable quick tests or rapid prototypes.
/// It leverages the EastTechRobot 'Test' mode functions to execute
/// routines.  Since it is separate from the 'product' robot
/// code (in EastTechRobot), it cannot directly use the various
/// member objects from that code.  Instead they can be accessed
/// through the EASTTECH_ROBOT_OBJ() macro.
///
/// A second, but currently unused, test approach is also
/// presented.  This approach attempts to mimic direct use of
/// the EastTechRobot object members by binding references to them.
///
////////////////////////////////////////////////////////////////
class EastTechRobotTest
{
public:
    static void InitializeCommonPointers();
    static void QuickTestCode();

    static void CtreSpeedControllerTest();
    static void RevSpeedControllerTest();
    static void TankDrive();
    static void SwerveDriveTest();
    static void SuperstructureTest();
    static void PneumaticsTest();

    static void TimeTest();
    static void ButtonChangeTest();
    static void AccelerometerTest();
    static void CandleLedsTest();
    static void RelayLedsTest();

private:
    // Objects for use in test routines
    static Joystick * m_pJoystick;

    // Alternate test approach (not currently used):
    // Singleton test object with members bound by reference to EastTechRobot member objects.
    /*
    EastTechRobotTest() :
        m_rpDebugOutput(EastTechRobot::GetRobotInstance()->m_pDebugOutput)
    {
    }
    static EastTechRobotTest * GetInstance() { return m_pRobotTestObj; }
    static void CreateInstance()
    {
        m_pRobotTestObj = new EastTechRobotTest();
    }

    static EastTechRobotTest * m_pRobotTestObj;
    DigitalOutput *& m_rpDebugOutput;
    */
};

// STATIC MEMBER DATA
Joystick * EastTechRobotTest::m_pJoystick = nullptr;



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::TestInit
///
/// The test init method.  This method is called once each time
/// the robot enters test mode.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::TestInit()
{
    RobotUtils::DisplayMessage("TestInit called.");

    EastTechRobotTest::InitializeCommonPointers();
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::TestPeriodic
///
/// The test control method.  This method is called
/// periodically while the robot is in test mode.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::TestPeriodic()
{
    // Log a mode change if one occurred
    CheckAndUpdateRobotMode(ROBOT_MODE_TEST);

    // Enable or disable routines for testing
    EastTechRobotTest::QuickTestCode();
    //EastTechRobotTest::CtreSpeedControllerTest();
    //EastTechRobotTest::RevSpeedControllerTest();
    //EastTechRobotTest::TankDrive();
    //EastTechRobotTest::SwerveDriveTest();
    //EastTechRobotTest::PneumaticsTest();
    //EastTechRobotTest::SuperstructureTest();
    //EastTechRobotTest::TimeTest();
    //EastTechRobotTest::ButtonChangeTest();
    //EastTechRobotTest::AccelerometerTest();
    //EastTechRobotTest::CandleLedsTest();
    //EastTechRobotTest::RelayLedsTest();
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobotTest::InitializeCommonPointers
///
/// Initializes any common test pointers by creating objects
/// for them to use.
///
////////////////////////////////////////////////////////////////
void EastTechRobotTest::InitializeCommonPointers()
{
    static bool bPointersInitialized = false;
    if (!bPointersInitialized)
    {
        // Only support one joystick in test code
        m_pJoystick = new Joystick(0);
        bPointersInitialized = true;
    }
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobotTest::QuickTestCode
///
/// Test code to try out for rapid prototyping.
///
////////////////////////////////////////////////////////////////
void EastTechRobotTest::QuickTestCode()
{
    // Rev only allows creating one motor object with a CAN ID.
    // Disable the actual swerve object creation.
    static CANSparkMax * m_pDriveSpark = new CANSparkMax(4, CANSparkLowLevel::MotorType::kBrushless);
    static CANSparkMax * m_pAngleSpark = new CANSparkMax(3, CANSparkLowLevel::MotorType::kBrushless);
    static SparkRelativeEncoder m_DriveSparkEncoder = m_pDriveSpark->GetEncoder();
    static SparkRelativeEncoder m_AngleSparkEncoder = m_pAngleSpark->GetEncoder();
    static SparkPIDController m_DrivePidController = m_pDriveSpark->GetPIDController();
    static SparkPIDController m_AnglePidController = m_pAngleSpark->GetPIDController();
    static CANcoder * m_pAngleCanCoder = new CANcoder(2, "canivore-8145");
    static bool bInit = false;
    
    if (!bInit)
    {
        m_pAngleSpark->RestoreFactoryDefaults();
        m_pAngleSpark->SetSmartCurrentLimit(20);
        m_pAngleSpark->SetInverted(false);
        m_pAngleSpark->SetIdleMode(CANSparkMax::IdleMode::kCoast);
        m_AngleSparkEncoder.SetPositionConversionFactor(360.0 / SwerveConfig::ANGLE_GEAR_RATIO);
        m_AnglePidController.SetP(0.028);
        m_AnglePidController.SetI(0.000);
        m_AnglePidController.SetD(0.0015);
        m_AnglePidController.SetFF(0.000);
        m_pAngleSpark->EnableVoltageCompensation(12.0);
        m_pAngleSpark->BurnFlash();

        CANcoderConfiguration canCoderConfig;
        canCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue::Unsigned_0To1;
        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue::CounterClockwise_Positive;
        (void)m_pAngleCanCoder->GetConfigurator().Apply(canCoderConfig);
        
        bInit = true;
        double absolutePositionDelta = m_pAngleCanCoder->GetAbsolutePosition().GetValueAsDouble() - 144.932;
        SmartDashboard::PutNumber("Debug A", m_AngleSparkEncoder.GetPosition());
        SmartDashboard::PutNumber("Debug B", absolutePositionDelta);
        m_AngleSparkEncoder.SetPosition(absolutePositionDelta);
        m_AnglePidController.SetReference(0.0, CANSparkMax::ControlType::kPosition);
    }

    // 2024 Team 8145 Neo Config (Bevels Right)
    // FL: 5-6-3, 190.459
    // FR: 3-4-2, 144.932
    // BL: 7-8-4, 127.178
    // BR: 1-2-1, 191.602
    SmartDashboard::PutNumber("Debug C", m_pAngleCanCoder->GetAbsolutePosition().GetValueAsDouble());
    SmartDashboard::PutNumber("Debug D", m_AngleSparkEncoder.GetPosition());
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobotTest::SuperstructureTest
///
/// Test code to try out functionality on the superstructure.
///
////////////////////////////////////////////////////////////////
void EastTechRobotTest::SuperstructureTest()
{
    static TalonFX * pTalonFx5 = new TalonFX(5);
    static TalonFX * pTalonFx6 = new TalonFX(6);
    static TalonFX * pTalonFx7 = new TalonFX(7);
    static TalonFX * pTalonFx8 = new TalonFX(8);
    static TalonFX * pTalonFx9 = new TalonFX(9);
    static TalonFX * pTalonFx10 = new TalonFX(10);
    
    while (m_pJoystick->GetRawButton(1))
    {
        pTalonFx5->Set(0.3);
        pTalonFx6->Set(0.3);
    }
    pTalonFx5->Set(0.0);
    pTalonFx6->Set(0.0);
    while (m_pJoystick->GetRawButton(2))
    {
        pTalonFx7->Set(0.3);
        pTalonFx8->Set(0.3);
    }
    pTalonFx7->Set(0.0);
    pTalonFx8->Set(0.0);
    while (m_pJoystick->GetRawButton(3))
    {
        pTalonFx9->Set(0.3);
        pTalonFx10->Set(0.3);
    }
    pTalonFx9->Set(0.0);
    pTalonFx10->Set(0.0);
    while (m_pJoystick->GetRawButton(4))
    {
        pTalonFx5->Set(0.3);
        pTalonFx5->Set(0.3);
        pTalonFx7->Set(0.5);
        pTalonFx8->Set(0.5);
        pTalonFx9->Set(1.0);
        pTalonFx10->Set(1.0);
    }
    pTalonFx5->Set(0.0);
    pTalonFx6->Set(0.0);
    pTalonFx7->Set(0.0);
    pTalonFx8->Set(0.0);
    pTalonFx9->Set(0.0);
    pTalonFx10->Set(0.0);
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobotTest::CtreSpeedControllerTest
///
/// Test code for CTRE speed controllers.
///
////////////////////////////////////////////////////////////////
void EastTechRobotTest::CtreSpeedControllerTest()
{
    static TalonFX * pLeft1 = new TalonFX(1);
    static TalonFX * pLeft2 = new TalonFX(2);
    static TalonFX * pRight1 = new TalonFX(3);
    static TalonFX * pRight2 = new TalonFX(4);
    
    while (m_pJoystick->GetRawButton(1))
    {
        pLeft1->Set(1.0);
        pLeft2->Set(1.0);
    }
    while (m_pJoystick->GetRawButton(2))
    {
        pLeft1->Set(-1.0);
        pLeft2->Set(-1.0);
    }
    while (m_pJoystick->GetRawButton(3))
    {
        pRight1->Set(1.0);
        pRight2->Set(1.0);
    }
    while (m_pJoystick->GetRawButton(4))
    {
        pRight1->Set(-1.0);
        pRight2->Set(-1.0);
    }
    
    pLeft1->Set(0.0);
    pLeft2->Set(0.0);
    pRight1->Set(0.0);
    pRight2->Set(0.0);
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobotTest::RevSpeedControllerTest
///
/// Test code for REV speed controllers.
///
////////////////////////////////////////////////////////////////
void EastTechRobotTest::RevSpeedControllerTest()
{
    static rev::CANSparkMax * pLeftNeo = new rev::CANSparkMax(1, rev::CANSparkLowLevel::MotorType::kBrushless);
    static rev::CANSparkMax * pRightNeo = new rev::CANSparkMax(2, rev::CANSparkLowLevel::MotorType::kBrushless);

    while (m_pJoystick->GetRawButton(1))
    {
        pLeftNeo->Set(1.0);
    }
    while (m_pJoystick->GetRawButton(2))
    {
        pLeftNeo->Set(-1.0);
    }
    while (m_pJoystick->GetRawButton(3))
    {
        pRightNeo->Set(1.0);
    }
    while (m_pJoystick->GetRawButton(4))
    {
        pRightNeo->Set(-1.0);
    }

    pLeftNeo->Set(0.0);
    pRightNeo->Set(0.0);
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobotTest::TankDrive
///
/// Test code for tank drive of the robot.
///
////////////////////////////////////////////////////////////////
void EastTechRobotTest::TankDrive()
{
    static TalonFX * pLeftDrive = new TalonFX(1);
    static TalonFX * pRightDrive = new TalonFX(2);
    pLeftDrive->Set(EASTTECH_ROBOT_OBJ()->m_pDriveController->GetAxisValue(1) * -1.0);
    pRightDrive->Set(EASTTECH_ROBOT_OBJ()->m_pDriveController->GetAxisValue(5) * -1.0);
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobotTest::SwerveDriveTest
///
/// Test code for swerve drive of the robot.
///
////////////////////////////////////////////////////////////////
void EastTechRobotTest::SwerveDriveTest()
{
    static SwerveDrive * pSwerveDrive = EASTTECH_ROBOT_OBJ()->m_pSwerveDrive;

    // Tests returning modules to absolute reference angles
    if (EASTTECH_ROBOT_OBJ()->m_pDriveController->DetectButtonChange(4))
    {
        // Not available yet
        //EASTTECH_ROBOT_OBJ()->m_pSwerveDrive->HomeModules();
    }

    // Dynamically switch between field relative and robot centric
    static bool bFieldRelative = true;
    if (EASTTECH_ROBOT_OBJ()->m_pDriveController->DetectButtonChange(5))
    {
        bFieldRelative = !bFieldRelative;
    }

    // Zero the gryo
    if (EASTTECH_ROBOT_OBJ()->m_pDriveController->DetectButtonChange(6))
    {
        pSwerveDrive->ZeroGyroYaw();
    }

    // Dynamically switch between arcade and GTA drive controls
    static bool bGtaControls = false;
    if (EASTTECH_ROBOT_OBJ()->m_pDriveController->DetectButtonChange(10))
    {
        bGtaControls = !bGtaControls;
    }

    // Get joystick inputs (x = strafe, y = translation)
    // logitech and xbox controller: strafe = kLeftX (0), translation = kLeftY(1) or triggers (2/3), rotation = kRightX (4)
    double translationAxis = 0.0;
    if (bGtaControls)
    {
        double lAxis = EASTTECH_ROBOT_OBJ()->m_pDriveController->GetAxisValue(2) * -1.0;
        double rAxis = EASTTECH_ROBOT_OBJ()->m_pDriveController->GetAxisValue(3);
        translationAxis = lAxis + rAxis;
    }
    else
    {
        translationAxis = EASTTECH_ROBOT_OBJ()->m_pDriveController->GetAxisValue(1) * -1.0;
    }
    double strafeAxis = EASTTECH_ROBOT_OBJ()->m_pDriveController->GetAxisValue(0) * -1.0;
    double rotationAxis = EASTTECH_ROBOT_OBJ()->m_pDriveController->GetAxisValue(4) * -1.0;

    strafeAxis = RobotUtils::Trim(strafeAxis, 0.10, -0.10);
    translationAxis = RobotUtils::Trim(translationAxis, 0.10, -0.10);
    rotationAxis = RobotUtils::Trim(rotationAxis, 0.10, -0.10);

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
    // Translation2d, double rotation, field relative, open loop
    pSwerveDrive->SetModuleStates(translation, rotationAxis, bFieldRelative, true);
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobotTest::PneumaticsTest
///
/// Test code for validating pneumatics.
///
////////////////////////////////////////////////////////////////
void EastTechRobotTest::PneumaticsTest()
{
    // The pneumatics library checks if channels are already in use
    // when creating the object.  The test code either has to pick
    // channels not in use (likely 6/7) or grab a reference to some
    // solenoid object from the actual robot code.
    //static DoubleSolenoid *& rpSolenoid = EASTTECH_ROBOT_OBJ()->m_pTalonCoolingSolenoid;
    static DoubleSolenoid * pSolenoid = new DoubleSolenoid(PneumaticsModuleType::CTREPCM, 6, 7);
    
    if (m_pJoystick->GetRawButton(1))
    {
        pSolenoid->Set(DoubleSolenoid::kForward);
    }
    else if (m_pJoystick->GetRawButton(2))
    {
        pSolenoid->Set(DoubleSolenoid::kReverse);
    }
    else if (m_pJoystick->GetRawButton(3))
    {
        pSolenoid->Set(DoubleSolenoid::kOff);
    }
    else
    {
    }
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobotTest::TimeTest
///
/// Test code for manually managing timing (including threads).
///
////////////////////////////////////////////////////////////////
void EastTechRobotTest::TimeTest()
{
    // Example code using standard library delays and time tracking
    static std::chrono::time_point<std::chrono::high_resolution_clock> currentTime;
    static std::chrono::time_point<std::chrono::high_resolution_clock> oldTime;
    
    currentTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = currentTime - oldTime;
    
    // Run for 100ms, sleep for 100ms
    const unsigned RUN_SLEEP_INTERVAL_MS = 100U;
    if (elapsed.count() > RUN_SLEEP_INTERVAL_MS)
    {
        auto start = std::chrono::high_resolution_clock::now();
        std::this_thread::sleep_for(std::chrono::milliseconds(RUN_SLEEP_INTERVAL_MS));
        auto end = std::chrono::high_resolution_clock::now();
        
        std::chrono::duration<double, std::milli> elapsed = end - start;
        std::cout << "Slept for " << elapsed.count() << " ms." << std::endl;
    }
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobotTest::ButtonChangeTest
///
/// Test code to verify button state change detection works.
///
////////////////////////////////////////////////////////////////
void EastTechRobotTest::ButtonChangeTest()
{
    // Sample code for testing the detect trigger change code
    if (EASTTECH_ROBOT_OBJ()->m_pDriveController->DetectButtonChange(1, EastTech::Controller::ButtonStateChanges::BUTTON_RELEASED))
    {
        RobotUtils::DisplayMessage("Trigger change detected!");
    }
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobotTest::AccelerometerTest
///
/// Test code to verify the built in accelerometer.
///
////////////////////////////////////////////////////////////////
void EastTechRobotTest::AccelerometerTest()
{
    // Test code for reading the built in accelerometer
    BuiltInAccelerometer * pAccelerometer = new BuiltInAccelerometer();
    double x = pAccelerometer->GetX();
    double y = pAccelerometer->GetY();
    double z = pAccelerometer->GetZ();
    RobotUtils::DisplayFormattedMessage("x: %f, y: %f, z: %f\n", x, y, z);
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobotTest::CandleLedsTest
///
/// Test code to verify functionality of CANdle controlled RGB
// LED strips.
///
////////////////////////////////////////////////////////////////
void EastTechRobotTest::CandleLedsTest()
{
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobotTest::RelayLedsTest
///
/// Test code to verify functionality of relay controlled RGB
/// LED strips.
///
////////////////////////////////////////////////////////////////
void EastTechRobotTest::RelayLedsTest()
{
    enum LedDisplayState
    {
        NONE,
        RED_ONLY,
        GREEN_ONLY,
        BLUE_ONLY,
        RED_GREEN,
        RED_BLUE,
        GREEN_BLUE,
        RED_GREEN_BLUE
    };
    static LedDisplayState displayState = NONE;

    // This may seem backward, but the LEDS work by creating
    // a voltage differential.  The LED strip has four lines,
    // 12V, red, green and blue.  The 12V line gets enabled by
    // one relay during initialization.  The RGB LEDs turn on
    // when there is a voltage differential, so 'on' is when
    // there is 0V on a RGB line (kOff) and 'off' is when there
    // is 12V on a RGB line (kForward).
    static const Relay::Value LEDS_ENABLED  = Relay::kForward;
    static const Relay::Value LEDS_DISABLED = Relay::kOff;
    static const Relay::Value LEDS_OFF      = Relay::kForward;
    static const Relay::Value LEDS_ON       = Relay::kOff;

    static Relay * pLedsEnableRelay = new Relay(0);     // Controls whether the LEDs will light up at all
    static Relay * pRedLedRelay     = new Relay(1);     // Controls whether or not the red LEDs are lit up
    static Relay * pGreenLedRelay   = new Relay(2);     // Controls whether or not the green LEDs are lit up
    static Relay * pBlueLedRelay    = new Relay(3);     // Controls whether or not the blue LEDs are lit up
    
    static std::chrono::time_point<std::chrono::high_resolution_clock> currentTime;
    static std::chrono::time_point<std::chrono::high_resolution_clock> oldTime;
    currentTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = currentTime - oldTime;
    if (elapsed.count() > 1000)
    {
        // kForward turns the LEDs off (voltage difference is zero)
        // kOff turns the LEDs on (voltage difference is +12V)
        switch (displayState)
        {
            case NONE:
            {
                pLedsEnableRelay->Set(LEDS_DISABLED);
                pRedLedRelay->Set(LEDS_OFF);
                pGreenLedRelay->Set(LEDS_OFF);
                pBlueLedRelay->Set(LEDS_OFF);
                displayState = RED_ONLY;
                break;
            }
            case RED_ONLY:
            {
                pLedsEnableRelay->Set(LEDS_ENABLED);
                pRedLedRelay->Set(LEDS_ON);
                pGreenLedRelay->Set(LEDS_OFF);
                pBlueLedRelay->Set(LEDS_OFF);
                displayState = GREEN_ONLY;
                break;
            }
            case GREEN_ONLY:
            {
                pRedLedRelay->Set(LEDS_OFF);
                pGreenLedRelay->Set(LEDS_ON);
                pBlueLedRelay->Set(LEDS_OFF);
                displayState = BLUE_ONLY;
                break;
            }
            case BLUE_ONLY:
            {
                pRedLedRelay->Set(LEDS_OFF);
                pGreenLedRelay->Set(LEDS_OFF);
                pBlueLedRelay->Set(LEDS_ON);
                displayState = RED_GREEN;
                break;
            }
            case RED_GREEN:
            {
                pRedLedRelay->Set(LEDS_ON);
                pGreenLedRelay->Set(LEDS_ON);
                pBlueLedRelay->Set(LEDS_OFF);
                displayState = RED_BLUE;
                break;
            }
            case RED_BLUE:
            {
                pRedLedRelay->Set(LEDS_ON);
                pGreenLedRelay->Set(LEDS_OFF);
                pBlueLedRelay->Set(LEDS_ON);
                displayState = GREEN_BLUE;
                break;
            }
            case GREEN_BLUE:
            {
                pRedLedRelay->Set(LEDS_OFF);
                pGreenLedRelay->Set(LEDS_ON);
                pBlueLedRelay->Set(LEDS_ON);
                displayState = RED_GREEN_BLUE;
                break;
            }
            case RED_GREEN_BLUE:
            {
                pRedLedRelay->Set(LEDS_ON);
                pGreenLedRelay->Set(LEDS_ON);
                pBlueLedRelay->Set(LEDS_ON);
                displayState = NONE;
                break;
            }
            default:
            {
                break;
            }
        }

        oldTime = currentTime;
    }
}
