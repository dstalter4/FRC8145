////////////////////////////////////////////////////////////////////////////////
/// @file   RobotCamera.hpp
/// @author David Stalter
///
/// @details
/// A class designed to support camera functionality on the robot.
///
/// Copyright (c) 2023 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

#ifndef ROBOTCAMERA_HPP
#define ROBOTCAMERA_HPP

// SYSTEM INCLUDES
// <none>

// C INCLUDES
#include "frc/smartdashboard/SmartDashboard.h"  // for smart dashboard support
#include "frc/Timer.h"                          // for creating a Timer
#include "networktables/NetworkTable.h"         // for interacting with network tables
#include "cameraserver/CameraServer.h"          // for camera support

// C++ INCLUDES
#include "RobotUtils.hpp"                       // for DisplayMessage()

// gcc 11.1 introduced this warning and no one cleaned up opencv yet
DISABLE_WARNING("-Wdeprecated-enum-enum-conversion")
#include "opencv2/imgproc/imgproc.hpp"          // for vision structures and routines
#include "opencv2/core/core.hpp"
#include "opencv2/core/types.hpp"
ENABLE_WARNING("-Wdeprecated-enum-enum-conversion")

using namespace frc;


////////////////////////////////////////////////////////////////
/// @class RobotCamera
///
/// Class that provides methods for interacting with the camera.
///
////////////////////////////////////////////////////////////////
class RobotCamera
{
public:
    
    enum CameraType
    {
        // These double as array indices, so use caution when modifying them
        FRONT_USB,
        BACK_USB,
        MAX_NUM_USB_CAMERAS
    };

    enum LimelightMode
    {
        // Values taken from the limelight documentation
        VISION_PROCESSOR    = 0,
        DRIVER_CAMERA       = 1
    };

    enum LimelightLedMode
    {
        // Values taken from the limelight documentation
        PIPELINE            = 0,
        ARRAY_OFF           = 1,
        ARRAY_BLINK         = 2,
        ARRAY_ON            = 3
    };
    
    // A structure for autonomous camera seeking operations
    struct AutonomousCamera
    {
    public:
        enum SeekDirection
        {
            SEEK_LEFT,
            SEEK_RIGHT
        };

        static bool AlignToTarget(SeekDirection seekDirection, const bool bEnableMotors = true);
        static void AlignToTargetSwerve();

    private:
        static Timer m_AutoCameraTimer;
        static double m_IntegralSum;

        static constexpr units::second_t MAX_CAMERA_SEARCH_TIME_S = 5.0_s;
        static constexpr double MAX_SEEK_MOTOR_SPEED = 0.25;
        static constexpr double KI = 0.0001;
        static constexpr double KP = 0.001;
        static constexpr double INTEGRAL_SUM_LIMIT_VALUE = 10000.0;
    };
    
    // Set whether or not full vision processing can occur
    inline static void SetFullProcessing(bool bState);
    
    // Pick a camera to use
    inline static void SetCamera(CameraType camera);
    
    // Toggle between cameras
    inline static void ToggleCamera();

    // Set the limelight mode
    inline static void SetLimelightPipeline(int32_t pipelineNum);

    // Set the limelight mode
    inline static void SetLimelightMode(LimelightMode mode);

    // Set the state of the limelight LED array
    inline static void SetLimelightLedMode(LimelightLedMode ledMode);
    
    // Toggle between what processed image is shown on the dashboard
    static void ToggleCameraProcessedImage();
    
    // The vision thread itself
    static void VisionThread();
    
    // Vision thread for using a limelight camera
    static void LimelightThread();

private:
    
    // Create the camera objects for any configured cameras
    static bool CreateConfiguredCameras();

    // Update values on the SmartDashboard
    static void UpdateSmartDashboard();

    // Process a mat through the vision pipeline
    static void ProcessImage();
    
    // Specific operations the vision pipeline will perform
    static void FilterImageHsv();
    static void ErodeImage();
    static void FindContours();
    static void FilterContours();

    // Process the filtered contours to find the reflective tape
    static void FindReflectiveTapeTarget();

    // Compute some useful information about the reflective tape
    static void CalculateReflectiveTapeValues();

    // Constructor
    RobotCamera();

    // Destructor, copy constructor, assignment operator
    ~RobotCamera();

    RobotCamera(const RobotCamera &) = delete;
    RobotCamera & operator=(const RobotCamera &) = delete;
    
    // MEMBER VARIABLES
    
    // A structure to hold measurements of a contour
    struct VisionTargetReport
    {
        double m_BoundingRectX;             // Bounding rectangle top left corner X
        double m_BoundingRectY;             // Bounding rectangle top left corner Y
        double m_BoundingRectWidth;         // Bounding rectangle width
        double m_BoundingRectHeight;        // Bounding rectangle height
        double m_BoundingRectArea;          // Bounding rectangle area
        double m_BoundingRectAspectRatio;   // Bounding rectangle aspect ratio
        
        double m_Area;                      // Contour area
        double m_Perimeter;                 // Contour perimeter
        double m_ConvexHullArea;            // Contour convex hull area
        double m_Solidity;                  // Contour solidity
        double m_Vertices;                  // Contour vertices count

        double m_PercentAreaToImageArea;    // Percentage of the area the contour occupies
        double m_TrapezoidPercent;          // Likelihood that this is a true rectangle
        double m_CameraDistanceX;           // Distance to the target from the camera, measured by width
        double m_CameraDistanceY;           // Distance to the target from the camera, measured by height
        double m_GroundDistance;            // Actual ground distance to the target
        bool   m_bTargetInRange;            // Remember the last result from full vision processing
        bool   m_bIsValid;                  // Indicates if the current report is valid
    };

    // A structure to hold information about a USB camera.
    // If Axis camera support is ever needed, this
    // will probably have to derive from a base class.
    struct UsbCameraInfo
    {
        cs::UsbCamera       m_UsbCam;                       // The camera object
        cs::CvSink          m_CamSink;                      // The sink for the camera
        bool                m_bIsPresent;                   // Indicates there is actual camera info for this entry
        int                 m_DeviceNum;                    // The hardware device number for the camera
        const CameraType    CAM_TYPE;                       // Camera type (e.g. front or back)
        const int           X_RESOLUTION;                   // Camera x resolution
        const int           Y_RESOLUTION;                   // Camera y resolution
        const int           FPS;                            // Camera frames per second

        // The values for resolution apparently matter, as
        // nothing shows up in the driver station at lower resolutions.
        static const int DEFAULT_X_RESOLUTION = 640;
        static const int DEFAULT_Y_RESOLUTION = 480;
        static const int DEFAULT_FPS = 30;
        
        // Consructor
        UsbCameraInfo(const CameraType camType, int devNum, const int xRes = DEFAULT_X_RESOLUTION, const int yRes = DEFAULT_Y_RESOLUTION, const int fps = DEFAULT_FPS);
    };
    
    // Represents the memory where the information on USB camera will be stored
    union UsbCameraStorage
    {
        // The storage as raw bytes and the UsbCameraInfo objects
        uint8_t m_RawStorage[MAX_NUM_USB_CAMERAS * sizeof(UsbCameraInfo)];
        UsbCameraInfo m_CamerasInfo[MAX_NUM_USB_CAMERAS];
        
        // Constructor
        UsbCameraStorage()
        {
            // Just zero out the memory
            std::memset(&m_RawStorage, 0U, sizeof(UsbCameraStorage));
        }
        
        // The destructor should never be called, but implement it anyway
        ~UsbCameraStorage()
        {
            for (int i = 0; i < MAX_NUM_USB_CAMERAS; i++)
            {
                if (m_CamerasInfo[i].m_bIsPresent)
                {
                    // These might not be strictly needed, but it's safer to destroy the object
                    m_CamerasInfo[i].m_UsbCam.~UsbCamera();
                    m_CamerasInfo[i].m_CamSink.~CvSink();
                }
            }
        }
    };
    
    // Camera related variables
    static std::shared_ptr<nt::NetworkTable>    m_pLimelightNetworkTable;           // Network table for the limelight camera
    static UsbCameraStorage                     m_UsbCameras;                       // Memory for storing the USB camera objects
    static UsbCameraInfo *                      m_pCurrentUsbCamera;                // Pointer to the currently selected USB camera object   
    static cs::CvSource                         m_CameraOutput;                     // Output source for processed images
    static int                                  m_NumUsbCamerasPresent;             // How many cameras are present on the robot
    
    // Mats
    static cv::Mat                              m_SourceMat;                        // The originating source mat from the current camera
    static cv::Mat                              m_ResizeOutputMat;                  // Resized source mat
    static cv::Mat                              m_HsvThresholdOutputMat;            // HSV filtered mat
    static cv::Mat                              m_ErodeOutputMat;                   // Erode output mat
    static cv::Mat                              m_ContoursMat;                      // Contours output mat
    static cv::Mat                              m_FilteredContoursMat;              // Filtered contours output mat
    static cv::Mat                              m_VisionTargetMat;                  // The best candidate vision target mat
    static cv::Mat *                            m_pDashboardMat;                    // Pointer to which mat should currently be sent to the dashboard
    
    // Image artifacts represented by std::vector
    static std::vector<std::vector<cv::Point>>  m_Contours;                         // Contours in the image
    static std::vector<std::vector<cv::Point>>  m_FilteredContours;                 // Filtered contours in the image
    
    // Misc
    static std::vector<VisionTargetReport>      m_ContourTargetReports;             // Stores information about the contours currently visible
    static VisionTargetReport                   m_VisionTargetReport;               // Information about the vision target
    static bool                                 m_bDoFullProcessing;                // Indicates whether or not full image processing should occur
    static unsigned                             m_CameraHeartBeat;                  // Keep alive for the camera thread
    
    // CONSTANTS
    
    static const unsigned                       CAMERA_THREAD_SLEEP_TIME_MS         = 100U;
    static const bool                           FRONT_USB_CAMERA_SUPPORTED          = true;
    static const bool                           BACK_USB_CAMERA_SUPPORTED           = false;
    static const char *                         CAMERA_OUTPUT_NAME;
    
    static constexpr double                     TARGET_WIDTH_INCHES                 =  2.0;
    static constexpr double                     TARGET_HEIGHT_INCHES                = 16.0;
    static constexpr double                     TARGET_HEIGHT_FROM_GROUND           =  2.0;
    //static constexpr double                     TARGET_MIN_AREA_PERCENT             = 0.0;
    //static constexpr double                     TARGET_MAX_AREA_PERCENT             = 100.0;
    //static constexpr double                     TARGET_RANGE_MIN                    = 132.0;
    //static constexpr double                     TARGET_RANGE_MAX                    = 192.0;
    //static constexpr double                     GROUND_DISTANCE_TOLERANCE           = 6.0;
    static constexpr double                     CAMERA_FOV_DEGREES                  = 50.0;
    static constexpr double                     CAMERA_DIAGONAL_FOV_DEGREES         = 78.0;
    static constexpr double                     CALIBRATED_CAMERA_ANGLE             = 21.5778173;
    static constexpr double                     DEGREES_TO_RADIANS                  = M_PI / 180.0;
    static constexpr double                     DECIMAL_TO_PERCENT                  = 100.0;
};



////////////////////////////////////////////////////////////////
/// @method RobotCamera::SetLimelightPipeline
///
/// This method sets the pipeline used by the limelight camera.
///
////////////////////////////////////////////////////////////////
inline void RobotCamera::SetLimelightPipeline(int32_t pipelineNum)
{
    m_pLimelightNetworkTable->PutNumber("pipeline", pipelineNum);
}



////////////////////////////////////////////////////////////////
/// @method RobotCamera::SetLimelightMode
///
/// This method sets the mode of the limelight camera.
///
////////////////////////////////////////////////////////////////
inline void RobotCamera::SetLimelightMode(LimelightMode mode)
{
    m_pLimelightNetworkTable->PutNumber("camMode", static_cast<int>(mode));
}



////////////////////////////////////////////////////////////////
/// @method RobotCamera::SetLimelightLedMode
///
/// This method sets the mode of the limelight camera.
///
////////////////////////////////////////////////////////////////
inline void RobotCamera::SetLimelightLedMode(LimelightLedMode ledMode)
{
    m_pLimelightNetworkTable->PutNumber("ledMode", static_cast<int>(ledMode));
}



////////////////////////////////////////////////////////////////
/// @method RobotCamera::SetFullProcessing
///
/// This method sets whether or not full vision processing
/// should occur.
///
////////////////////////////////////////////////////////////////
inline void RobotCamera::SetFullProcessing(bool bState)
{
    m_bDoFullProcessing = bState;
    
    if (!m_bDoFullProcessing)
    {
        // If processing was previously enabled,
        // need to switch back to the default mat.
        m_pDashboardMat = &m_SourceMat;
        SmartDashboard::PutString("Camera Output", "Default");
    }
}



////////////////////////////////////////////////////////////////
/// @method RobotCamera::SetCamera
///
/// This method sets which camera is active.
///
////////////////////////////////////////////////////////////////
inline void RobotCamera::SetCamera(CameraType camera)
{
    // Make sure the camera is present before trying to switch
    if (m_UsbCameras.m_CamerasInfo[camera].m_bIsPresent)
    {
        m_pCurrentUsbCamera = &m_UsbCameras.m_CamerasInfo[camera];
    }
    else
    {
        RobotUtils::DisplayMessage("Desired camera not present/configured.");
    }
}



////////////////////////////////////////////////////////////////
/// @method RobotCamera::ToggleCamera
///
/// This method toggles between which camera is active.
///
////////////////////////////////////////////////////////////////
inline void RobotCamera::ToggleCamera()
{
    CameraType nextCam = (m_pCurrentUsbCamera->CAM_TYPE == FRONT_USB) ? BACK_USB : FRONT_USB;
    SetCamera(nextCam);
}

#endif // ROBOTCAMERA_HPP
