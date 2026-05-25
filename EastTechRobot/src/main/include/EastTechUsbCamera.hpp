////////////////////////////////////////////////////////////////////////////////
/// @file   EastTechUsbCamera.hpp
/// @author David Stalter
///
/// @details
/// A class designed to support USB camera functionality on the robot.
///
/// Copyright (c) 2024 East Technical High School
////////////////////////////////////////////////////////////////////////////////

#ifndef EASTTECHUSBCAMERA_HPP
#define EASTTECHUSBCAMERA_HPP

// SYSTEM INCLUDES
// <none>

// C INCLUDES
#include "frc/smartdashboard/SmartDashboard.h"  // for smart dashboard support
#include "frc/Timer.h"                          // for creating a Timer

// C++ INCLUDES
#include "RobotUtils.hpp"                       // for DisplayMessage()

// gcc 11.1 introduced this warning and mat.hpp hasn't been fully cleaned up yet
DISABLE_WARNING("-Wdeprecated-enum-enum-conversion")
#include "opencv2/core/mat.hpp"
ENABLE_WARNING("-Wdeprecated-enum-enum-conversion")
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/core/types.hpp"

// Indirectly includes opencv stuff, must happen after the warning disable/enable
#include "cameraserver/CameraServer.h"

using namespace frc;


////////////////////////////////////////////////////////////////
/// @class EastTechUsbCamera
///
/// Class that provides methods for interacting with USB cameras.
/// Note: A lot of this is legacy code that has not been used
///       for a long time.
///
////////////////////////////////////////////////////////////////
class EastTechUsbCamera
{
public:
    
    enum CameraType
    {
        // These double as array indices, so use caution when modifying them
        FRONT_USB,
        BACK_USB,
        MAX_NUM_USB_CAMERAS
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
    
    // Toggle between what processed image is shown on the dashboard
    static void ToggleCameraProcessedImage();
    
    // The vision thread itself
    static void UsbVisionThread();

private:

    // Update values on the SmartDashboard
    static void UpdateSmartDashboard();

    // Create the camera objects for any configured cameras
    static bool CreateConfiguredCameras();

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
    EastTechUsbCamera();

    // Destructor, copy constructor, assignment operator
    ~EastTechUsbCamera();

    EastTechUsbCamera(const EastTechUsbCamera &) = delete;
    EastTechUsbCamera & operator=(const EastTechUsbCamera &) = delete;
    
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
};



////////////////////////////////////////////////////////////////
/// @method EastTechUsbCamera::SetFullProcessing
///
/// This method sets whether or not full vision processing
/// should occur.
///
////////////////////////////////////////////////////////////////
inline void EastTechUsbCamera::SetFullProcessing(bool bState)
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
/// @method EastTechUsbCamera::SetCamera
///
/// This method sets which camera is active.
///
////////////////////////////////////////////////////////////////
inline void EastTechUsbCamera::SetCamera(CameraType camera)
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
/// @method EastTechUsbCamera::ToggleCamera
///
/// This method toggles between which camera is active.
///
////////////////////////////////////////////////////////////////
inline void EastTechUsbCamera::ToggleCamera()
{
    CameraType nextCam = (m_pCurrentUsbCamera->CAM_TYPE == FRONT_USB) ? BACK_USB : FRONT_USB;
    SetCamera(nextCam);
}

#endif // EASTTECHUSBCAMERA_HPP
