﻿#ifndef ZED_WRAPPER_NODELET_H
#define ZED_WRAPPER_NODELET_H

///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2020, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

#include "sl_tools.h"

#include <sl/Camera.hpp>

#include <diagnostic_updater/diagnostic_updater.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

// Dynamic reconfiguration
#include <zed_nodelets/ZedConfig.h>

// Services
#include <zed_interfaces/reset_odometry.h>
#include <zed_interfaces/reset_tracking.h>
#include <zed_interfaces/set_led_status.h>
#include <zed_interfaces/set_pose.h>
#include <zed_interfaces/start_remote_stream.h>
#include <zed_interfaces/start_svo_recording.h>
#include <zed_interfaces/stop_remote_stream.h>
#include <zed_interfaces/stop_svo_recording.h>
#include <zed_interfaces/toggle_led.h>

// Topics
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <stereo_msgs/DisparityImage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

using namespace std;

namespace zed_nodelets
{
class ZEDWrapperNodelet : public nodelet::Nodelet
{
  typedef enum _dyn_params
  {
    DATAPUB_FREQ = 0,
    CONFIDENCE = 1,
    TEXTURE_CONF = 2,
    POINTCLOUD_FREQ = 3,
    BRIGHTNESS = 4,
    CONTRAST = 5,
    HUE = 6,
    SATURATION = 7,
    SHARPNESS = 8,
    GAMMA = 9,
    AUTO_EXP_GAIN = 10,
    GAIN = 11,
    EXPOSURE = 12,
    AUTO_WB = 13,
    WB_TEMP = 14
  } DynParams;

public:
  /*! \brief Default constructor
   */
  ZEDWrapperNodelet();

  /*! \brief \ref ZEDWrapperNodelet destructor
   */
  virtual ~ZEDWrapperNodelet();

protected:
  void checkVersion();
  bool initCamera();
  bool initZedParams();
  void checkCameraModel();
  void initDynamicReconfigure();
  void initAllPublishers();
  void initAllServices();

  /*! \brief Initialization function called by the Nodelet base class
   */
  virtual void onInit();

  void setupGeneral();
  void setupVideo();
  void setupDepth();
  void setupTracking();
  void setupSensors();
  void setupSvo();
  void setupCoordinateFrames();
  void logTFFrames();
  void setupTFBroadcasting();
  void setupDynamicParameters();

  /*! \brief Reads parameters from the param server
   */
  void readParameters();

  ros::WallTimer wall_timer_;
  ros::Time sim_clock_base_time;
  /*! \brief Publish the current time when use_sim_time is true
   * \param t : the ros::Time to send in the clock message
   */
  void publishSimClock(const ros::Time& stamp);

  /*! \brief sim clock update thread
   */
  void sim_clock_update(const ros::WallTimerEvent& e);

  /*! \brief get ZED image time or current time depending on params
   */
  ros::Time getTimestamp();

  /*! \brief update dynamic reconfigure parameters in the camera
   */
  bool updateCameraWithDynParams();

  /*! \brief ZED camera polling thread function
   */
  void device_poll_thread_func();

  /*! \brief Publish the pose of the camera in "Map" frame with a ros Publisher
   * \param t : the ros::Time to stamp the image
   */
  void publishPose(ros::Time t);

  /*! \brief Publish the pose of the camera in "Odom" frame with a ros Publisher
   * \param base2odomTransf : Transformation representing the camera pose
   * from base frame to odom frame
   * \param slPose : latest odom pose from ZED SDK
   * \param t : the ros::Time to stamp the image
   */
  void publishOdom(tf2::Transform odom2baseTransf, sl::Pose& slPose, ros::Time t);

  /*! \brief Publish the pose of the camera in "Map" frame as a transformation
   * \param baseTransform : Transformation representing the camera pose from
   * odom frame to map frame
   * \param t : the ros::Time to stamp the image
   */
  void publishPoseFrame(tf2::Transform baseTransform, ros::Time t);

  /*! \brief Publish the odometry of the camera in "Odom" frame as a
   * transformation
   * \param odomTransf : Transformation representing the camera pose from
   * base frame to odom frame
   * \param t : the ros::Time to stamp the image
   */
  void publishOdomFrame(tf2::Transform odomTransf, ros::Time t);

  /*!
   * \brief Publish IMU frame once as static TF
   */
  void publishStaticImuFrame(const ros::Time& t);

  /*! \brief Publish a sl::Mat image with a ros Publisher
   * \param imgMsgPtr : the image message to publish
   * \param img : the image to publish
   * \param pubImg : the publisher object to use (different image publishers
   * exist)
   * \param camInfoMsg : the camera_info to be published with image
   * \param imgFrameId : the id of the reference frame of the image (different
   * image frames exist)
   * \param t : the ros::Time to stamp the image
   */
  void publishImage(sensor_msgs::ImagePtr imgMsgPtr, sl::Mat img, image_transport::CameraPublisher& pubImg,
                    sensor_msgs::CameraInfoPtr camInfoMsg, string imgFrameId, ros::Time t);

  /*! \brief Publish a sl::Mat depth image with a ros Publisher
   * \param imgMsgPtr : the depth image topic message to publish
   * \param depth : the depth image to publish
   * \param t : the ros::Time to stamp the depth image
   */
  void publishDepth(sensor_msgs::ImagePtr imgMsgPtr, sl::Mat depth, ros::Time t);

  /*! \brief Publish the informations of a camera with a ros Publisher
   * \param cam_info_msg : the information message to publish
   * \param pub_cam_info : the publisher object to use
   * \param t : the ros::Time to stamp the message
   */
  void publishCamInfo(sensor_msgs::CameraInfoPtr camInfoMsg, ros::Publisher pubCamInfo, ros::Time t);

  /*! \brief Publish a sl::Mat disparity image with a ros Publisher
   * \param disparity : the disparity image to publish
   * \param t : the ros::Time to stamp the depth image
   */
  void publishDisparity(sl::Mat disparity, ros::Time t);

  /*! \brief Publish sensors data and TF
   * \param t : the ros::Time to stamp the depth image
   */
  void publishSensData(ros::Time t = ros::Time(0));

  /*! \brief Get the information of the ZED cameras and store them in an
   * information message
   * \param zed : the sl::zed::Camera* pointer to an instance
   * \param left_cam_info_msg : the information message to fill with the left
   * camera informations
   * \param right_cam_info_msg : the information message to fill with the right
   * camera informations
   * \param left_frame_id : the id of the reference frame of the left camera
   * \param right_frame_id : the id of the reference frame of the right camera
   */
  void fillCamInfo(sl::Camera& zed, sensor_msgs::CameraInfoPtr leftCamInfoMsg,
                   sensor_msgs::CameraInfoPtr rightCamInfoMsg, string leftFrameId, string rightFrameId,
                   bool rawParam = false);

  /*! \brief Get the information of the ZED cameras and store them in an
   * information message for depth topics
   * \param zed : the sl::zed::Camera* pointer to an instance
   * \param depth_info_msg : the information message to fill with the left
   * camera informations
   * \param frame_id : the id of the reference frame of the left camera
   */
  void fillCamDepthInfo(sl::Camera& zed, sensor_msgs::CameraInfoPtr depth_info_msg, string frame_id);

  /* \bried Check if FPS and Resolution chosen by user are correct.
   *        Modifies FPS to match correct value.
   */
  void checkResolFps();

  /*! \brief Callback to handle dynamic reconfigure events in ROS
   */
  void callback_dynamicReconf(zed_nodelets::ZedConfig& config, uint32_t level);

  bool getCamData(
      const ros::Time update_stamp,
      const bool get_left,
      const bool get_left_raw,
      const bool get_right,
      const bool get_right_raw,
      const bool get_depth,
      const bool get_disparity,
      const bool get_conf_map,
      sl::Mat& mat_left, sl::Mat& mat_left_raw,
      sl::Mat& mat_right, sl::Mat& mat_right_raw,
      sl::Mat& mat_depth, sl::Mat& mat_disp, sl::Mat& mat_conf,
      ros::Time& stamp,
      sl::Timestamp& grab_ts, sl::Timestamp& lastZedTs);

  /*! \brief Callback to publish Video and Depth data
   * \param e : the ros::TimerEvent binded to the callback
   */
  void callback_pubVideoDepth(const ros::TimerEvent& e);

  /*! \brief Callback to publish Path data with a ROS publisher.
   * \param e : the ros::TimerEvent binded to the callback
   */
  void callback_pubPath(const ros::TimerEvent& e);

  /*! \brief Callback to publish Sensor Data with a ROS publisher.
   * \param e : the ros::TimerEvent binded to the callback
   */
  void callback_pubSensorsData(const ros::TimerEvent& e);

  /*! \brief Callback to update node diagnostic status
   * \param stat : node status
   */
  void callback_updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat);

  /*! \brief Service callback to reset_tracking service
   * Tracking pose is reinitialized to the value available in the ROS Param
   * server
   */
  bool on_reset_tracking(zed_interfaces::reset_tracking::Request& req, zed_interfaces::reset_tracking::Response& res);

  /*! \brief Service callback to reset_odometry service
   *        Odometry is reset to clear drift and odometry frame gets the latest
   * pose
   *        from ZED tracking.
   */
  bool on_reset_odometry(zed_interfaces::reset_odometry::Request& req, zed_interfaces::reset_odometry::Response& res);

  /*! \brief Service callback to set_pose service
   *        Tracking pose is set to the new values
   */
  bool on_set_pose(zed_interfaces::set_pose::Request& req, zed_interfaces::set_pose::Response& res);

  /*! \brief convenience function to try different compressions when starting recording
   */
  sl::ERROR_CODE enableRecordingAllCompressions(sl::RecordingParameters& recParams);

  /*! \brief Service callback to start_svo_recording service
   */
  bool on_start_svo_recording(zed_interfaces::start_svo_recording::Request& req,
                              zed_interfaces::start_svo_recording::Response& res);

  /*! \brief Service callback to stop_svo_recording service
   */
  bool on_stop_svo_recording(zed_interfaces::stop_svo_recording::Request& req,
                             zed_interfaces::stop_svo_recording::Response& res);

  /*! \brief Service callback to start_remote_stream service
   */
  bool on_start_remote_stream(zed_interfaces::start_remote_stream::Request& req,
                              zed_interfaces::start_remote_stream::Response& res);

  /*! \brief Service callback to stop_remote_stream service
   */
  bool on_stop_remote_stream(zed_interfaces::stop_remote_stream::Request& req,
                             zed_interfaces::stop_remote_stream::Response& res);

  /*! \brief Service callback to set_led_status service
   */
  bool on_set_led_status(zed_interfaces::set_led_status::Request& req, zed_interfaces::set_led_status::Response& res);

  /*! \brief Service callback to toggle_led service
   */
  bool on_toggle_led(zed_interfaces::toggle_led::Request& req, zed_interfaces::toggle_led::Response& res);

  /*! \brief Utility to initialize the pose variables
   */
  bool set_pose(float xt, float yt, float zt, float rr, float pr, float yr);

  /*! \brief Utility to initialize the most used transforms
   */
  void initTransforms();

  /*! \brief Utility to initialize the static transform from Sensor to Base
   */
  bool getSens2BaseTransform();

  /*! \brief Utility to initialize the static transform from Sensor to Camera
   */
  bool getSens2CameraTransform();

  /*! \brief Utility to initialize the static transform from Camera to Base
   */
  bool getCamera2BaseTransform();

  /* \bried Start tracking
   */
  void start_pos_tracking();

  /*! \brief Generates an univoque color for each object class ID
   */
  inline sl::float3 generateColorClass(int idx)
  {
    sl::float3 clr;
    clr.r = static_cast<uint8_t>(33 + (idx * 456262));
    clr.g = static_cast<uint8_t>(233 + (idx * 1564684));
    clr.b = static_cast<uint8_t>(133 + (idx * 76873242));
    return clr / 255.f;
  }

  /*! \brief Update Dynamic reconfigure server parameters from member variables
   */
  void updateDynamicReconfigureServer();

private:
  uint64_t mFrameCount = 0;

  // SDK version
  int mVerMajor;
  int mVerMinor;
  int mVerSubMinor;

  // ROS
  ros::NodeHandle mNh;
  ros::NodeHandle mNhNs;
  std::thread mDevicePollThread;
  std::thread mPcThread;  // Point Cloud thread

  bool mStopNode = false;

  const double mSensPubRate = 400.0;

  // Publishers
  image_transport::CameraPublisher mPubLeft;      //
  image_transport::CameraPublisher mPubRawLeft;   //
  image_transport::CameraPublisher mPubRight;     //
  image_transport::CameraPublisher mPubRawRight;  //
  image_transport::CameraPublisher mPubDepth;     //
  image_transport::CameraPublisher mPubConfMap;

  ros::Publisher mPubSimClock;

  ros::Publisher mPubDisparity;  //
  ros::Publisher mPubCloud;
  ros::Publisher mPubPose;
  ros::Publisher mPubPoseCov;
  ros::Publisher mPubOdom;
  ros::Publisher mPubOdomPath;
  ros::Publisher mPubMapPath;
  ros::Publisher mPubImu;
  ros::Publisher mPubImuRaw;
  ros::Publisher mPubImuTemp;
  ros::Publisher mPubImuMag;
  // ros::Publisher mPubImuMagRaw;
  ros::Publisher mPubPressure;
  ros::Publisher mPubTempL;
  ros::Publisher mPubTempR;
  ros::Publisher mPubCamImuTransf;

  // Timers
  ros::Timer mImuTimer;
  ros::Timer mPathTimer;
  ros::Timer mVideoDepthTimer;

  // Services
  ros::ServiceServer mSrvSetInitPose;
  ros::ServiceServer mSrvResetOdometry;
  ros::ServiceServer mSrvResetTracking;
  ros::ServiceServer mSrvSvoStartRecording;
  ros::ServiceServer mSrvSvoStopRecording;
  ros::ServiceServer mSrvSvoStartStream;
  ros::ServiceServer mSrvSvoStopStream;
  ros::ServiceServer mSrvSetLedStatus;
  ros::ServiceServer mSrvToggleLed;

  // ----> Topics (ONLY THOSE NOT CHANGING WHILE NODE RUNS)
  // Camera info
  sensor_msgs::CameraInfoPtr mLeftCamInfoMsg;
  sensor_msgs::CameraInfoPtr mRightCamInfoMsg;
  sensor_msgs::CameraInfoPtr mLeftCamInfoRawMsg;
  sensor_msgs::CameraInfoPtr mRightCamInfoRawMsg;
  sensor_msgs::CameraInfoPtr mDepthCamInfoMsg;

  geometry_msgs::TransformPtr mCameraImuTransfMgs;
  // <---- Topics

  // ROS TF
  tf2_ros::TransformBroadcaster mTransformPoseBroadcaster;
  tf2_ros::TransformBroadcaster mTransformOdomBroadcaster;
  tf2_ros::StaticTransformBroadcaster mStaticTransformImuBroadcaster;

  std::string mDepthFrameId;
  std::string mDepthOptFrameId;

  std::string mDisparityFrameId;
  std::string mDisparityOptFrameId;

  std::string mConfidenceFrameId;
  std::string mConfidenceOptFrameId;

  std::string mCloudFrameId;

  std::string mMapFrameId;
  std::string mOdometryFrameId;
  std::string mBaseFrameId;
  std::string mCameraFrameId;

  std::string mRightCamFrameId;
  std::string mRightCamOptFrameId;
  std::string mLeftCamFrameId;
  std::string mLeftCamOptFrameId;
  std::string mImuFrameId;

  std::string mBaroFrameId;
  std::string mMagFrameId;
  std::string mTempLeftFrameId;
  std::string mTempRightFrameId;

  bool mPublishTf;
  bool mPublishMapTf;
  bool mPublishImuTf;
  bool mCameraFlip;
  bool mCameraSelfCalib;

  // Launch file parameters
  std::string mCameraName;
  sl::RESOLUTION mCamResol;
  int mCamFrameRate;
  sl::DEPTH_MODE mDepthMode;
  sl::SENSING_MODE mCamSensingMode;
  int mGpuId;
  int mZedId;
  int mDepthStabilization;
  std::string mAreaMemDbPath;
  std::string mSvoFilepath;
  std::string mRecordSvoFilepath;
  std::string mRemoteStreamAddr;
  bool mSensTimestampSync;
  double mPathPubRate;
  int mPathMaxCount;
  bool mVerbose;
  bool mSvoMode = false;
  double mCamMinDepth;
  double mCamMaxDepth;

  // Positional tracking
  bool mPosTrackingEnabled=false;
  bool mPosTrackingActivated=false;
  bool mPosTrackingReady=false;
  bool mTwoDMode = false;
  double mFixedZValue = 0.0;
  bool mFloorAlignment = false;
  bool mImuFusion = true;
  bool mGrabActive = false;  // Indicate if camera grabbing is active (at least one topic subscribed)
  sl::ERROR_CODE mConnStatus;
  sl::ERROR_CODE mGrabStatus;
  sl::POSITIONAL_TRACKING_STATE mPosTrackingStatus;
  bool mSensPublishing = false;

  // Last frame time
  ros::Time mPrevFrameTimestamp;
  ros::Time mFrameTimestamp;

  // Positional Tracking variables
  sl::Pose mLastZedPose;  // Sensor to Map transform
  sl::Transform mInitialPoseSl;
  std::vector<float> mInitialBasePose;
  std::vector<geometry_msgs::PoseStamped> mOdomPath;
  std::vector<geometry_msgs::PoseStamped> mMapPath;

  // IMU TF
  tf2::Transform mLastImuPose;

  // TF Transforms
  tf2::Transform mMap2OdomTransf;       // Coordinates of the odometry frame in map frame
  tf2::Transform mOdom2BaseTransf;      // Coordinates of the base in odometry frame
  tf2::Transform mMap2BaseTransf;       // Coordinates of the base in map frame
  tf2::Transform mMap2CameraTransf;     // Coordinates of the camera in base frame
  tf2::Transform mSensor2BaseTransf;    // Coordinates of the base frame in sensor frame
  tf2::Transform mSensor2CameraTransf;  // Coordinates of the camera frame in sensor frame
  tf2::Transform mCamera2BaseTransf;    // Coordinates of the base frame in camera frame

  bool mSensor2BaseTransfValid = false;
  bool mSensor2CameraTransfValid = false;
  bool mCamera2BaseTransfValid = false;
  bool mStaticImuFramePublished = false;

  // initialization Transform listener
  boost::shared_ptr<tf2_ros::Buffer> mTfBuffer;
  boost::shared_ptr<tf2_ros::TransformListener> mTfListener;

  // Zed object
  sl::InitParameters mZedParams;
  sl::Camera mZed;
  unsigned int mZedSerialNumber;
  sl::MODEL mZedUserCamModel;   // Camera model set by ROS Param
  sl::MODEL mZedRealCamModel;   // Real camera model by SDK API
  unsigned int mCamFwVersion;   // Camera FW version
  unsigned int mSensFwVersion;  // Sensors FW version

  // Dynamic Parameters
  int mCamBrightness = 4;
  int mCamContrast = 4;
  int mCamHue = 0;
  int mCamSaturation = 4;
  int mCamSharpness = 3;
  int mCamGamma = 1;
  bool mCamAutoExposure = true;
  int mCamGain = 100;
  int mCamExposure = 100;
  bool mCamAutoWB = true;
  int mCamWB = 4200;

  int mCamDepthConfidence = 50;
  int mCamDepthTextureConf = 100;
  double mVideoDepthFreq = 15.;

  double mCamImageResizeFactor = 1.0;
  double mCamDepthResizeFactor = 1.0;

  // flags  
  bool mUseSimTime = false;
  bool mTriggerAutoExposure = true;
  bool mTriggerAutoWB = true;
  bool mComputeDepth;
  bool mOpenniDepthMode;  // 16 bit UC data in mm else 32F in m, for more info -> http://www.ros.org/reps/rep-0118.html
  bool mPoseSmoothing = false;  // Always disabled. Enable only for AR/VR applications
  bool mAreaMemory;
  bool mInitOdomWithPose;
  bool mResetOdom = false;
  bool mUseOldExtrinsic = false;
  bool mUpdateDynParams = false;
  bool mPublishingData = false;

  // SVO recording
  bool mRecording = false;
  sl::RecordingStatus mRecStatus;
  sl::SVO_COMPRESSION_MODE mSvoComprMode;

  // Streaming
  bool mStreaming = false;

  // Mat
  int mCamWidth;
  int mCamHeight;
  sl::Resolution mMatResolVideo;
  sl::Resolution mMatResolDepth;

  // Thread Sync
  std::mutex mCloseZedMutex;
  std::mutex mCamDataMutex;
  std::mutex mPcMutex;
  std::mutex mRecMutex;
  std::mutex mPosTrkMutex;
  std::mutex mDynParMutex;
  std::condition_variable mPcDataReadyCondVar;
  bool mPcDataReady;
  std::condition_variable mRgbDepthDataRetrievedCondVar;
  bool mRgbDepthDataRetrieved;

  // Dynamic reconfigure
  boost::recursive_mutex mDynServerMutex;  // To avoid Dynamic Reconfigure Server warning
  boost::shared_ptr<dynamic_reconfigure::Server<zed_nodelets::ZedConfig>> mDynRecServer;

  // Diagnostic
  float mTempLeft = -273.15f;
  float mTempRight = -273.15f;
  std::unique_ptr<sl_tools::CSmartMean> mElabPeriodMean_sec;
  std::unique_ptr<sl_tools::CSmartMean> mGrabPeriodMean_usec;
  std::unique_ptr<sl_tools::CSmartMean> mVideoDepthPeriodMean_sec;
  std::unique_ptr<sl_tools::CSmartMean> mSensPeriodMean_usec;

  diagnostic_updater::Updater mDiagUpdater;  // Diagnostic Updater

  // Camera IMU transform
  sl::Transform mSlCamImuTransf;
  geometry_msgs::TransformStamped mStaticImuTransformStamped;
};  // class ZEDROSWrapperNodelet
}  // namespace zed_nodelets

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(zed_nodelets::ZEDWrapperNodelet, nodelet::Nodelet);

#endif  // ZED_WRAPPER_NODELET_H
