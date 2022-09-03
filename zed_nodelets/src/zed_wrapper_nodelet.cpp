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

#include "zed_nodelets/zed_wrapper_nodelet.hpp"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <chrono>
#include <csignal>

#ifndef NDEBUG
#include <ros/console.h>
#endif

//#define DEBUG_SENS_TS 1

namespace zed_nodelets
{
#ifndef DEG2RAD
#define DEG2RAD 0.017453293
#define RAD2DEG 57.295777937
#endif

#define MAG_FREQ 50.
#define BARO_FREQ 25.

ZEDWrapperNodelet::ZEDWrapperNodelet() : Nodelet()
{
}

ZEDWrapperNodelet::~ZEDWrapperNodelet()
{
  if (mDevicePollThread.joinable())
  {
    mDevicePollThread.join();
  }
}

void ZEDWrapperNodelet::initAllPublishers()
{
  // Image publishers
  image_transport::ImageTransport it_zed(mNhNs);

  mPubLeft = it_zed.advertiseCamera("left/image_rect_color", 1);
  mPubRawLeft = it_zed.advertiseCamera("left_raw/image_raw_color", 1);
  mPubRight = it_zed.advertiseCamera("right/image_rect_color", 1);
  mPubRawRight = it_zed.advertiseCamera("right_raw/image_raw_color", 1);
  mPubDepth = it_zed.advertiseCamera("depth/depth_registered", 1);
  mPubConfMap = it_zed.advertiseCamera("confidence/confidence_map", 1);

  if (mUseSimTime)
  {
    mPubSimClock = mNhNs.advertise<rosgraph_msgs::Clock>("/clock", 2);
  }

  mPubDisparity = mNhNs.advertise<stereo_msgs::DisparityImage>("disparity/disparity_topic",
      static_cast<int>(mVideoDepthFreq));
  mPubPose = mNhNs.advertise<geometry_msgs::PoseStamped>("pose", 1);
  mPubPoseCov = mNhNs.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_with_covariance", 1);
  mPubOdom = mNhNs.advertise<nav_msgs::Odometry>("odom", 1);

  // Camera Path
  if (mPathPubRate > 0)
  {
    mPubOdomPath = mNhNs.advertise<nav_msgs::Path>("path_odom", 1, true);
    mPubMapPath = mNhNs.advertise<nav_msgs::Path>("path_map", 1, true);

    mPathTimer = mNhNs.createTimer(ros::Duration(1.0 / mPathPubRate), &ZEDWrapperNodelet::callback_pubPath, this);

    if (mPathMaxCount != -1)
    {
      NODELET_DEBUG_STREAM("Path vectors reserved " << mPathMaxCount << " poses.");
      mOdomPath.reserve(mPathMaxCount);
      mMapPath.reserve(mPathMaxCount);

      NODELET_DEBUG_STREAM("Path vector sizes: " << mOdomPath.size() << " " << mMapPath.size());
    }
  }
  else
  {
    NODELET_INFO_STREAM("Path topics not published -> mPathPubRate: " << mPathPubRate);
  }

  // Sensor publishers
  if (mZedRealCamModel != sl::MODEL::ZED)
  {
    // IMU Publishers
    mPubImu = mNhNs.advertise<sensor_msgs::Imu>("imu/data", 1 /*static_cast<int>(mSensPubRate)*/);
    mPubImuRaw = mNhNs.advertise<sensor_msgs::Imu>("imu/data_raw", 1 /*static_cast<int>(mSensPubRate)*/);

    mPubImuMag = mNhNs.advertise<sensor_msgs::MagneticField>("imu/mag", 1 /*MAG_FREQ*/);

    if (mZedRealCamModel == sl::MODEL::ZED2 || mZedRealCamModel == sl::MODEL::ZED2i)
    {
      string temp_topic_root = "temperature";
      // IMU temperature sensor
      const string imu_temp_topic = temp_topic_root + "/imu";
      mPubImuTemp = mNhNs.advertise<sensor_msgs::Temperature>(imu_temp_topic, 1 /*static_cast<int>(mSensPubRate)*/);

      // Atmospheric pressure
      mPubPressure = mNhNs.advertise<sensor_msgs::FluidPressure>("atm_press", 1 /*static_cast<int>(BARO_FREQ)*/);

      // CMOS sensor temperatures
      const string temp_topic_left = temp_topic_root + "/left";
      mPubTempL = mNhNs.advertise<sensor_msgs::Temperature>(temp_topic_left, 1 /*static_cast<int>(BARO_FREQ)*/);

      const string temp_topic_right = temp_topic_root + "/right";
      mPubTempR = mNhNs.advertise<sensor_msgs::Temperature>(temp_topic_right, 1 /*static_cast<int>(BARO_FREQ)*/);
    }

    // Publish camera imu transform in a latched topic
    if (mZedRealCamModel != sl::MODEL::ZED)
    {
      const string cam_imu_tr_topic = "left_cam_imu_transform";
      mPubCamImuTransf = mNhNs.advertise<geometry_msgs::Transform>(cam_imu_tr_topic, 1, true);

      sl::Orientation sl_rot = mSlCamImuTransf.getOrientation();
      sl::Translation sl_tr = mSlCamImuTransf.getTranslation();

      mCameraImuTransfMgs = boost::make_shared<geometry_msgs::Transform>();

      mCameraImuTransfMgs->rotation.x = sl_rot.ox;
      mCameraImuTransfMgs->rotation.y = sl_rot.oy;
      mCameraImuTransfMgs->rotation.z = sl_rot.oz;
      mCameraImuTransfMgs->rotation.w = sl_rot.ow;

      mCameraImuTransfMgs->translation.x = sl_tr.x;
      mCameraImuTransfMgs->translation.y = sl_tr.y;
      mCameraImuTransfMgs->translation.z = sl_tr.z;

      NODELET_DEBUG("Camera-IMU Rotation: \n %s", sl_rot.getRotationMatrix().getInfos().c_str());
      NODELET_DEBUG("Camera-IMU Translation: \n %g %g %g", sl_tr.x, sl_tr.y, sl_tr.z);

      mPubCamImuTransf.publish(mCameraImuTransfMgs);
    }

    if (!mSvoMode && !mSensTimestampSync)
    {
      // init so sensor callback will have a timestamp to work with on first publish
      mFrameTimestamp = getTimestamp();
      mImuTimer = mNhNs.createTimer(ros::Duration(1.0 / (mSensPubRate * 1.5)),
                                    &ZEDWrapperNodelet::callback_pubSensorsData, this);
      mSensPeriodMean_usec.reset(new sl_tools::CSmartMean(mSensPubRate / 2));
    }
    else
    {
      mSensPeriodMean_usec.reset(new sl_tools::CSmartMean(mCamFrameRate / 2));
    }
  }
}

void ZEDWrapperNodelet::initAllServices()
{
  mSrvSetInitPose = mNhNs.advertiseService("set_pose", &ZEDWrapperNodelet::on_set_pose, this);
  mSrvResetOdometry = mNhNs.advertiseService("reset_odometry", &ZEDWrapperNodelet::on_reset_odometry, this);
  mSrvResetTracking = mNhNs.advertiseService("reset_tracking", &ZEDWrapperNodelet::on_reset_tracking, this);
  mSrvSvoStartRecording =
      mNhNs.advertiseService("start_svo_recording", &ZEDWrapperNodelet::on_start_svo_recording, this);
  mSrvSvoStopRecording = mNhNs.advertiseService("stop_svo_recording", &ZEDWrapperNodelet::on_stop_svo_recording, this);

  mSrvSetLedStatus = mNhNs.advertiseService("set_led_status", &ZEDWrapperNodelet::on_set_led_status, this);
  mSrvToggleLed = mNhNs.advertiseService("toggle_led", &ZEDWrapperNodelet::on_toggle_led, this);
  mSrvSvoStartStream = mNhNs.advertiseService("start_remote_stream", &ZEDWrapperNodelet::on_start_remote_stream, this);
  mSrvSvoStopStream = mNhNs.advertiseService("stop_remote_stream", &ZEDWrapperNodelet::on_stop_remote_stream, this);
}

void ZEDWrapperNodelet::checkVersion()
{
  std::string ver = sl_tools::getSDKVersion(mVerMajor, mVerMinor, mVerSubMinor);
  NODELET_INFO_STREAM("SDK version : " << ver);

  if (mVerMajor < 3)
  {
    NODELET_ERROR("This version of the ZED ROS Wrapper is designed to be used with ZED SDK v3.x");
    ros::shutdown();
    raise(SIGINT);
    raise(SIGABRT);
    exit(-1);
  }
}

// Try to initialize the ZED
bool ZEDWrapperNodelet::initCamera()
{
  if (!mSvoFilepath.empty() || !mRemoteStreamAddr.empty())
  {
    if (!mSvoFilepath.empty())
    {
      mZedParams.input.setFromSVOFile(mSvoFilepath.c_str());
      /**
        from sl/Camera.hpp
        When playing back an SVO file, each call to \ref Camera::grab() will extract a new frame and use it.
        However, this ignores the real capture rate of the images saved in the SVO file.
        Enabling this parameter will bring the SDK closer to a real simulation when playing back a file by
        using the images' timestamps. However, calls to \ref Camera::grab() will return an error when trying
        to play too fast, and frames will be dropped when playing too slowly.
      */
      mZedParams.svo_real_time_mode = false;

      // TODO ADD PARAMETER FOR SVO REAL TIME
    }
    else if (!mRemoteStreamAddr.empty())
    {
      std::vector<std::string> configStream = sl_tools::split_string(mRemoteStreamAddr, ':');
      sl::String ip = sl::String(configStream.at(0).c_str());

      if (configStream.size() == 2)
      {
        mZedParams.input.setFromStream(ip, atoi(configStream.at(1).c_str()));
      }
      else
      {
        mZedParams.input.setFromStream(ip);
      }
    }

    mSvoMode = true;
  }
  else
  {
    mZedParams.camera_fps = mCamFrameRate;
    mZedParams.camera_resolution = static_cast<sl::RESOLUTION>(mCamResol);

    if (mZedSerialNumber == 0)
    {
      mZedParams.input.setFromCameraID(mZedId);
    }
    else
    {
      bool waiting_for_camera = true;

      while (waiting_for_camera)
      {
        // Ctrl+C check
        if (!mNhNs.ok())
        {
          stop();

          NODELET_DEBUG("ZED pool thread finished");
          return false;
        }

        sl::DeviceProperties prop = sl_tools::getZEDFromSN(mZedSerialNumber);

        if (prop.id < -1 || prop.camera_state == sl::CAMERA_STATE::NOT_AVAILABLE)
        {
          std::string msg = "ZED SN" + to_string(mZedSerialNumber) + " not detected ! Please connect this ZED";
          NODELET_INFO_STREAM(msg.c_str());
          ros::WallDuration(2.0).sleep();
        }
        else
        {
          waiting_for_camera = false;
          mZedParams.input.setFromCameraID(prop.id);
        }
      }
    }
  }
  return true;
}

void ZEDWrapperNodelet::checkCameraModel()
{
  mZedRealCamModel = mZed.getCameraInformation().camera_model;

  if (mZedRealCamModel == sl::MODEL::ZED)
  {
    if (mZedUserCamModel != mZedRealCamModel)
    {
      NODELET_WARN("Camera model does not match user parameter. Please modify "
                   "the value of the parameter 'camera_model' to 'zed'");
    }
  }
  else if (mZedRealCamModel == sl::MODEL::ZED_M)
  {
    if (mZedUserCamModel != mZedRealCamModel)
    {
      NODELET_WARN("Camera model does not match user parameter. Please modify "
                   "the value of the parameter 'camera_model' to 'zedm'");
    }
#if ZED_SDK_MAJOR_VERSION == 3 && ZED_SDK_MINOR_VERSION < 1
    mSlCamImuTransf = mZed.getCameraInformation().camera_imu_transform;
#else
    mSlCamImuTransf = mZed.getCameraInformation().sensors_configuration.camera_imu_transform;
#endif

    NODELET_INFO("Camera-IMU Transform: \n %s", mSlCamImuTransf.getInfos().c_str());
  }
  else if (mZedRealCamModel == sl::MODEL::ZED2)
  {
    if (mZedUserCamModel != mZedRealCamModel)
    {
      NODELET_WARN("Camera model does not match user parameter. Please modify "
                   "the value of the parameter 'camera_model' to 'zed2'");
    }

#if ZED_SDK_MAJOR_VERSION == 3 && ZED_SDK_MINOR_VERSION < 1
    mSlCamImuTransf = mZed.getCameraInformation().camera_imu_transform;
#else
    mSlCamImuTransf = mZed.getCameraInformation().sensors_configuration.camera_imu_transform;
#endif

    NODELET_INFO("Camera-IMU Transform: \n %s", mSlCamImuTransf.getInfos().c_str());
  }
  else if (mZedRealCamModel == sl::MODEL::ZED2i)
  {
    if (mZedUserCamModel != mZedRealCamModel)
    {
      NODELET_WARN("Camera model does not match user parameter. Please modify "
                   "the value of the parameter 'camera_model' to 'zed2i'");
    }

#if ZED_SDK_MAJOR_VERSION == 3 && ZED_SDK_MINOR_VERSION < 1
    mSlCamImuTransf = mZed.getCameraInformation().camera_imu_transform;
#else
    mSlCamImuTransf = mZed.getCameraInformation().sensors_configuration.camera_imu_transform;
#endif

    NODELET_INFO("Camera-IMU Transform: \n %s", mSlCamImuTransf.getInfos().c_str());
  }

  NODELET_INFO_STREAM(" * CAMERA MODEL\t -> " << sl::toString(mZedRealCamModel).c_str());
  mZedSerialNumber = mZed.getCameraInformation().serial_number;
  NODELET_INFO_STREAM(" * Serial Number -> " << mZedSerialNumber);

  if (!mSvoMode)
  {
#if ZED_SDK_MAJOR_VERSION == 3 && ZED_SDK_MINOR_VERSION < 1
    mCamFwVersion = mZed.getCameraInformation().camera_firmware_version;
#else
    mCamFwVersion = mZed.getCameraInformation().camera_configuration.firmware_version;
#endif

    NODELET_INFO_STREAM(" * Camera FW Version -> " << mCamFwVersion);
    if (mZedRealCamModel != sl::MODEL::ZED)
    {
#if ZED_SDK_MAJOR_VERSION == 3 && ZED_SDK_MINOR_VERSION < 1
      mSensFwVersion = mZed.getCameraInformation().sensors_firmware_version;
#else
      mSensFwVersion = mZed.getCameraInformation().sensors_configuration.firmware_version;
#endif
      NODELET_INFO_STREAM(" * Sensors FW Version -> " << mSensFwVersion);
    }
  }
  else
  {
    NODELET_INFO_STREAM(" * Input type\t -> " << sl::toString(mZed.getCameraInformation().input_type).c_str());
  }

  mDiagUpdater.setHardwareIDf("%s - s/n: %d [GPU #%d]", sl::toString(mZedRealCamModel).c_str(), mZedSerialNumber,
                              mGpuId);
}

bool ZEDWrapperNodelet::initZedParams()
{
  mZedParams.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
  NODELET_INFO_STREAM(" * Camera coordinate system\t-> " << sl::toString(mZedParams.coordinate_system));

  mZedParams.coordinate_units = sl::UNIT::METER;
  mZedParams.depth_mode = static_cast<sl::DEPTH_MODE>(mDepthMode);
  mZedParams.sdk_verbose = mVerbose;
  mZedParams.sdk_gpu_id = mGpuId;
  mZedParams.depth_stabilization = mDepthStabilization;
  mZedParams.camera_image_flip = mCameraFlip;
  mZedParams.depth_minimum_distance = static_cast<float>(mCamMinDepth);
  mZedParams.depth_maximum_distance = static_cast<float>(mCamMaxDepth);
  mZedParams.camera_disable_self_calib = !mCameraSelfCalib;

  mZedParams.enable_image_enhancement = true;  // Always active

  mDiagUpdater.add("ZED Diagnostic", this, &ZEDWrapperNodelet::callback_updateDiagnostic);
  mDiagUpdater.setHardwareID("ZED camera");

  mConnStatus = sl::ERROR_CODE::CAMERA_NOT_DETECTED;

  NODELET_INFO_STREAM(" *** Opening " << sl::toString(mZedUserCamModel) << "...");
  while (mConnStatus != sl::ERROR_CODE::SUCCESS)
  {
    mConnStatus = mZed.open(mZedParams);
    NODELET_INFO_STREAM("ZED connection -> " << sl::toString(mConnStatus));
    ros::WallDuration(2.0).sleep();

    if (!mNhNs.ok())
    {
      stop();
      return false;
    }

    mDiagUpdater.update();
  }
  NODELET_INFO_STREAM(" ...  " << sl::toString(mZedRealCamModel) << " ready");

  // CUdevice devid;
  cuCtxGetDevice(&mGpuId);

  NODELET_INFO_STREAM("ZED SDK running on GPU #" << mGpuId);

  // Disable AEC_AGC and Auto Whitebalance to trigger it if use set to automatic
  mZed.setCameraSettings(sl::VIDEO_SETTINGS::AEC_AGC, 0);
  mZed.setCameraSettings(sl::VIDEO_SETTINGS::WHITEBALANCE_AUTO, 0);

  return true;
}

void ZEDWrapperNodelet::initDynamicReconfigure()
{
  mDynRecServer = boost::make_shared<dynamic_reconfigure::Server<zed_nodelets::ZedConfig>>(mDynServerMutex);
  dynamic_reconfigure::Server<zed_nodelets::ZedConfig>::CallbackType f;
  f = boost::bind(&ZEDWrapperNodelet::callback_dynamicReconf, this, boost::placeholders::_1, boost::placeholders::_2);
  mDynRecServer->setCallback(f);
#if 0
  // TODO(lucasw) don't care about these config defaults, want the class member defaults
  zed_nodelets::ZedConfig config;
  mDynRecServer->getConfigDefault(config);
  {
    std::lock_guard<std::mutex> lock(mDynServerMutex);
    mDynRecServer->updateConfig(config);
  }
#endif
  updateDynamicReconfigureServer();
}

void ZEDWrapperNodelet::onInit()
{
  // Node handlers
  mNh = getMTNodeHandle();
  mNhNs = getMTPrivateNodeHandle();

  mStopNode = false;
  mPcDataReady = false;

#if 0
#ifndef NDEBUG
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }
#endif
#endif

  NODELET_INFO("********** Starting nodelet '%s' **********", getName().c_str());

  checkVersion();

  readParameters();
  initTransforms();

  if (mOpenniDepthMode)
  {
    NODELET_INFO_STREAM("Openni depth mode activated -> Units: mm, Encoding: TYPE_16UC1");
  }

  // Create camera info
  mLeftCamInfoMsg.reset(new sensor_msgs::CameraInfo());
  mRightCamInfoMsg.reset(new sensor_msgs::CameraInfo());
  mLeftCamInfoRawMsg.reset(new sensor_msgs::CameraInfo());
  mRightCamInfoRawMsg.reset(new sensor_msgs::CameraInfo());
  mDepthCamInfoMsg.reset(new sensor_msgs::CameraInfo());

  // Initialization transformation listener
  mTfBuffer.reset(new tf2_ros::Buffer);
  mTfListener.reset(new tf2_ros::TransformListener(*mTfBuffer));

  if (!initCamera()) {
    return;
  }

  if (!initZedParams()) {
    return;
  }

  checkCameraModel();

  initDynamicReconfigure();
  initAllPublishers();
  initAllServices();

  // start recording immediately if record parameter set
  if (mRecordSvoFilepath != "") {
    std::string info = "";
    startRecording(mRecordSvoFilepath, info);
    if (mRecording == false) {
      return;
    }
  }

  // Start pool thread
  mDevicePollThread = std::thread(&ZEDWrapperNodelet::device_poll_thread_func, this);

  if (mUseSimTime) {
    // TODO(lucasw) 0.02-0.05 are probably fine
    wall_timer_ =  mNhNs.createWallTimer(ros::WallDuration(0.01), &ZEDWrapperNodelet::sim_clock_update, this);
  }

  // Start data publishing timer
  mVideoDepthTimer =
      mNhNs.createTimer(ros::Duration(1.0 / mVideoDepthFreq), &ZEDWrapperNodelet::callback_pubVideoDepth, this);
}

void ZEDWrapperNodelet::setupGeneral()
{
  NODELET_INFO_STREAM("*** GENERAL PARAMETERS ***");

  mNhNs.getParam("general/camera_name", mCameraName);
  NODELET_INFO_STREAM(" * Camera Name\t\t\t-> " << mCameraName.c_str());
  int resol;
  mNhNs.getParam("general/resolution", resol);
  mCamResol = static_cast<sl::RESOLUTION>(resol);
  NODELET_INFO_STREAM(" * Camera Resolution\t\t-> " << sl::toString(mCamResol).c_str());
  mNhNs.getParam("general/grab_frame_rate", mCamFrameRate);
  checkResolFps();
  NODELET_INFO_STREAM(" * Camera Grab Framerate\t-> " << mCamFrameRate);

  mNhNs.getParam("general/gpu_id", mGpuId);
  NODELET_INFO_STREAM(" * Gpu ID\t\t\t-> " << mGpuId);
  mNhNs.getParam("general/zed_id", mZedId);
  NODELET_INFO_STREAM(" * Camera ID\t\t\t-> " << mGpuId);
  mNhNs.getParam("general/verbose", mVerbose);
  NODELET_INFO_STREAM(" * Verbose\t\t\t-> " << (mVerbose ? "ENABLED" : "DISABLED"));
  mNhNs.param<bool>("general/camera_flip", mCameraFlip, false);
  NODELET_INFO_STREAM(" * Camera Flip\t\t\t-> " << (mCameraFlip ? "ENABLED" : "DISABLED"));
  mNhNs.param<bool>("general/self_calib", mCameraSelfCalib, true);
  NODELET_INFO_STREAM(" * Self calibration\t\t-> " << (mCameraSelfCalib ? "ENABLED" : "DISABLED"));

  int tmp_sn = 0;
  mNhNs.getParam("general/serial_number", tmp_sn);

  if (tmp_sn > 0)
  {
    mZedSerialNumber = static_cast<int>(tmp_sn);
    NODELET_INFO_STREAM(" * Serial number\t\t-> " << mZedSerialNumber);
  }

  string camera_model;
  mNhNs.getParam("general/camera_model", camera_model);

  if (camera_model == "zed")
  {
    mZedUserCamModel = sl::MODEL::ZED;
    NODELET_INFO_STREAM(" * Camera Model by param\t-> " << camera_model);
  }
  else if (camera_model == "zedm")
  {
    mZedUserCamModel = sl::MODEL::ZED_M;
    NODELET_INFO_STREAM(" * Camera Model by param\t-> " << camera_model);
  }
  else if (camera_model == "zed2")
  {
    mZedUserCamModel = sl::MODEL::ZED2;
    NODELET_INFO_STREAM(" * Camera Model by param\t-> " << camera_model);
  }
  else if (camera_model == "zed2i")
  {
    mZedUserCamModel = sl::MODEL::ZED2i;
    NODELET_INFO_STREAM(" * Camera Model by param\t-> " << camera_model);
  }
  else
  {
    NODELET_ERROR_STREAM("Camera model not valid: " << camera_model);
  }
}

void ZEDWrapperNodelet::setupVideo()
{
  NODELET_INFO_STREAM("*** VIDEO PARAMETERS ***");

  mNhNs.getParam("video/img_downsample_factor", mCamImageResizeFactor);
  NODELET_INFO_STREAM(" * Image resample factor\t-> " << mCamImageResizeFactor);

  mNhNs.getParam("video/extrinsic_in_camera_frame", mUseOldExtrinsic);
  NODELET_INFO_STREAM(" * Extrinsic param. frame\t-> "
                      << (mUseOldExtrinsic ? "X RIGHT - Y DOWN - Z FWD" : "X FWD - Y LEFT - Z UP"));
}

void ZEDWrapperNodelet::setupDepth()
{
  NODELET_INFO_STREAM("*** DEPTH PARAMETERS ***");

  int depth_mode;
  mNhNs.getParam("depth/quality", depth_mode);
  mDepthMode = static_cast<sl::DEPTH_MODE>(depth_mode);
  NODELET_INFO_STREAM(" * Depth quality\t\t-> " << sl::toString(mDepthMode).c_str());
  int sensing_mode;
  mNhNs.getParam("depth/sensing_mode", sensing_mode);
  mCamSensingMode = static_cast<sl::SENSING_MODE>(sensing_mode);
  NODELET_INFO_STREAM(" * Depth Sensing mode\t\t-> " << sl::toString(mCamSensingMode).c_str());
  mNhNs.getParam("depth/openni_depth_mode", mOpenniDepthMode);
  NODELET_INFO_STREAM(" * OpenNI mode\t\t\t-> " << (mOpenniDepthMode ? "ENABLED" : "DISABLED"));
  mNhNs.getParam("depth/depth_stabilization", mDepthStabilization);
  NODELET_INFO_STREAM(" * Depth Stabilization\t\t-> " << (mDepthStabilization ? "ENABLED" : "DISABLED"));
  mNhNs.getParam("depth/min_depth", mCamMinDepth);
  NODELET_INFO_STREAM(" * Minimum depth\t\t-> " << mCamMinDepth << " m");
  mNhNs.getParam("depth/max_depth", mCamMaxDepth);
  NODELET_INFO_STREAM(" * Maximum depth\t\t-> " << mCamMaxDepth << " m");
  mNhNs.getParam("depth/depth_downsample_factor", mCamDepthResizeFactor);
  NODELET_INFO_STREAM(" * Depth resample factor\t-> " << mCamDepthResizeFactor);
}

void ZEDWrapperNodelet::setupTracking()
{
  NODELET_INFO_STREAM("*** POSITIONAL TRACKING PARAMETERS ***");

  mNhNs.param<bool>("pos_tracking/pos_tracking_enabled", mPosTrackingEnabled, true);
  NODELET_INFO_STREAM(" * Positional tracking\t\t-> " << (mPosTrackingEnabled ? "ENABLED" : "DISABLED"));
  mNhNs.getParam("pos_tracking/path_pub_rate", mPathPubRate);
  NODELET_INFO_STREAM(" * Path rate\t\t\t-> " << mPathPubRate << " Hz");
  mNhNs.getParam("pos_tracking/path_max_count", mPathMaxCount);
  NODELET_INFO_STREAM(" * Path history size\t\t-> " << (mPathMaxCount == -1) ? std::string("INFINITE") :
                                                                               std::to_string(mPathMaxCount));

  if (mPathMaxCount < 2 && mPathMaxCount != -1)
  {
    mPathMaxCount = 2;
  }

  mNhNs.getParam("pos_tracking/initial_base_pose", mInitialBasePose);

  mNhNs.getParam("pos_tracking/area_memory_db_path", mAreaMemDbPath);
  NODELET_INFO_STREAM(" * Odometry DB path\t\t-> " << mAreaMemDbPath.c_str());
  mNhNs.param<bool>("pos_tracking/area_memory", mAreaMemory, false);
  NODELET_INFO_STREAM(" * Spatial Memory\t\t-> " << (mAreaMemory ? "ENABLED" : "DISABLED"));
  mNhNs.param<bool>("pos_tracking/imu_fusion", mImuFusion, true);
  NODELET_INFO_STREAM(" * IMU Fusion\t\t\t-> " << (mImuFusion ? "ENABLED" : "DISABLED"));
  mNhNs.param<bool>("pos_tracking/floor_alignment", mFloorAlignment, false);
  NODELET_INFO_STREAM(" * Floor alignment\t\t-> " << (mFloorAlignment ? "ENABLED" : "DISABLED"));
  mNhNs.param<bool>("pos_tracking/init_odom_with_first_valid_pose", mInitOdomWithPose, true);
  NODELET_INFO_STREAM(" * Init Odometry with first valid pose data -> "
                      << (mInitOdomWithPose ? "ENABLED" : "DISABLED"));
  mNhNs.param<bool>("pos_tracking/two_d_mode", mTwoDMode, false);
  NODELET_INFO_STREAM(" * Two D mode\t\t\t-> " << (mTwoDMode ? "ENABLED" : "DISABLED"));
  mNhNs.param<double>("pos_tracking/fixed_z_value", mFixedZValue, 0.0);

  if (mTwoDMode)
  {
    NODELET_INFO_STREAM(" * Fixed Z value\t\t-> " << mFixedZValue);
  }
}

void ZEDWrapperNodelet::setupSensors()
{
  NODELET_INFO_STREAM("*** SENSORS PARAMETERS ***");

  mNhNs.getParam("sensors/sensors_timestamp_sync", mSensTimestampSync);
  NODELET_INFO_STREAM(" * Sensors timestamp sync\t-> " << (mSensTimestampSync ? "ENABLED" : "DISABLED"));
}

void ZEDWrapperNodelet::setupSvo()
{
  NODELET_INFO_STREAM("*** SVO PARAMETERS ***");

  mNhNs.param<std::string>("svo_file", mSvoFilepath, std::string());
  NODELET_INFO_STREAM(" * SVO input file: \t\t-> " << mSvoFilepath.c_str());

  mNhNs.param<std::string>("record_svo_file", mRecordSvoFilepath, std::string());
  if (mRecordSvoFilepath != "") {
   // NODELET_WARN_STREAM(" * Record SVO input file: \t\t-> " << mRecordSvoFilepath.c_str());
  }
  NODELET_WARN_STREAM(" * Record SVO input file: \t\t-> '" << mRecordSvoFilepath << "'");

  mNhNs.getParam("/use_sim_time", mUseSimTime);
  NODELET_INFO_STREAM(" * Use Sim Time\t\t\t-> " << mUseSimTime);

  int svo_compr = 0;
  mNhNs.getParam("general/svo_compression", svo_compr);

  if (svo_compr >= static_cast<int>(sl::SVO_COMPRESSION_MODE::LAST))
  {
    NODELET_WARN_STREAM("The parameter `general/svo_compression` has an invalid value. Please check it in the "
                        "configuration file `common.yaml`");

    svo_compr = 0;
  }

  mSvoComprMode = static_cast<sl::SVO_COMPRESSION_MODE>(svo_compr);

  NODELET_INFO_STREAM(" * SVO REC compression\t\t-> " << sl::toString(mSvoComprMode));

  // Remote Stream
  mNhNs.param<std::string>("stream", mRemoteStreamAddr, std::string());
}

void ZEDWrapperNodelet::setupCoordinateFrames()
{
  mNhNs.param<std::string>("pos_tracking/map_frame", mMapFrameId, "map");
  mNhNs.param<std::string>("pos_tracking/odometry_frame", mOdometryFrameId, "odom");
  mNhNs.param<std::string>("general/base_frame", mBaseFrameId, "base_link");

  mCameraFrameId = mCameraName + "_camera_center";
  mImuFrameId = mCameraName + "_imu_link";
  mLeftCamFrameId = mCameraName + "_left_camera_frame";
  mLeftCamOptFrameId = mCameraName + "_left_camera_optical_frame";
  mRightCamFrameId = mCameraName + "_right_camera_frame";
  mRightCamOptFrameId = mCameraName + "_right_camera_optical_frame";

  mBaroFrameId = mCameraName + "_baro_link";
  mMagFrameId = mCameraName + "_mag_link";
  mTempLeftFrameId = mCameraName + "_temp_left_link";
  mTempRightFrameId = mCameraName + "_temp_right_link";

  mDepthFrameId = mLeftCamFrameId;
  mDepthOptFrameId = mLeftCamOptFrameId;

  // Note: Depth image frame id must match color image frame id
  mCloudFrameId = mDepthOptFrameId;
  mDisparityFrameId = mDepthFrameId;
  mDisparityOptFrameId = mDepthOptFrameId;
  mConfidenceFrameId = mDepthFrameId;
  mConfidenceOptFrameId = mDepthOptFrameId;

  logTFFrames();
}

// TODO(lucasw) have this take a log level argument
void ZEDWrapperNodelet::logTFFrames()
{
  std::stringstream ss;
  ss << "*** COORDINATE FRAMES ***" << "\n";;

  ss << " * map_frame\t\t\t-> " << mMapFrameId << "\n";
  ss << " * odometry_frame\t\t-> " << mOdometryFrameId << "\n";
  ss << " * base_frame\t\t\t-> " << mBaseFrameId << "\n";
  ss << " * camera_frame\t\t\t-> " << mCameraFrameId << "\n";
  ss << " * imu_link\t\t\t-> " << mImuFrameId << "\n";
  ss << " * left_camera_frame\t\t-> " << mLeftCamFrameId << "\n";
  ss << " * left_camera_optical_frame\t-> " << mLeftCamOptFrameId << "\n";
  ss << " * right_camera_frame\t\t-> " << mRightCamFrameId << "\n";
  ss << " * right_camera_optical_frame\t-> " << mRightCamOptFrameId << "\n";
  ss << " * depth_frame\t\t\t-> " << mDepthFrameId << "\n";
  ss << " * depth_optical_frame\t\t-> " << mDepthOptFrameId << "\n";
  ss << " * disparity_frame\t\t-> " << mDisparityFrameId << "\n";
  ss << " * disparity_optical_frame\t-> " << mDisparityOptFrameId << "\n";
  ss << " * confidence_frame\t\t-> " << mConfidenceFrameId << "\n";
  ss << " * confidence_optical_frame\t-> " << mConfidenceOptFrameId << "\n";

  NODELET_DEBUG_STREAM(ss.str());
}

void ZEDWrapperNodelet::setupTFBroadcasting()
{
  mNhNs.param<bool>("pos_tracking/publish_tf", mPublishTf, true);
  NODELET_INFO_STREAM(" * Broadcast odometry TF\t-> " << (mPublishTf ? "ENABLED" : "DISABLED"));
  mNhNs.param<bool>("pos_tracking/publish_map_tf", mPublishMapTf, true);
  NODELET_INFO_STREAM(" * Broadcast map pose TF\t-> "
                      << (mPublishTf ? (mPublishMapTf ? "ENABLED" : "DISABLED") : "DISABLED"));
  mNhNs.param<bool>("sensors/publish_imu_tf", mPublishImuTf, true);
  NODELET_INFO_STREAM(" * Broadcast IMU pose TF\t-> " << (mPublishImuTf ? "ENABLED" : "DISABLED"));
}

// TODO(lucasw) dynamic reconfigure should do this automatically (minus the summary text)
void ZEDWrapperNodelet::setupDynamicParameters()
{
  std::stringstream ss;

  ss << "*** DYNAMIC PARAMETERS (Init. values) ***\n";

  mNhNs.getParam("depth_confidence", mCamDepthConfidence);
  ss << " * [DYN] Depth confidence\t-> " << mCamDepthConfidence << "\n";
  mNhNs.getParam("depth_texture_conf", mCamDepthTextureConf);
  ss << " * [DYN] Depth texture conf.\t-> " << mCamDepthTextureConf << "\n";

  mNhNs.getParam("pub_frame_rate", mVideoDepthFreq);
  ss << " * [DYN] pub_frame_rate\t\t-> " << mVideoDepthFreq << " Hz" << "\n";
  mNhNs.getParam("brightness", mCamBrightness);
  ss << " * [DYN] brightness\t\t-> " << mCamBrightness << "\n";
  mNhNs.getParam("contrast", mCamContrast);
  ss << " * [DYN] contrast\t\t-> " << mCamContrast << "\n";
  mNhNs.getParam("hue", mCamHue);
  ss << " * [DYN] hue\t\t\t-> " << mCamHue << "\n";
  mNhNs.getParam("saturation", mCamSaturation);
  ss << " * [DYN] saturation\t\t-> " << mCamSaturation << "\n";
  mNhNs.getParam("sharpness", mCamSharpness);
  ss << " * [DYN] sharpness\t\t-> " << mCamSharpness << "\n";
#if (ZED_SDK_MAJOR_VERSION == 3 && ZED_SDK_MINOR_VERSION >= 1)
  mNhNs.getParam("gamma", mCamGamma);
  ss << " * [DYN] gamma\t\t\t-> " << mCamGamma << "\n";
#endif
  mNhNs.getParam("auto_exposure_gain", mCamAutoExposure);
  ss << " * [DYN] auto_exposure_gain\t-> " << (mCamAutoExposure ? "ENABLED" : "DISABLED") << "\n";
  mNhNs.getParam("gain", mCamGain);
  mNhNs.getParam("exposure", mCamExposure);
  if (!mCamAutoExposure)
  {
    ss << "  * [DYN] gain\t\t-> " << mCamGain << "\n";
    ss << "  * [DYN] exposure\t\t-> " << mCamExposure << "\n";
  }
  mNhNs.getParam("auto_whitebalance", mCamAutoWB);
  ss << " * [DYN] auto_whitebalance\t-> " << (mCamAutoWB ? "ENABLED" : "DISABLED") << "\n";
  mNhNs.getParam("whitebalance_temperature", mCamWB );
  if (!mCamAutoWB)
  {
    ss << "  * [DYN] whitebalance_temperature\t\t-> " << mCamWB << "\n";
  }

  if (mCamAutoExposure)
  {
    mTriggerAutoExposure = true;
  }
  if (mCamAutoWB)
  {
    mTriggerAutoWB = true;
  }

  NODELET_INFO_STREAM(ss.str());
}

void ZEDWrapperNodelet::readParameters()
{
  // Get parameters from rosparams
  setupGeneral();
  setupVideo();
  setupDepth();
  setupTracking();
  setupSensors();
  setupSvo();
  setupCoordinateFrames();
  setupTFBroadcasting();
  setupDynamicParameters();
}

void ZEDWrapperNodelet::checkResolFps()
{
  switch (mCamResol)
  {
    case sl::RESOLUTION::HD2K:
      if (mCamFrameRate != 15)
      {
        NODELET_WARN_STREAM("Wrong FrameRate (" << mCamFrameRate << ") for the resolution HD2K. Set to 15 FPS.");
        mCamFrameRate = 15;
      }

      break;

    case sl::RESOLUTION::HD1080:
      if (mCamFrameRate == 15 || mCamFrameRate == 30)
      {
        break;
      }

      if (mCamFrameRate > 15 && mCamFrameRate < 30)
      {
        NODELET_WARN_STREAM("Wrong FrameRate (" << mCamFrameRate << ") for the resolution HD1080. Set to 15 FPS.");
        mCamFrameRate = 15;
      }
      else if (mCamFrameRate > 30)
      {
        NODELET_WARN_STREAM("Wrong FrameRate (" << mCamFrameRate << ") for the resolution HD1080. Set to 30 FPS.");
        mCamFrameRate = 30;
      }
      else
      {
        NODELET_WARN_STREAM("Wrong FrameRate (" << mCamFrameRate << ") for the resolution HD1080. Set to 15 FPS.");
        mCamFrameRate = 15;
      }

      break;

    case sl::RESOLUTION::HD720:
      if (mCamFrameRate == 15 || mCamFrameRate == 30 || mCamFrameRate == 60)
      {
        break;
      }

      if (mCamFrameRate > 15 && mCamFrameRate < 30)
      {
        NODELET_WARN_STREAM("Wrong FrameRate (" << mCamFrameRate << ") for the resolution HD720. Set to 15 FPS.");
        mCamFrameRate = 15;
      }
      else if (mCamFrameRate > 30 && mCamFrameRate < 60)
      {
        NODELET_WARN_STREAM("Wrong FrameRate (" << mCamFrameRate << ") for the resolution HD720. Set to 30 FPS.");
        mCamFrameRate = 30;
      }
      else if (mCamFrameRate > 60)
      {
        NODELET_WARN_STREAM("Wrong FrameRate (" << mCamFrameRate << ") for the resolution HD720. Set to 60 FPS.");
        mCamFrameRate = 60;
      }
      else
      {
        NODELET_WARN_STREAM("Wrong FrameRate (" << mCamFrameRate << ") for the resolution HD720. Set to 15 FPS.");
        mCamFrameRate = 15;
      }

      break;

    case sl::RESOLUTION::VGA:
      if (mCamFrameRate == 15 || mCamFrameRate == 30 || mCamFrameRate == 60 || mCamFrameRate == 100)
      {
        break;
      }

      if (mCamFrameRate > 15 && mCamFrameRate < 30)
      {
        NODELET_WARN_STREAM("Wrong FrameRate (" << mCamFrameRate << ") for the resolution VGA. Set to 15 FPS.");
        mCamFrameRate = 15;
      }
      else if (mCamFrameRate > 30 && mCamFrameRate < 60)
      {
        NODELET_WARN_STREAM("Wrong FrameRate (" << mCamFrameRate << ") for the resolution VGA. Set to 30 FPS.");
        mCamFrameRate = 30;
      }
      else if (mCamFrameRate > 60 && mCamFrameRate < 100)
      {
        NODELET_WARN_STREAM("Wrong FrameRate (" << mCamFrameRate << ") for the resolution VGA. Set to 60 FPS.");
        mCamFrameRate = 60;
      }
      else if (mCamFrameRate > 100)
      {
        NODELET_WARN_STREAM("Wrong FrameRate (" << mCamFrameRate << ") for the resolution VGA. Set to 100 FPS.");
        mCamFrameRate = 100;
      }
      else
      {
        NODELET_WARN_STREAM("Wrong FrameRate (" << mCamFrameRate << ") for the resolution VGA. Set to 15 FPS.");
        mCamFrameRate = 15;
      }

      break;

    default:
      NODELET_WARN_STREAM("Invalid resolution. Set to HD720 @ 30 FPS");
      mCamResol = sl::RESOLUTION::HD720;
      mCamFrameRate = 30;
  }
}

void ZEDWrapperNodelet::initTransforms()
{
  // According to REP 105 -> http://www.ros.org/reps/rep-0105.html

  // base_link <- odom <- map
  //     ^                 |
  //     |                 |
  //     -------------------

  // ----> Dynamic transforms
  mOdom2BaseTransf.setIdentity();   // broadcasted if `publish_tf` is true
  mMap2OdomTransf.setIdentity();    // broadcasted if `publish_map_tf` is true
  mMap2BaseTransf.setIdentity();    // used internally, but not broadcasted
  mMap2CameraTransf.setIdentity();  // used internally, but not broadcasted
                                    // <---- Dynamic transforms
}

bool ZEDWrapperNodelet::getCamera2BaseTransform()
{
  NODELET_DEBUG("Getting static TF from '%s' to '%s'", mCameraFrameId.c_str(), mBaseFrameId.c_str());

  mCamera2BaseTransfValid = false;
  static bool first_error = true;

  // ----> Static transforms
  // Sensor to Base link
  try
  {
    // Save the transformation
    geometry_msgs::TransformStamped c2b =
        mTfBuffer->lookupTransform(mCameraFrameId, mBaseFrameId, ros::Time(0), ros::Duration(0.1));

    // Get the TF2 transformation
    tf2::fromMsg(c2b.transform, mCamera2BaseTransf);

    double roll, pitch, yaw;
    tf2::Matrix3x3(mCamera2BaseTransf.getRotation()).getRPY(roll, pitch, yaw);

    NODELET_INFO("Static transform Camera Center to Base [%s -> %s]", mCameraFrameId.c_str(), mBaseFrameId.c_str());
    NODELET_INFO(" * Translation: {%.3f,%.3f,%.3f}", mCamera2BaseTransf.getOrigin().x(),
                 mCamera2BaseTransf.getOrigin().y(), mCamera2BaseTransf.getOrigin().z());
    NODELET_INFO(" * Rotation: {%.3f,%.3f,%.3f}", roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);
  }
  catch (tf2::TransformException& ex)
  {
    if (!first_error)
    {
      NODELET_DEBUG_THROTTLE(1.0, "Transform error: %s", ex.what());
      NODELET_WARN_THROTTLE(1.0, "The tf from '%s' to '%s' is not available.", mCameraFrameId.c_str(),
                            mBaseFrameId.c_str());
      NODELET_WARN_THROTTLE(1.0,
                            "Note: one of the possible cause of the problem is the absense of an instance "
                            "of the `robot_state_publisher` node publishing the correct static TF transformations "
                            "or a modified URDF not correctly reproducing the ZED "
                            "TF chain '%s' -> '%s' -> '%s'",
                            mBaseFrameId.c_str(), mCameraFrameId.c_str(), mDepthFrameId.c_str());
      first_error = false;
    }

    mCamera2BaseTransf.setIdentity();
    return false;
  }

  // <---- Static transforms
  mCamera2BaseTransfValid = true;
  return true;
}

bool ZEDWrapperNodelet::getSens2CameraTransform()
{
  NODELET_DEBUG("Getting static TF from '%s' to '%s'", mDepthFrameId.c_str(), mCameraFrameId.c_str());

  mSensor2CameraTransfValid = false;

  static bool first_error = true;

  // ----> Static transforms
  // Sensor to Camera Center
  try
  {
    // Save the transformation
    geometry_msgs::TransformStamped s2c =
        mTfBuffer->lookupTransform(mDepthFrameId, mCameraFrameId, ros::Time(0), ros::Duration(0.1));
    // Get the TF2 transformation
    tf2::fromMsg(s2c.transform, mSensor2CameraTransf);

    double roll, pitch, yaw;
    tf2::Matrix3x3(mSensor2CameraTransf.getRotation()).getRPY(roll, pitch, yaw);

    NODELET_INFO("Static transform Sensor to Camera Center [%s -> %s]", mDepthFrameId.c_str(), mCameraFrameId.c_str());
    NODELET_INFO(" * Translation: {%.3f,%.3f,%.3f}", mSensor2CameraTransf.getOrigin().x(),
                 mSensor2CameraTransf.getOrigin().y(), mSensor2CameraTransf.getOrigin().z());
    NODELET_INFO(" * Rotation: {%.3f,%.3f,%.3f}", roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);
  }
  catch (tf2::TransformException& ex)
  {
    if (!first_error)
    {
      NODELET_DEBUG_THROTTLE(1.0, "Transform error: %s", ex.what());
      NODELET_WARN_THROTTLE(1.0, "The tf from '%s' to '%s' is not available.", mDepthFrameId.c_str(),
                            mCameraFrameId.c_str());
      NODELET_WARN_THROTTLE(1.0,
                            "Note: one of the possible cause of the problem is the absense of an instance "
                            "of the `robot_state_publisher` node publishing the correct static TF transformations "
                            "or a modified URDF not correctly reproducing the ZED "
                            "TF chain '%s' -> '%s' -> '%s'",
                            mBaseFrameId.c_str(), mCameraFrameId.c_str(), mDepthFrameId.c_str());
      first_error = false;
    }

    mSensor2CameraTransf.setIdentity();
    return false;
  }

  // <---- Static transforms

  mSensor2CameraTransfValid = true;
  return true;
}

bool ZEDWrapperNodelet::getSens2BaseTransform()
{
  NODELET_DEBUG("Getting static TF from '%s' to '%s'", mDepthFrameId.c_str(), mBaseFrameId.c_str());

  mSensor2BaseTransfValid = false;
  static bool first_error = true;

  // ----> Static transforms
  // Sensor to Base link
  try
  {
    // Save the transformation
    geometry_msgs::TransformStamped s2b =
        mTfBuffer->lookupTransform(mDepthFrameId, mBaseFrameId, ros::Time(0), ros::Duration(0.1));
    // Get the TF2 transformation
    tf2::fromMsg(s2b.transform, mSensor2BaseTransf);

    double roll, pitch, yaw;
    tf2::Matrix3x3(mSensor2BaseTransf.getRotation()).getRPY(roll, pitch, yaw);

    NODELET_INFO("Static transform Sensor to Base [%s -> %s]", mDepthFrameId.c_str(), mBaseFrameId.c_str());
    NODELET_INFO(" * Translation: {%.3f,%.3f,%.3f}", mSensor2BaseTransf.getOrigin().x(),
                 mSensor2BaseTransf.getOrigin().y(), mSensor2BaseTransf.getOrigin().z());
    NODELET_INFO(" * Rotation: {%.3f,%.3f,%.3f}", roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);
  }
  catch (tf2::TransformException& ex)
  {
    if (!first_error)
    {
      NODELET_DEBUG_THROTTLE(1.0, "Transform error: %s", ex.what());
      NODELET_WARN_THROTTLE(1.0, "The tf from '%s' to '%s' is not available.", mDepthFrameId.c_str(),
                            mBaseFrameId.c_str());
      NODELET_WARN_THROTTLE(1.0,
                            "Note: one of the possible cause of the problem is the absense of an instance "
                            "of the `robot_state_publisher` node publishing the correct static TF transformations "
                            "or a modified URDF not correctly reproducing the ZED "
                            "TF chain '%s' -> '%s' -> '%s'",
                            mBaseFrameId.c_str(), mCameraFrameId.c_str(), mDepthFrameId.c_str());
      first_error = false;
    }

    mSensor2BaseTransf.setIdentity();
    return false;
  }

  // <---- Static transforms

  mSensor2BaseTransfValid = true;
  return true;
}

bool ZEDWrapperNodelet::set_pose(float xt, float yt, float zt, float rr, float pr, float yr)
{
  initTransforms();

  if (!mSensor2BaseTransfValid)
  {
    getSens2BaseTransform();
  }

  if (!mSensor2CameraTransfValid)
  {
    getSens2CameraTransform();
  }

  if (!mCamera2BaseTransfValid)
  {
    getCamera2BaseTransform();
  }

  // Apply Base to sensor transform
  tf2::Transform initPose;
  tf2::Vector3 origin(xt, yt, zt);
  initPose.setOrigin(origin);
  tf2::Quaternion quat;
  quat.setRPY(rr, pr, yr);
  initPose.setRotation(quat);

  initPose = initPose * mSensor2BaseTransf.inverse();

  // SL pose
  sl::float3 t_vec;
  t_vec[0] = initPose.getOrigin().x();
  t_vec[1] = initPose.getOrigin().y();
  t_vec[2] = initPose.getOrigin().z();

  sl::float4 q_vec;
  q_vec[0] = initPose.getRotation().x();
  q_vec[1] = initPose.getRotation().y();
  q_vec[2] = initPose.getRotation().z();
  q_vec[3] = initPose.getRotation().w();

  sl::Translation trasl(t_vec);
  sl::Orientation orient(q_vec);
  mInitialPoseSl.setTranslation(trasl);
  mInitialPoseSl.setOrientation(orient);

  return (mSensor2BaseTransfValid & mSensor2CameraTransfValid & mCamera2BaseTransfValid);
}

bool ZEDWrapperNodelet::on_set_pose(zed_interfaces::set_pose::Request& req, zed_interfaces::set_pose::Response& res)
{
  mInitialBasePose.resize(6);
  mInitialBasePose[0] = req.x;
  mInitialBasePose[1] = req.y;
  mInitialBasePose[2] = req.z;
  mInitialBasePose[3] = req.R;
  mInitialBasePose[4] = req.P;
  mInitialBasePose[5] = req.Y;

  if (!set_pose(mInitialBasePose[0], mInitialBasePose[1], mInitialBasePose[2], mInitialBasePose[3], mInitialBasePose[4],
                mInitialBasePose[5]))
  {
    res.done = false;
    return false;
  }

  std::lock_guard<std::mutex> lock(mPosTrkMutex);

  // Disable tracking
  mPosTrackingActivated = false;
  mZed.disablePositionalTracking();

  // Restart tracking
  start_pos_tracking();

  res.done = true;
  return true;
}

bool ZEDWrapperNodelet::on_reset_tracking(zed_interfaces::reset_tracking::Request& req,
                                          zed_interfaces::reset_tracking::Response& res)
{
  if (!mPosTrackingActivated)
  {
    res.reset_done = false;
    return false;
  }

  if (!set_pose(mInitialBasePose[0], mInitialBasePose[1], mInitialBasePose[2], mInitialBasePose[3], mInitialBasePose[4],
                mInitialBasePose[5]))
  {
    res.reset_done = false;
    return false;
  }

  std::lock_guard<std::mutex> lock(mPosTrkMutex);

  // Disable tracking
  mPosTrackingActivated = false;
  mZed.disablePositionalTracking();

  // Restart tracking
  start_pos_tracking();

  res.reset_done = true;
  return true;
}

bool ZEDWrapperNodelet::on_reset_odometry(zed_interfaces::reset_odometry::Request& req,
                                          zed_interfaces::reset_odometry::Response& res)
{
  mResetOdom = true;
  res.reset_done = true;
  return true;
}

void ZEDWrapperNodelet::start_pos_tracking()
{
  NODELET_INFO_STREAM("*** Starting Positional Tracking ***");

  NODELET_INFO(" * Waiting for valid static transformations...");

  bool transformOk = false;
  ros::Duration elapsed(0.0);

  auto start = ros::Time::now();

  do
  {
    transformOk = set_pose(mInitialBasePose[0], mInitialBasePose[1], mInitialBasePose[2], mInitialBasePose[3],
                           mInitialBasePose[4], mInitialBasePose[5]);
    elapsed = ros::Time::now() - start;

    if (elapsed > ros::Duration(3.0))
    {
      NODELET_WARN(" !!! Failed to get static transforms. Is the 'ROBOT STATE PUBLISHER' node correctly working? ");
      break;
    }

    ros::WallDuration(0.1).sleep();
  } while (transformOk == false);

  if (transformOk)
  {
    NODELET_DEBUG("Time required to get valid static transforms: %g sec", elapsed.toSec());
  }

  NODELET_INFO("Initial ZED left camera pose (ZED pos. tracking): ");
  NODELET_INFO(" * T: [%g,%g,%g]", mInitialPoseSl.getTranslation().x, mInitialPoseSl.getTranslation().y,
               mInitialPoseSl.getTranslation().z);
  NODELET_INFO(" * Q: [%g,%g,%g,%g]", mInitialPoseSl.getOrientation().ox, mInitialPoseSl.getOrientation().oy,
               mInitialPoseSl.getOrientation().oz, mInitialPoseSl.getOrientation().ow);

  if (mAreaMemDbPath != "" && !sl_tools::file_exist(mAreaMemDbPath))
  {
    mAreaMemDbPath = "";
    NODELET_WARN("area_memory_db_path path doesn't exist or is unreachable.");
  }

  // Tracking parameters
  sl::PositionalTrackingParameters trackParams;

  trackParams.area_file_path = mAreaMemDbPath.c_str();

  mPoseSmoothing = false;  // Always false. Pose Smoothing is to be enabled only for VR/AR applications
  trackParams.enable_pose_smoothing = mPoseSmoothing;

  trackParams.enable_area_memory = mAreaMemory;
  trackParams.enable_imu_fusion = mImuFusion;
  trackParams.initial_world_transform = mInitialPoseSl;

  trackParams.set_floor_as_origin = mFloorAlignment;

  sl::ERROR_CODE err = mZed.enablePositionalTracking(trackParams);

  if (err == sl::ERROR_CODE::SUCCESS)
  {
    mPosTrackingActivated = true;
  }
  else
  {
    mPosTrackingActivated = false;

    NODELET_WARN("Tracking not activated: %s", sl::toString(err).c_str());
  }
}

void ZEDWrapperNodelet::publishOdom(tf2::Transform odom2baseTransf, sl::Pose& slPose, ros::Time t)
{
  nav_msgs::OdometryPtr odomMsg = boost::make_shared<nav_msgs::Odometry>();

  odomMsg->header.stamp = t;
  odomMsg->header.frame_id = mOdometryFrameId;  // frame
  odomMsg->child_frame_id = mBaseFrameId;       // camera_frame
  // conversion from Tranform to message
  geometry_msgs::Transform base2odom = tf2::toMsg(odom2baseTransf);
  // Add all value in odometry message
  odomMsg->pose.pose.position.x = base2odom.translation.x;
  odomMsg->pose.pose.position.y = base2odom.translation.y;
  odomMsg->pose.pose.position.z = base2odom.translation.z;
  odomMsg->pose.pose.orientation.x = base2odom.rotation.x;
  odomMsg->pose.pose.orientation.y = base2odom.rotation.y;
  odomMsg->pose.pose.orientation.z = base2odom.rotation.z;
  odomMsg->pose.pose.orientation.w = base2odom.rotation.w;

  // Odometry pose covariance

  for (size_t i = 0; i < odomMsg->pose.covariance.size(); i++)
  {
    odomMsg->pose.covariance[i] = static_cast<double>(slPose.pose_covariance[i]);

    if (mTwoDMode)
    {
      if (i == 14 || i == 21 || i == 28)
      {
        odomMsg->pose.covariance[i] = 1e-9;  // Very low covariance if 2D mode
      }
      else if ((i >= 2 && i <= 4) || (i >= 8 && i <= 10) || (i >= 12 && i <= 13) || (i >= 15 && i <= 16) ||
               (i >= 18 && i <= 20) || (i == 22) || (i >= 24 && i <= 27))
      {
        odomMsg->pose.covariance[i] = 0.0;
      }
    }
  }

  // Publish odometry message
  mPubOdom.publish(odomMsg);
}

void ZEDWrapperNodelet::publishPose(ros::Time t)
{
  tf2::Transform base_pose;
  base_pose.setIdentity();

  if (mPublishMapTf)
  {
    base_pose = mMap2BaseTransf;
  }
  else if (mPublishTf)
  {
    base_pose = mOdom2BaseTransf;
  }

  std_msgs::Header header;
  header.stamp = t;
  header.frame_id = mMapFrameId;
  geometry_msgs::Pose pose;

  // conversion from Tranform to message
  geometry_msgs::Transform base2frame = tf2::toMsg(base_pose);

  // Add all value in Pose message
  pose.position.x = base2frame.translation.x;
  pose.position.y = base2frame.translation.y;
  pose.position.z = base2frame.translation.z;
  pose.orientation.x = base2frame.rotation.x;
  pose.orientation.y = base2frame.rotation.y;
  pose.orientation.z = base2frame.rotation.z;
  pose.orientation.w = base2frame.rotation.w;

  if (mPubPose.getNumSubscribers() > 0)
  {
    geometry_msgs::PoseStamped poseNoCov;

    poseNoCov.header = header;
    poseNoCov.pose = pose;

    // Publish pose stamped message
    mPubPose.publish(poseNoCov);
  }

  if (mPubPoseCov.getNumSubscribers() > 0)
  {
    geometry_msgs::PoseWithCovarianceStampedPtr poseCovMsg =
        boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>();

    poseCovMsg->header = header;
    poseCovMsg->pose.pose = pose;

    // Odometry pose covariance if available
    for (size_t i = 0; i < poseCovMsg->pose.covariance.size(); i++)
    {
      poseCovMsg->pose.covariance[i] = static_cast<double>(mLastZedPose.pose_covariance[i]);

      if (mTwoDMode)
      {
        if (i == 14 || i == 21 || i == 28)
        {
          poseCovMsg->pose.covariance[i] = 1e-9;  // Very low covariance if 2D mode
        }
        else if ((i >= 2 && i <= 4) || (i >= 8 && i <= 10) || (i >= 12 && i <= 13) || (i >= 15 && i <= 16) ||
                 (i >= 18 && i <= 20) || (i == 22) || (i >= 24 && i <= 27))
        {
          poseCovMsg->pose.covariance[i] = 0.0;
        }
      }
    }

    // Publish pose with covariance stamped message
    mPubPoseCov.publish(poseCovMsg);
  }
}

void ZEDWrapperNodelet::publishStaticImuFrame(const ros::Time& stamp)
{
  // Publish IMU TF as static TF
  if (!mPublishImuTf)
  {
    return;
  }

  if (mStaticImuFramePublished)
  {
    return;
  }

  NODELET_WARN_STREAM_ONCE("static imu transform " << stamp);
  mStaticImuTransformStamped.header.stamp = stamp;
  mStaticImuTransformStamped.header.frame_id = mLeftCamFrameId;
  mStaticImuTransformStamped.child_frame_id = mImuFrameId;
  sl::Translation sl_tr = mSlCamImuTransf.getTranslation();
  mStaticImuTransformStamped.transform.translation.x = sl_tr.x;
  mStaticImuTransformStamped.transform.translation.y = sl_tr.y;
  mStaticImuTransformStamped.transform.translation.z = sl_tr.z;
  sl::Orientation sl_or = mSlCamImuTransf.getOrientation();
  mStaticImuTransformStamped.transform.rotation.x = sl_or.ox;
  mStaticImuTransformStamped.transform.rotation.y = sl_or.oy;
  mStaticImuTransformStamped.transform.rotation.z = sl_or.oz;
  mStaticImuTransformStamped.transform.rotation.w = sl_or.ow;

  // Publish transformation
  mStaticTransformImuBroadcaster.sendTransform(mStaticImuTransformStamped);

  NODELET_INFO_STREAM("Published static transform '" << mImuFrameId << "' -> '" << mLeftCamFrameId << "' " << stamp);

  mStaticImuFramePublished = true;
}

void ZEDWrapperNodelet::publishOdomFrame(tf2::Transform odomTransf, ros::Time t)
{
  // ----> Avoid duplicated TF publishing
  static ros::Time last_stamp;

  if( t==last_stamp )
  {
    return;
  }
  last_stamp = t;
  // <---- Avoid duplicated TF publishing

  if (!mSensor2BaseTransfValid)
  {
    getSens2BaseTransform();
  }

  if (!mSensor2CameraTransfValid)
  {
    getSens2CameraTransform();
  }

  if (!mCamera2BaseTransfValid)
  {
    getCamera2BaseTransform();
  }

  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = t;
  transformStamped.header.frame_id = mOdometryFrameId;
  transformStamped.child_frame_id = mBaseFrameId;
  // conversion from Tranform to message
  transformStamped.transform = tf2::toMsg(odomTransf);
  // Publish transformation
  mTransformOdomBroadcaster.sendTransform(transformStamped);

  //NODELET_INFO_STREAM( "Published ODOM TF with TS: " << t );
}

void ZEDWrapperNodelet::publishPoseFrame(tf2::Transform baseTransform, ros::Time t)
{
  // ----> Avoid duplicated TF publishing
  static ros::Time last_stamp;

  if( t==last_stamp )
  {
    return;
  }
  last_stamp = t;
  // <---- Avoid duplicated TF publishing

  if (!mSensor2BaseTransfValid)
  {
    getSens2BaseTransform();
  }

  if (!mSensor2CameraTransfValid)
  {
    getSens2CameraTransform();
  }

  if (!mCamera2BaseTransfValid)
  {
    getCamera2BaseTransform();
  }

  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = t;
  transformStamped.header.frame_id = mMapFrameId;
  transformStamped.child_frame_id = mOdometryFrameId;
  // conversion from Tranform to message
  transformStamped.transform = tf2::toMsg(baseTransform);
  // Publish transformation
  mTransformPoseBroadcaster.sendTransform(transformStamped);

  // NODELET_INFO_STREAM( "Published POSE TF with TS: " << t );
}

void ZEDWrapperNodelet::publishImage(sensor_msgs::ImagePtr imgMsgPtr, sl::Mat img,
                                     image_transport::CameraPublisher& pubImg, sensor_msgs::CameraInfoPtr camInfoMsg,
                                     string imgFrameId, ros::Time t)
{
  camInfoMsg->header.stamp = t;
  sl_tools::imageToROSmsg(imgMsgPtr, img, imgFrameId, t);
  pubImg.publish(imgMsgPtr, camInfoMsg);
}

void ZEDWrapperNodelet::publishDepth(sensor_msgs::ImagePtr imgMsgPtr, sl::Mat depth, ros::Time t)
{
  mDepthCamInfoMsg->header.stamp = t;

  // NODELET_DEBUG_STREAM("mOpenniDepthMode: " << mOpenniDepthMode);

  if (!mOpenniDepthMode)
  {
    // NODELET_INFO("Using float32");
    sl_tools::imageToROSmsg(imgMsgPtr, depth, mDepthOptFrameId, t);
    mPubDepth.publish(imgMsgPtr, mDepthCamInfoMsg);

    return;
  }

#if (ZED_SDK_MAJOR_VERSION == 3 && ZED_SDK_MINOR_VERSION < 4)
  // OPENNI CONVERSION (meter -> millimeters - float32 -> uint16)
  if (!imgMsgPtr)
  {
    imgMsgPtr = boost::make_shared<sensor_msgs::Image>();
  }

  imgMsgPtr->header.stamp = t;
  imgMsgPtr->header.frame_id = mDepthOptFrameId;
  imgMsgPtr->height = depth.getHeight();
  imgMsgPtr->width = depth.getWidth();

  int num = 1;  // for endianness detection
  imgMsgPtr->is_bigendian = !(*(char*)&num == 1);

  imgMsgPtr->step = imgMsgPtr->width * sizeof(uint16_t);
  imgMsgPtr->encoding = sensor_msgs::image_encodings::TYPE_16UC1;

  size_t size = imgMsgPtr->step * imgMsgPtr->height;
  imgMsgPtr->data.resize(size);

  uint16_t* data = (uint16_t*)(&imgMsgPtr->data[0]);

  int dataSize = imgMsgPtr->width * imgMsgPtr->height;
  sl::float1* depthDataPtr = depth.getPtr<sl::float1>();

  for (int i = 0; i < dataSize; i++)
  {
    *(data++) = static_cast<uint16_t>(std::round(*(depthDataPtr++) * 1000));  // in mm, rounded
  }
  mPubDepth.publish(imgMsgPtr, mDepthCamInfoMsg);
#else
  // NODELET_INFO("Using depth16");
  sl_tools::imageToROSmsg(imgMsgPtr, depth, mDepthOptFrameId, t);
  mPubDepth.publish(imgMsgPtr, mDepthCamInfoMsg);
#endif
}

void ZEDWrapperNodelet::publishDisparity(sl::Mat disparity, ros::Time t)
{
  sl::CameraInformation zedParam = mZed.getCameraInformation(mMatResolDepth);

  sensor_msgs::ImagePtr disparityImgMsg = boost::make_shared<sensor_msgs::Image>();
  stereo_msgs::DisparityImagePtr disparityMsg = boost::make_shared<stereo_msgs::DisparityImage>();

  sl_tools::imageToROSmsg(disparityImgMsg, disparity, mDisparityFrameId, t);

  disparityMsg->image = *disparityImgMsg;
  disparityMsg->header = disparityMsg->image.header;

#if ZED_SDK_MAJOR_VERSION == 3 && ZED_SDK_MINOR_VERSION < 1
  disparityMsg->f = zedParam.calibration_parameters.left_cam.fx;
  disparityMsg->T = zedParam.calibration_parameters.T.x;
#else
  disparityMsg->f = zedParam.camera_configuration.calibration_parameters.left_cam.fx;
  disparityMsg->T = zedParam.camera_configuration.calibration_parameters.getCameraBaseline();
#endif

  if (disparityMsg->T > 0)
  {
    disparityMsg->T *= -1.0f;
  }

  disparityMsg->min_disparity = disparityMsg->f * disparityMsg->T / mZed.getInitParameters().depth_minimum_distance;
  disparityMsg->max_disparity = disparityMsg->f * disparityMsg->T / mZed.getInitParameters().depth_maximum_distance;

  mPubDisparity.publish(disparityMsg);
}

void ZEDWrapperNodelet::publishCamInfo(sensor_msgs::CameraInfoPtr camInfoMsg, ros::Publisher pubCamInfo, ros::Time t)
{
  static int seq = 0;
  camInfoMsg->header.stamp = t;
  camInfoMsg->header.seq = seq;
  pubCamInfo.publish(camInfoMsg);
  seq++;
}

void ZEDWrapperNodelet::fillCamInfo(sl::Camera& zed, sensor_msgs::CameraInfoPtr leftCamInfoMsg,
                                    sensor_msgs::CameraInfoPtr rightCamInfoMsg, string leftFrameId, string rightFrameId,
                                    bool rawParam /*= false*/)
{
  sl::CalibrationParameters zedParam;

#if ZED_SDK_MAJOR_VERSION == 3 && ZED_SDK_MINOR_VERSION < 1
  if (rawParam)
  {
    zedParam = zed.getCameraInformation(mMatResolVideo).calibration_parameters_raw;  // ok
  }
  else
  {
    zedParam = zed.getCameraInformation(mMatResolVideo).calibration_parameters;  // ok
  }
#else
  if (rawParam)
  {
    zedParam = zed.getCameraInformation(mMatResolVideo).camera_configuration.calibration_parameters_raw;
  }
  else
  {
    zedParam = zed.getCameraInformation(mMatResolVideo).camera_configuration.calibration_parameters;
  }
#endif

  float baseline = zedParam.getCameraBaseline();
  leftCamInfoMsg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
  rightCamInfoMsg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
  leftCamInfoMsg->D.resize(5);
  rightCamInfoMsg->D.resize(5);
  leftCamInfoMsg->D[0] = zedParam.left_cam.disto[0];    // k1
  leftCamInfoMsg->D[1] = zedParam.left_cam.disto[1];    // k2
  leftCamInfoMsg->D[2] = zedParam.left_cam.disto[4];    // k3
  leftCamInfoMsg->D[3] = zedParam.left_cam.disto[2];    // p1
  leftCamInfoMsg->D[4] = zedParam.left_cam.disto[3];    // p2
  rightCamInfoMsg->D[0] = zedParam.right_cam.disto[0];  // k1
  rightCamInfoMsg->D[1] = zedParam.right_cam.disto[1];  // k2
  rightCamInfoMsg->D[2] = zedParam.right_cam.disto[4];  // k3
  rightCamInfoMsg->D[3] = zedParam.right_cam.disto[2];  // p1
  rightCamInfoMsg->D[4] = zedParam.right_cam.disto[3];  // p2
  leftCamInfoMsg->K.fill(0.0);
  rightCamInfoMsg->K.fill(0.0);
  leftCamInfoMsg->K[0] = static_cast<double>(zedParam.left_cam.fx);
  leftCamInfoMsg->K[2] = static_cast<double>(zedParam.left_cam.cx);
  leftCamInfoMsg->K[4] = static_cast<double>(zedParam.left_cam.fy);
  leftCamInfoMsg->K[5] = static_cast<double>(zedParam.left_cam.cy);
  leftCamInfoMsg->K[8] = 1.0;
  rightCamInfoMsg->K[0] = static_cast<double>(zedParam.right_cam.fx);
  rightCamInfoMsg->K[2] = static_cast<double>(zedParam.right_cam.cx);
  rightCamInfoMsg->K[4] = static_cast<double>(zedParam.right_cam.fy);
  rightCamInfoMsg->K[5] = static_cast<double>(zedParam.right_cam.cy);
  rightCamInfoMsg->K[8] = 1.0;
  leftCamInfoMsg->R.fill(0.0);
  rightCamInfoMsg->R.fill(0.0);

  for (size_t i = 0; i < 3; i++)
  {
    // identity
    rightCamInfoMsg->R[i + i * 3] = 1;
    leftCamInfoMsg->R[i + i * 3] = 1;
  }

#if ZED_SDK_MAJOR_VERSION == 3 && ZED_SDK_MINOR_VERSION < 1
  if (rawParam)
  {
    std::vector<float> R_ = sl_tools::convertRodrigues(zedParam.R);
    float* p = R_.data();

    for (int i = 0; i < 9; i++)
    {
      rightCamInfoMsg->R[i] = p[i];
    }
  }
#else
  if (rawParam)
  {
    if (mUseOldExtrinsic)
    {  // Camera frame (Z forward, Y down, X right)

      std::vector<float> R_ = sl_tools::convertRodrigues(zedParam.R);
      float* p = R_.data();

      for (int i = 0; i < 9; i++)
      {
        rightCamInfoMsg->R[i] = p[i];
      }
    }
    else
    {  // ROS frame (X forward, Z up, Y left)
      for (int i = 0; i < 9; i++)
      {
        rightCamInfoMsg->R[i] = zedParam.stereo_transform.getRotationMatrix().r[i];
      }
    }
  }
#endif

  leftCamInfoMsg->P.fill(0.0);
  rightCamInfoMsg->P.fill(0.0);
  leftCamInfoMsg->P[0] = static_cast<double>(zedParam.left_cam.fx);
  leftCamInfoMsg->P[2] = static_cast<double>(zedParam.left_cam.cx);
  leftCamInfoMsg->P[5] = static_cast<double>(zedParam.left_cam.fy);
  leftCamInfoMsg->P[6] = static_cast<double>(zedParam.left_cam.cy);
  leftCamInfoMsg->P[10] = 1.0;
  // http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
  rightCamInfoMsg->P[3] = static_cast<double>(-1 * zedParam.left_cam.fx * baseline);
  rightCamInfoMsg->P[0] = static_cast<double>(zedParam.right_cam.fx);
  rightCamInfoMsg->P[2] = static_cast<double>(zedParam.right_cam.cx);
  rightCamInfoMsg->P[5] = static_cast<double>(zedParam.right_cam.fy);
  rightCamInfoMsg->P[6] = static_cast<double>(zedParam.right_cam.cy);
  rightCamInfoMsg->P[10] = 1.0;
  leftCamInfoMsg->width = rightCamInfoMsg->width = static_cast<uint32_t>(mMatResolVideo.width);
  leftCamInfoMsg->height = rightCamInfoMsg->height = static_cast<uint32_t>(mMatResolVideo.height);
  leftCamInfoMsg->header.frame_id = leftFrameId;
  rightCamInfoMsg->header.frame_id = rightFrameId;
}

void ZEDWrapperNodelet::fillCamDepthInfo(sl::Camera& zed, sensor_msgs::CameraInfoPtr depth_info_msg, string frame_id)
{
  sl::CalibrationParameters zedParam;

#if ZED_SDK_MAJOR_VERSION == 3 && ZED_SDK_MINOR_VERSION < 1
  zedParam = zed.getCameraInformation(mMatResolDepth).calibration_parameters;
#else
  zedParam = zed.getCameraInformation(mMatResolDepth).camera_configuration.calibration_parameters;
#endif

  float baseline = zedParam.getCameraBaseline();
  depth_info_msg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
  depth_info_msg->D.resize(5);
  depth_info_msg->D[0] = zedParam.left_cam.disto[0];  // k1
  depth_info_msg->D[1] = zedParam.left_cam.disto[1];  // k2
  depth_info_msg->D[2] = zedParam.left_cam.disto[4];  // k3
  depth_info_msg->D[3] = zedParam.left_cam.disto[2];  // p1
  depth_info_msg->D[4] = zedParam.left_cam.disto[3];  // p2
  depth_info_msg->K.fill(0.0);
  depth_info_msg->K[0] = static_cast<double>(zedParam.left_cam.fx);
  depth_info_msg->K[2] = static_cast<double>(zedParam.left_cam.cx);
  depth_info_msg->K[4] = static_cast<double>(zedParam.left_cam.fy);
  depth_info_msg->K[5] = static_cast<double>(zedParam.left_cam.cy);
  depth_info_msg->K[8] = 1.0;
  depth_info_msg->R.fill(0.0);

  for (size_t i = 0; i < 3; i++)
  {
    // identity
    depth_info_msg->R[i + i * 3] = 1;
  }

  depth_info_msg->P.fill(0.0);
  depth_info_msg->P[0] = static_cast<double>(zedParam.left_cam.fx);
  depth_info_msg->P[2] = static_cast<double>(zedParam.left_cam.cx);
  depth_info_msg->P[5] = static_cast<double>(zedParam.left_cam.fy);
  depth_info_msg->P[6] = static_cast<double>(zedParam.left_cam.cy);
  depth_info_msg->P[10] = 1.0;
  // http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
  depth_info_msg->width = static_cast<uint32_t>(mMatResolDepth.width);
  depth_info_msg->height = static_cast<uint32_t>(mMatResolDepth.height);
  depth_info_msg->header.frame_id = frame_id;
}

void ZEDWrapperNodelet::updateDynamicReconfigureServer()
{
  zed_nodelets::ZedConfig config;
  {
    // TODO(lucasw) the purpose of this lock is to have a consistent snapshot of the parameters?
    std::lock_guard<std::mutex> lock(mDynParMutex);
    config.auto_exposure_gain = mCamAutoExposure;
    config.auto_whitebalance = mCamAutoWB;
    config.brightness = mCamBrightness;
    config.depth_confidence = mCamDepthConfidence;
    config.depth_texture_conf = mCamDepthTextureConf;
    config.contrast = mCamContrast;
    config.exposure = mCamExposure;
    config.gain = mCamGain;
    config.hue = mCamHue;
    config.saturation = mCamSaturation;
    config.sharpness = mCamSharpness;
    config.gamma = mCamGamma;
    // TODO(lucasw) get rid of this 100 scale factor
    config.whitebalance_temperature = mCamWB / 100;
    // config.point_cloud_freq = mPointCloudFreq;
    config.pub_frame_rate = mVideoDepthFreq;
  }

  // std::lock_guard<std::mutex> lock(mDynServerMutex);
  mDynServerMutex.lock();
  mDynRecServer->updateConfig(config);
  mDynServerMutex.unlock();

  mUpdateDynParams = false;
}

void ZEDWrapperNodelet::callback_dynamicReconf(zed_nodelets::ZedConfig& config, uint32_t level)
{
  std::lock_guard<std::mutex> lock(mDynParMutex);

  DynParams param = static_cast<DynParams>(level);

  switch (param)
  {
    case DATAPUB_FREQ:
      if (config.pub_frame_rate > mCamFrameRate)
      {
        mVideoDepthFreq = mCamFrameRate;
        NODELET_WARN_STREAM("'pub_frame_rate' cannot be major than camera grabbing framerate. Set to "
                            << mVideoDepthFreq);

        mUpdateDynParams = true;
      }
      else
      {
        mVideoDepthFreq = config.pub_frame_rate;
        NODELET_INFO("Reconfigure Video and Depth pub. frequency: %g", mVideoDepthFreq);
      }

      mVideoDepthTimer.setPeriod(ros::Duration(1.0 / mVideoDepthFreq));
      break;

    case CONFIDENCE:
      mCamDepthConfidence = config.depth_confidence;
      NODELET_INFO("Reconfigure confidence threshold: %d", mCamDepthConfidence);
      break;

    case TEXTURE_CONF:
      mCamDepthTextureConf = config.depth_texture_conf;
      NODELET_INFO("Reconfigure texture confidence threshold: %d", mCamDepthTextureConf);
      break;

    case BRIGHTNESS:
      mCamBrightness = config.brightness;
      NODELET_INFO("Reconfigure image brightness: %d", mCamBrightness);
      break;

    case CONTRAST:
      mCamContrast = config.contrast;
      NODELET_INFO("Reconfigure image contrast: %d", mCamContrast);
      break;

    case HUE:
      mCamHue = config.hue;
      NODELET_INFO("Reconfigure image hue: %d", mCamHue);
      break;

    case SATURATION:
      mCamSaturation = config.saturation;
      NODELET_INFO("Reconfigure image saturation: %d", mCamSaturation);
      break;

    case SHARPNESS:
      mCamSharpness = config.sharpness;
      NODELET_INFO("Reconfigure image sharpness: %d", mCamSharpness);
      break;

    case GAMMA:
#if (ZED_SDK_MAJOR_VERSION == 3 && ZED_SDK_MINOR_VERSION >= 1)
      mCamGamma = config.gamma;
      NODELET_INFO("Reconfigure image gamma: %d", mCamGamma);
#else
      NODELET_DEBUG_STREAM("Gamma Control is not available for SDK older that v3.1");
#endif
      break;

    case AUTO_EXP_GAIN:
      mCamAutoExposure = config.auto_exposure_gain;
      NODELET_INFO_STREAM("Reconfigure auto exposure/gain: " << mCamAutoExposure ? "ENABLED" : "DISABLED");
      if (!mCamAutoExposure)
      {
        mZed.setCameraSettings(sl::VIDEO_SETTINGS::AEC_AGC, 0);
        mTriggerAutoExposure = false;
      }
      else
      {
        mTriggerAutoExposure = true;
      }
      break;

    case GAIN:
      mCamGain = config.gain;
      if (mCamAutoExposure)
      {
        NODELET_WARN("Reconfigure gain has no effect if 'auto_exposure_gain' is enabled");
      }
      else
      {
        NODELET_INFO("Reconfigure gain: %d", mCamGain);
      }
      break;

    case EXPOSURE:
      mCamExposure = config.exposure;
      if (mCamAutoExposure)
      {
        NODELET_WARN("Reconfigure exposure has no effect if 'auto_exposure_gain' is enabled");
      }
      else
      {
        NODELET_INFO("Reconfigure exposure: %d", mCamExposure);
      }
      break;

    case AUTO_WB:
      mCamAutoWB = config.auto_whitebalance;
      NODELET_INFO_STREAM("Reconfigure auto white balance: " << mCamAutoWB ? "ENABLED" : "DISABLED");
      if (!mCamAutoWB)
      {
        mZed.setCameraSettings(sl::VIDEO_SETTINGS::WHITEBALANCE_AUTO, 0);
        mTriggerAutoWB = false;
      }
      else
      {
        mTriggerAutoWB = true;
      }
      break;

    case WB_TEMP:
      mCamWB = config.whitebalance_temperature * 100;
      if (mCamAutoWB)
      {
        NODELET_WARN("Reconfigure white balance temperature has no effect if 'auto_whitebalance' is enabled");
      }
      else
      {
        NODELET_INFO("Reconfigure white balance temperature: %d", mCamWB);
      }
      break;

    default:
      NODELET_DEBUG_STREAM("dynamicReconfCallback Unknown param: " << level);
  }
}

bool ZEDWrapperNodelet::getCamData(
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
    sl::Timestamp& grab_ts, sl::Timestamp& lastZedTs)
{
  bool retrieved = false;

  sl::Timestamp rgb_ts = 0;  // used to check RGB/Depth sync

  std::lock_guard<std::mutex> lock(mCamDataMutex);

  // ----> Retrieve all required image data
  if (get_left)
  {
    mZed.retrieveImage(mat_left, sl::VIEW::LEFT, sl::MEM::CPU, mMatResolVideo);
    retrieved = true;
    rgb_ts = mat_left.timestamp;
    grab_ts = mat_left.timestamp;
  }
  if (get_left_raw)
  {
    mZed.retrieveImage(mat_left_raw, sl::VIEW::LEFT_UNRECTIFIED, sl::MEM::CPU, mMatResolVideo);
    retrieved = true;
    grab_ts = mat_left_raw.timestamp;
  }
  if (get_right)
  {
    mZed.retrieveImage(mat_right, sl::VIEW::RIGHT, sl::MEM::CPU, mMatResolVideo);
    retrieved = true;
    grab_ts = mat_right.timestamp;
  }
  if (get_right_raw)
  {
    mZed.retrieveImage(mat_right_raw, sl::VIEW::RIGHT_UNRECTIFIED, sl::MEM::CPU, mMatResolVideo);
    retrieved = true;
    grab_ts = mat_right_raw.timestamp;
  }
  if (get_depth)
  {
#if (ZED_SDK_MAJOR_VERSION == 3 && ZED_SDK_MINOR_VERSION < 4)
    mZed.retrieveMeasure(mat_depth, sl::MEASURE::DEPTH, sl::MEM::CPU, mMatResolDepth);
#else
    if (!mOpenniDepthMode)
    {
      mZed.retrieveMeasure(mat_depth, sl::MEASURE::DEPTH, sl::MEM::CPU, mMatResolDepth);
    }
    else
    {
      mZed.retrieveMeasure(mat_depth, sl::MEASURE::DEPTH_U16_MM, sl::MEM::CPU, mMatResolDepth);
    }
#endif
    retrieved = true;
    grab_ts = mat_depth.timestamp;

    // used to check RGB/Depth sync
    const sl::Timestamp depth_ts = mat_depth.timestamp;

    if (rgb_ts.data_ns != 0 && (depth_ts.data_ns != rgb_ts.data_ns))
    {
      NODELET_WARN_STREAM("!!!!! DEPTH/RGB ASYNC !!!!! - Delta: " << 1e-9 * static_cast<double>(depth_ts - rgb_ts)
                                                                  << " sec");
    }
  }
  if (get_disparity)
  {
    mZed.retrieveMeasure(mat_disp, sl::MEASURE::DISPARITY, sl::MEM::CPU, mMatResolDepth);
    retrieved = true;
    grab_ts = mat_disp.timestamp;
  }
  if (get_conf_map)
  {
    mZed.retrieveMeasure(mat_conf, sl::MEASURE::CONFIDENCE, sl::MEM::CPU, mMatResolDepth);
    retrieved = true;
    grab_ts = mat_conf.timestamp;
  }
  // <---- Retrieve all required image data

  // ----> Data ROS timestamp
  stamp = sl_tools::slTime2Ros(grab_ts);
  if (mSvoMode)
  {
    // grab_ts is 0 when in svo playback mode
    stamp = update_stamp;
  }
  NODELET_INFO_STREAM_ONCE("time " << stamp);
  // <---- Data ROS timestamp

  // ----> Publish sensor data if sync is required by user or SVO
  if (mZedRealCamModel != sl::MODEL::ZED)
  {
    if (mSensTimestampSync)
    {
      // NODELET_INFO_STREAM("tot_sub: " << tot_sub << " - retrieved: " << retrieved << " -
      // grab_ts.data_ns!=lastZedTs.data_ns: " << (grab_ts.data_ns!=lastZedTs.data_ns));
      //
      const bool get_any = (get_left || get_left_raw || get_right || get_right_raw ||
                           get_depth || get_disparity || get_conf_map);
      if (get_any && retrieved && (grab_ts.data_ns != lastZedTs.data_ns))
      {
        // NODELET_INFO("CALLBACK");
        publishSensData(stamp);
      }
    }
    else if (mSvoMode)
    {
      publishSensData(stamp);
    }
  }

  return retrieved;
}

void ZEDWrapperNodelet::callback_pubVideoDepth(const ros::TimerEvent& e)
{
  // TODO(lucasw) wrap these up in a struct
  const bool get_left = mPubLeft.getNumSubscribers() > 0;
  const bool get_left_raw = mPubRawLeft.getNumSubscribers() > 0;
  const bool get_right = mPubRight.getNumSubscribers() > 0;
  const bool get_right_raw = mPubRawRight.getNumSubscribers() > 0;
  const bool get_depth = mPubDepth.getNumSubscribers() > 0;
  const bool get_disparity = mPubDisparity.getNumSubscribers() > 0;
  const bool get_conf_map = mPubConfMap.getNumSubscribers() > 0;

  sl::Mat mat_left, mat_left_raw;
  sl::Mat mat_right, mat_right_raw;
  sl::Mat mat_depth, mat_disp, mat_conf;

  ros::Time stamp;
  sl::Timestamp grab_ts = 0;
  sl::Timestamp lastZedTs = 0;  // Used to calculate stable publish frequency

  const bool retrieved = getCamData(e.current_real,
                                    get_left, get_left_raw, get_right, get_right_raw,
                                    get_depth, get_disparity, get_conf_map,
                                    mat_left, mat_left_raw,
                                    mat_right, mat_right_raw,
                                    mat_depth, mat_disp, mat_conf,
                                    stamp, grab_ts, lastZedTs);

  // ----> Notify grab thread that all data are synchronized and a new grab can be done
  // mRgbDepthDataRetrievedCondVar.notify_one();
  // mRgbDepthDataRetrieved = true;
  // <---- Notify grab thread that all data are synchronized and a new grab can be done

  if (!retrieved)
  {
    mPublishingData = false;
    lastZedTs = 0;
    return;
  }
  mPublishingData = true;

  // ----> Check if a grab has been done before publishing the same images
  if (grab_ts.data_ns == lastZedTs.data_ns)
  {
    // Data not updated by a grab calling in the grab thread
    return;
  }
  if (lastZedTs.data_ns != 0)
  {
    double period_sec = static_cast<double>(grab_ts.data_ns - lastZedTs.data_ns) / 1e9;
    // NODELET_DEBUG_STREAM( "PUBLISHING PERIOD: " << period_sec << " sec @" << 1./period_sec << " Hz") ;

    mVideoDepthPeriodMean_sec->addValue(period_sec);
    // NODELET_DEBUG_STREAM( "MEAN PUBLISHING PERIOD: " << mVideoDepthPeriodMean_sec->getMean() << " sec @"
    // << 1./mVideoDepthPeriodMean_sec->getMean() << " Hz") ;
  }
  lastZedTs = grab_ts;
  // <---- Check if a grab has been done before publishing the same images

  // Publish the left = rgb image if someone has subscribed to
  if (get_left)
  {
    sensor_msgs::ImagePtr leftImgMsg = boost::make_shared<sensor_msgs::Image>();
    publishImage(leftImgMsg, mat_left, mPubLeft, mLeftCamInfoMsg, mLeftCamOptFrameId, stamp);
  }

  // Publish the left_raw = rgb_raw image if someone has subscribed to
  if (get_left_raw)
  {
    sensor_msgs::ImagePtr rawLeftImgMsg = boost::make_shared<sensor_msgs::Image>();
    publishImage(rawLeftImgMsg, mat_left_raw, mPubRawLeft, mLeftCamInfoRawMsg, mLeftCamOptFrameId, stamp);
  }

  // Publish the right image if someone has subscribed to
  if (get_right)
  {
    sensor_msgs::ImagePtr rightImgMsg = boost::make_shared<sensor_msgs::Image>();
    publishImage(rightImgMsg, mat_right, mPubRight, mRightCamInfoMsg, mRightCamOptFrameId, stamp);
  }

  // Publish the right raw image if someone has subscribed to
  if (get_right_raw)
  {
    sensor_msgs::ImagePtr rawRightImgMsg = boost::make_shared<sensor_msgs::Image>();
    publishImage(rawRightImgMsg, mat_right_raw, mPubRawRight, mRightCamInfoRawMsg, mRightCamOptFrameId, stamp);
  }

  // Publish the depth image if someone has subscribed to
  if (get_depth)
  {
    sensor_msgs::ImagePtr depthImgMsg = boost::make_shared<sensor_msgs::Image>();
    publishDepth(depthImgMsg, mat_depth, stamp);
  }

  // Publish the disparity image if someone has subscribed to
  if (get_disparity)
  {
    publishDisparity(mat_disp, stamp);
  }

  // Publish the confidence map if someone has subscribed to
  if (get_conf_map)
  {
    sensor_msgs::ImagePtr confMapMsg = boost::make_shared<sensor_msgs::Image>();
    sl_tools::imageToROSmsg(confMapMsg, mat_conf, mConfidenceOptFrameId, stamp);
    // TODO(lucasw) reuse depth cam info, probably is correct for confidence map?
    mDepthCamInfoMsg->header.stamp = stamp;
    mPubConfMap.publish(confMapMsg, mDepthCamInfoMsg);
  }
}

void ZEDWrapperNodelet::callback_pubPath(const ros::TimerEvent& e)
{
  NODELET_INFO_STREAM_ONCE("pub path " << e.current_real);
  uint32_t mapPathSub = mPubMapPath.getNumSubscribers();
  uint32_t odomPathSub = mPubOdomPath.getNumSubscribers();

  geometry_msgs::PoseStamped odomPose;
  geometry_msgs::PoseStamped mapPose;

  odomPose.header.stamp = mFrameTimestamp;
  odomPose.header.frame_id = mMapFrameId;  // frame
  // conversion from Tranform to message
  geometry_msgs::Transform base2odom = tf2::toMsg(mOdom2BaseTransf);

  // Add all value in Pose message
  // TODO(lucasw) generic message converter
  odomPose.pose.position.x = base2odom.translation.x;
  odomPose.pose.position.y = base2odom.translation.y;
  odomPose.pose.position.z = base2odom.translation.z;
  odomPose.pose.orientation.x = base2odom.rotation.x;
  odomPose.pose.orientation.y = base2odom.rotation.y;
  odomPose.pose.orientation.z = base2odom.rotation.z;
  odomPose.pose.orientation.w = base2odom.rotation.w;

  mapPose.header.stamp = mFrameTimestamp;
  mapPose.header.frame_id = mMapFrameId;  // frame
  // conversion from Tranform to message
  geometry_msgs::Transform base2map = tf2::toMsg(mMap2BaseTransf);
  // Add all value in Pose message
  // TODO(lucasw) generic message converter
  mapPose.pose.position.x = base2map.translation.x;
  mapPose.pose.position.y = base2map.translation.y;
  mapPose.pose.position.z = base2map.translation.z;
  mapPose.pose.orientation.x = base2map.rotation.x;
  mapPose.pose.orientation.y = base2map.rotation.y;
  mapPose.pose.orientation.z = base2map.rotation.z;
  mapPose.pose.orientation.w = base2map.rotation.w;

  // Circular vector
  if (mPathMaxCount != -1)
  {
    if (mOdomPath.size() == mPathMaxCount)
    {
      NODELET_DEBUG("Path vectors full: rotating ");
      std::rotate(mOdomPath.begin(), mOdomPath.begin() + 1, mOdomPath.end());
      std::rotate(mMapPath.begin(), mMapPath.begin() + 1, mMapPath.end());

      mMapPath[mPathMaxCount - 1] = mapPose;
      mOdomPath[mPathMaxCount - 1] = odomPose;
    }
    else
    {
      // NODELET_DEBUG_STREAM("Path vectors adding last available poses");
      mMapPath.push_back(mapPose);
      mOdomPath.push_back(odomPose);
    }
  }
  else
  {
    // NODELET_DEBUG_STREAM("No limit path vectors, adding last available poses");
    mMapPath.push_back(mapPose);
    mOdomPath.push_back(odomPose);
  }

  if (mapPathSub > 0)
  {
    nav_msgs::PathPtr mapPath = boost::make_shared<nav_msgs::Path>();
    mapPath->header.frame_id = mMapFrameId;
    mapPath->header.stamp = mFrameTimestamp;
    mapPath->poses = mMapPath;

    mPubMapPath.publish(mapPath);
  }

  if (odomPathSub > 0)
  {
    nav_msgs::PathPtr odomPath = boost::make_shared<nav_msgs::Path>();
    odomPath->header.frame_id = mMapFrameId;
    odomPath->header.stamp = mFrameTimestamp;
    odomPath->poses = mOdomPath;

    mPubOdomPath.publish(odomPath);
  }
}

void ZEDWrapperNodelet::callback_pubSensorsData(const ros::TimerEvent& e)
{
  // NODELET_INFO("callback_pubSensorsData");
  // TODO(lucasw) the only other user of this mutex is stop()
  std::lock_guard<std::mutex> lock(mCloseZedMutex);

  if (!mZed.isOpened())
  {
    return;
  }

  publishSensData();
}

void ZEDWrapperNodelet::publishSensData(ros::Time t)
{
  // NODELET_INFO("publishSensData");

  uint32_t imu_SubNumber = mPubImu.getNumSubscribers();
  uint32_t imu_RawSubNumber = mPubImuRaw.getNumSubscribers();
  uint32_t imu_TempSubNumber = 0;
  uint32_t imu_MagSubNumber = 0;
  uint32_t pressSubNumber = 0;
  uint32_t tempLeftSubNumber = 0;
  uint32_t tempRightSubNumber = 0;

  if (mZedRealCamModel == sl::MODEL::ZED2 || mZedRealCamModel == sl::MODEL::ZED2i)
  {
    imu_TempSubNumber = mPubImuTemp.getNumSubscribers();
    imu_MagSubNumber = mPubImuMag.getNumSubscribers();
    pressSubNumber = mPubPressure.getNumSubscribers();
    tempLeftSubNumber = mPubTempL.getNumSubscribers();
    tempRightSubNumber = mPubTempR.getNumSubscribers();
  }

  uint32_t tot_sub = imu_SubNumber + imu_RawSubNumber + imu_TempSubNumber + imu_MagSubNumber + pressSubNumber +
                     tempLeftSubNumber + tempRightSubNumber;

  if (tot_sub > 0)
  {
    mSensPublishing = true;
  }
  else
  {
    mSensPublishing = false;
  }

  bool sensors_data_published = false;

  ros::Time ts_imu;
  ros::Time ts_baro;
  ros::Time ts_mag;

  static ros::Time lastTs_imu = ros::Time();
  static ros::Time lastTs_baro = ros::Time();
  static ros::Time lastT_mag = ros::Time();

  sl::SensorsData sens_data;

  // TODO(lucasw) is a mUseSimTime check needed in here?
  if (mSvoMode || mSensTimestampSync)
  {
    if (mZed.getSensorsData(sens_data, sl::TIME_REFERENCE::IMAGE) != sl::ERROR_CODE::SUCCESS)
    {
      NODELET_DEBUG("Not retrieved sensors data in IMAGE REFERENCE TIME");
      return;
    }
  }
  else
  {
    if (mZed.getSensorsData(sens_data, sl::TIME_REFERENCE::CURRENT) != sl::ERROR_CODE::SUCCESS)
    {
      NODELET_DEBUG("Not retrieved sensors data in CURRENT REFERENCE TIME");
      return;
    }
  }

  if (t != ros::Time(0))
  {
    ts_imu = t;
    ts_baro = t;
    ts_mag = t;
  }
  else
  {
    ts_imu = sl_tools::slTime2Ros(sens_data.imu.timestamp);
    ts_baro = sl_tools::slTime2Ros(sens_data.barometer.timestamp);
    ts_mag = sl_tools::slTime2Ros(sens_data.magnetometer.timestamp);
  }
  // TODO(lucasw) in svo mode the above timestamps need to be set to ros::Time::now()?

  bool new_imu_data = ts_imu != lastTs_imu;
  bool new_baro_data = ts_baro != lastTs_baro;
  bool new_mag_data = ts_mag != lastT_mag;

  if (!new_imu_data && !new_baro_data && !new_mag_data)
  {
    NODELET_DEBUG("No updated sensors data");
    return;
  }

  // ----> Publish odometry tf only if enabled
  if (mPublishTf && mPosTrackingReady && new_imu_data)
  {
    NODELET_DEBUG("Publishing TF");

    publishOdomFrame(mOdom2BaseTransf, ts_imu);  // publish the base Frame in odometry frame

    if (mPublishMapTf)
    {
      publishPoseFrame(mMap2OdomTransf, ts_imu);  // publish the odometry Frame in map frame
    }
  }

  if (new_imu_data)
  {
    if (mPublishImuTf && !mStaticImuFramePublished)
    {
      publishStaticImuFrame(ts_imu);
    }
  }

  // <---- Publish odometry tf only if enabled

  if (mZedRealCamModel == sl::MODEL::ZED2 || mZedRealCamModel == sl::MODEL::ZED2i)
  {
    // Update temperatures for Diagnostic
    sens_data.temperature.get(sl::SensorsData::TemperatureData::SENSOR_LOCATION::ONBOARD_LEFT, mTempLeft);
    sens_data.temperature.get(sl::SensorsData::TemperatureData::SENSOR_LOCATION::ONBOARD_RIGHT, mTempRight);
  }

  if (imu_TempSubNumber > 0 && new_imu_data)
  {
    lastTs_imu = ts_imu;

    sensor_msgs::TemperaturePtr imuTempMsg = boost::make_shared<sensor_msgs::Temperature>();

    imuTempMsg->header.stamp = ts_imu;

#ifdef DEBUG_SENS_TS
    static ros::Time old_ts;
    if (old_ts == imuTempMsg->header.stamp)
    {
      NODELET_WARN_STREAM("Publishing IMU data with old timestamp " << old_ts);
    }
    old_ts = imuTempMsg->header.stamp;
#endif

    imuTempMsg->header.frame_id = mImuFrameId;
    float imu_temp;
    sens_data.temperature.get(sl::SensorsData::TemperatureData::SENSOR_LOCATION::IMU, imu_temp);
    imuTempMsg->temperature = static_cast<double>(imu_temp);
    imuTempMsg->variance = 0.0;

    sensors_data_published = true;
    mPubImuTemp.publish(imuTempMsg);
  }
  else
  {
    NODELET_DEBUG("No new IMU temp.");
  }

  if (sens_data.barometer.is_available && new_baro_data)
  {
    lastTs_baro = ts_baro;

    if (pressSubNumber > 0)
    {
      sensor_msgs::FluidPressurePtr pressMsg = boost::make_shared<sensor_msgs::FluidPressure>();

      pressMsg->header.stamp = ts_baro;

#ifdef DEBUG_SENS_TS
      static ros::Time old_ts;
      if (old_ts == pressMsg->header.stamp)
      {
        NODELET_WARN_STREAM("Publishing BARO data with old timestamp " << old_ts);
      }
      old_ts = pressMsg->header.stamp;
#endif
      pressMsg->header.frame_id = mBaroFrameId;
      pressMsg->fluid_pressure = sens_data.barometer.pressure * 1e-2;  // Pascal
      pressMsg->variance = 1.0585e-2;

      sensors_data_published = true;
      mPubPressure.publish(pressMsg);
    }

    if (tempLeftSubNumber > 0)
    {
      sensor_msgs::TemperaturePtr tempLeftMsg = boost::make_shared<sensor_msgs::Temperature>();

      tempLeftMsg->header.stamp = ts_baro;

#ifdef DEBUG_SENS_TS
      static ros::Time old_ts;
      if (old_ts == tempLeftMsg->header.stamp)
      {
        NODELET_WARN_STREAM("Publishing BARO data with old timestamp " << old_ts);
      }
      old_ts = tempLeftMsg->header.stamp;
#endif

      tempLeftMsg->header.frame_id = mTempLeftFrameId;
      tempLeftMsg->temperature = static_cast<double>(mTempLeft);
      tempLeftMsg->variance = 0.0;

      sensors_data_published = true;
      mPubTempL.publish(tempLeftMsg);
    }

    if (tempRightSubNumber > 0)
    {
      sensor_msgs::TemperaturePtr tempRightMsg = boost::make_shared<sensor_msgs::Temperature>();

      tempRightMsg->header.stamp = ts_baro;

#ifdef DEBUG_SENS_TS
      static ros::Time old_ts;
      if (old_ts == tempRightMsg->header.stamp)
      {
        NODELET_WARN_STREAM("Publishing BARO data with old timestamp " << old_ts);
      }
      old_ts = tempRightMsg->header.stamp;
#endif

      tempRightMsg->header.frame_id = mTempRightFrameId;
      tempRightMsg->temperature = static_cast<double>(mTempRight);
      tempRightMsg->variance = 0.0;

      sensors_data_published = true;
      mPubTempR.publish(tempRightMsg);
    }
  }
  else
  {
    NODELET_DEBUG("No new BAROM. DATA");
  }

  if (imu_MagSubNumber > 0)
  {
    if (sens_data.magnetometer.is_available && new_mag_data)
    {
      lastT_mag = ts_mag;

      sensor_msgs::MagneticFieldPtr magMsg = boost::make_shared<sensor_msgs::MagneticField>();

      magMsg->header.stamp = ts_mag;

#ifdef DEBUG_SENS_TS
      static ros::Time old_ts;
      if (old_ts == magMsg->header.stamp)
      {
        NODELET_WARN_STREAM("Publishing MAG data with old timestamp " << old_ts);
      }
      old_ts = magMsg->header.stamp;
#endif

      magMsg->header.frame_id = mMagFrameId;
      magMsg->magnetic_field.x = sens_data.magnetometer.magnetic_field_calibrated.x * 1e-6;  // Tesla
      magMsg->magnetic_field.y = sens_data.magnetometer.magnetic_field_calibrated.y * 1e-6;  // Tesla
      magMsg->magnetic_field.z = sens_data.magnetometer.magnetic_field_calibrated.z * 1e-6;  // Tesla
      magMsg->magnetic_field_covariance[0] = 0.039e-6;
      magMsg->magnetic_field_covariance[1] = 0.0f;
      magMsg->magnetic_field_covariance[2] = 0.0f;
      magMsg->magnetic_field_covariance[3] = 0.0f;
      magMsg->magnetic_field_covariance[4] = 0.037e-6;
      magMsg->magnetic_field_covariance[5] = 0.0f;
      magMsg->magnetic_field_covariance[6] = 0.0f;
      magMsg->magnetic_field_covariance[7] = 0.0f;
      magMsg->magnetic_field_covariance[8] = 0.047e-6;

      sensors_data_published = true;
      mPubImuMag.publish(magMsg);
    }
  }
  else
  {
    NODELET_DEBUG("No new MAG. DATA");
  }

  if (imu_SubNumber > 0 && new_imu_data)
  {
    lastTs_imu = ts_imu;

    sensor_msgs::ImuPtr imuMsg = boost::make_shared<sensor_msgs::Imu>();

    imuMsg->header.stamp = ts_imu;

#ifdef DEBUG_SENS_TS
    static ros::Time old_ts;
    if (old_ts == imuMsg->header.stamp)
    {
      NODELET_WARN_STREAM("Publishing IMU data with old timestamp " << old_ts);
    }
    else
    {
      NODELET_INFO_STREAM("Publishing IMU data with new timestamp. Freq: " << 1. / (ts_imu.toSec() - old_ts.toSec()));
      old_ts = imuMsg->header.stamp;
    }
#endif

    imuMsg->header.frame_id = mImuFrameId;

    imuMsg->orientation.x = sens_data.imu.pose.getOrientation()[0];
    imuMsg->orientation.y = sens_data.imu.pose.getOrientation()[1];
    imuMsg->orientation.z = sens_data.imu.pose.getOrientation()[2];
    imuMsg->orientation.w = sens_data.imu.pose.getOrientation()[3];

    imuMsg->angular_velocity.x = sens_data.imu.angular_velocity[0] * DEG2RAD;
    imuMsg->angular_velocity.y = sens_data.imu.angular_velocity[1] * DEG2RAD;
    imuMsg->angular_velocity.z = sens_data.imu.angular_velocity[2] * DEG2RAD;

    imuMsg->linear_acceleration.x = sens_data.imu.linear_acceleration[0];
    imuMsg->linear_acceleration.y = sens_data.imu.linear_acceleration[1];
    imuMsg->linear_acceleration.z = sens_data.imu.linear_acceleration[2];

    for (int i = 0; i < 3; ++i)
    {
      int r = 0;

      if (i == 0)
      {
        r = 0;
      }
      else if (i == 1)
      {
        r = 1;
      }
      else
      {
        r = 2;
      }

      imuMsg->orientation_covariance[i * 3 + 0] = sens_data.imu.pose_covariance.r[r * 3 + 0] * DEG2RAD * DEG2RAD;
      imuMsg->orientation_covariance[i * 3 + 1] = sens_data.imu.pose_covariance.r[r * 3 + 1] * DEG2RAD * DEG2RAD;
      imuMsg->orientation_covariance[i * 3 + 2] = sens_data.imu.pose_covariance.r[r * 3 + 2] * DEG2RAD * DEG2RAD;

      imuMsg->linear_acceleration_covariance[i * 3 + 0] = sens_data.imu.linear_acceleration_covariance.r[r * 3 + 0];
      imuMsg->linear_acceleration_covariance[i * 3 + 1] = sens_data.imu.linear_acceleration_covariance.r[r * 3 + 1];
      imuMsg->linear_acceleration_covariance[i * 3 + 2] = sens_data.imu.linear_acceleration_covariance.r[r * 3 + 2];

      imuMsg->angular_velocity_covariance[i * 3 + 0] =
          sens_data.imu.angular_velocity_covariance.r[r * 3 + 0] * DEG2RAD * DEG2RAD;
      imuMsg->angular_velocity_covariance[i * 3 + 1] =
          sens_data.imu.angular_velocity_covariance.r[r * 3 + 1] * DEG2RAD * DEG2RAD;
      imuMsg->angular_velocity_covariance[i * 3 + 2] =
          sens_data.imu.angular_velocity_covariance.r[r * 3 + 2] * DEG2RAD * DEG2RAD;
    }

    sensors_data_published = true;
    mPubImu.publish(imuMsg);
  }
  else
  {
    NODELET_DEBUG("No new IMU DATA");
  }

  if (imu_RawSubNumber > 0 && new_imu_data)
  {
    lastTs_imu = ts_imu;

    sensor_msgs::ImuPtr imuRawMsg = boost::make_shared<sensor_msgs::Imu>();

    imuRawMsg->header.stamp = ts_imu;
    imuRawMsg->header.frame_id = mImuFrameId;
    imuRawMsg->angular_velocity.x = sens_data.imu.angular_velocity[0] * DEG2RAD;
    imuRawMsg->angular_velocity.y = sens_data.imu.angular_velocity[1] * DEG2RAD;
    imuRawMsg->angular_velocity.z = sens_data.imu.angular_velocity[2] * DEG2RAD;
    imuRawMsg->linear_acceleration.x = sens_data.imu.linear_acceleration[0];
    imuRawMsg->linear_acceleration.y = sens_data.imu.linear_acceleration[1];
    imuRawMsg->linear_acceleration.z = sens_data.imu.linear_acceleration[2];

    for (int i = 0; i < 3; ++i)
    {
      int r = 0;

      if (i == 0)
      {
        r = 0;
      }
      else if (i == 1)
      {
        r = 1;
      }
      else
      {
        r = 2;
      }

      imuRawMsg->linear_acceleration_covariance[i * 3 + 0] = sens_data.imu.linear_acceleration_covariance.r[r * 3 + 0];
      imuRawMsg->linear_acceleration_covariance[i * 3 + 1] = sens_data.imu.linear_acceleration_covariance.r[r * 3 + 1];
      imuRawMsg->linear_acceleration_covariance[i * 3 + 2] = sens_data.imu.linear_acceleration_covariance.r[r * 3 + 2];
      imuRawMsg->angular_velocity_covariance[i * 3 + 0] =
          sens_data.imu.angular_velocity_covariance.r[r * 3 + 0] * DEG2RAD * DEG2RAD;
      imuRawMsg->angular_velocity_covariance[i * 3 + 1] =
          sens_data.imu.angular_velocity_covariance.r[r * 3 + 1] * DEG2RAD * DEG2RAD;
      imuRawMsg->angular_velocity_covariance[i * 3 + 2] =
          sens_data.imu.angular_velocity_covariance.r[r * 3 + 2] * DEG2RAD * DEG2RAD;
    }

    // Orientation data is not available in "data_raw" -> See ROS REP145
    // http://www.ros.org/reps/rep-0145.html#topics
    imuRawMsg->orientation_covariance[0] = -1;
    sensors_data_published = true;
    mPubImuRaw.publish(imuRawMsg);
  }

  // ----> Update Diagnostic
  if (sensors_data_published)
  {
    // Publish freq calculation
    static std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();

    double elapsed_usec = std::chrono::duration_cast<std::chrono::microseconds>(now - last_time).count();
    last_time = now;

    mSensPeriodMean_usec->addValue(elapsed_usec);
  }
  // <---- Update Diagnostic
}

void ZEDWrapperNodelet::publishSimClock(const ros::Time& stamp)
{
  {
    boost::posix_time::ptime posix_time = stamp.toBoost();
    const std::string iso_time_str = boost::posix_time::to_iso_extended_string(posix_time);
    // NODELET_INFO_STREAM_THROTTLE(1.0, "time " << iso_time_str);
    NODELET_INFO_STREAM_ONCE("time " << iso_time_str);
  }

  NODELET_WARN_STREAM_ONCE("using sim clock " << stamp);
  rosgraph_msgs::Clock clock;
  clock.clock = stamp;
  mPubSimClock.publish(clock);
}

ros::Time ZEDWrapperNodelet::getTimestamp()
{
  ros::Time stamp;
  if (mSvoMode && !mUseSimTime)
  {
    // TODO(lucasw) does it matter which one?
    stamp = ros::Time::now();
    // mFrameTimestamp = sl_tools::slTime2Ros(mZed.getTimestamp(sl::TIME_REFERENCE::CURRENT));
    NODELET_WARN_STREAM_ONCE("Using current time instead of image time " << stamp);
  }
  else
  {
    // TODO(lucasw) if no images have arrived yet, this is zero, or something else?
    // In the svo file it appears to be something else, slightly in advance of the first
    // frame.
    stamp = sl_tools::slTime2Ros(mZed.getTimestamp(sl::TIME_REFERENCE::IMAGE));
    NODELET_WARN_STREAM_ONCE("Using zed image time " << stamp);
  }
  return stamp;
}

void ZEDWrapperNodelet::sim_clock_update(const ros::WallTimerEvent& e)
{
  // TODO(lucasw) mutex
  publishSimClock(sim_clock_base_time);
  // TODO(lucasw) have ability to roll forward faster than real time
  sim_clock_base_time += ros::Duration(0.001);
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

// TODO(lucasw) pass in all the params in the dynamic reconfigure struct
bool ZEDWrapperNodelet::updateCameraWithDynParams()
{
  // NODELET_DEBUG_STREAM( "[" << mFrameCount << "] device_poll_thread_func MUTEX LOCK");
  std::lock_guard<std::mutex> lock(mDynParMutex);
  bool updated = false;

  const int brightness = mZed.getCameraSettings(sl::VIDEO_SETTINGS::BRIGHTNESS);
  if (brightness != mCamBrightness)
  {
    mZed.setCameraSettings(sl::VIDEO_SETTINGS::BRIGHTNESS, mCamBrightness);
    NODELET_DEBUG_STREAM("mCamBrightness changed: " << mCamBrightness << " <- " << brightness);
    updated = true;
  }

  const int contrast = mZed.getCameraSettings(sl::VIDEO_SETTINGS::CONTRAST);
  if (contrast != mCamContrast)
  {
    mZed.setCameraSettings(sl::VIDEO_SETTINGS::CONTRAST, mCamContrast);
    NODELET_DEBUG_STREAM("mCamContrast changed: " << mCamContrast << " <- " << contrast);
    updated = true;
  }

  const int hue = mZed.getCameraSettings(sl::VIDEO_SETTINGS::HUE);
  if (hue != mCamHue)
  {
    mZed.setCameraSettings(sl::VIDEO_SETTINGS::HUE, mCamHue);
    NODELET_DEBUG_STREAM("mCamHue changed: " << mCamHue << " <- " << hue);
    updated = true;
  }

  const int saturation = mZed.getCameraSettings(sl::VIDEO_SETTINGS::SATURATION);
  if (saturation != mCamSaturation)
  {
    mZed.setCameraSettings(sl::VIDEO_SETTINGS::SATURATION, mCamSaturation);
    NODELET_DEBUG_STREAM("mCamSaturation changed: " << mCamSaturation << " <- " << saturation);
    updated = true;
  }

  const int sharpness = mZed.getCameraSettings(sl::VIDEO_SETTINGS::SHARPNESS);
  if (sharpness != mCamSharpness)
  {
    mZed.setCameraSettings(sl::VIDEO_SETTINGS::SHARPNESS, mCamSharpness);
    NODELET_DEBUG_STREAM("mCamSharpness changed: " << mCamSharpness << " <- " << sharpness);
    updated = true;
  }

#if (ZED_SDK_MAJOR_VERSION == 3 && ZED_SDK_MINOR_VERSION >= 1)
  const int gamma = mZed.getCameraSettings(sl::VIDEO_SETTINGS::GAMMA);
  if (gamma != mCamGamma)
  {
    mZed.setCameraSettings(sl::VIDEO_SETTINGS::GAMMA, mCamGamma);
    NODELET_DEBUG_STREAM("mCamGamma changed: " << mCamGamma << " <- " << gamma);
    updated = true;
  }
#endif

  if (mCamAutoExposure)
  {
    if (mTriggerAutoExposure)
    {
      mZed.setCameraSettings(sl::VIDEO_SETTINGS::AEC_AGC, 1);
      mTriggerAutoExposure = false;
    }
  }
  else
  {
    const int exposure = mZed.getCameraSettings(sl::VIDEO_SETTINGS::EXPOSURE);
    if (exposure != mCamExposure)
    {
      mZed.setCameraSettings(sl::VIDEO_SETTINGS::EXPOSURE, mCamExposure);
      NODELET_DEBUG_STREAM("mCamExposure changed: " << mCamExposure << " <- " << exposure);
      updated = true;
    }

    const int gain = mZed.getCameraSettings(sl::VIDEO_SETTINGS::GAIN);
    if (gain != mCamGain)
    {
      mZed.setCameraSettings(sl::VIDEO_SETTINGS::GAIN, mCamGain);
      NODELET_DEBUG_STREAM("mCamGain changed: " << mCamGain << " <- " << gain);
      updated = true;
    }
  }

  if (mCamAutoWB)
  {
    if (mTriggerAutoWB)
    {
      mZed.setCameraSettings(sl::VIDEO_SETTINGS::WHITEBALANCE_AUTO, 1);
      mTriggerAutoWB = false;
    }
  }
  else
  {
    const int wb = mZed.getCameraSettings(sl::VIDEO_SETTINGS::WHITEBALANCE_TEMPERATURE);
    if (wb != mCamWB)
    {
      mZed.setCameraSettings(sl::VIDEO_SETTINGS::WHITEBALANCE_TEMPERATURE, mCamWB);
      NODELET_DEBUG_STREAM("mCamWB changed: " << mCamWB << " <- " << wb);
      updated = true;
    }
  }

  return updated;
}

sl::ERROR_CODE ZEDWrapperNodelet::enableRecordingAllCompressions(sl::RecordingParameters& recParams)
{
  auto err = mZed.enableRecording(recParams);

  // TODO(lucasw) roll this into a loop
  if (err == sl::ERROR_CODE::SVO_UNSUPPORTED_COMPRESSION)
  {
    recParams.compression_mode = mSvoComprMode == sl::SVO_COMPRESSION_MODE::H265 ? sl::SVO_COMPRESSION_MODE::H264 :
                                                                                   sl::SVO_COMPRESSION_MODE::H265;

    NODELET_WARN_STREAM("The chosen " << sl::toString(mSvoComprMode).c_str() << "mode is not available. Trying "
                                      << sl::toString(recParams.compression_mode).c_str());

    err = mZed.enableRecording(recParams);

    if (err == sl::ERROR_CODE::SVO_UNSUPPORTED_COMPRESSION)
    {
      NODELET_WARN_STREAM(sl::toString(recParams.compression_mode).c_str()
                          << "not available. Trying " << sl::toString(sl::SVO_COMPRESSION_MODE::H264).c_str());
      recParams.compression_mode = sl::SVO_COMPRESSION_MODE::H264;
      err = mZed.enableRecording(recParams);

      if (err == sl::ERROR_CODE::SVO_UNSUPPORTED_COMPRESSION)
      {
        recParams.compression_mode = sl::SVO_COMPRESSION_MODE::LOSSLESS;
        err = mZed.enableRecording(recParams);
      }
    }
  }
  return err;
}

void ZEDWrapperNodelet::startRecording(const std::string& filename, std::string& info)
{
  std::lock_guard<std::mutex> lock(mRecMutex);

  sl::RecordingParameters recParams;
  recParams.compression_mode = mSvoComprMode;
  recParams.video_filename = filename.c_str();
  mRecordSvoFilepath = filename;
  const auto err = enableRecordingAllCompressions(recParams);

  if (err != sl::ERROR_CODE::SUCCESS)
  {
    info += sl::toString(err).c_str();
    mRecording = false;
    return;
  }

  mSvoComprMode = recParams.compression_mode;
  NODELET_INFO_STREAM("SVO started recording '" << mRecordSvoFilepath << "' "
                      << sl::toString(recParams.compression_mode));
  mRecording = true;
}

void ZEDWrapperNodelet::stopRecording()
{
  std::lock_guard<std::mutex> lock(mRecMutex);
  if (!mRecording)
  {
    return;
  }
  mZed.disableRecording();
  NODELET_INFO_STREAM("SVO stopped recording '" << mRecordSvoFilepath << "'");
  mRecording = false;
}

void ZEDWrapperNodelet::updateRecordingStatus()
{
  std::lock_guard<std::mutex> lock(mRecMutex);

  if (!mRecording)
  {
    return;
  }

  mRecStatus = mZed.getRecordingStatus();

  if (!mRecStatus.status)
  {
    NODELET_ERROR_THROTTLE(1.0, "Error saving frame to SVO");
  }

  mDiagUpdater.force_update();
}

void ZEDWrapperNodelet::stop()
{
  std::lock_guard<std::mutex> lock(mCloseZedMutex);
  // TODO(lucasw) nothing looks at mStopNode
  mStopNode = true;
  stopRecording();
  mZed.close();
}

// TODO(lucasw) turn this into a ros update
void ZEDWrapperNodelet::device_poll_thread_func()
{
  ros::Rate loop_rate(mCamFrameRate);

  mRecording = false;

  mElabPeriodMean_sec.reset(new sl_tools::CSmartMean(mCamFrameRate));
  mGrabPeriodMean_usec.reset(new sl_tools::CSmartMean(mCamFrameRate));
  mVideoDepthPeriodMean_sec.reset(new sl_tools::CSmartMean(mCamFrameRate));

  // Timestamp initialization
  mFrameTimestamp = getTimestamp();

  // TODO(lucasw) mutex
  sim_clock_base_time = mFrameTimestamp;

  mPrevFrameTimestamp = mFrameTimestamp;

  mPosTrackingActivated = false;
  mRecording = false;

  // Get the parameters of the ZED images
#if ZED_SDK_MAJOR_VERSION == 3 && ZED_SDK_MINOR_VERSION < 1
  mCamWidth = mZed.getCameraInformation().camera_resolution.width;
  mCamHeight = mZed.getCameraInformation().camera_resolution.height;
#else
  mCamWidth = mZed.getCameraInformation().camera_configuration.resolution.width;
  mCamHeight = mZed.getCameraInformation().camera_configuration.resolution.height;
#endif
  NODELET_DEBUG_STREAM("Camera Frame size : " << mCamWidth << "x" << mCamHeight);
  int v_w = static_cast<int>(mCamWidth * mCamImageResizeFactor);
  int v_h = static_cast<int>(mCamHeight * mCamImageResizeFactor);
  mMatResolVideo = sl::Resolution(v_w, v_h);
  NODELET_DEBUG_STREAM("Image Mat size : " << mMatResolVideo.width << "x" << mMatResolVideo.height);
  int d_w = static_cast<int>(mCamWidth * mCamDepthResizeFactor);
  int d_h = static_cast<int>(mCamHeight * mCamDepthResizeFactor);
  mMatResolDepth = sl::Resolution(d_w, d_h);
  NODELET_DEBUG_STREAM("Depth Mat size : " << mMatResolDepth.width << "x" << mMatResolDepth.height);

  // Create and fill the camera information messages
  fillCamInfo(mZed, mLeftCamInfoMsg, mRightCamInfoMsg, mLeftCamOptFrameId, mRightCamOptFrameId);
  fillCamInfo(mZed, mLeftCamInfoRawMsg, mRightCamInfoRawMsg, mLeftCamOptFrameId, mRightCamOptFrameId, true);
  fillCamDepthInfo(mZed, mDepthCamInfoMsg, mLeftCamOptFrameId);

  // the reference camera is the Left one (next to the ZED logo)

  sl::RuntimeParameters runParams;
  runParams.sensing_mode = static_cast<sl::SENSING_MODE>(mCamSensingMode);

  // TODO(lucasw) is there a call to mZed in here that rolls it forward a frame in mSvoMode, or does it
  // have a timer and keeps time independently, can only do real time?
  // Main loop
  while (mNhNs.ok())
  {
    if (stop_recording_) {
      stopRecording();
      stop_recording_ = false;
    }

    // Check for subscribers
    uint32_t leftSubnumber = mPubLeft.getNumSubscribers();
    uint32_t leftRawSubnumber = mPubRawLeft.getNumSubscribers();
    uint32_t rightSubnumber = mPubRight.getNumSubscribers();
    uint32_t rightRawSubnumber = mPubRawRight.getNumSubscribers();
    uint32_t depthSubnumber = mPubDepth.getNumSubscribers();
    uint32_t disparitySubnumber = mPubDisparity.getNumSubscribers();
    uint32_t poseSubnumber = mPubPose.getNumSubscribers();
    uint32_t poseCovSubnumber = mPubPoseCov.getNumSubscribers();
    uint32_t odomSubnumber = mPubOdom.getNumSubscribers();
    uint32_t confMapSubnumber = mPubConfMap.getNumSubscribers();
    uint32_t pathSubNumber = mPubMapPath.getNumSubscribers() + mPubOdomPath.getNumSubscribers();

    mGrabActive =
        mRecording || mStreaming || mPosTrackingEnabled || mPosTrackingActivated ||
        ((leftSubnumber + leftRawSubnumber + rightSubnumber + rightRawSubnumber +
          poseCovSubnumber + odomSubnumber + confMapSubnumber /*+ imuSubnumber + imuRawsubnumber*/ + pathSubNumber) > 0);

    // Run the loop only if there is some subscribers or SVO is active
    if (mGrabActive)
    {
      std::lock_guard<std::mutex> lock(mPosTrkMutex);

      // Note: ones tracking is started it is never stopped anymore to not lose tracking information
      bool computeTracking = (mPosTrackingEnabled || mPosTrackingActivated || (mComputeDepth & mDepthStabilization) ||
                              poseSubnumber > 0 || poseCovSubnumber > 0 || odomSubnumber > 0 || pathSubNumber > 0);

      // Start the tracking?
      if ((computeTracking) && !mPosTrackingActivated && (mDepthMode != sl::DEPTH_MODE::NONE))
      {
        start_pos_tracking();
      }

      // Detect if one of the subscriber need to have the depth information
      mComputeDepth = mDepthMode != sl::DEPTH_MODE::NONE &&
                      (computeTracking || ((depthSubnumber + disparitySubnumber + poseSubnumber +
                       poseCovSubnumber + odomSubnumber + confMapSubnumber) > 0));

      if (mComputeDepth)
      {
        runParams.confidence_threshold = mCamDepthConfidence;
#if ZED_SDK_MAJOR_VERSION == 3 && ZED_SDK_MINOR_VERSION < 2
        runParams.textureness_confidence_threshold = mCamDepthTextureConf;
#else
        runParams.texture_confidence_threshold = mCamDepthTextureConf;
#endif
        runParams.enable_depth = true;  // Ask to compute the depth
      }
      else
      {
        runParams.enable_depth = false;  // Ask to not compute the depth
      }

      if (mSvoMode) {
        // give even more time for subsribers like rosbag to keep up
        ros::WallDuration(0.1).sleep();
      }

      // TODO(lucasw) have dynamic reconfigure skip parameter to loop on grap to skip ahead,
      // or use sdk features to jump in position
      {
        std::lock_guard<std::mutex> lock(mCamDataMutex);
        mGrabStatus = mZed.grab(runParams);
      }

      std::chrono::steady_clock::time_point start_elab = std::chrono::steady_clock::now();

      // cout << toString(grab_status) << endl;
      if (mGrabStatus != sl::ERROR_CODE::SUCCESS)
      {
        // Detect if a error occurred (for example:
        // the zed have been disconnected) and
        // re-initialize the ZED

        NODELET_INFO_STREAM_ONCE(toString(mGrabStatus));

        // TODO(lucasw) if the status is END OF SVO FILE REACHED then loop it optionally
        if (mGrabStatus == sl::ERROR_CODE::END_OF_SVOFILE_REACHED) {
          stop();
          return;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        if ((ros::Time::now() - mPrevFrameTimestamp).toSec() > 5 && !mSvoMode)
        {
          stop();

          mConnStatus = sl::ERROR_CODE::CAMERA_NOT_DETECTED;

          while (mConnStatus != sl::ERROR_CODE::SUCCESS)
          {
            if (!mNhNs.ok())
            {
              stop();
              return;
            }

            int id = sl_tools::checkCameraReady(mZedSerialNumber);

            if (id >= 0)
            {
              mZedParams.input.setFromCameraID(id);
              mConnStatus = mZed.open(mZedParams);  // Try to initialize the ZED
              NODELET_INFO_STREAM(toString(mConnStatus));
            }
            else
            {
              NODELET_INFO_STREAM("Waiting for the ZED (S/N " << mZedSerialNumber << ") to be re-connected");
            }

            mDiagUpdater.force_update();
            ros::WallDuration(2.0).sleep();
          }

          mPosTrackingActivated = false;

          computeTracking = mPosTrackingEnabled || mDepthStabilization || poseSubnumber > 0 || poseCovSubnumber > 0 || odomSubnumber > 0;

          if (computeTracking)
          {  // Start the tracking
            start_pos_tracking();
          }
        }

        mDiagUpdater.update();

        continue;
      }

      mFrameCount++;

      // SVO recording
      updateRecordingStatus();

      // Timestamp
      mPrevFrameTimestamp = mFrameTimestamp;

      // Publish freq calculation
      static std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();
      std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();

      double elapsed_usec = std::chrono::duration_cast<std::chrono::microseconds>(now - last_time).count();
      last_time = now;

      mGrabPeriodMean_usec->addValue(elapsed_usec);

      // NODELET_INFO_STREAM("Grab time: " << elapsed_usec / 1000 << " msec");
      mFrameTimestamp = getTimestamp();

      const ros::Time stamp = mFrameTimestamp;  // Timestamp
      sim_clock_base_time = stamp;

      // ----> Camera Settings
      if (!mSvoMode && mFrameCount % 5 == 0)
      {
        if (updateCameraWithDynParams())
        {
          mUpdateDynParams = true;
        }
      }

      if (mUpdateDynParams)
      {
        NODELET_DEBUG("Update Dynamic Parameters in server");
        updateDynamicReconfigureServer();
      }
      // <---- Camera Settings

      // Publish the odometry if someone has subscribed to
      if (computeTracking)
      {
        if (!mSensor2BaseTransfValid)
        {
          getSens2BaseTransform();
        }

        if (!mSensor2CameraTransfValid)
        {
          getSens2CameraTransform();
        }

        if (!mCamera2BaseTransfValid)
        {
          getCamera2BaseTransform();
        }

        if (!mInitOdomWithPose)
        {
          sl::Pose deltaOdom;
          mPosTrackingStatus = mZed.getPosition(deltaOdom, sl::REFERENCE_FRAME::CAMERA);

          sl::Translation translation = deltaOdom.getTranslation();
          sl::Orientation quat = deltaOdom.getOrientation();

#if 0
                    NODELET_DEBUG("delta ODOM [%s] - %.2f,%.2f,%.2f %.2f,%.2f,%.2f,%.2f",
                                  sl::toString(mTrackingStatus).c_str(),
                                  translation(0), translation(1), translation(2),
                                  quat(0), quat(1), quat(2), quat(3));

                    NODELET_DEBUG_STREAM("ODOM -> Tracking Status: " << sl::toString(mTrackingStatus));
#endif

          if (mPosTrackingStatus == sl::POSITIONAL_TRACKING_STATE::OK ||
              mPosTrackingStatus == sl::POSITIONAL_TRACKING_STATE::SEARCHING ||
              mPosTrackingStatus == sl::POSITIONAL_TRACKING_STATE::FPS_TOO_LOW)
          {
            // Transform ZED delta odom pose in TF2 Transformation
            geometry_msgs::Transform deltaTransf;
            deltaTransf.translation.x = translation(0);
            deltaTransf.translation.y = translation(1);
            deltaTransf.translation.z = translation(2);
            deltaTransf.rotation.x = quat(0);
            deltaTransf.rotation.y = quat(1);
            deltaTransf.rotation.z = quat(2);
            deltaTransf.rotation.w = quat(3);
            tf2::Transform deltaOdomTf;
            tf2::fromMsg(deltaTransf, deltaOdomTf);
            // delta odom from sensor to base frame
            tf2::Transform deltaOdomTf_base = mSensor2BaseTransf.inverse() * deltaOdomTf * mSensor2BaseTransf;

            // Propagate Odom transform in time
            mOdom2BaseTransf = mOdom2BaseTransf * deltaOdomTf_base;

            if (mTwoDMode)
            {
              tf2::Vector3 tr_2d = mOdom2BaseTransf.getOrigin();
              tr_2d.setZ(mFixedZValue);
              mOdom2BaseTransf.setOrigin(tr_2d);

              double roll, pitch, yaw;
              tf2::Matrix3x3(mOdom2BaseTransf.getRotation()).getRPY(roll, pitch, yaw);

              tf2::Quaternion quat_2d;
              quat_2d.setRPY(0.0, 0.0, yaw);

              mOdom2BaseTransf.setRotation(quat_2d);
            }

#if 0  //#ifndef NDEBUG // Enable for TF checking
                        double roll, pitch, yaw;
                        tf2::Matrix3x3(mOdom2BaseTransf.getRotation()).getRPY(roll, pitch, yaw);

                        NODELET_DEBUG("+++ Odometry [%s -> %s] - {%.3f,%.3f,%.3f} {%.3f,%.3f,%.3f}",
                                      mOdometryFrameId.c_str(), mBaseFrameId.c_str(),
                                      mOdom2BaseTransf.getOrigin().x(), mOdom2BaseTransf.getOrigin().y(), mOdom2BaseTransf.getOrigin().z(),
                                      roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);
#endif

            // Publish odometry message
            if (odomSubnumber > 0)
            {
              publishOdom(mOdom2BaseTransf, deltaOdom, stamp);
            }

            mPosTrackingReady = true;
          }
        }
        else if (mFloorAlignment)
        {
          NODELET_WARN_THROTTLE(5.0, "Odometry will be published as soon as the floor as been detected for the first "
                                     "time");
        }
      }

      // Publish the zed camera pose if someone has subscribed to
      if (computeTracking)
      {
        static sl::POSITIONAL_TRACKING_STATE oldStatus;
        mPosTrackingStatus = mZed.getPosition(mLastZedPose, sl::REFERENCE_FRAME::WORLD);

        sl::Translation translation = mLastZedPose.getTranslation();
        sl::Orientation quat = mLastZedPose.getOrientation();

#if 0  //#ifndef NDEBUG // Enable for TF checking
                double roll, pitch, yaw;
                tf2::Matrix3x3(tf2::Quaternion(quat.ox, quat.oy, quat.oz, quat.ow)).getRPY(roll, pitch, yaw);

                NODELET_DEBUG("Sensor POSE [%s -> %s] - {%.2f,%.2f,%.2f} {%.2f,%.2f,%.2f}",
                              mLeftCamFrameId.c_str(), mMapFrameId.c_str(),
                              translation.x, translation.y, translation.z,
                              roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);

                NODELET_DEBUG_STREAM("MAP -> Tracking Status: " << sl::toString(mTrackingStatus));

#endif

        if (mPosTrackingStatus == sl::POSITIONAL_TRACKING_STATE::OK ||
            mPosTrackingStatus ==
                sl::POSITIONAL_TRACKING_STATE::SEARCHING /*|| status == sl::POSITIONAL_TRACKING_STATE::FPS_TOO_LOW*/)
        {
          // Transform ZED pose in TF2 Transformation
          geometry_msgs::Transform map2sensTransf;

          map2sensTransf.translation.x = translation(0);
          map2sensTransf.translation.y = translation(1);
          map2sensTransf.translation.z = translation(2);
          map2sensTransf.rotation.x = quat(0);
          map2sensTransf.rotation.y = quat(1);
          map2sensTransf.rotation.z = quat(2);
          map2sensTransf.rotation.w = quat(3);
          tf2::Transform map_to_sens_transf;
          tf2::fromMsg(map2sensTransf, map_to_sens_transf);

          mMap2BaseTransf = map_to_sens_transf * mSensor2BaseTransf;  // Base position in map frame

          if (mTwoDMode)
          {
            tf2::Vector3 tr_2d = mMap2BaseTransf.getOrigin();
            tr_2d.setZ(mFixedZValue);
            mMap2BaseTransf.setOrigin(tr_2d);

            double roll, pitch, yaw;
            tf2::Matrix3x3(mMap2BaseTransf.getRotation()).getRPY(roll, pitch, yaw);

            tf2::Quaternion quat_2d;
            quat_2d.setRPY(0.0, 0.0, yaw);

            mMap2BaseTransf.setRotation(quat_2d);
          }

#if 0  //#ifndef NDEBUG // Enable for TF checking
                    double roll, pitch, yaw;
                    tf2::Matrix3x3(mMap2BaseTransf.getRotation()).getRPY(roll, pitch, yaw);

                    NODELET_DEBUG("*** Base POSE [%s -> %s] - {%.3f,%.3f,%.3f} {%.3f,%.3f,%.3f}",
                                  mMapFrameId.c_str(), mBaseFrameId.c_str(),
                                  mMap2BaseTransf.getOrigin().x(), mMap2BaseTransf.getOrigin().y(), mMap2BaseTransf.getOrigin().z(),
                                  roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);
#endif

          bool initOdom = false;

          if (!(mFloorAlignment))
          {
            initOdom = mInitOdomWithPose;
          }
          else
          {
            initOdom = (mPosTrackingStatus == sl::POSITIONAL_TRACKING_STATE::OK) & mInitOdomWithPose;
          }

          if (initOdom || mResetOdom)
          {
            NODELET_INFO("Odometry aligned to last tracking pose");

            // Propagate Odom transform in time
            mOdom2BaseTransf = mMap2BaseTransf;
            mMap2BaseTransf.setIdentity();

            if (odomSubnumber > 0)
            {
              // Publish odometry message
              publishOdom(mOdom2BaseTransf, mLastZedPose, stamp);
            }

            mInitOdomWithPose = false;
            mResetOdom = false;
          }
          else
          {
            // Transformation from map to odometry frame
            // mMap2OdomTransf = mOdom2BaseTransf.inverse() * mMap2BaseTransf;
            mMap2OdomTransf = mMap2BaseTransf * mOdom2BaseTransf.inverse();

#if 0  //#ifndef NDEBUG // Enable for TF checking
                        double roll, pitch, yaw;
                        tf2::Matrix3x3(mMap2OdomTransf.getRotation()).getRPY(roll, pitch, yaw);

                        NODELET_DEBUG("+++ Diff [%s -> %s] - {%.3f,%.3f,%.3f} {%.3f,%.3f,%.3f}",
                                      mMapFrameId.c_str(), mOdometryFrameId.c_str(),
                                      mMap2OdomTransf.getOrigin().x(), mMap2OdomTransf.getOrigin().y(), mMap2OdomTransf.getOrigin().z(),
                                      roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);
#endif
          }

          // Publish Pose message
          if ((poseSubnumber + poseCovSubnumber) > 0)
          {
            publishPose(stamp);
          }

          mPosTrackingReady = true;
        }

        oldStatus = mPosTrackingStatus;
      }

      if (mZedRealCamModel == sl::MODEL::ZED)
      {
        // Publish pose tf only if enabled
        if (mPublishTf)
        {
          // Note, the frame is published, but its values will only change if
          // someone has subscribed to odom
          publishOdomFrame(mOdom2BaseTransf, stamp);  // publish the base Frame in odometry frame

          if (mPublishMapTf)
          {
            // Note, the frame is published, but its values will only change if
            // someone has subscribed to map
            publishPoseFrame(mMap2OdomTransf, stamp);  // publish the odometry Frame in map frame
          }
        }
      }

#if 0  //#ifndef NDEBUG // Enable for TF checking
       // Double check: map_to_pose must be equal to mMap2BaseTransf

            tf2::Transform map_to_base;

            try {
                // Save the transformation from base to frame
                geometry_msgs::TransformStamped b2m =
                        mTfBuffer->lookupTransform(mMapFrameId, mBaseFrameId, ros::Time(0));
                // Get the TF2 transformation
                tf2::fromMsg(b2m.transform, map_to_base);
            } catch (tf2::TransformException& ex) {
                NODELET_DEBUG_THROTTLE(1.0, "Transform error: %s", ex.what());
                NODELET_WARN_THROTTLE(1.0, "The tf from '%s' to '%s' is not available.",
                                      mMapFrameId.c_str(), mBaseFrameId.c_str());
            }

            double roll, pitch, yaw;
            tf2::Matrix3x3(map_to_base.getRotation()).getRPY(roll, pitch, yaw);

            NODELET_DEBUG("*** Check [%s -> %s] - {%.3f,%.3f,%.3f} {%.3f,%.3f,%.3f}",
                          mMapFrameId.c_str(), mBaseFrameId.c_str(),
                          map_to_base.getOrigin().x(), map_to_base.getOrigin().y(), map_to_base.getOrigin().z(),
                          roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);

            NODELET_DEBUG("*******************************");
#endif
      std::chrono::steady_clock::time_point end_elab = std::chrono::steady_clock::now();

      double elab_usec = std::chrono::duration_cast<std::chrono::microseconds>(end_elab - start_elab).count();

      double mean_elab_sec = mElabPeriodMean_sec->addValue(elab_usec / 1000000.);

      static int count_warn = 0;

      if (!loop_rate.sleep())
      {
        if (mean_elab_sec > (1. / mCamFrameRate))
        {
          if (++count_warn > 10)
          {
            NODELET_DEBUG_THROTTLE(1.0, "Working thread is not synchronized with the Camera frame rate");
            NODELET_DEBUG_STREAM_THROTTLE(1.0, "Expected cycle time: " << loop_rate.expectedCycleTime()
                                                                       << " - Real cycle time: " << mean_elab_sec);
            NODELET_WARN_STREAM_THROTTLE(10.0, "Elaboration takes longer (" << mean_elab_sec
                                                                            << " sec) than requested "
                                                                               "by the FPS rate ("
                                                                            << loop_rate.expectedCycleTime()
                                                                            << " sec). Please consider to "
                                                                               "lower the 'frame_rate' setting or to "
                                                                               "reduce the power requirements reducing "
                                                                               "the resolutions.");
          }

          loop_rate.reset();
        }
        else
        {
          count_warn = 0;
        }
      }
    }
    else
    {
      NODELET_DEBUG_THROTTLE(5.0, "No topics subscribed by users");

      if (mZedRealCamModel == sl::MODEL::ZED || !mPublishImuTf)
      {
        // Publish odometry tf only if enabled
        if (mPublishTf)
        {
          // TODO(lucasw) or just ust mFrameTimestamp?
          const ros::Time t = getTimestamp();

          publishOdomFrame(mOdom2BaseTransf, t);  // publish the base Frame in odometry frame

          if (mPublishMapTf)
          {
            publishPoseFrame(mMap2OdomTransf, t);  // publish the odometry Frame in map frame
          }

          if (mPublishImuTf && !mStaticImuFramePublished)
          {
            publishStaticImuFrame(t);
          }
        }
      }

      // No subscribers, we just wait
      ros::WallDuration(0.01).sleep();
      loop_rate.reset();
    }

    mDiagUpdater.update();
  }  // while loop

  stop();
}

void ZEDWrapperNodelet::callback_updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  if (mConnStatus != sl::ERROR_CODE::SUCCESS)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, sl::toString(mConnStatus).c_str());
    return;
  }

  if (mGrabActive)
  {
    if (mGrabStatus == sl::ERROR_CODE::SUCCESS /*|| mGrabStatus == sl::ERROR_CODE::NOT_A_NEW_FRAME*/)
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Camera grabbing");

      double freq = 1000000. / mGrabPeriodMean_usec->getMean();
      double freq_perc = 100. * freq / mCamFrameRate;
      stat.addf("Capture", "Mean Frequency: %.1f Hz (%.1f%%)", freq, freq_perc);

      stat.addf("General Processing", "Mean Time: %.3f sec (Max. %.3f sec)", mElabPeriodMean_sec->getMean(),
                1. / mCamFrameRate);

      if (mPublishingData)
      {
        freq = 1. / mVideoDepthPeriodMean_sec->getMean();
        freq_perc = 100. * freq / mVideoDepthFreq;
        stat.addf("Video/Depth Publish", "Mean Frequency: %.1f Hz (%.1f%%)", freq, freq_perc);
      }

      if (mSvoMode)
      {
        int frame = mZed.getSVOPosition();
        int totFrames = mZed.getSVONumberOfFrames();
        double svo_perc = 100. * (static_cast<double>(frame) / totFrames);

        stat.addf("Playing SVO", "Frame: %d/%d (%.1f%%)", frame, totFrames, svo_perc);
      }

      if (mComputeDepth)
      {
        stat.add("Depth status", "ACTIVE");

        if (mFloorAlignment)
        {
          if (mInitOdomWithPose)
          {
            stat.add("Floor Detection", "NOT INITIALIZED");
          }
          else
          {
            stat.add("Floor Detection", "INITIALIZED");
          }
        }

        if (mPosTrackingActivated)
        {
          stat.addf("Tracking status", "%s", sl::toString(mPosTrackingStatus).c_str());
        }
        else
        {
          stat.add("Pos. Tracking status", "INACTIVE");
        }
      }
      else
      {
        stat.add("Depth status", "INACTIVE");
      }
    }
    else
    {
      stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Camera error: %s", sl::toString(mGrabStatus).c_str());
    }
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Waiting for data subscriber");
    stat.add("Capture", "INACTIVE");
  }

  if (mSensPublishing)
  {
    double freq = 1000000. / mSensPeriodMean_usec->getMean();
    double freq_perc = 100. * freq / mSensPubRate;
    stat.addf("IMU", "Mean Frequency: %.1f Hz (%.1f%%)", freq, freq_perc);
  }
  else
  {
    stat.add("IMU", "Topics not subscribed");
  }

  if (mZedRealCamModel == sl::MODEL::ZED2 || mZedRealCamModel == sl::MODEL::ZED2i)
  {
    stat.addf("Left CMOS Temp.", "%.1f °C", mTempLeft);
    stat.addf("Right CMOS Temp.", "%.1f °C", mTempRight);

    if (mTempLeft > 70.f || mTempRight > 70.f)
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Camera temperature");
    }
  }
  else
  {
    stat.add("Left CMOS Temp.", "N/A");
    stat.add("Right CMOS Temp.", "N/A");
  }

  if (mRecording)
  {
    if (!mRecStatus.status)
    {
      if (mGrabActive)
      {
        stat.add("SVO Recording", "ERROR");
        stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Error adding frames to SVO file while recording. Check "
                                                              "free disk space");
      }
      else
      {
        stat.add("SVO Recording", "WAITING");
      }
    }
    else
    {
      stat.add("SVO Recording", "ACTIVE");
      stat.addf("SVO compression time", "%g msec", mRecStatus.average_compression_time);
      stat.addf("SVO compression ratio", "%.1f%%", mRecStatus.average_compression_ratio);
    }
  }
  else
  {
    stat.add("SVO Recording", "NOT ACTIVE");
  }
}

bool ZEDWrapperNodelet::on_start_svo_recording(zed_interfaces::start_svo_recording::Request& req,
                                               zed_interfaces::start_svo_recording::Response& res)
{
  if (mRecording)
  {
    res.result = false;
    res.info = "Recording was just active";
    return false;
  }

  // Check filename
  if (req.svo_filename.empty())
  {
    req.svo_filename = "zed.svo";
  }

  res.info = "SVO Recording started";
  res.info += " '" + req.svo_filename + "' ";
  startRecording(req.svo_filename, res.info);
  res.info += " ";
  res.info += sl::toString(mSvoComprMode).c_str();
  res.result = mRecording;

  return true;
}

// TODO(lucasw) this is frequently crashing
bool ZEDWrapperNodelet::on_stop_svo_recording(zed_interfaces::stop_svo_recording::Request& req,
                                              zed_interfaces::stop_svo_recording::Response& res)
{
  if (!mRecording)
  {
    res.done = false;
    res.info = "Recording was not active";
    return false;
  }

  res.info = "SVO Recording stop commanded '" +  mRecordSvoFilepath + "'";
  stop_recording_ = true;
  // TODO(lucasw) if a second on_stop_svo_recording is triggered during the sleep
  // does it re-enter here, foul this up?
  NODELET_INFO_STREAM(res.info);
  ros::Duration(0.25).sleep();
  res.done = !stop_recording_;
  NODELET_INFO_STREAM("stop recording: " << int(res.done));
  return true;
}

bool ZEDWrapperNodelet::on_start_remote_stream(zed_interfaces::start_remote_stream::Request& req,
                                               zed_interfaces::start_remote_stream::Response& res)
{
  if (mStreaming)
  {
    res.result = false;
    res.info = "SVO remote streaming was just active";
    return false;
  }

  sl::StreamingParameters params;
  params.codec = static_cast<sl::STREAMING_CODEC>(req.codec);
  params.port = req.port;
  params.bitrate = req.bitrate;
  params.gop_size = req.gop_size;
  params.adaptative_bitrate = req.adaptative_bitrate;

  if ((params.gop_size < -1) || (params.gop_size > 256))
  {
    mStreaming = false;

    res.result = false;
    res.info = "`gop_size` wrong (";
    res.info += params.gop_size;
    res.info += "). Remote streaming not started";

    NODELET_ERROR_STREAM(res.info);
    return false;
  }

  if (params.port % 2 != 0)
  {
    mStreaming = false;

    res.result = false;
    res.info = "`port` must be an even number. Remote streaming not started";

    NODELET_ERROR_STREAM(res.info);
    return false;
  }

  sl::ERROR_CODE err = mZed.enableStreaming(params);

  if (err != sl::ERROR_CODE::SUCCESS)
  {
    mStreaming = false;

    res.result = false;
    res.info = sl::toString(err).c_str();

    NODELET_ERROR_STREAM("Remote streaming not started (" << res.info << ")");

    return false;
  }

  mStreaming = true;

  NODELET_INFO_STREAM("Remote streaming STARTED");

  res.result = true;
  res.info = "Remote streaming STARTED";
  return true;
}

bool ZEDWrapperNodelet::on_stop_remote_stream(zed_interfaces::stop_remote_stream::Request& req,
                                              zed_interfaces::stop_remote_stream::Response& res)
{
  if (mStreaming)
  {
    mZed.disableStreaming();
  }

  mStreaming = false;
  NODELET_INFO_STREAM("SVO remote streaming STOPPED");

  res.done = true;
  return true;
}

bool ZEDWrapperNodelet::on_set_led_status(zed_interfaces::set_led_status::Request& req,
                                          zed_interfaces::set_led_status::Response& res)
{
  if (mCamFwVersion < 1523)
  {
    NODELET_WARN_STREAM("To set the status of the blue LED the camera must be updated to FW 1523 or newer");
    return false;
  }

  mZed.setCameraSettings(sl::VIDEO_SETTINGS::LED_STATUS, req.led_enabled ? 1 : 0);

  return true;
}

bool ZEDWrapperNodelet::on_toggle_led(zed_interfaces::toggle_led::Request& req,
                                      zed_interfaces::toggle_led::Response& res)
{
  if (mCamFwVersion < 1523)
  {
    NODELET_WARN_STREAM("To set the status of the blue LED the camera must be updated to FW 1523 or newer");
    return false;
  }

  int status = mZed.getCameraSettings(sl::VIDEO_SETTINGS::LED_STATUS);
  int new_status = status == 0 ? 1 : 0;
  mZed.setCameraSettings(sl::VIDEO_SETTINGS::LED_STATUS, new_status);

  return (new_status == 1);
}

}  // namespace zed_nodelets

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(zed_nodelets::ZEDWrapperNodelet, nodelet::Nodelet);
