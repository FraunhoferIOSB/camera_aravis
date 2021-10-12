/****************************************************************************
 *
 * camera_aravis
 *
 * Copyright © 2019 Fraunhofer FKIE, Straw Lab, van Breugel Lab, and contributors
 * Authors: Dominik A. Klein,
 * 			Floris van Breugel,
 * 			Andrew Straw,
 * 			Steve Safarik
 *
 * Licensed under the LGPL, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.gnu.org/licenses/lgpl-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/

#include "../include/camera_aravis/camera_aravis_nodelet.h"

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(camera_aravis::CameraAravisNodelet, nodelet::Nodelet)

namespace camera_aravis
{

void renameImg(sensor_msgs::ImagePtr& in, sensor_msgs::ImagePtr& out, const std::string out_format) {
  if (!in) {
    ROS_WARN("camera_aravis::renameImg(): no input image given.");
    return;
  }

  // make a shallow copy (in-place operation on input)
  out = in;

  out->encoding = out_format;
}

void shift(uint16_t* data, const size_t length, const size_t digits) {
  for (size_t i=0; i<length; ++i) {
    data[i] <<= digits;
  }
}

void shiftImg(sensor_msgs::ImagePtr& in, sensor_msgs::ImagePtr& out, const size_t n_digits, const std::string out_format)
{
  if (!in) {
    ROS_WARN("camera_aravis::shiftImg(): no input image given.");
    return;
  }

  // make a shallow copy (in-place operation on input)
  out = in;

  // shift
  shift(reinterpret_cast<uint16_t*>(out->data.data()), out->data.size()/2, n_digits);
  out->encoding = out_format;
}

void interleaveImg(sensor_msgs::ImagePtr& in, sensor_msgs::ImagePtr& out, const size_t n_digits, const std::string out_format)
{
  if (!in) {
    ROS_WARN("camera_aravis::interleaveImg(): no input image given.");
    return;
  }

  if (!out) {
    out.reset(new sensor_msgs::Image);
    ROS_INFO("camera_aravis::interleaveImg(): no output image given. Reserved a new one.");
  }

  out->header = in->header;
  out->height = in->height;
  out->width = in->width;
  out->is_bigendian = in->is_bigendian;
  out->step = in->step;
  out->data.resize(in->data.size());

  const size_t n_bytes = in->data.size() / (3 * in->width * in->height);
  uint8_t* c0 = in->data.data();
  uint8_t* c1 = in->data.data() + (in->data.size() / 3);
  uint8_t* c2 = in->data.data() + (2 * in->data.size() / 3);
  uint8_t* o = out->data.data();

  for (uint32_t h=0; h<in->height; ++h) {
    for (uint32_t w=0; w<in->width; ++w) {
      for (size_t i=0; i<n_bytes; ++i) {
        o[i] = c0[i];
        o[i+n_bytes] = c1[i];
        o[i+2*n_bytes] = c2[i];
      }
      c0 += n_bytes;
      c1 += n_bytes;
      c2 += n_bytes;
      o += 3*n_bytes;
    }
  }

  if (n_digits>0) {
    shift(reinterpret_cast<uint16_t*>(out->data.data()), out->data.size()/2, n_digits);
  }
  out->encoding = out_format;
}

void unpack10p32Img(sensor_msgs::ImagePtr& in, sensor_msgs::ImagePtr& out, const std::string out_format) {
  if (!in) {
    ROS_WARN("camera_aravis::unpack10pImg(): no input image given.");
    return;
  }

  if (!out) {
    out.reset(new sensor_msgs::Image);
    ROS_INFO("camera_aravis::unpack10pImg(): no output image given. Reserved a new one.");
  }

  out->header = in->header;
  out->height = in->height;
  out->width = in->width;
  out->is_bigendian = in->is_bigendian;
  out->step = (3*in->step)/2;
  out->data.resize((3*in->data.size())/2);

  // change pixel bit alignment from every 3*10+2 = 32 Bit = 4 Byte format LSB
  //  byte 3 | byte 2 | byte 1 | byte 0
  // 00CCCCCC CCCCBBBB BBBBBBAA AAAAAAAA
  // into 3*16 = 48 Bit = 6 Byte format
  //  bytes 5+4       | bytes 3+2       | bytes 1+0
  // CCCCCCCC CC000000 BBBBBBBB BB000000 AAAAAAAA AA000000

  uint8_t* from = in->data.data();
  uint16_t* to = reinterpret_cast<uint16_t*>(out->data.data());
  // unpack a full RGB pixel per iteration
  for (size_t i=0; i<in->data.size()/4; ++i) {

    std::memcpy(to, from, 2);
    to[0] <<= 6;

    std::memcpy(&to[1], &from[1], 2);
    to[1] <<= 4;
    to[1] &= 0b1111111111000000;

    std::memcpy(&to[2], &from[2], 2);
    to[2] <<= 2;
    to[2] &= 0b1111111111000000;

    to+=3;
    from+=4;
  }

  out->encoding = out_format;
}

void unpack10PackedImg(sensor_msgs::ImagePtr& in, sensor_msgs::ImagePtr& out, const std::string out_format) {
  if (!in) {
    ROS_WARN("camera_aravis::unpack10pImg(): no input image given.");
    return;
  }

  if (!out) {
    out.reset(new sensor_msgs::Image);
    ROS_INFO("camera_aravis::unpack10pImg(): no output image given. Reserved a new one.");
  }

  out->header = in->header;
  out->height = in->height;
  out->width = in->width;
  out->is_bigendian = in->is_bigendian;
  out->step = (3*in->step)/2;
  out->data.resize((3*in->data.size())/2);

  // change pixel bit alignment from every 3*10+2 = 32 Bit = 4 Byte format
  //  byte 3 | byte 2 | byte 1 | byte 0
  // AAAAAAAA BBBBBBBB CCCCCCCC 00CCBBAA
  // into 3*16 = 48 Bit = 6 Byte format
  //  bytes 5+4       | bytes 3+2       | bytes 1+0
  // CCCCCCCC CC000000 BBBBBBBB BB000000 AAAAAAAA AA000000

  // note that in this old style GigE format, byte 0 contains the lsb of C, B as well as A

  uint8_t* from = in->data.data();
  uint8_t* to = out->data.data();
  // unpack a RGB pixel per iteration
  for (size_t i=0; i<in->data.size()/4; ++i) {

    to[0] = from[0]<<6;
    to[1] = from[3];
    to[2] = (from[0] & 0b00001100)<<4;
    to[3] = from[2];
    to[4] = (from[0] & 0b00110000)<<2;
    to[5] = from[1];

    to+=6;
    from+=4;
  }
  out->encoding = out_format;
}


void unpack10pMonoImg(sensor_msgs::ImagePtr& in, sensor_msgs::ImagePtr& out, const std::string out_format) {
  if (!in) {
    ROS_WARN("camera_aravis::unpack10pImg(): no input image given.");
    return;
  }

  if (!out) {
    out.reset(new sensor_msgs::Image);
    ROS_INFO("camera_aravis::unpack10pImg(): no output image given. Reserved a new one.");
  }

  out->header = in->header;
  out->height = in->height;
  out->width = in->width;
  out->is_bigendian = in->is_bigendian;
  out->step = (8*in->step)/5;
  out->data.resize((8*in->data.size())/5);

  // change pixel bit alignment from every 4*10 = 40 Bit = 5 Byte format LSB
  // byte 4  | byte 3 | byte 2 | byte 1 | byte 0
  // DDDDDDDD DDCCCCCC CCCCBBBB BBBBBBAA AAAAAAAA
  // into 4*16 = 64 Bit = 8 Byte format
  // bytes 7+6        | bytes 5+4       | bytes 3+2       | bytes 1+0
  // DDDDDDDD DD000000 CCCCCCCC CC000000 BBBBBBBB BB000000 AAAAAAAA AA000000

  uint8_t* from = in->data.data();
  uint16_t* to = reinterpret_cast<uint16_t*>(out->data.data());
  // unpack 4 mono pixels per iteration
  for (size_t i=0; i<in->data.size()/5; ++i) {

    std::memcpy(to, from, 2);
    to[0] <<= 6;

    std::memcpy(&to[1], &from[1], 2);
    to[1] <<= 4;
    to[1] &= 0b1111111111000000;

    std::memcpy(&to[2], &from[2], 2);
    to[2] <<= 2;
    to[2] &= 0b1111111111000000;

    std::memcpy(&to[3], &from[3], 2);
    to[3] &= 0b1111111111000000;

    to+=4;
    from+=5;
  }
  out->encoding = out_format;
}

void unpack10PackedMonoImg(sensor_msgs::ImagePtr& in, sensor_msgs::ImagePtr& out, const std::string out_format) {
  if (!in) {
    ROS_WARN("camera_aravis::unpack10pImg(): no input image given.");
    return;
  }

  if (!out) {
    out.reset(new sensor_msgs::Image);
    ROS_INFO("camera_aravis::unpack10pImg(): no output image given. Reserved a new one.");
  }

  out->header = in->header;
  out->height = in->height;
  out->width = in->width;
  out->is_bigendian = in->is_bigendian;
  out->step = (4*in->step)/3;
  out->data.resize((4*in->data.size())/3);

  // change pixel bit alignment from every 2*10+4 = 24 Bit = 3 Byte format
  //  byte 2 | byte 1 | byte 0
  // BBBBBBBB 00BB00AA AAAAAAAA
  // into 2*16 = 32 Bit = 4 Byte format
  //  bytes 3+2       | bytes 1+0
  // BBBBBBBB BB000000 AAAAAAAA AA000000

  // note that in this old style GigE format, byte 1 contains the lsb of B as well as A

  uint8_t* from = in->data.data();
  uint8_t* to = out->data.data();
  // unpack 4 mono pixels per iteration
  for (size_t i=0; i<in->data.size()/3; ++i) {

    to[0] = from[1]<<6;
    to[1] = from[0];

    to[2] = from[1] & 0b11000000;
    to[3] = from[2];

    to+=4;
    from+=3;
  }
  out->encoding = out_format;
}

void unpack12pImg(sensor_msgs::ImagePtr& in, sensor_msgs::ImagePtr& out, const std::string out_format) {
  if (!in) {
    ROS_WARN("camera_aravis::unpack12pImg(): no input image given.");
    return;
  }

  if (!out) {
    out.reset(new sensor_msgs::Image);
    ROS_INFO("camera_aravis::unpack12pImg(): no output image given. Reserved a new one.");
  }

  out->header = in->header;
  out->height = in->height;
  out->width = in->width;
  out->is_bigendian = in->is_bigendian;
  out->step = (4*in->step)/3;
  out->data.resize((4*in->data.size())/3);

  // change pixel bit alignment from every 2*12 = 24 Bit = 3 Byte format LSB
  //  byte 2 | byte 1 | byte 0
  // BBBBBBBB BBBBAAAA AAAAAAAA
  // into 2*16 = 32 Bit = 4 Byte format
  //  bytes 3+2       | bytes 1+0
  // BBBBBBBB BBBB0000 AAAAAAAA AAAA0000

  uint8_t* from = in->data.data();
  uint16_t* to = reinterpret_cast<uint16_t*>(out->data.data());
  // unpack 2 values per iteration
  for (size_t i=0; i<in->data.size()/3; ++i) {

    std::memcpy(to, from, 2);
    to[0] <<= 4;

    std::memcpy(&to[1], &from[1], 2);
    to[1] &= 0b1111111111110000;

    to+=2;
    from+=3;
  }
  out->encoding = out_format;
}

void unpack12PackedImg(sensor_msgs::ImagePtr& in, sensor_msgs::ImagePtr& out, const std::string out_format) {
  if (!in) {
    ROS_WARN("camera_aravis::unpack12pImg(): no input image given.");
    return;
  }

  if (!out) {
    out.reset(new sensor_msgs::Image);
    ROS_INFO("camera_aravis::unpack12pImg(): no output image given. Reserved a new one.");
  }

  out->header = in->header;
  out->height = in->height;
  out->width = in->width;
  out->is_bigendian = in->is_bigendian;
  out->step = (4*in->step)/3;
  out->data.resize((4*in->data.size())/3);

  // change pixel bit alignment from every 2*12 = 24 Bit = 3 Byte format
  //  byte 2 | byte 1 | byte 0
  // BBBBBBBB BBBBAAAA AAAAAAAA
  // into 2*16 = 32 Bit = 4 Byte format
  //  bytes 3+2       | bytes 1+0
  // BBBBBBBB BBBB0000 AAAAAAAA AAAA0000

  // note that in this old style GigE format, byte 1 contains the lsb of B as well as A

  uint8_t* from = in->data.data();
  uint8_t* to = out->data.data();
  // unpack 2 values per iteration
  for (size_t i=0; i<in->data.size()/3; ++i) {

    to[0] = from[1]<<4;
    to[1] = from[0];

    to[2] = from[1] & 0b11110000;
    to[3] = from[2];

    to+=4;
    from+=3;
  }
  out->encoding = out_format;
}

void unpack565pImg(sensor_msgs::ImagePtr& in, sensor_msgs::ImagePtr& out, const std::string out_format) {
  if (!in) {
    ROS_WARN("camera_aravis::unpack565pImg(): no input image given.");
    return;
  }

  if (!out) {
    out.reset(new sensor_msgs::Image);
    ROS_INFO("camera_aravis::unpack565pImg(): no output image given. Reserved a new one.");
  }

  out->header = in->header;
  out->height = in->height;
  out->width = in->width;
  out->is_bigendian = in->is_bigendian;
  out->step = (3*in->step)/2;
  out->data.resize((3*in->data.size())/2);

  // change pixel bit alignment from every 5+6+5 = 16 Bit = 2 Byte format LSB
  //  byte 1 | byte 0
  // CCCCCBBB BBBAAAAA
  // into 3*8 = 24 Bit = 3 Byte format
  //  byte 2 | byte 1 | byte 0
  // CCCCC000 BBBBBB00 AAAAA000

  uint8_t* from = in->data.data();
  uint8_t* to = out->data.data();
  // unpack a whole RGB pixel per iteration
  for (size_t i=0; i<in->data.size()/2; ++i) {
    to[0] = from[0] << 3;

    to[1] = from[0] >> 3;
    to[1] |= (from[1]<<5);
    to[1] &= 0b11111100;

    to[2] = from[1] & 0b11111000;

    to+=3;
    from+=2;
  }
  out->encoding = out_format;
}

CameraAravisNodelet::CameraAravisNodelet()
{
}

CameraAravisNodelet::~CameraAravisNodelet()
{
  for(int i=0; i < p_streams_.size(); i++) {
    if(p_streams_[i]) {
      arv_stream_set_emit_signals(p_streams_[i], FALSE);
    }
  }

  spawning_ = false;
  if (spawn_stream_thread_.joinable())
  {
    spawn_stream_thread_.join();
  }

  software_trigger_active_ = false;
  if (software_trigger_thread_.joinable())
  {
    software_trigger_thread_.join();
  }

  tf_thread_active_ = false;
  if (tf_dyn_thread_.joinable())
  {
    tf_dyn_thread_.join();
  }


  for(int i=0; i < p_streams_.size(); i++) {
    guint64 n_completed_buffers = 0;
    guint64 n_failures = 0;
    guint64 n_underruns = 0;
    arv_stream_get_statistics(p_streams_[i], &n_completed_buffers, &n_failures, &n_underruns);
    ROS_INFO("Completed buffers = %Lu", (unsigned long long ) n_completed_buffers);
    ROS_INFO("Failures          = %Lu", (unsigned long long ) n_failures);
    ROS_INFO("Underruns         = %Lu", (unsigned long long ) n_underruns);
    if (arv_camera_is_gv_device(p_camera_))
    {
    guint64 n_resent;
    guint64 n_missing;
    arv_gv_stream_get_statistics(reinterpret_cast<ArvGvStream*>(p_streams_[i]), &n_resent, &n_missing);
    ROS_INFO("Resent buffers    = %Lu", (unsigned long long ) n_resent);
    ROS_INFO("Missing           = %Lu", (unsigned long long ) n_missing);
    }
  }


  if (p_device_)
  {
    arv_device_execute_command(p_device_, "AcquisitionStop");
  }

  for(int i = 0; i < p_streams_.size(); i++) {
      g_object_unref(p_streams_[i]);
  }
  g_object_unref(p_camera_);
}

void CameraAravisNodelet::onInit()
{
  ros::NodeHandle pnh = getPrivateNodeHandle();

  // Print out some useful info.
  ROS_INFO("Attached cameras:");
  arv_update_device_list();
  uint n_interfaces = arv_get_n_interfaces();
  ROS_INFO("# Interfaces: %d", n_interfaces);

  uint n_devices = arv_get_n_devices();
  ROS_INFO("# Devices: %d", n_devices);
  for (uint i = 0; i < n_devices; i++)
    ROS_INFO("Device%d: %s", i, arv_get_device_id(i));

  if (n_devices == 0)
  {
    ROS_ERROR("No cameras detected.");
    return;
  }

  // Get the camera guid as a parameter or use the first device.
  std::string guid;
  if (pnh.hasParam("guid"))
  {
    pnh.getParam("guid", guid);
  }

  // Get PTP timestamp parameter
  pnh.param<bool>("use_ptp_timestamp", use_ptp_stamp_, false);

  // Open the camera, and set it up.
  while (!p_camera_)
  {
    if (guid.empty())
    {
      ROS_INFO("Opening: (any)");
      p_camera_ = arv_camera_new(NULL);
    }
    else
    {
      ROS_INFO_STREAM("Opening: " << guid);
      p_camera_ = arv_camera_new(guid.c_str());
    }
    ros::Duration(1.0).sleep();
    ros::spinOnce();
  }

  p_device_ = arv_camera_get_device(p_camera_);
  ROS_INFO("Opened: %s-%s", arv_camera_get_vendor_name(p_camera_),
           arv_device_get_string_feature_value(p_device_, "DeviceSerialNumber"));

  // Start the dynamic_reconfigure server.
  reconfigure_server_.reset(new dynamic_reconfigure::Server<Config>(reconfigure_mutex_, pnh));
  reconfigure_server_->getConfigDefault(config_);
  reconfigure_server_->getConfigMin(config_min_);
  reconfigure_server_->getConfigMax(config_max_);

  // See which features exist in this camera device
  discoverFeatures();

  // Check the number of streams for this camera
  num_streams_ = 0;
  num_streams_ = arv_device_get_integer_feature_value(p_device_, "DeviceStreamChannelCount");
  // if this return 0, try the deprecated GevStreamChannelCount in case this is an older camera
  if (num_streams_ == 0) {
    num_streams_ = arv_device_get_integer_feature_value(p_device_, "GevStreamChannelCount");
  }
  ROS_INFO("Number of supported stream channels %i.", (int) num_streams_);

  std::string stream_channel_args;
  if (pnh.getParam("channel_names", stream_channel_args)) {
    parseStringArgs(stream_channel_args, stream_names_);
  } else {
    stream_names_ = { "" };
  }

  std::string pixel_format_args;
  std::vector<std::string> pixel_formats;
  pnh.param("PixelFormat", pixel_format_args, pixel_format_args);
  parseStringArgs(pixel_format_args, pixel_formats);

  std::string calib_url_args;
  std::vector<std::string> calib_urls;
  pnh.param("camera_info_url", calib_url_args, calib_url_args);
  parseStringArgs(calib_url_args, calib_urls);

  // check if every stream channel has been given a channel name
  if (stream_names_.size() < num_streams_) {
    num_streams_ = stream_names_.size();
  }

  // initialize the sensor structs
  for(int i = 0; i < num_streams_; i++) {
    sensors_.push_back(new Sensor());
    p_streams_.push_back(NULL);
    p_buffer_pools_.push_back(CameraBufferPool::Ptr());
    p_camera_info_managers_.push_back(NULL);
    camera_infos_.push_back(sensor_msgs::CameraInfoPtr());
    cam_pubs_.push_back(image_transport::CameraPublisher());
  }

  // Get parameter bounds.
  arv_camera_get_exposure_time_bounds(p_camera_, &config_min_.ExposureTime, &config_max_.ExposureTime);
  arv_camera_get_gain_bounds(p_camera_, &config_min_.Gain, &config_max_.Gain);
  for(int i = 0; i < num_streams_; i++) {
    arv_camera_gv_select_stream_channel(p_camera_,i);
    arv_camera_get_sensor_size(p_camera_, &sensors_[i]->width, &sensors_[i]->height);
  }
  arv_camera_get_width_bounds(p_camera_, &roi_.width_min, &roi_.width_max);
  arv_camera_get_height_bounds(p_camera_, &roi_.height_min, &roi_.height_max);
  arv_camera_get_frame_rate_bounds(p_camera_, &config_min_.AcquisitionFrameRate, &config_max_.AcquisitionFrameRate);
  if (implemented_features_["FocusPos"])
  {
    gint64 focus_min64, focus_max64;
    arv_device_get_integer_feature_bounds(p_device_, "FocusPos", &focus_min64, &focus_max64);
    config_min_.FocusPos = focus_min64;
    config_max_.FocusPos = focus_max64;
  }
  else
  {
    config_min_.FocusPos = 0;
    config_max_.FocusPos = 0;
  }

  for(int i = 0; i < num_streams_; i++) {
    arv_camera_gv_select_stream_channel(p_camera_,i);

    // Initial camera settings.
    if (implemented_features_["ExposureTime"])
      arv_camera_set_exposure_time(p_camera_, config_.ExposureTime);
    else if (implemented_features_["ExposureTimeAbs"])
      arv_device_set_float_feature_value(p_device_, "ExposureTimeAbs", config_.ExposureTime);
    if (implemented_features_["Gain"])
      arv_camera_set_gain(p_camera_, config_.Gain);
    if (implemented_features_["AcquisitionFrameRateEnable"])
      arv_device_set_integer_feature_value(p_device_, "AcquisitionFrameRateEnable", 1);
    if (implemented_features_["AcquisitionFrameRate"])
      arv_camera_set_frame_rate(p_camera_, config_.AcquisitionFrameRate);

    // init default to full sensor resolution
    arv_camera_set_region (p_camera_, 0, 0, roi_.width_max, roi_.height_max);

    // Set up the triggering.
    if (implemented_features_["TriggerMode"] && implemented_features_["TriggerSelector"])
    {
      arv_device_set_string_feature_value(p_device_, "TriggerSelector", "FrameStart");
      arv_device_set_string_feature_value(p_device_, "TriggerMode", "Off");
    }

    // possibly set or override from given parameter
    writeCameraFeaturesFromRosparam();
  }

  // get current state of camera for config_
  arv_camera_get_region(p_camera_, &roi_.x, &roi_.y, &roi_.width, &roi_.height);
  config_.AcquisitionMode =
      implemented_features_["AcquisitionMode"] ? arv_device_get_string_feature_value(p_device_, "AcquisitionMode") :
          "Continuous";
  config_.AcquisitionFrameRate =
      implemented_features_["AcquisitionFrameRate"] ? arv_camera_get_frame_rate(p_camera_) : 0.0;
  config_.ExposureAuto =
      implemented_features_["ExposureAuto"] ? arv_device_get_string_feature_value(p_device_, "ExposureAuto") : "Off";
  config_.ExposureTime = implemented_features_["ExposureTime"] ? arv_camera_get_exposure_time(p_camera_) : 0.0;
  config_.GainAuto =
      implemented_features_["GainAuto"] ? arv_device_get_string_feature_value(p_device_, "GainAuto") : "Off";
  config_.Gain = implemented_features_["Gain"] ? arv_camera_get_gain(p_camera_) : 0.0;
  config_.TriggerMode =
      implemented_features_["TriggerMode"] ? arv_device_get_string_feature_value(p_device_, "TriggerMode") : "Off";
  config_.TriggerSource =
      implemented_features_["TriggerSource"] ? arv_device_get_string_feature_value(p_device_, "TriggerSource") :
          "Software";

  // get pixel format name and translate into corresponding ROS name
  for(int i = 0; i < num_streams_; i++) {
    arv_camera_gv_select_stream_channel(p_camera_,i);
    arv_device_set_string_feature_value(p_device_, "PixelFormat", pixel_formats[i].c_str());
    sensors_[i]->pixel_format = std::string(arv_device_get_string_feature_value(p_device_, "PixelFormat"));
    const auto sensor_iter = CONVERSIONS_DICTIONARY.find(sensors_[i]->pixel_format);
    if (sensor_iter!=CONVERSIONS_DICTIONARY.end()) {
      convert_format = sensor_iter->second;
    }
    else {
      ROS_WARN_STREAM("There is no known conversion from " << sensors_[i]->pixel_format << " to a usual ROS image encoding. Likely you need to implement one.");
    }

    sensors_[i]->n_bits_pixel = ARV_PIXEL_FORMAT_BIT_PER_PIXEL(
        arv_device_get_integer_feature_value(p_device_, "PixelFormat"));
    config_.FocusPos =
        implemented_features_["FocusPos"] ? arv_device_get_integer_feature_value(p_device_, "FocusPos") : 0;

  }

  // Get other (non GenIcam) parameter current values.
  pnh.param<double>("softwaretriggerrate", config_.softwaretriggerrate, config_.softwaretriggerrate);
  pnh.param<int>("mtu", config_.mtu, config_.mtu);
  pnh.param<std::string>("frame_id", config_.frame_id, config_.frame_id);
  pnh.param<bool>("auto_master", config_.AutoMaster, config_.AutoMaster);
  pnh.param<bool>("auto_slave", config_.AutoSlave, config_.AutoSlave);
  setAutoMaster(config_.AutoMaster);
  setAutoSlave(config_.AutoSlave);

  // should we publish tf transforms to camera optical frame?
  bool pub_tf_optical;
  pnh.param<bool>("publish_tf", pub_tf_optical, false);
  if (pub_tf_optical)
  {
    tf_optical_.header.frame_id = config_.frame_id;
    tf_optical_.child_frame_id = config_.frame_id + "_optical";
    tf_optical_.transform.translation.x = 0.0;
    tf_optical_.transform.translation.y = 0.0;
    tf_optical_.transform.translation.z = 0.0;
    tf_optical_.transform.rotation.x = -0.5;
    tf_optical_.transform.rotation.y = 0.5;
    tf_optical_.transform.rotation.z = -0.5;
    tf_optical_.transform.rotation.w = 0.5;

    double tf_publish_rate;
    pnh.param<double>("tf_publish_rate", tf_publish_rate, 0);
    if (tf_publish_rate > 0.)
    {
      // publish dynamic tf at given rate (recommended when running as a Nodelet, since latching has bugs-by-design)
      p_tb_.reset(new tf2_ros::TransformBroadcaster());
      tf_dyn_thread_ = std::thread(&CameraAravisNodelet::publishTfLoop, this, tf_publish_rate);
    }
    else
    {
      // publish static tf only once (latched)
      p_stb_.reset(new tf2_ros::StaticTransformBroadcaster());
      tf_optical_.header.stamp = ros::Time::now();
      p_stb_->sendTransform(tf_optical_);
    }
  }

  // default calibration url is [DeviceSerialNumber/DeviceID].yaml
  ArvGcNode *p_gc_node;
  GError *error = NULL;

  p_gc_node = arv_device_get_feature(p_device_, "DeviceSerialNumber");

  if(calib_urls[0].empty()) {
    if( arv_gc_feature_node_is_implemented( ARV_GC_FEATURE_NODE(p_gc_node), &error) ) {
      GType device_serial_return_type = arv_gc_feature_node_get_value_type( ARV_GC_FEATURE_NODE(p_gc_node));
      // If the feature DeviceSerialNumber is not string, it indicates that the camera is using an older version of the genicam SFNC.
      // Older camera models do not have a DeviceSerialNumber as string, but as integer and often set to 0.
      // In those cases use the outdated DeviceID (deprecated since genicam SFNC v2.0).
      if (device_serial_return_type == G_TYPE_STRING) {
        calib_urls[0] = arv_device_get_string_feature_value(p_device_, "DeviceSerialNumber");
        calib_urls[0] += ".yaml";
      } else if (device_serial_return_type == G_TYPE_INT64) {
        calib_urls[0] = arv_device_get_string_feature_value(p_device_, "DeviceID");
        calib_urls[0] += ".yaml";
      }
    }
  }

  for(int i = 0; i < num_streams_; i++) {
    // Start the camerainfo manager.
    p_camera_info_managers_[i].reset(new camera_info_manager::CameraInfoManager(pnh, stream_names_[i], calib_urls[i]));
    ROS_INFO("Reset %s Camera Info Manager", stream_names_[i].c_str());
    ROS_INFO("%s Calib URL: %s", stream_names_[i].c_str(), calib_urls[i].c_str());
  }

  // update the reconfigure config
  reconfigure_server_->setConfigMin(config_min_);
  reconfigure_server_->setConfigMax(config_max_);
  reconfigure_server_->updateConfig(config_);
  ros::Duration(2.0).sleep();
  reconfigure_server_->setCallback(boost::bind(&CameraAravisNodelet::rosReconfigureCallback, this, _1, _2));

  // Print information.
  ROS_INFO("    Using Camera Configuration:");
  ROS_INFO("    ---------------------------");
  ROS_INFO("    Vendor name          = %s", arv_device_get_string_feature_value(p_device_, "DeviceVendorName"));
  ROS_INFO("    Model name           = %s", arv_device_get_string_feature_value(p_device_, "DeviceModelName"));
  ROS_INFO("    Device id            = %s", arv_device_get_string_feature_value(p_device_, "DeviceUserID"));
  ROS_INFO("    Serial number        = %s", arv_device_get_string_feature_value(p_device_, "DeviceSerialNumber"));
  ROS_INFO(
      "    Type                 = %s",
      arv_camera_is_uv_device(p_camera_) ? "USB3Vision" :
          (arv_camera_is_gv_device(p_camera_) ? "GigEVision" : "Other"));
  ROS_INFO("    Sensor width         = %d", sensors_[0]->width);
  ROS_INFO("    Sensor height        = %d", sensors_[0]->height);
  ROS_INFO("    ROI x,y,w,h          = %d, %d, %d, %d", roi_.x, roi_.y, roi_.width, roi_.height);
  ROS_INFO("    Pixel format         = %s", sensors_[0]->pixel_format.c_str());
  ROS_INFO("    BitsPerPixel         = %lu", sensors_[0]->n_bits_pixel);
  ROS_INFO(
      "    Acquisition Mode     = %s",
      implemented_features_["AcquisitionMode"] ? arv_device_get_string_feature_value(p_device_, "AcquisitionMode") :
          "(not implemented in camera)");
  ROS_INFO(
      "    Trigger Mode         = %s",
      implemented_features_["TriggerMode"] ? arv_device_get_string_feature_value(p_device_, "TriggerMode") :
          "(not implemented in camera)");
  ROS_INFO(
      "    Trigger Source       = %s",
      implemented_features_["TriggerSource"] ? arv_device_get_string_feature_value(p_device_, "TriggerSource") :
          "(not implemented in camera)");
  ROS_INFO("    Can set FrameRate:     %s", implemented_features_["AcquisitionFrameRate"] ? "True" : "False");
  if (implemented_features_["AcquisitionFrameRate"])
  {
    ROS_INFO("    AcquisitionFrameRate = %g hz", config_.AcquisitionFrameRate);
  }

  ROS_INFO("    Can set Exposure:      %s", implemented_features_["ExposureTime"] ? "True" : "False");
  if (implemented_features_["ExposureTime"])
  {
    ROS_INFO("    Can set ExposureAuto:  %s", implemented_features_["ExposureAuto"] ? "True" : "False");
    ROS_INFO("    Exposure             = %g us in range [%g,%g]", config_.ExposureTime, config_min_.ExposureTime,
             config_max_.ExposureTime);
  }

  ROS_INFO("    Can set Gain:          %s", implemented_features_["Gain"] ? "True" : "False");
  if (implemented_features_["Gain"])
  {
    ROS_INFO("    Can set GainAuto:      %s", implemented_features_["GainAuto"] ? "True" : "False");
    ROS_INFO("    Gain                 = %f %% in range [%f,%f]", config_.Gain, config_min_.Gain, config_max_.Gain);
  }

  ROS_INFO("    Can set FocusPos:      %s", implemented_features_["FocusPos"] ? "True" : "False");

  if (implemented_features_["GevSCPSPacketSize"])
    ROS_INFO("    Network mtu          = %lu", arv_device_get_integer_feature_value(p_device_, "GevSCPSPacketSize"));

  ROS_INFO("    ---------------------------");

  // Reset PTP clock
  if (use_ptp_stamp_)
    resetPtpClock();

  // spwan camera stream in thread, so onInit() is not blocked
  spawning_ = true;
  spawn_stream_thread_ = std::thread(&CameraAravisNodelet::spawnStream, this);
}

void CameraAravisNodelet::spawnStream()
{
  ros::NodeHandle nh  = getNodeHandle();
  ros::NodeHandle pnh = getPrivateNodeHandle();
  std::string guid;
  if (pnh.hasParam("guid"))
  {
    pnh.getParam("guid", guid);
  }

  while (spawning_)
  {
    for(int i = 0; i < num_streams_; i++) {
      while (true) {
        arv_camera_gv_select_stream_channel(p_camera_, i);
        p_streams_[i] = arv_camera_create_stream(p_camera_, NULL, NULL);

        if (p_streams_[i])
        {
          // Load up some buffers.
          arv_camera_gv_select_stream_channel(p_camera_, i);
          const gint n_bytes_payload_stream_ = arv_camera_get_payload(p_camera_);

          p_buffer_pools_[i].reset(new CameraBufferPool(p_streams_[i], n_bytes_payload_stream_, 10));

          if (arv_camera_is_gv_device(p_camera_))
          {
            tuneGvStream(reinterpret_cast<ArvGvStream*>(p_streams_[i]));
          }
          break;
        }
        else
        {
          ROS_WARN("Stream %i: Could not create image stream for %s.  Retrying...", i, guid.c_str());
          ros::Duration(1.0).sleep();
          ros::spinOnce();
        }
      }
    }
  }

  // Monitor whether anyone is subscribed to the camera stream
  std::vector<image_transport::SubscriberStatusCallback> image_cbs_;
  std::vector<ros::SubscriberStatusCallback> info_cbs_;

  image_transport::SubscriberStatusCallback image_cb = [this](const image_transport::SingleSubscriberPublisher &ssp)
  { this->rosConnectCallback();};
  ros::SubscriberStatusCallback info_cb = [this](const ros::SingleSubscriberPublisher &ssp)
  { this->rosConnectCallback();};

  for(int i = 0; i < num_streams_; i++) {
    image_transport::ImageTransport *p_transport;
    // Set up image_raw
    std::string topic_name = ros::this_node::getNamespace();
    p_transport = new image_transport::ImageTransport(pnh);
    if(num_streams_ != 1 || !stream_names_[i].empty()) {
      topic_name += "/" + stream_names_[i];
    }

    cam_pubs_[i] = p_transport->advertiseCamera(
      ros::names::remap(topic_name + "/image_raw"),
      1, image_cb, image_cb, info_cb, info_cb);
  }

  // Connect signals with callbacks.
  for(int i = 0; i < num_streams_; i++) {
    StreamIdData* data = new StreamIdData();
    data->can = this;
    data->stream_id = i;
    g_signal_connect(p_streams_[i], "new-buffer", (GCallback)CameraAravisNodelet::newBufferReadyCallback, data);
  }
  g_signal_connect(p_device_, "control-lost", (GCallback)CameraAravisNodelet::controlLostCallback, this);

  for(int i = 0; i < num_streams_; i++) {
    arv_stream_set_emit_signals(p_streams_[i], TRUE);
  }

  if (std::any_of(cam_pubs_.begin(), cam_pubs_.end(),
    [](image_transport::CameraPublisher pub){ return pub.getNumSubscribers() > 0; })
  ) {
    arv_camera_start_acquisition(p_camera_);
  }

  ROS_INFO("Done initializing camera_aravis.");
}

void CameraAravisNodelet::resetPtpClock()
{
  std::string ptp_status(arv_device_get_string_feature_value(p_device_, "GevIEEE1588Status"));
  if (ptp_status != std::string("Slave"))
  {
    ROS_INFO("camera_aravis: Reset ptp clock");
    arv_device_set_boolean_feature_value(p_device_, "GevIEEE1588", false);
    arv_device_set_boolean_feature_value(p_device_, "GevIEEE1588", true);
  }
}

void CameraAravisNodelet::cameraAutoInfoCallback(const CameraAutoInfoConstPtr &msg_ptr)
{
  if (config_.AutoSlave && p_device_)
  {

    if (auto_params_.exposure_time != msg_ptr->exposure_time && implemented_features_["ExposureTime"])
    {
      arv_device_set_float_feature_value(p_device_, "ExposureTime", msg_ptr->exposure_time);
    }

    if (implemented_features_["Gain"])
    {
      if (auto_params_.gain != msg_ptr->gain)
      {
        if (implemented_features_["GainSelector"])
        {
          arv_device_set_string_feature_value(p_device_, "GainSelector", "All");
        }
        arv_device_set_float_feature_value(p_device_, "Gain", msg_ptr->gain);
      }

      if (implemented_features_["GainSelector"])
      {
        if (auto_params_.gain_red != msg_ptr->gain_red)
        {
          arv_device_set_string_feature_value(p_device_, "GainSelector", "Red");
          arv_device_set_float_feature_value(p_device_, "Gain", msg_ptr->gain_red);
        }

        if (auto_params_.gain_green != msg_ptr->gain_green)
        {
          arv_device_set_string_feature_value(p_device_, "GainSelector", "Green");
          arv_device_set_float_feature_value(p_device_, "Gain", msg_ptr->gain_green);
        }

        if (auto_params_.gain_blue != msg_ptr->gain_blue)
        {
          arv_device_set_string_feature_value(p_device_, "GainSelector", "Blue");
          arv_device_set_float_feature_value(p_device_, "Gain", msg_ptr->gain_blue);
        }
      }
    }

    if (implemented_features_["BlackLevel"])
    {
      if (auto_params_.black_level != msg_ptr->black_level)
      {
        if (implemented_features_["BlackLevelSelector"])
        {
          arv_device_set_string_feature_value(p_device_, "BlackLevelSelector", "All");
        }
        arv_device_set_float_feature_value(p_device_, "BlackLevel", msg_ptr->black_level);
      }

      if (implemented_features_["BlackLevelSelector"])
      {
        if (auto_params_.bl_red != msg_ptr->bl_red)
        {
          arv_device_set_string_feature_value(p_device_, "BlackLevelSelector", "Red");
          arv_device_set_float_feature_value(p_device_, "BlackLevel", msg_ptr->bl_red);
        }

        if (auto_params_.bl_green != msg_ptr->bl_green)
        {
          arv_device_set_string_feature_value(p_device_, "BlackLevelSelector", "Green");
          arv_device_set_float_feature_value(p_device_, "BlackLevel", msg_ptr->bl_green);
        }

        if (auto_params_.bl_blue != msg_ptr->bl_blue)
        {
          arv_device_set_string_feature_value(p_device_, "BlackLevelSelector", "Blue");
          arv_device_set_float_feature_value(p_device_, "BlackLevel", msg_ptr->bl_blue);
        }
      }
    }

    // White balance as TIS is providing
    if (strcmp("The Imaging Source Europe GmbH", arv_camera_get_vendor_name(p_camera_)) == 0)
    {
      arv_device_set_integer_feature_value(p_device_, "WhiteBalanceRedRegister", (int)(auto_params_.wb_red * 255.));
      arv_device_set_integer_feature_value(p_device_, "WhiteBalanceGreenRegister", (int)(auto_params_.wb_green * 255.));
      arv_device_set_integer_feature_value(p_device_, "WhiteBalanceBlueRegister", (int)(auto_params_.wb_blue * 255.));
    }
    else if (implemented_features_["BalanceRatio"] && implemented_features_["BalanceRatioSelector"])
    {
      if (auto_params_.wb_red != msg_ptr->wb_red)
      {
        arv_device_set_string_feature_value(p_device_, "BalanceRatioSelector", "Red");
        arv_device_set_float_feature_value(p_device_, "BalanceRatio", msg_ptr->wb_red);
      }

      if (auto_params_.wb_green != msg_ptr->wb_green)
      {
        arv_device_set_string_feature_value(p_device_, "BalanceRatioSelector", "Green");
        arv_device_set_float_feature_value(p_device_, "BalanceRatio", msg_ptr->wb_green);
      }

      if (auto_params_.wb_blue != msg_ptr->wb_blue)
      {
        arv_device_set_string_feature_value(p_device_, "BalanceRatioSelector", "Blue");
        arv_device_set_float_feature_value(p_device_, "BalanceRatio", msg_ptr->wb_blue);
      }
    }

    auto_params_ = *msg_ptr;
  }
}

void CameraAravisNodelet::syncAutoParameters()
{
  auto_params_.exposure_time = auto_params_.gain = auto_params_.gain_red = auto_params_.gain_green =
      auto_params_.gain_blue = auto_params_.black_level = auto_params_.bl_red = auto_params_.bl_green =
          auto_params_.bl_blue = auto_params_.wb_red = auto_params_.wb_green = auto_params_.wb_blue =
              std::numeric_limits<double>::quiet_NaN();

  if (p_device_)
  {
    if (implemented_features_["ExposureTime"])
    {
      auto_params_.exposure_time = arv_device_get_float_feature_value(p_device_, "ExposureTime");
    }

    if (implemented_features_["Gain"])
    {
      if (implemented_features_["GainSelector"])
      {
        arv_device_set_string_feature_value(p_device_, "GainSelector", "All");
      }
      auto_params_.gain = arv_device_get_float_feature_value(p_device_, "Gain");
      if (implemented_features_["GainSelector"])
      {
        arv_device_set_string_feature_value(p_device_, "GainSelector", "Red");
        auto_params_.gain_red = arv_device_get_float_feature_value(p_device_, "Gain");
        arv_device_set_string_feature_value(p_device_, "GainSelector", "Green");
        auto_params_.gain_green = arv_device_get_float_feature_value(p_device_, "Gain");
        arv_device_set_string_feature_value(p_device_, "GainSelector", "Blue");
        auto_params_.gain_blue = arv_device_get_float_feature_value(p_device_, "Gain");
      }
    }

    if (implemented_features_["BlackLevel"])
    {
      if (implemented_features_["BlackLevelSelector"])
      {
        arv_device_set_string_feature_value(p_device_, "BlackLevelSelector", "All");
      }
      auto_params_.black_level = arv_device_get_float_feature_value(p_device_, "BlackLevel");
      if (implemented_features_["BlackLevelSelector"])
      {
        arv_device_set_string_feature_value(p_device_, "BlackLevelSelector", "Red");
        auto_params_.bl_red = arv_device_get_float_feature_value(p_device_, "BlackLevel");
        arv_device_set_string_feature_value(p_device_, "BlackLevelSelector", "Green");
        auto_params_.bl_green = arv_device_get_float_feature_value(p_device_, "BlackLevel");
        arv_device_set_string_feature_value(p_device_, "BlackLevelSelector", "Blue");
        auto_params_.bl_blue = arv_device_get_float_feature_value(p_device_, "BlackLevel");
      }
    }

    // White balance as TIS is providing
    if (strcmp("The Imaging Source Europe GmbH", arv_camera_get_vendor_name(p_camera_)) == 0)
    {
      auto_params_.wb_red = arv_device_get_integer_feature_value(p_device_, "WhiteBalanceRedRegister") / 255.;
      auto_params_.wb_green = arv_device_get_integer_feature_value(p_device_, "WhiteBalanceGreenRegister") / 255.;
      auto_params_.wb_blue = arv_device_get_integer_feature_value(p_device_, "WhiteBalanceBlueRegister") / 255.;
    }
    // the standard way
    else if (implemented_features_["BalanceRatio"] && implemented_features_["BalanceRatioSelector"])
    {
      arv_device_set_string_feature_value(p_device_, "BalanceRatioSelector", "Red");
      auto_params_.wb_red = arv_device_get_float_feature_value(p_device_, "BalanceRatio");
      arv_device_set_string_feature_value(p_device_, "BalanceRatioSelector", "Green");
      auto_params_.wb_green = arv_device_get_float_feature_value(p_device_, "BalanceRatio");
      arv_device_set_string_feature_value(p_device_, "BalanceRatioSelector", "Blue");
      auto_params_.wb_blue = arv_device_get_float_feature_value(p_device_, "BalanceRatio");
    }
  }
}

void CameraAravisNodelet::setAutoMaster(bool value)
{
  if (value)
  {
    syncAutoParameters();
    auto_pub_ = getNodeHandle().advertise<CameraAutoInfo>(ros::names::remap("camera_auto_info"), 1, true);
  }
  else
  {
    auto_pub_.shutdown();
  }
}

void CameraAravisNodelet::setAutoSlave(bool value)
{
  if (value)
  {
    // deactivate all auto functions
    if (implemented_features_["ExposureAuto"])
    {
      arv_device_set_string_feature_value(p_device_, "ExposureAuto", "Off");
    }
    if (implemented_features_["GainAuto"])
    {
      arv_device_set_string_feature_value(p_device_, "GainAuto", "Off");
    }
    if (implemented_features_["GainAutoBalance"])
    {
      arv_device_set_string_feature_value(p_device_, "GainAutoBalance", "Off");
    }
    if (implemented_features_["BlackLevelAuto"])
    {
      arv_device_set_string_feature_value(p_device_, "BlackLevelAuto", "Off");
    }
    if (implemented_features_["BlackLevelAutoBalance"])
    {
      arv_device_set_string_feature_value(p_device_, "BlackLevelAutoBalance", "Off");
    }
    if (implemented_features_["BalanceWhiteAuto"])
    {
      arv_device_set_string_feature_value(p_device_, "BalanceWhiteAuto", "Off");
    }
    syncAutoParameters();
    auto_sub_ = getNodeHandle().subscribe(ros::names::remap("camera_auto_info"), 1,
                                          &CameraAravisNodelet::cameraAutoInfoCallback, this);
  }
  else
  {
    auto_sub_.shutdown();
  }
}

// Extra stream options for GigEVision streams.
void CameraAravisNodelet::tuneGvStream(ArvGvStream *p_stream)
{
  gboolean b_auto_buffer = FALSE;
  gboolean b_packet_resend = TRUE;
  unsigned int timeout_packet = 40; // milliseconds
  unsigned int timeout_frame_retention = 200;

  if (p_stream)
  {
    if (!ARV_IS_GV_STREAM(p_stream))
    {
      ROS_WARN("Stream is not a GV_STREAM");
      return;
    }

    if (b_auto_buffer)
      g_object_set(p_stream, "socket-buffer", ARV_GV_STREAM_SOCKET_BUFFER_AUTO, "socket-buffer-size", 0,
      NULL);
    if (!b_packet_resend)
      g_object_set(p_stream, "packet-resend",
                   b_packet_resend ? ARV_GV_STREAM_PACKET_RESEND_ALWAYS : ARV_GV_STREAM_PACKET_RESEND_NEVER,
                   NULL);
    g_object_set(p_stream, "packet-timeout", timeout_packet * 1000, "frame-retention", timeout_frame_retention * 1000,
    NULL);
  }
}

void CameraAravisNodelet::rosReconfigureCallback(Config &config, uint32_t level)
{
  reconfigure_mutex_.lock();
  std::string tf_prefix = tf::getPrefixParam(getNodeHandle());
  ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);

  if (config.frame_id == "")
    config.frame_id = this->getName();

  // Limit params to legal values.
  config.AcquisitionFrameRate = CLAMP(config.AcquisitionFrameRate, config_min_.AcquisitionFrameRate,
                                      config_max_.AcquisitionFrameRate);
  config.ExposureTime = CLAMP(config.ExposureTime, config_min_.ExposureTime, config_max_.ExposureTime);
  config.Gain = CLAMP(config.Gain, config_min_.Gain, config_max_.Gain);
  config.FocusPos = CLAMP(config.FocusPos, config_min_.FocusPos, config_max_.FocusPos);
  config.frame_id = tf::resolve(tf_prefix, config.frame_id);

  if (use_ptp_stamp_)
    resetPtpClock();

  // stop auto functions if slave
  if (config.AutoSlave)
  {
    config.ExposureAuto = "Off";
    config.GainAuto = "Off";
    // todo: all other auto functions "Off"
  }

  // reset values controlled by auto functions
  if (config.ExposureAuto.compare("Off") != 0)
  {
    config.ExposureTime = config_.ExposureTime;
    ROS_WARN("ExposureAuto is active. Cannot manually set ExposureTime.");
  }
  if (config.GainAuto.compare("Off") != 0)
  {
    config.Gain = config_.Gain;
    ROS_WARN("GainAuto is active. Cannot manually set Gain.");
  }

  // reset FrameRate when triggered
  if (config.TriggerMode.compare("Off") != 0)
  {
    config.AcquisitionFrameRate = config_.AcquisitionFrameRate;
    ROS_WARN("TriggerMode is active. Cannot manually set AcquisitionFrameRate.");
  }

  // Find valid user changes we need to react to.
  const bool changed_auto_master = (config_.AutoMaster != config.AutoMaster);
  const bool changed_auto_slave = (config_.AutoSlave != config.AutoSlave);
  const bool changed_acquisition_frame_rate = (config_.AcquisitionFrameRate != config.AcquisitionFrameRate);
  const bool changed_exposure_auto = (config_.ExposureAuto != config.ExposureAuto);
  const bool changed_exposure_time = (config_.ExposureTime != config.ExposureTime);
  const bool changed_gain_auto = (config_.GainAuto != config.GainAuto);
  const bool changed_gain = (config_.Gain != config.Gain);
  const bool changed_acquisition_mode = (config_.AcquisitionMode != config.AcquisitionMode);
  const bool changed_trigger_mode = (config_.TriggerMode != config.TriggerMode);
  const bool changed_trigger_source = (config_.TriggerSource != config.TriggerSource) || changed_trigger_mode;
  const bool changed_focus_pos = (config_.FocusPos != config.FocusPos);
  const bool changed_mtu = (config_.mtu != config.mtu);

  if (changed_auto_master)
  {
    setAutoMaster(config.AutoMaster);
  }

  if (changed_auto_slave)
  {
    setAutoSlave(config.AutoSlave);
  }

  // Set params into the camera.
  if (changed_exposure_time)
  {
    if (implemented_features_["ExposureTime"])
    {
      ROS_INFO("Set ExposureTime = %f us", config.ExposureTime);
      arv_camera_set_exposure_time(p_camera_, config.ExposureTime);
    }
    else
      ROS_INFO("Camera does not support ExposureTime.");
  }

  if (changed_gain)
  {
    if (implemented_features_["Gain"])
    {
      ROS_INFO("Set gain = %f", config.Gain);
      arv_camera_set_gain(p_camera_, config.Gain);
    }
    else
      ROS_INFO("Camera does not support Gain or GainRaw.");
  }

  if (changed_exposure_auto)
  {
    if (implemented_features_["ExposureAuto"] && implemented_features_["ExposureTime"])
    {
      ROS_INFO("Set ExposureAuto = %s", config.ExposureAuto.c_str());
      arv_device_set_string_feature_value(p_device_, "ExposureAuto", config.ExposureAuto.c_str());
      if (config.ExposureAuto.compare("Once") == 0)
      {
        ros::Duration(2.0).sleep();
        config.ExposureTime = arv_camera_get_exposure_time(p_camera_);
        ROS_INFO("Get ExposureTime = %f us", config.ExposureTime);
        config.ExposureAuto = "Off";
      }
    }
    else
      ROS_INFO("Camera does not support ExposureAuto.");
  }
  if (changed_gain_auto)
  {
    if (implemented_features_["GainAuto"] && implemented_features_["Gain"])
    {
      ROS_INFO("Set GainAuto = %s", config.GainAuto.c_str());
      arv_device_set_string_feature_value(p_device_, "GainAuto", config.GainAuto.c_str());
      if (config.GainAuto.compare("Once") == 0)
      {
        ros::Duration(2.0).sleep();
        config.Gain = arv_camera_get_gain(p_camera_);
        ROS_INFO("Get Gain = %f", config.Gain);
        config.GainAuto = "Off";
      }
    }
    else
      ROS_INFO("Camera does not support GainAuto.");
  }

  if (changed_acquisition_frame_rate)
  {
    if (implemented_features_["AcquisitionFrameRate"])
    {
      ROS_INFO("Set frame rate = %f Hz", config.AcquisitionFrameRate);
      arv_camera_set_frame_rate(p_camera_, config.AcquisitionFrameRate);
    }
    else
      ROS_INFO("Camera does not support AcquisitionFrameRate.");
  }

  if (changed_trigger_mode)
  {
    if (implemented_features_["TriggerMode"])
    {
      ROS_INFO("Set TriggerMode = %s", config.TriggerMode.c_str());
      arv_device_set_string_feature_value(p_device_, "TriggerMode", config.TriggerMode.c_str());
    }
    else
      ROS_INFO("Camera does not support TriggerMode.");
  }

  if (changed_trigger_source)
  {
    // delete old software trigger thread if active
    software_trigger_active_ = false;
    if (software_trigger_thread_.joinable())
    {
      software_trigger_thread_.join();
    }

    if (implemented_features_["TriggerSource"])
    {
      ROS_INFO("Set TriggerSource = %s", config.TriggerSource.c_str());
      arv_device_set_string_feature_value(p_device_, "TriggerSource", config.TriggerSource.c_str());
    }
    else
    {
      ROS_INFO("Camera does not support TriggerSource.");
    }

    // activate on demand
    if (config.TriggerMode.compare("On") == 0 && config.TriggerSource.compare("Software") == 0)
    {
      if (implemented_features_["TriggerSoftware"])
      {
        config_.softwaretriggerrate = config.softwaretriggerrate;
        ROS_INFO("Set softwaretriggerrate = %f", 1000.0 / ceil(1000.0 / config.softwaretriggerrate));

        // Turn on software timer callback.
        software_trigger_thread_ = std::thread(&CameraAravisNodelet::softwareTriggerLoop, this);
      }
      else
      {
        ROS_INFO("Camera does not support TriggerSoftware command.");
      }
    }
  }

  if (changed_focus_pos)
  {
    if (implemented_features_["FocusPos"])
    {
      ROS_INFO("Set FocusPos = %d", config.FocusPos);
      arv_device_set_integer_feature_value(p_device_, "FocusPos", config.FocusPos);
      ros::Duration(1.0).sleep();
      config.FocusPos = arv_device_get_integer_feature_value(p_device_, "FocusPos");
      ROS_INFO("Get FocusPos = %d", config.FocusPos);
    }
    else
      ROS_INFO("Camera does not support FocusPos.");
  }

  if (changed_mtu)
  {
    if (implemented_features_["GevSCPSPacketSize"])
    {
      ROS_INFO("Set mtu = %d", config.mtu);
      arv_device_set_integer_feature_value(p_device_, "GevSCPSPacketSize", config.mtu);
      ros::Duration(1.0).sleep();
      config.mtu = arv_device_get_integer_feature_value(p_device_, "GevSCPSPacketSize");
      ROS_INFO("Get mtu = %d", config.mtu);
    }
    else
      ROS_INFO("Camera does not support mtu (i.e. GevSCPSPacketSize).");
  }

  if (changed_acquisition_mode)
  {
    if (implemented_features_["AcquisitionMode"])
    {
      ROS_INFO("Set AcquisitionMode = %s", config.AcquisitionMode.c_str());
      arv_device_set_string_feature_value(p_device_, "AcquisitionMode", config.AcquisitionMode.c_str());

      ROS_INFO("AcquisitionStop");
      arv_device_execute_command(p_device_, "AcquisitionStop");
      ROS_INFO("AcquisitionStart");
      arv_device_execute_command(p_device_, "AcquisitionStart");
    }
    else
      ROS_INFO("Camera does not support AcquisitionMode.");
  }

  // adopt new config
  config_ = config;
  reconfigure_mutex_.unlock();
}

void CameraAravisNodelet::rosConnectCallback()
{
  if (p_device_)
  {
    if (std::all_of(cam_pubs_.begin(), cam_pubs_.end(),
      [](image_transport::CameraPublisher pub){ return pub.getNumSubscribers() == 0; })
    ){
      arv_device_execute_command(p_device_, "AcquisitionStop"); // don't waste CPU if nobody is listening!
    }
    else
    {
      arv_device_execute_command(p_device_, "AcquisitionStart");
    }
  }
}

void CameraAravisNodelet::newBufferReadyCallback(ArvStream *p_stream, gpointer can_instance)
{

  // workaround to get access to the instance from a static method
  StreamIdData *data = (StreamIdData*) can_instance;
  CameraAravisNodelet *p_can = (CameraAravisNodelet*) data->can;
  size_t stream_id = data->stream_id;
  image_transport::CameraPublisher *p_cam_pub = &p_can->cam_pubs_[stream_id];

  if( p_can->stream_names_[stream_id].empty() )
  {
    newBufferReady(p_stream, p_can, p_can->config_.frame_id, stream_id);
  } else {
    const std::string stream_frame_id = p_can->config_.frame_id + "/" + p_can->stream_names_[stream_id];
    newBufferReady(p_stream, p_can, stream_frame_id, stream_id);
  }

}

void CameraAravisNodelet::newBufferReady(ArvStream *p_stream, CameraAravisNodelet *p_can, std::string frame_id, size_t stream_id)
{
  ArvBuffer *p_buffer = arv_stream_try_pop_buffer(p_stream);

  // check if we risk to drop the next image because of not enough buffers left
  gint n_available_buffers;
  arv_stream_get_n_buffers(p_stream, &n_available_buffers, NULL);
  if (n_available_buffers == 0)
  {
    p_can->p_buffer_pools_[stream_id]->allocateBuffers(1);
  }

  if (p_buffer != NULL)
  {
    if (arv_buffer_get_status(p_buffer) == ARV_BUFFER_STATUS_SUCCESS && p_can->p_buffer_pools_[stream_id]
        && p_can->cam_pubs_[stream_id].getNumSubscribers())
    {
      (p_can->n_buffers_)++;
      // get the image message which wraps around this buffer
      sensor_msgs::ImagePtr msg_ptr = (*(p_can->p_buffer_pools_[stream_id]))[p_buffer];
      // fill the meta information of image message
      // get acquisition time
      guint64 t;
      if (p_can->use_ptp_stamp_)
        t = arv_buffer_get_timestamp(p_buffer);
      else
        t = arv_buffer_get_system_timestamp(p_buffer);
      msg_ptr->header.stamp.fromNSec(t);
      // get frame sequence number
      msg_ptr->header.seq = arv_buffer_get_frame_id(p_buffer);
      // fill other stream properties
      msg_ptr->header.frame_id = frame_id;
      msg_ptr->width = p_can->roi_.width;
      msg_ptr->height = p_can->roi_.height;
      msg_ptr->encoding = p_can->sensors_[stream_id]->pixel_format;
      msg_ptr->step = (msg_ptr->width * p_can->sensors_[stream_id]->n_bits_pixel)/8;

      // do the magic of conversion into a ROS format
      if (p_can->convert_format) {
        sensor_msgs::ImagePtr cvt_msg_ptr = p_can->p_buffer_pools_[stream_id]->getRecyclableImg();
        p_can->convert_format(msg_ptr, cvt_msg_ptr);
        msg_ptr = cvt_msg_ptr;
      }

      // get current CameraInfo data
      if (!p_can->camera_infos_[stream_id]) {
        p_can->camera_infos_[stream_id].reset(new sensor_msgs::CameraInfo);
      }
      (*p_can->camera_infos_[stream_id]) = p_can->p_camera_info_managers_[stream_id]->getCameraInfo();
      p_can->camera_infos_[stream_id]->header = msg_ptr->header;
      p_can->camera_infos_[stream_id]->width = p_can->roi_.width;
      p_can->camera_infos_[stream_id]->height = p_can->roi_.height;

      p_can->cam_pubs_[stream_id].publish(msg_ptr, p_can->camera_infos_[stream_id]);

      // check PTP status, camera cannot recover from "Faulty" by itself
      if (p_can->use_ptp_stamp_)
        p_can->resetPtpClock();
    }
    else
    {
      ROS_WARN("(%s) Frame error: %s", frame_id.c_str(), szBufferStatusFromInt[arv_buffer_get_status(p_buffer)]);

      arv_stream_push_buffer(p_stream, p_buffer);
    }
  }

  // publish current lighting settings if this camera is configured as master
  if (p_can->config_.AutoMaster)
  {
    p_can->syncAutoParameters();
    p_can->auto_pub_.publish(p_can->auto_params_);
  }
}

void CameraAravisNodelet::controlLostCallback(ArvDevice *p_gv_device, gpointer can_instance)
{
  CameraAravisNodelet *p_can = (CameraAravisNodelet*)can_instance;
  ROS_ERROR("Control to aravis device lost.");
  nodelet::NodeletUnload unload_service;
  unload_service.request.name = p_can->getName();
  if (false == ros::service::call(ros::this_node::getName() + "/unload_nodelet", unload_service))
  {
    ros::shutdown();
  }
}

void CameraAravisNodelet::softwareTriggerLoop()
{
  software_trigger_active_ = true;
  ROS_INFO("Software trigger started.");
  std::chrono::system_clock::time_point next_time = std::chrono::system_clock::now();
  while (ros::ok() && software_trigger_active_)
  {
    next_time += std::chrono::milliseconds(size_t(std::round(1000.0 / config_.softwaretriggerrate)));
    if (std::any_of(cam_pubs_.begin(), cam_pubs_.end(),
      [](image_transport::CameraPublisher pub){ return pub.getNumSubscribers() > 0; })
    )
    {
      arv_device_execute_command(p_device_, "TriggerSoftware");
    }
    if (next_time > std::chrono::system_clock::now())
    {
      std::this_thread::sleep_until(next_time);
    }
    else
    {
      ROS_WARN("Camera Aravis: Missed a software trigger event.");
      next_time = std::chrono::system_clock::now();
    }
  }
  ROS_INFO("Software trigger stopped.");
}

void CameraAravisNodelet::publishTfLoop(double rate)
{
  // Publish optical transform for the camera
  ROS_WARN("Publishing dynamic camera transforms (/tf) at %g Hz", rate);

  tf_thread_active_ = true;

  ros::Rate loop_rate(rate);

  while (ros::ok() && tf_thread_active_)
  {
    // Update the header for publication
    tf_optical_.header.stamp = ros::Time::now();
    ++tf_optical_.header.seq;
    p_tb_->sendTransform(tf_optical_);

    loop_rate.sleep();
  }
}

void CameraAravisNodelet::discoverFeatures()
{
  implemented_features_.clear();
  if (!p_device_)
    return;

  // get the root node of genicam description
  ArvGc *gc = arv_device_get_genicam(p_device_);
  if (!gc)
    return;

  std::list<ArvDomNode*> todo;
  todo.push_front((ArvDomNode*)arv_gc_get_node(gc, "Root"));
  GError *error = NULL;

  while (!todo.empty())
  {
    // get next entry
    ArvDomNode *node = todo.front();
    todo.pop_front();
    const std::string name(arv_dom_node_get_node_name(node));

    // Do the indirection
    if (name[0] == 'p')
    {
      if (name.compare("pInvalidator") == 0)
      {
        continue;
      }
      ArvDomNode *inode = (ArvDomNode*)arv_gc_get_node(gc,
                                                       arv_dom_node_get_node_value(arv_dom_node_get_first_child(node)));
      if (inode)
        todo.push_front(inode);
      continue;
    }

    // check for implemented feature
    if (ARV_IS_GC_FEATURE_NODE(node))
    {
      //if (!(ARV_IS_GC_CATEGORY(node) || ARV_IS_GC_ENUM_ENTRY(node) /*|| ARV_IS_GC_PORT(node)*/)) {
      ArvGcFeatureNode *fnode = ARV_GC_FEATURE_NODE(node);
      const std::string fname(arv_gc_feature_node_get_name(fnode));
      const bool usable = arv_gc_feature_node_is_available(fnode, &error)
          && arv_gc_feature_node_is_implemented(fnode, &error);
      ROS_INFO_STREAM("Feature " << fname << " is " << usable);
      implemented_features_.emplace(fname, usable);
      //}
    }

//		if (ARV_IS_GC_PROPERTY_NODE(node)) {
//			ArvGcPropertyNode* pnode = ARV_GC_PROPERTY_NODE(node);
//			const std::string pname(arv_gc_property_node_get_name(pnode));
//			ROS_INFO_STREAM("Property " << pname << " found");
//		}

    // add children in todo-list
    ArvDomNodeList *children = arv_dom_node_get_child_nodes(node);
    const uint l = arv_dom_node_list_get_length(children);
    for (uint i = 0; i < l; ++i)
    {
      todo.push_front(arv_dom_node_list_get_item(children, i));
    }
  }
}

void CameraAravisNodelet::parseStringArgs(std::string in_arg_string, std::vector<std::string> &out_args) {
  size_t array_start = 0;
  size_t array_end = in_arg_string.length();
  if(array_start != std::string::npos && array_end != std::string::npos) {
        // parse the string into an array of parameters
        std::stringstream ss(in_arg_string.substr(array_start, array_end - array_start));
        while (ss.good()) {
          std::string temp;
          getline( ss, temp, ',');
          boost::trim_left(temp);
          boost::trim_right(temp);
          out_args.push_back(temp);
        }
  } else {
    // add just the one argument onto the vector
    out_args.push_back(in_arg_string);
  }
}

// WriteCameraFeaturesFromRosparam()
// Read ROS parameters from this node's namespace, and see if each parameter has a similarly named & typed feature in the camera.  Then set the
// camera feature to that value.  For example, if the parameter camnode/Gain is set to 123.0, then we'll write 123.0 to the Gain feature
// in the camera.
//
// Note that the datatype of the parameter *must* match the datatype of the camera feature, and this can be determined by
// looking at the camera's XML file.  Camera enum's are string parameters, camera bools are false/true parameters (not 0/1),
// integers are integers, doubles are doubles, etc.
//
void CameraAravisNodelet::writeCameraFeaturesFromRosparam()
{
  XmlRpc::XmlRpcValue xml_rpc_params;
  XmlRpc::XmlRpcValue::iterator iter;
  ArvGcNode *p_gc_node;
  GError *error = NULL;

  getPrivateNodeHandle().getParam(this->getName(), xml_rpc_params);

  if (xml_rpc_params.getType() == XmlRpc::XmlRpcValue::TypeStruct)
  {
    for (iter = xml_rpc_params.begin(); iter != xml_rpc_params.end(); iter++)
    {
      std::string key = iter->first;

      p_gc_node = arv_device_get_feature(p_device_, key.c_str());
      if (p_gc_node && arv_gc_feature_node_is_implemented(ARV_GC_FEATURE_NODE(p_gc_node), &error))
      {
        //				unsigned long	typeValue = arv_gc_feature_node_get_value_type((ArvGcFeatureNode *)pGcNode);
        //				ROS_INFO("%s cameratype=%lu, rosparamtype=%d", key.c_str(), typeValue, static_cast<int>(iter->second.getType()));

        // We'd like to check the value types too, but typeValue is often given as G_TYPE_INVALID, so ignore it.
        switch (iter->second.getType())
        {
          case XmlRpc::XmlRpcValue::TypeBoolean: //if ((iter->second.getType()==XmlRpc::XmlRpcValue::TypeBoolean))// && (typeValue==G_TYPE_INT64))
          {
            int value = (bool)iter->second;
            arv_device_set_integer_feature_value(p_device_, key.c_str(), value);
            ROS_INFO("Read parameter (bool) %s: %d", key.c_str(), value);
          }
            break;

          case XmlRpc::XmlRpcValue::TypeInt: //if ((iter->second.getType()==XmlRpc::XmlRpcValue::TypeInt))// && (typeValue==G_TYPE_INT64))
          {
            int value = (int)iter->second;
            arv_device_set_integer_feature_value(p_device_, key.c_str(), value);
            ROS_INFO("Read parameter (int) %s: %d", key.c_str(), value);
          }
            break;

          case XmlRpc::XmlRpcValue::TypeDouble: //if ((iter->second.getType()==XmlRpc::XmlRpcValue::TypeDouble))// && (typeValue==G_TYPE_DOUBLE))
          {
            double value = (double)iter->second;
            arv_device_set_float_feature_value(p_device_, key.c_str(), value);
            ROS_INFO("Read parameter (float) %s: %f", key.c_str(), value);
          }
            break;

          case XmlRpc::XmlRpcValue::TypeString: //if ((iter->second.getType()==XmlRpc::XmlRpcValue::TypeString))// && (typeValue==G_TYPE_STRING))
          {
            std::string value = (std::string)iter->second;
            arv_device_set_string_feature_value(p_device_, key.c_str(), value.c_str());
            ROS_INFO("Read parameter (string) %s: %s", key.c_str(), value.c_str());
          }
            break;

          case XmlRpc::XmlRpcValue::TypeInvalid:
          case XmlRpc::XmlRpcValue::TypeDateTime:
          case XmlRpc::XmlRpcValue::TypeBase64:
          case XmlRpc::XmlRpcValue::TypeArray:
          case XmlRpc::XmlRpcValue::TypeStruct:
          default:
            ROS_WARN("Unhandled rosparam type in writeCameraFeaturesFromRosparam()");
        }
      }
    }
  }
}

} // end namespace camera_aravis
