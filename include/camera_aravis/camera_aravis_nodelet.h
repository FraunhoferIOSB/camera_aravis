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

#ifndef INCLUDE_CAMERA_ARAVIS_CAMERA_ARAVIS_NODELET_H_
#define INCLUDE_CAMERA_ARAVIS_CAMERA_ARAVIS_NODELET_H_

extern "C" {
#include <arv.h>
}

#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <algorithm>
#include <functional>
#include <cctype>
#include <memory>
#include <atomic>
#include <thread>
#include <chrono>
#include <unordered_map>

#include <glib.h>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <nodelet/NodeletUnload.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int64.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <boost/algorithm/string/trim.hpp>

#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/SensorLevels.h>
#include <tf/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <camera_aravis/CameraAravisConfig.h>
#include <camera_aravis/CameraAutoInfo.h>

#include "camera_buffer_pool.h"

namespace camera_aravis
{

typedef CameraAravisConfig Config;

// Conversions from integers to Arv types.
const char *szBufferStatusFromInt[] = {"ARV_BUFFER_STATUS_SUCCESS", "ARV_BUFFER_STATUS_CLEARED",
                                       "ARV_BUFFER_STATUS_TIMEOUT", "ARV_BUFFER_STATUS_MISSING_PACKETS",
                                       "ARV_BUFFER_STATUS_WRONG_PACKET_ID", "ARV_BUFFER_STATUS_SIZE_MISMATCH",
                                       "ARV_BUFFER_STATUS_FILLING", "ARV_BUFFER_STATUS_ABORTED"};

// Conversion functions from Genicam to ROS formats
typedef std::function<void(sensor_msgs::ImagePtr& in, sensor_msgs::ImagePtr& out)> ConversionFunction;
void renameImg(sensor_msgs::ImagePtr& in, sensor_msgs::ImagePtr& out, const std::string out_format);
void shiftImg(sensor_msgs::ImagePtr& in, sensor_msgs::ImagePtr& out, const size_t n_digits, const std::string out_format);
void interleaveImg(sensor_msgs::ImagePtr& in, sensor_msgs::ImagePtr& out, const size_t n_digits, const std::string out_format);
void unpack10p32Img(sensor_msgs::ImagePtr& in, sensor_msgs::ImagePtr& out, const std::string out_format);
void unpack10PackedImg(sensor_msgs::ImagePtr& in, sensor_msgs::ImagePtr& out, const std::string out_format);
void unpack10pMonoImg(sensor_msgs::ImagePtr& in, sensor_msgs::ImagePtr& out, const std::string out_format);
void unpack10PackedMonoImg(sensor_msgs::ImagePtr& in, sensor_msgs::ImagePtr& out, const std::string out_format);
void unpack12pImg(sensor_msgs::ImagePtr& in, sensor_msgs::ImagePtr& out, const std::string out_format);
void unpack12PackedImg(sensor_msgs::ImagePtr& in, sensor_msgs::ImagePtr& out, const std::string out_format);
void unpack565pImg(sensor_msgs::ImagePtr& in, sensor_msgs::ImagePtr& out, const std::string out_format);

const std::map<std::string, ConversionFunction> CONVERSIONS_DICTIONARY =
{
 // equivalent to official ROS color encodings
 { "RGB8", boost::bind(&renameImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::RGB8) },
 { "RGBa8", boost::bind(&renameImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::RGBA8) },
 { "RGB16", boost::bind(&renameImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::RGB16) },
 { "RGBa16", boost::bind(&renameImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::RGBA16) },
 { "BGR8", boost::bind(&renameImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::BGR8) },
 { "BGRa8", boost::bind(&renameImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::BGRA8) },
 { "BGR16", boost::bind(&renameImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::BGR16) },
 { "BGRa16", boost::bind(&renameImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::BGRA16) },
 { "Mono8", boost::bind(&renameImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::MONO8) },
 { "Raw8", boost::bind(&renameImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::MONO8) },
 { "R8", boost::bind(&renameImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::MONO8) },
 { "G8", boost::bind(&renameImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::MONO8) },
 { "B8", boost::bind(&renameImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::MONO8) },
 { "Mono16", boost::bind(&renameImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::MONO16) },
 { "Raw16", boost::bind(&renameImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::MONO16) },
 { "R16", boost::bind(&renameImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::MONO16) },
 { "G16", boost::bind(&renameImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::MONO16) },
 { "B16", boost::bind(&renameImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::MONO16) },
 { "BayerRG8", boost::bind(&renameImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::BAYER_RGGB8) },
 { "BayerBG8", boost::bind(&renameImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::BAYER_BGGR8) },
 { "BayerGB8", boost::bind(&renameImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::BAYER_GBRG8) },
 { "BayerGR8", boost::bind(&renameImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::BAYER_GRBG8) },
 { "BayerRG16", boost::bind(&renameImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::BAYER_RGGB16) },
 { "BayerBG16", boost::bind(&renameImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::BAYER_BGGR16) },
 { "BayerGB16", boost::bind(&renameImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::BAYER_GBRG16) },
 { "BayerGR16", boost::bind(&renameImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::BAYER_GRBG16) },
 { "YUV422_8_UYVY", boost::bind(&renameImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::YUV422) },
 { "YUV422_8", boost::bind(&renameImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::YUV422) },
 // non-color contents
 { "Data8", boost::bind(&renameImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::TYPE_8UC1) },
 { "Confidence8", boost::bind(&renameImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::TYPE_8UC1) },
 { "Data8s", boost::bind(&renameImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::TYPE_8SC1) },
 { "Data16", boost::bind(&renameImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::TYPE_16UC1) },
 { "Confidence16", boost::bind(&renameImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::TYPE_16UC1) },
 { "Data16s", boost::bind(&renameImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::TYPE_16SC1) },
 { "Data32s", boost::bind(&renameImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::TYPE_32SC1) },
 { "Data32f", boost::bind(&renameImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::TYPE_32FC1) },
 { "Confidence32f", boost::bind(&renameImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::TYPE_32FC1) },
 { "Data64f", boost::bind(&renameImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::TYPE_64FC1) },
 // unthrifty formats. Shift away padding Bits for use with ROS.
 { "Mono10", boost::bind(&shiftImg, boost::placeholders::_1, boost::placeholders::_2, 6, sensor_msgs::image_encodings::MONO16) },
 { "Mono12", boost::bind(&shiftImg, boost::placeholders::_1, boost::placeholders::_2, 4, sensor_msgs::image_encodings::MONO16) },
 { "Mono14", boost::bind(&shiftImg, boost::placeholders::_1, boost::placeholders::_2, 2, sensor_msgs::image_encodings::MONO16) },
 { "RGB10", boost::bind(&shiftImg, boost::placeholders::_1, boost::placeholders::_2, 6, sensor_msgs::image_encodings::RGB16) },
 { "RGB12", boost::bind(&shiftImg, boost::placeholders::_1, boost::placeholders::_2, 4, sensor_msgs::image_encodings::RGB16) },
 { "BGR10", boost::bind(&shiftImg, boost::placeholders::_1, boost::placeholders::_2, 6, sensor_msgs::image_encodings::BGR16) },
 { "BGR12", boost::bind(&shiftImg, boost::placeholders::_1, boost::placeholders::_2, 4, sensor_msgs::image_encodings::BGR16) },
 { "BayerRG10", boost::bind(&shiftImg, boost::placeholders::_1, boost::placeholders::_2, 6, sensor_msgs::image_encodings::BAYER_RGGB16) },
 { "BayerBG10", boost::bind(&shiftImg, boost::placeholders::_1, boost::placeholders::_2, 6, sensor_msgs::image_encodings::BAYER_BGGR16) },
 { "BayerGB10", boost::bind(&shiftImg, boost::placeholders::_1, boost::placeholders::_2, 6, sensor_msgs::image_encodings::BAYER_GBRG16) },
 { "BayerGR10", boost::bind(&shiftImg, boost::placeholders::_1, boost::placeholders::_2, 6, sensor_msgs::image_encodings::BAYER_GRBG16) },
 { "BayerRG12", boost::bind(&shiftImg, boost::placeholders::_1, boost::placeholders::_2, 4, sensor_msgs::image_encodings::BAYER_RGGB16) },
 { "BayerBG12", boost::bind(&shiftImg, boost::placeholders::_1, boost::placeholders::_2, 4, sensor_msgs::image_encodings::BAYER_BGGR16) },
 { "BayerGB12", boost::bind(&shiftImg, boost::placeholders::_1, boost::placeholders::_2, 4, sensor_msgs::image_encodings::BAYER_GBRG16) },
 { "BayerGR12", boost::bind(&shiftImg, boost::placeholders::_1, boost::placeholders::_2, 4, sensor_msgs::image_encodings::BAYER_GRBG16) },
 // planar instead pixel-by-pixel encodings
 { "RGB8_Planar", boost::bind(&interleaveImg, boost::placeholders::_1, boost::placeholders::_2, 0, sensor_msgs::image_encodings::RGB8) },
 { "RGB10_Planar", boost::bind(&interleaveImg, boost::placeholders::_1, boost::placeholders::_2, 6, sensor_msgs::image_encodings::RGB16) },
 { "RGB12_Planar", boost::bind(&interleaveImg, boost::placeholders::_1, boost::placeholders::_2, 4, sensor_msgs::image_encodings::RGB16) },
 { "RGB16_Planar", boost::bind(&interleaveImg, boost::placeholders::_1, boost::placeholders::_2, 0, sensor_msgs::image_encodings::RGB16) },
 // packed, non-Byte aligned formats
 { "Mono10p", boost::bind(&unpack10pMonoImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::MONO16) },
 { "RGB10p", boost::bind(&unpack10p32Img, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::RGB16) },
 { "RGB10p32", boost::bind(&unpack10p32Img, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::RGB16) },
 { "RGBa10p", boost::bind(&unpack10p32Img, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::RGBA16) },
 { "BGR10p", boost::bind(&unpack10p32Img, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::BGR16) },
 { "BGRa10p", boost::bind(&unpack10p32Img, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::BGRA16) },
 { "BayerRG10p", boost::bind(&unpack10pMonoImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::BAYER_RGGB16) },
 { "BayerBG10p", boost::bind(&unpack10pMonoImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::BAYER_BGGR16) },
 { "BayerGB10p", boost::bind(&unpack10pMonoImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::BAYER_GBRG16) },
 { "BayerGR10p", boost::bind(&unpack10pMonoImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::BAYER_GRBG16) },
 { "Mono12p", boost::bind(&unpack12pImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::MONO16) },
 { "RGB12p", boost::bind(&unpack12pImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::RGB16) },
 { "RGBa12p", boost::bind(&unpack12pImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::RGBA16) },
 { "BGR12p", boost::bind(&unpack12pImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::BGR16) },
 { "BGRa12p", boost::bind(&unpack12pImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::BGRA16) },
 { "BayerRG12p", boost::bind(&unpack12pImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::BAYER_RGGB16) },
 { "BayerBG12p", boost::bind(&unpack12pImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::BAYER_BGGR16) },
 { "BayerGB12p", boost::bind(&unpack12pImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::BAYER_GBRG16) },
 { "BayerGR12p", boost::bind(&unpack12pImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::BAYER_GRBG16) },
 { "RGB565p", boost::bind(&unpack565pImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::RGB8) },
 { "BGR565p", boost::bind(&unpack565pImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::BGR8) },
 // GigE-Vision specific format naming
 { "RGB10V1Packed", boost::bind(&unpack10PackedImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::RGB16) },
 { "RGB10V2Packed", boost::bind(&unpack10p32Img, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::RGB16) },
 { "RGB12V1Packed", boost::bind(&unpack12PackedImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::RGB16) },
 { "Mono10Packed", boost::bind(&unpack10PackedMonoImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::MONO16) },
 { "Mono12Packed", boost::bind(&unpack12PackedImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::MONO16) },
 { "BayerRG10Packed", boost::bind(&unpack10PackedMonoImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::BAYER_RGGB16) },
 { "BayerBG10Packed", boost::bind(&unpack10PackedMonoImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::BAYER_BGGR16) },
 { "BayerGB10Packed", boost::bind(&unpack10PackedMonoImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::BAYER_GBRG16) },
 { "BayerGR10Packed", boost::bind(&unpack10PackedMonoImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::BAYER_GRBG16) },
 { "BayerRG12Packed", boost::bind(&unpack12PackedImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::BAYER_RGGB16) },
 { "BayerBG12Packed", boost::bind(&unpack12PackedImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::BAYER_BGGR16) },
 { "BayerGB12Packed", boost::bind(&unpack12PackedImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::BAYER_GBRG16) },
 { "BayerGR12Packed", boost::bind(&unpack12PackedImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::BAYER_GRBG16) },
 { "YUV422Packed", boost::bind(&renameImg, boost::placeholders::_1, boost::placeholders::_2, sensor_msgs::image_encodings::YUV422) }
};

class CameraAravisNodelet : public nodelet::Nodelet
{
public:
  CameraAravisNodelet();
  virtual ~CameraAravisNodelet();

private:
  virtual void onInit() override;

protected:
  // apply auto functions from a ros message
  void cameraAutoInfoCallback(const CameraAutoInfoConstPtr &msg_ptr);

  void syncAutoParameters();
  void setAutoMaster(bool value);
  void setAutoSlave(bool value);

  // Extra stream options for GigEVision streams.
  void tuneGvStream(ArvGvStream *p_stream);

  void rosReconfigureCallback(Config &config, uint32_t level);

  // Start and stop camera on demand
  void rosConnectCallback();

  // Callback to wrap and send recorded image as ROS message
  static void newBufferReadyCallback(ArvStream *p_stream, gpointer can_instance);

  // Buffer Callback Helper
  static void newBufferReady(ArvStream *p_stream, CameraAravisNodelet *p_can, std::string frame_id, size_t stream_id);

  // Clean-up if aravis device is lost
  static void controlLostCallback(ArvDevice *p_gv_device, gpointer can_instance);

  // triggers a shot at regular intervals, sleeps in between
  void softwareTriggerLoop();

  void publishTfLoop(double rate);

  void discoverFeatures();

  static void parseStringArgs(std::string in_arg_string, std::vector<std::string> &out_args);

  // WriteCameraFeaturesFromRosparam()
  // Read ROS parameters from this node's namespace, and see if each parameter has a similarly named & typed feature in the camera.  Then set the
  // camera feature to that value.  For example, if the parameter camnode/Gain is set to 123.0, then we'll write 123.0 to the Gain feature
  // in the camera.
  //
  // Note that the datatype of the parameter *must* match the datatype of the camera feature, and this can be determined by
  // looking at the camera's XML file.  Camera enum's are string parameters, camera bools are false/true parameters (not 0/1),
  // integers are integers, doubles are doubles, etc.
  void writeCameraFeaturesFromRosparam();

  std::unique_ptr<dynamic_reconfigure::Server<Config> > reconfigure_server_;
  boost::recursive_mutex reconfigure_mutex_;

  std::vector<image_transport::CameraPublisher> cam_pubs_;
  std::vector<std::unique_ptr<camera_info_manager::CameraInfoManager>> p_camera_info_managers_;
  std::vector<sensor_msgs::CameraInfoPtr> camera_infos_;

  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> p_stb_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> p_tb_;
  geometry_msgs::TransformStamped tf_optical_;
  std::thread tf_dyn_thread_;
  std::atomic_bool tf_thread_active_;

  CameraAutoInfo auto_params_;
  ros::Publisher auto_pub_;
  ros::Subscriber auto_sub_;

  Config config_;
  Config config_min_;
  Config config_max_;

  std::thread software_trigger_thread_;
  std::atomic_bool software_trigger_active_;
  size_t n_buffers_ = 0;

  std::unordered_map<std::string, const bool> implemented_features_;

  struct
  {
    int32_t x = 0;
    int32_t y = 0;
    int32_t width = 0;
    int32_t width_min = 0;
    int32_t width_max = 0;
    int32_t height = 0;
    int32_t height_min = 0;
    int32_t height_max = 0;
  } roi_;

  struct Sensor
  {
    int32_t width = 0;
    int32_t height = 0;
    std::string pixel_format;
    size_t n_bits_pixel = 0;
  };

  std::vector<Sensor *> sensors_;

  struct StreamIdData
  {
    CameraAravisNodelet* can;
    size_t stream_id;
  };

  ArvCamera *p_camera_ = NULL;
  ArvDevice *p_device_ = NULL;
  gint num_streams_;
  std::vector<ArvStream *> p_streams_;
  std::vector<std::string> stream_names_;
  std::vector<CameraBufferPool::Ptr> p_buffer_pools_;
  int32_t acquire_ = 0;
  ConversionFunction convert_format;
};

} // end namespace camera_aravis

#endif /* INCLUDE_CAMERA_ARAVIS_CAMERA_ARAVIS_NODELET_H_ */
