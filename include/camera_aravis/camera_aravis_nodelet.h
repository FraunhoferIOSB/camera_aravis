/****************************************************************************
 *
 * camera_aravis
 *
 * Copyright © 2022 Fraunhofer IOSB and contributors
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 51 Franklin St, Fifth Floor,
 * Boston, MA  02110-1301, USA.
 *
 ****************************************************************************/

#ifndef CAMERA_ARAVIS_CAMERA_ARAVIS_NODELET
#define CAMERA_ARAVIS_CAMERA_ARAVIS_NODELET

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
#include <camera_aravis/ExtendedCameraInfo.h>

#include <camera_aravis/get_integer_feature_value.h>
#include <camera_aravis/set_integer_feature_value.h>
#include <camera_aravis/get_float_feature_value.h>
#include <camera_aravis/set_float_feature_value.h>
#include <camera_aravis/get_string_feature_value.h>
#include <camera_aravis/set_string_feature_value.h>
#include <camera_aravis/get_boolean_feature_value.h>
#include <camera_aravis/set_boolean_feature_value.h>

#include <camera_aravis/camera_buffer_pool.h>
#include <camera_aravis/conversion_utils.h>

namespace camera_aravis
{

typedef CameraAravisConfig Config;

class CameraAravisNodelet : public nodelet::Nodelet
{
public:
  CameraAravisNodelet();
  virtual ~CameraAravisNodelet();

private:
  bool verbose_ = false;
  std::string guid_ = "";
  bool use_ptp_stamp_ = false;
  bool pub_ext_camera_info_ = false;
  bool pub_tf_optical_ = false;

  ArvCamera *p_camera_ = NULL;
  ArvDevice *p_device_ = NULL;

  gint num_streams_ = 0;
  std::vector<ArvStream *> p_streams_;
  std::vector<std::string> stream_names_;
  std::vector<CameraBufferPool::Ptr> p_buffer_pools_;
  int32_t acquire_ = 0;
  std::vector<ConversionFunction> convert_formats;

  virtual void onInit() override;
  void spawnStream();


protected:
  // reset PTP clock
  void resetPtpClock();

  // apply auto functions from a ros message
  void cameraAutoInfoCallback(const CameraAutoInfoConstPtr &msg_ptr);

  void syncAutoParameters();
  void setAutoMaster(bool value);
  void setAutoSlave(bool value);

  void setExtendedCameraInfo(std::string channel_name, size_t stream_id);
  void fillExtendedCameraInfoMessage(ExtendedCameraInfo &msg);

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

  // Services
  ros::ServiceServer get_integer_service_;
  bool getIntegerFeatureCallback(camera_aravis::get_integer_feature_value::Request& request, camera_aravis::get_integer_feature_value::Response& response);

  ros::ServiceServer get_float_service_;
  bool getFloatFeatureCallback(camera_aravis::get_float_feature_value::Request& request, camera_aravis::get_float_feature_value::Response& response);

  ros::ServiceServer get_string_service_;
  bool getStringFeatureCallback(camera_aravis::get_string_feature_value::Request& request, camera_aravis::get_string_feature_value::Response& response);

  ros::ServiceServer get_boolean_service_;
  bool getBooleanFeatureCallback(camera_aravis::get_boolean_feature_value::Request& request, camera_aravis::get_boolean_feature_value::Response& response);

  ros::ServiceServer set_integer_service_;
  bool setIntegerFeatureCallback(camera_aravis::set_integer_feature_value::Request& request, camera_aravis::set_integer_feature_value::Response& response);

  ros::ServiceServer set_float_service_;
  bool setFloatFeatureCallback(camera_aravis::set_float_feature_value::Request& request, camera_aravis::set_float_feature_value::Response& response);

  ros::ServiceServer set_string_service_;
  bool setStringFeatureCallback(camera_aravis::set_string_feature_value::Request& request, camera_aravis::set_string_feature_value::Response& response);

  ros::ServiceServer set_boolean_service_;
  bool setBooleanFeatureCallback(camera_aravis::set_boolean_feature_value::Request& request, camera_aravis::set_boolean_feature_value::Response& response);

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
  std::vector<std::unique_ptr<ros::NodeHandle>> p_camera_info_node_handles_;
  std::vector<sensor_msgs::CameraInfoPtr> camera_infos_;

  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> p_stb_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> p_tb_;
  geometry_msgs::TransformStamped tf_optical_;
  std::thread tf_dyn_thread_;
  std::atomic_bool tf_thread_active_;

  CameraAutoInfo auto_params_;
  ros::Publisher auto_pub_;
  ros::Subscriber auto_sub_;

  boost::recursive_mutex extended_camera_info_mutex_;
  std::vector<ros::Publisher> extended_camera_info_pubs_;

  Config config_;
  Config config_min_;
  Config config_max_;

  std::atomic<bool> spawning_;
  std::thread       spawn_stream_thread_;

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
};

} // end namespace camera_aravis

#endif /* CAMERA_ARAVIS_CAMERA_ARAVIS_NODELET */
