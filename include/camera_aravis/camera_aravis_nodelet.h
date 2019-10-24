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

extern "C"
{
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

#include <dynamic_reconfigure/server.h>
#include <driver_base/SensorLevels.h>
#include <tf/transform_listener.h>
#include <camera_aravis/CameraAravisConfig.h>

#include "camera_buffer_pool.h"

#include <XmlRpc.h>

namespace camera_aravis
{

typedef CameraAravisConfig Config;

struct NODEEX
{
	const char *sz_name;
	const char *sz_tag;
	ArvDomNode *p_node;
	ArvDomNode *p_node_sibling;
};

#define ARV_PIXEL_FORMAT_BYTE_PER_PIXEL(pixel_format) ((((pixel_format) >> 16) & 0xff) >> 3)

// Conversions from integers to Arv types.
const char	*szBufferStatusFromInt[] = {
										"ARV_BUFFER_STATUS_SUCCESS",
										"ARV_BUFFER_STATUS_CLEARED",
										"ARV_BUFFER_STATUS_TIMEOUT",
										"ARV_BUFFER_STATUS_MISSING_PACKETS",
										"ARV_BUFFER_STATUS_WRONG_PACKET_ID",
										"ARV_BUFFER_STATUS_SIZE_MISMATCH",
										"ARV_BUFFER_STATUS_FILLING",
										"ARV_BUFFER_STATUS_ABORTED"
										};

class CameraAravisNodelet : public nodelet::Nodelet
{
public:
	CameraAravisNodelet();
	virtual ~CameraAravisNodelet();

private:
	virtual void onInit() override;

protected:
	// Extra stream options for GigEVision streams.
	void tuneGvStream(ArvGvStream* p_stream);

	void rosReconfigureCallback(Config &config, uint32_t level);

	void rosConnectCallback();

	static void newBufferReadyCallback(ArvStream *p_stream, gpointer can_instance);

	static void controlLostCallback (ArvDevice *p_gv_device, gpointer can_instance);

	void softwareTriggerThread();

	// Get the child and the child's sibling, where <p___> indicates an indirection.
	static NODEEX getGcFirstChild(ArvGc *p_genicam, NODEEX nodeex);

	// Get the sibling and the sibling's sibling, where <p___> indicates an indirection.
	static NODEEX getGcNextSibling(ArvGc *p_genicam, NODEEX nodeex);

	// Walk the DOM tree, i.e. the tree represented by the XML file in the camera, and that contains all the various features, parameters, etc.
	static void printDOMTree(ArvGc *p_genicam, NODEEX nodeex, const int n_indent);

	// WriteCameraFeaturesFromRosparam()
	// Read ROS parameters from this node's namespace, and see if each parameter has a similarly named & typed feature in the camera.  Then set the
	// camera feature to that value.  For example, if the parameter camnode/Gain is set to 123.0, then we'll write 123.0 to the Gain feature
	// in the camera.
	//
	// Note that the datatype of the parameter *must* match the datatype of the camera feature, and this can be determined by
	// looking at the camera's XML file.  Camera enum's are string parameters, camera bools are false/true parameters (not 0/1),
	// integers are integers, doubles are doubles, etc.
	void writeCameraFeaturesFromRosparam();

	std::unique_ptr<dynamic_reconfigure::Server<Config> >	reconfigure_server_;
	boost::recursive_mutex									reconfigure_mutex_;

	image_transport::CameraPublisher 						publisher_;
	std::unique_ptr<camera_info_manager::CameraInfoManager> p_camera_info_manager_;
	sensor_msgs::CameraInfoPtr 								camera_info_;

	Config 									config_;
	Config 									config_min_;
	Config 									config_max_;

	std::thread								software_trigger_;
	std::atomic_bool						software_trigger_active_;
	size_t									n_buffers_ = 0;

	struct {
		bool								acquisition_frame_rate			= false;
		bool								acquisition_frame_rate_enable	= false;
		bool								gain							= false;
		bool								exposure_time					= false;
		bool								exposure_auto					= false;
		bool								gain_auto						= false;
		bool								focus_pos						= false;
		bool								trigger_selector				= false;
		bool								trigger_source					= false;
		bool								trigger_mode					= false;
		bool								acquisition_mode				= false;
		bool								mtu								= false;
	} is_implemented_;

	struct {
		int32_t    							x							= 0;
		int32_t        						y							= 0;
		int32_t        						width						= 0;
		int32_t								width_min					= 0;
		int32_t								width_max					= 0;
		int32_t        						height						= 0;
		int32_t								height_min					= 0;
		int32_t								height_max					= 0;
	} roi_;

	struct {
		int32_t                             width						= 0;
		int32_t                             height						= 0;
		std::string					        sz_pixel_format;
		size_t								n_bytes_pixel				= 0;
	} sensor_;


	ros::NodeHandle 					    p_h_node_;
	ArvCamera 							   *p_camera_					= NULL;
	ArvDevice 							   *p_device_					= NULL;
	ArvStream                              *p_stream_					= NULL;
	CameraBufferPool::Ptr					p_buffer_pool_;
	int32_t									mtu_						= 0;
	int32_t									acquire_					= 0;
	std::string							    key_acquisition_frame_rate_;
};

} // end namespace camera_aravis


#endif /* INCLUDE_CAMERA_ARAVIS_CAMERA_ARAVIS_NODELET_H_ */
