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

CameraAravisNodelet::CameraAravisNodelet() {
}

CameraAravisNodelet::~CameraAravisNodelet() {
	if (p_stream_) {
		arv_stream_set_emit_signals(p_stream_, FALSE);
	}

	software_trigger_active_ = false;
	if (software_trigger_.joinable()) {
		software_trigger_.join();
	}

	guint64 n_completed_buffers;
	guint64 n_failures;
	guint64 n_underruns;
	arv_stream_get_statistics(p_stream_, &n_completed_buffers, &n_failures, &n_underruns);
	ROS_INFO ("Completed buffers = %Lu", (unsigned long long) n_completed_buffers);
	ROS_INFO ("Failures          = %Lu", (unsigned long long) n_failures);
	ROS_INFO ("Underruns         = %Lu", (unsigned long long) n_underruns);
	if (arv_camera_is_gv_device(p_camera_))
	{
		guint64 n_resent;
		guint64 n_missing;
		arv_gv_stream_get_statistics(reinterpret_cast<ArvGvStream *>(p_stream_), &n_resent, &n_missing);
		ROS_INFO ("Resent buffers    = %Lu", (unsigned long long) n_resent);
		ROS_INFO ("Missing           = %Lu", (unsigned long long) n_missing);
	}

	if (p_device_) {
		arv_device_execute_command (p_device_, "AcquisitionStop");
	}

	g_object_unref (p_stream_);
	g_object_unref (p_camera_);
}

void CameraAravisNodelet::onInit() {
	p_h_node_ = getNodeHandle();

	// Print out some useful info.
	ROS_INFO ("Attached cameras:");
	arv_update_device_list();
	uint n_interfaces = arv_get_n_interfaces();
	ROS_INFO ("# Interfaces: %d", n_interfaces);

	uint n_devices = arv_get_n_devices();
	ROS_INFO ("# Devices: %d", n_devices);
	for (uint i=0; i<n_devices; i++)
		ROS_INFO ("Device%d: %s", i, arv_get_device_id(i));

	if (n_devices==0) {
		ROS_ERROR ("No cameras detected.");
		return;
	}

	// Get the camera guid as a parameter or use the first device.
	const char *psz_guid = NULL;
	std::string sz_guid;
	if (p_h_node_.hasParam("guid"))	{
		p_h_node_.getParam("guid", sz_guid);
		psz_guid = sz_guid.c_str();
	}

	// Open the camera, and set it up.
	while (!p_camera_)
	{
		ROS_INFO("Opening: %s", psz_guid ? psz_guid : "(any)");
		p_camera_ = arv_camera_new(psz_guid);
		ros::Duration(1.0).sleep();
		ros::spinOnce();
	}

	p_device_ = arv_camera_get_device(p_camera_);
	ROS_INFO("Opened: %s-%s", arv_camera_get_vendor_name(p_camera_), arv_device_get_string_feature_value(p_device_, "DeviceSerialNumber"));

	// See if some basic camera features exist.
	ArvGcNode* p_gc_node = arv_device_get_feature (p_device_, "AcquisitionMode");
	GError *error = NULL;
	is_implemented_.acquisition_mode = ARV_GC_FEATURE_NODE (p_gc_node) ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (p_gc_node), &error) : FALSE;

	p_gc_node = arv_device_get_feature (p_device_, "GainRaw");
	is_implemented_.gain = ARV_GC_FEATURE_NODE (p_gc_node) ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (p_gc_node), &error) : FALSE;
	p_gc_node = arv_device_get_feature (p_device_, "Gain");
	is_implemented_.gain |= ARV_GC_FEATURE_NODE (p_gc_node) ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (p_gc_node), &error) : FALSE;

	p_gc_node = arv_device_get_feature (p_device_, "ExposureTime");
	is_implemented_.exposure_time = ARV_GC_FEATURE_NODE (p_gc_node) ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (p_gc_node), &error) : FALSE;

	p_gc_node = arv_device_get_feature (p_device_, "ExposureAuto");
	is_implemented_.exposure_auto = ARV_GC_FEATURE_NODE (p_gc_node) ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (p_gc_node), &error) : FALSE;

	p_gc_node = arv_device_get_feature (p_device_, "GainAuto");
	is_implemented_.gain_auto = ARV_GC_FEATURE_NODE (p_gc_node) ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (p_gc_node), &error) : FALSE;

	p_gc_node = arv_device_get_feature (p_device_, "TriggerSelector");
	is_implemented_.trigger_selector = ARV_GC_FEATURE_NODE (p_gc_node) ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (p_gc_node), &error) : FALSE;

	p_gc_node = arv_device_get_feature (p_device_, "TriggerSource");
	is_implemented_.trigger_source = ARV_GC_FEATURE_NODE (p_gc_node) ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (p_gc_node), &error) : FALSE;

	p_gc_node = arv_device_get_feature (p_device_, "TriggerMode");
	is_implemented_.trigger_mode = ARV_GC_FEATURE_NODE (p_gc_node) ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (p_gc_node), &error) : FALSE;

	p_gc_node = arv_device_get_feature (p_device_, "FocusPos");
	is_implemented_.focus_pos = ARV_GC_FEATURE_NODE (p_gc_node) ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (p_gc_node), &error) : FALSE;

	p_gc_node = arv_device_get_feature (p_device_, "GevSCPSPacketSize");
	is_implemented_.mtu = ARV_GC_FEATURE_NODE (p_gc_node) ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (p_gc_node), &error) : FALSE;

	p_gc_node = arv_device_get_feature (p_device_, "AcquisitionFrameRateEnable");
	is_implemented_.acquisition_frame_rate_enable = ARV_GC_FEATURE_NODE (p_gc_node) ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (p_gc_node), &error) : FALSE;

	// Find the key name for framerate.
	const char	*p_key_acquisition_frame_rate[2] = {"AcquisitionFrameRate", "AcquisitionFrameRateAbs"};
	for (uint i=0; i<2; i++)
	{
		p_gc_node = arv_device_get_feature (p_device_, p_key_acquisition_frame_rate[i]);
		is_implemented_.acquisition_frame_rate = p_gc_node ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (p_gc_node), &error) : FALSE;
		if (is_implemented_.acquisition_frame_rate)
		{
			key_acquisition_frame_rate_ = p_key_acquisition_frame_rate[i];
			break;
		}
	}

	// Get parameter bounds.
	arv_camera_get_exposure_time_bounds	(p_camera_, &config_min_.ExposureTime, &config_max_.ExposureTime);
	arv_camera_get_gain_bounds			(p_camera_, &config_min_.Gain, &config_max_.Gain);
	arv_camera_get_sensor_size			(p_camera_, &sensor_.width, &sensor_.height);
	arv_camera_get_width_bounds			(p_camera_, &roi_.width_min, &roi_.width_max);
	arv_camera_get_height_bounds		(p_camera_, &roi_.height_min, &roi_.height_max);

	if (is_implemented_.focus_pos)
	{
		gint64 focus_min64, focus_max64;
		arv_device_get_integer_feature_bounds (p_device_, "FocusPos", &focus_min64, &focus_max64);
		config_min_.FocusPos = focus_min64;
		config_max_.FocusPos = focus_max64;
	}
	else
	{
		config_min_.FocusPos = 0;
		config_max_.FocusPos = 0;
	}

	config_min_.AcquisitionFrameRate =    0.0;
	config_max_.AcquisitionFrameRate = 1000.0;

	// Initial camera settings.
	if (is_implemented_.exposure_time)
		arv_camera_set_exposure_time(p_camera_, config_.ExposureTime);
	if (is_implemented_.gain)
		arv_camera_set_gain(p_camera_, config_.Gain);
	if (is_implemented_.acquisition_frame_rate_enable)
		arv_device_set_integer_feature_value(p_device_, "AcquisitionFrameRateEnable", 1);
	if (is_implemented_.acquisition_frame_rate)
		arv_camera_set_frame_rate(p_camera_, config_.AcquisitionFrameRate);

	// Set up the triggering.
	if (is_implemented_.trigger_mode && is_implemented_.trigger_selector)
	{
		arv_device_set_string_feature_value(p_device_, "TriggerSelector", "FrameStart");
		arv_device_set_string_feature_value(p_device_, "TriggerMode", "Off");
	}

	writeCameraFeaturesFromRosparam ();

	// Start the camerainfo manager.
	p_camera_info_manager_.reset(new camera_info_manager::CameraInfoManager(p_h_node_, arv_device_get_string_feature_value (p_device_, "DeviceUserID")));

	// Start the dynamic_reconfigure server.
	reconfigure_server_.reset(new dynamic_reconfigure::Server<Config>(reconfigure_mutex_));
	reconfigure_server_->getConfigDefault(config_);
	// Get parameter current values.
	arv_camera_get_region (p_camera_, &roi_.x, &roi_.y, &roi_.width, &roi_.height);
	config_.ExposureTime 	= is_implemented_.exposure_time ? arv_device_get_float_feature_value (p_device_, "ExposureTime") : 0;
	config_.Gain      		= is_implemented_.gain ? arv_camera_get_gain (p_camera_) : 0.0;
	sensor_.sz_pixel_format	= std::string(arv_device_get_string_feature_value(p_device_, "PixelFormat"));
	std::transform(sensor_.sz_pixel_format.begin(), sensor_.sz_pixel_format.end(), sensor_.sz_pixel_format.begin(), [](unsigned char c){ return std::tolower(c); });
	sensor_.n_bytes_pixel	= ARV_PIXEL_FORMAT_BYTE_PER_PIXEL(arv_device_get_integer_feature_value(p_device_, "PixelFormat"));
	config_.FocusPos  		= is_implemented_.focus_pos ? arv_device_get_integer_feature_value (p_device_, "FocusPos") : 0;

	reconfigure_mutex_.lock();
	reconfigure_server_->updateConfig(config_);
	reconfigure_mutex_.unlock();
	ros::Duration(2.0).sleep();
	reconfigure_server_->setCallback(boost::bind(&CameraAravisNodelet::rosReconfigureCallback, this, _1, _2));

	// Print information.
	ROS_INFO ("    Using Camera Configuration:");
	ROS_INFO ("    ---------------------------");
	ROS_INFO ("    Vendor name          = %s", arv_device_get_string_feature_value (p_device_, "DeviceVendorName"));
	ROS_INFO ("    Model name           = %s", arv_device_get_string_feature_value (p_device_, "DeviceModelName"));
	ROS_INFO ("    Device id            = %s", arv_device_get_string_feature_value (p_device_, "DeviceUserID"));
	ROS_INFO ("    Serial number        = %s", arv_device_get_string_feature_value (p_device_, "DeviceSerialNumber"));
	ROS_INFO ("    Type                 = %s", arv_camera_is_uv_device(p_camera_) ? "USB3Vision" : (arv_camera_is_gv_device(p_camera_) ? "GigEVision" : "Other"));
	ROS_INFO ("    Sensor width         = %d", sensor_.width);
	ROS_INFO ("    Sensor height        = %d", sensor_.height);
	ROS_INFO ("    ROI x,y,w,h          = %d, %d, %d, %d", roi_.x, roi_.y, roi_.width, roi_.height);
	ROS_INFO ("    Pixel format         = %s", sensor_.sz_pixel_format.c_str());
	ROS_INFO ("    BytesPerPixel        = %lu", sensor_.n_bytes_pixel);
	ROS_INFO ("    Acquisition Mode     = %s", is_implemented_.acquisition_mode ? arv_device_get_string_feature_value (p_device_, "AcquisitionMode") : "(not implemented in camera)");
	ROS_INFO ("    Trigger Mode         = %s", is_implemented_.trigger_mode ? arv_device_get_string_feature_value (p_device_, "TriggerMode") : "(not implemented in camera)");
	ROS_INFO ("    Trigger Source       = %s", is_implemented_.trigger_source ? arv_device_get_string_feature_value(p_device_, "TriggerSource") : "(not implemented in camera)");
	ROS_INFO ("    Can set FrameRate:     %s", is_implemented_.acquisition_frame_rate ? "True" : "False");
	if (is_implemented_.acquisition_frame_rate)
	{
		config_.AcquisitionFrameRate = arv_device_get_float_feature_value (p_device_, key_acquisition_frame_rate_.c_str());
		ROS_INFO ("    AcquisitionFrameRate = %g hz", config_.AcquisitionFrameRate);
	}

	ROS_INFO ("    Can set Exposure:      %s", is_implemented_.exposure_time ? "True" : "False");
	if (is_implemented_.exposure_time)
	{
		ROS_INFO ("    Can set ExposureAuto:  %s", is_implemented_.exposure_auto ? "True" : "False");
		ROS_INFO ("    Exposure             = %g us in range [%g,%g]", config_.ExposureTime, config_min_.ExposureTime, config_max_.ExposureTime);
	}

	ROS_INFO ("    Can set Gain:          %s", is_implemented_.gain ? "True" : "False");
	if (is_implemented_.gain)
	{
		ROS_INFO ("    Can set GainAuto:      %s", is_implemented_.gain_auto ? "True" : "False");
		ROS_INFO ("    Gain                 = %f %% in range [%f,%f]", config_.Gain, config_min_.Gain, config_max_.Gain);
	}

	ROS_INFO ("    Can set FocusPos:      %s", is_implemented_.focus_pos ? "True" : "False");

	if (is_implemented_.mtu)
		ROS_INFO ("    Network mtu          = %lu", arv_device_get_integer_feature_value(p_device_, "GevSCPSPacketSize"));

	ROS_INFO ("    ---------------------------");


	//		// Print the tree of camera features, with their values.
	//		ROS_INFO ("    ----------------------------------------------------------------------------------");
	//		NODEEX		 nodeex;
	//		ArvGc	*pGenicam=0;
	//		pGenicam = arv_device_get_genicam(global.pDevice);
	//
	//		nodeex.szName = "Root";
	//		nodeex.pNode = (ArvDomNode	*)arv_gc_get_node(pGenicam, nodeex.szName);
	//		nodeex.pNodeSibling = NULL;
	//		PrintDOMTree(pGenicam, nodeex, 0);
	//		ROS_INFO ("    ----------------------------------------------------------------------------------");


	while (true)
	{
		p_stream_ = arv_device_create_stream(p_device_, NULL, NULL);
		if (p_stream_)
		{
			// Load up some buffers.
			const gint n_bytes_payload = arv_camera_get_payload(p_camera_);
			p_buffer_pool_.reset(new CameraBufferPool(p_stream_, n_bytes_payload));
			if (arv_camera_is_gv_device(p_camera_))
			{
				tuneGvStream(reinterpret_cast<ArvGvStream *>(p_stream_));
			}
			break;
		}
		else
		{
			ROS_WARN("Could not create image stream for %s.  Retrying...", psz_guid);
			ros::Duration(1.0).sleep();
			ros::spinOnce();
		}
	}

	// Monitor whether anyone is subscribed to the camera stream
	image_transport::SubscriberStatusCallback image_cb = boost::bind(&CameraAravisNodelet::rosConnectCallback, this);
	ros::SubscriberStatusCallback info_cb = boost::bind(&CameraAravisNodelet::rosConnectCallback, this);
	// Set up image_raw
	image_transport::ImageTransport	*p_transport = new image_transport::ImageTransport(p_h_node_);
	publisher_ = p_transport->advertiseCamera(this->getName() + "/image_raw", 1, image_cb, image_cb, info_cb, info_cb);

	// Connect signals with callbacks.
	g_signal_connect (p_stream_, "new-buffer", (GCallback)CameraAravisNodelet::newBufferReadyCallback, this);

	//control_cb_fct_= boost::bind(&CameraAravisNodelet::controlLostCallback, this, _1);
	g_signal_connect (p_device_, "control-lost", (GCallback)CameraAravisNodelet::controlLostCallback, this);

	arv_stream_set_emit_signals(p_stream_, TRUE);

	if (publisher_.getNumSubscribers())
		arv_camera_start_acquisition(p_camera_);
}

// Extra stream options for GigEVision streams.
void CameraAravisNodelet::tuneGvStream(ArvGvStream* p_stream)
{
	gboolean 		b_auto_buffer = FALSE;
	gboolean 		b_packet_resend = TRUE;
	unsigned int 	timeout_packet = 40; // milliseconds
	unsigned int 	timeout_frame_retention = 200;

	if (p_stream)
	{
		if (!ARV_IS_GV_STREAM (p_stream))
		{
			ROS_WARN("Stream is not a GV_STREAM");
			return;
		}

		if (b_auto_buffer)
			g_object_set (p_stream,
					"socket-buffer",
					ARV_GV_STREAM_SOCKET_BUFFER_AUTO,
					"socket-buffer-size", 0,
					NULL);
		if (!b_packet_resend)
			g_object_set (p_stream,
					"packet-resend",
					b_packet_resend ? ARV_GV_STREAM_PACKET_RESEND_ALWAYS : ARV_GV_STREAM_PACKET_RESEND_NEVER,
							NULL);
		g_object_set (p_stream,
				"packet-timeout", timeout_packet * 1000,
				"frame-retention", timeout_frame_retention * 1000,
				NULL);
	}
}

void CameraAravisNodelet::rosReconfigureCallback(Config &config, uint32_t level)
{
	reconfigure_mutex_.lock();
	std::string tf_prefix = tf::getPrefixParam(p_h_node_);
	ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);

	if (config.frame_id == "")
		config.frame_id = "camera";

	// Limit params to legal values.
	config.AcquisitionFrameRate		= CLAMP(config.AcquisitionFrameRate, config_min_.AcquisitionFrameRate, config_max_.AcquisitionFrameRate);
	config.ExposureTime				= CLAMP(config.ExposureTime,	config_min_.ExposureTime, config_max_.ExposureTime);
	config.Gain          			= CLAMP(config.Gain, config_min_.Gain, config_max_.Gain);
	config.FocusPos       			= CLAMP(config.FocusPos, config_min_.FocusPos, config_max_.FocusPos);
	config.frame_id   				= tf::resolve(tf_prefix, config.frame_id);

	// Find what the user changed.
	const bool changed_acquire					= (config_.Acquire != config.Acquire);
	const bool changed_acquisition_frame_rate	= (config_.AcquisitionFrameRate != config.AcquisitionFrameRate);
	const bool changed_exposure_auto			= (config_.ExposureAuto != config.ExposureAuto);
	const bool changed_exposure_time			= (config_.ExposureTime != config.ExposureTime);
	const bool changed_gain_auto				= (config_.GainAuto != config.GainAuto);
	const bool changed_gain						= (config_.Gain != config.Gain);
	const bool changed_acquisition_mode			= (config_.AcquisitionMode != config.AcquisitionMode);
	const bool changed_trigger_mode				= (config_.TriggerMode != config.TriggerMode);
	const bool changed_trigger_source			= (config_.TriggerSource != config.TriggerSource);
	const bool changed_softwarerate				= (config_.softwaretriggerrate != config.softwaretriggerrate);
	const bool changed_frame_id					= (config_.frame_id != config.frame_id);
	const bool changed_focus_pos				= (config_.FocusPos != config.FocusPos);
	const bool changed_mtu						= (config_.mtu != config.mtu);

	// Adjust other controls dependent on what the user changed.
	if (changed_exposure_time || changed_gain_auto ||
			((changed_acquisition_frame_rate || changed_gain || changed_frame_id ||
					changed_acquisition_mode || changed_trigger_source || changed_softwarerate) && config.ExposureAuto.compare("Once")==0))
		config.ExposureAuto = "Off";

	if (changed_gain || changed_exposure_auto ||
			((changed_acquisition_frame_rate || changed_exposure_time || changed_frame_id ||
					changed_acquisition_mode || changed_trigger_source || changed_softwarerate) && config.GainAuto.compare("Once")==0))
		config.GainAuto = "Off";

	if (changed_acquisition_frame_rate)
		config.TriggerMode = "Off";

	// Set params into the camera.
	if (changed_exposure_time)
	{
		if (is_implemented_.exposure_time)
		{
			ROS_INFO ("Set ExposureTime = %f us", config.ExposureTime);
			arv_camera_set_exposure_time(p_camera_, config.ExposureTime);
		}
		else
			ROS_INFO ("Camera does not support ExposureTime.");
	}

	if (changed_gain)
	{
		if (is_implemented_.gain)
		{
			ROS_INFO ("Set gain = %f", config.Gain);
			//arv_device_set_integer_feature_value (global.pDevice, "GainRaw", config.GainRaw);
			arv_camera_set_gain (p_camera_, config.Gain);
		}
		else
			ROS_INFO ("Camera does not support Gain or GainRaw.");
	}

	if (changed_exposure_auto)
	{
		if (is_implemented_.exposure_auto && is_implemented_.exposure_time)
		{
			ROS_INFO ("Set ExposureAuto = %s", config.ExposureAuto.c_str());
			arv_device_set_string_feature_value (p_device_, "ExposureAuto", config.ExposureAuto.c_str());
			if (config.ExposureAuto.compare("Once")==0)
			{
				ros::Duration(2.0).sleep();
				config.ExposureTime = arv_camera_get_exposure_time(p_camera_);
				ROS_INFO ("Get ExposureTime = %f us", config.ExposureTime);
				config.ExposureAuto = "Off";
			}
		}
		else
			ROS_INFO ("Camera does not support ExposureAuto.");
	}
	if (changed_gain_auto)
	{
		if (is_implemented_.gain_auto && is_implemented_.gain)
		{
			ROS_INFO ("Set GainAuto = %s", config.GainAuto.c_str());
			arv_device_set_string_feature_value (p_device_, "GainAuto", config.GainAuto.c_str());
			if (config.GainAuto.compare("Once")==0)
			{
				ros::Duration(2.0).sleep();
				//config.GainRaw = arv_device_get_integer_feature_value (global.pDevice, "GainRaw");
				config.Gain = arv_camera_get_gain (p_camera_);
				ROS_INFO ("Get Gain = %f", config.Gain);
				config.GainAuto = "Off";
			}
		}
		else
			ROS_INFO ("Camera does not support GainAuto.");
	}

	if (changed_acquisition_frame_rate)
	{
		if (is_implemented_.acquisition_frame_rate)
		{
			ROS_INFO ("Set frame rate = %f Hz", config.AcquisitionFrameRate);
			arv_camera_set_frame_rate(p_camera_, config.AcquisitionFrameRate);
		}
		else
			ROS_INFO ("Camera does not support AcquisitionFrameRate.");
	}

	if (changed_trigger_mode)
	{
		if (is_implemented_.trigger_mode)
		{
			ROS_INFO ("Set TriggerMode = %s", config.TriggerMode.c_str());
			arv_device_set_string_feature_value (p_device_, "TriggerMode", config.TriggerMode.c_str());
		}
		else
			ROS_INFO ("Camera does not support TriggerMode.");
	}

	if (changed_trigger_source)
	{
		if (is_implemented_.trigger_source)
		{
			ROS_INFO ("Set TriggerSource = %s", config.TriggerSource.c_str());
			arv_device_set_string_feature_value (p_device_, "TriggerSource", config.TriggerSource.c_str());
		}
		else
			ROS_INFO ("Camera does not support TriggerSource.");
	}

	if ((changed_trigger_mode || changed_trigger_source || changed_softwarerate) &&
			config.TriggerMode.compare("On")==0 && config.TriggerSource.compare("Software")==0)
	{
		if (is_implemented_.acquisition_frame_rate)
		{
			// The software rate is limited by the camera's internal framerate.  Bump up the camera's internal framerate if necessary.
			config.AcquisitionFrameRate = config_max_.AcquisitionFrameRate;
			ROS_INFO ("Set frame rate = %f Hz", config.AcquisitionFrameRate);
			arv_camera_set_frame_rate(p_camera_, config.AcquisitionFrameRate);
		}
	}

	//if (changed_trigger_source || changed_softwarerate)
	if (changed_trigger_source)
	{
		// delete software trigger if active
		software_trigger_active_ = false;
		if (software_trigger_.joinable()) {
			software_trigger_.join();
		}
		// activate on demand
		if (config.TriggerSource.compare("Software")==0)
		{
			ROS_INFO ("Set softwaretriggerrate = %f", 1000.0/ceil(1000.0 / config.softwaretriggerrate));

			// Turn on software timer callback.
			//id_software_trigger_timer_ = g_timeout_add ((guint)ceil(1000.0 / config.softwaretriggerrate), boost::bind(boost::mem_fn(&CameraAravisNodelet::softwareTriggerCallback), *this, _1), p_camera_);
			software_trigger_ = std::thread(&CameraAravisNodelet::softwareTriggerThread, this);
		}
	}

	if (changed_focus_pos)
	{
		if (is_implemented_.focus_pos)
		{
			ROS_INFO ("Set FocusPos = %d", config.FocusPos);
			arv_device_set_integer_feature_value(p_device_, "FocusPos", config.FocusPos);
			ros::Duration(1.0).sleep();
			config.FocusPos = arv_device_get_integer_feature_value(p_device_, "FocusPos");
			ROS_INFO ("Get FocusPos = %d", config.FocusPos);
		}
		else
			ROS_INFO ("Camera does not support FocusPos.");
	}

	if (changed_mtu)
	{
		if (is_implemented_.mtu)
		{
			ROS_INFO ("Set mtu = %d", config.mtu);
			arv_device_set_integer_feature_value(p_device_, "GevSCPSPacketSize", config.mtu);
			ros::Duration(1.0).sleep();
			config.mtu = arv_device_get_integer_feature_value(p_device_, "GevSCPSPacketSize");
			ROS_INFO ("Get mtu = %d", config.mtu);
		}
		else
			ROS_INFO ("Camera does not support mtu (i.e. GevSCPSPacketSize).");
	}

	if (changed_acquisition_mode)
	{
		if (is_implemented_.acquisition_mode)
		{
			ROS_INFO ("Set AcquisitionMode = %s", config.AcquisitionMode.c_str());
			arv_device_set_string_feature_value (p_device_, "AcquisitionMode", config.AcquisitionMode.c_str());

			ROS_INFO("AcquisitionStop");
			arv_device_execute_command (p_device_, "AcquisitionStop");
			ROS_INFO("AcquisitionStart");
			arv_device_execute_command (p_device_, "AcquisitionStart");
		}
		else
			ROS_INFO ("Camera does not support AcquisitionMode.");
	}

	if (changed_acquire)
	{
		if (config.Acquire)
		{
			ROS_INFO("AcquisitionStart");
			arv_device_execute_command (p_device_, "AcquisitionStart");
		}
		else
		{
			ROS_INFO("AcquisitionStop");
			arv_device_execute_command (p_device_, "AcquisitionStop");
		}
	}

	// adopt new config
	config_ = config;
	reconfigure_mutex_.unlock();
}

void CameraAravisNodelet::rosConnectCallback()
{
	if (p_device_) {
		if (publisher_.getNumSubscribers() == 0) {
			arv_device_execute_command (p_device_, "AcquisitionStop"); // don't waste CPU if nobody is listening!
		}
		else if (config_.Acquire) {
			arv_device_execute_command (p_device_, "AcquisitionStart");
		}
	}
}

void CameraAravisNodelet::newBufferReadyCallback (ArvStream *p_stream, gpointer can_instance)
{
	// workaround to get access to the instance from a static method
	CameraAravisNodelet* p_can = (CameraAravisNodelet*) can_instance;

	ArvBuffer *p_buffer = arv_stream_try_pop_buffer(p_stream);

	// check if we risk to drop the next image because of not enough buffers left
	gint n_available_buffers;
	arv_stream_get_n_buffers(p_stream, &n_available_buffers, NULL);
	if (n_available_buffers==0) {
		p_can->p_buffer_pool_->allocateBuffers(1);
	}

	if (p_buffer != NULL)
	{
		if (arv_buffer_get_status (p_buffer) == ARV_BUFFER_STATUS_SUCCESS
				&& p_can->p_buffer_pool_
				&& p_can->publisher_.getNumSubscribers())
		{
			(p_can->n_buffers_)++;
			// get the image message which wraps around this buffer
			sensor_msgs::ImagePtr msg_ptr = (*(p_can->p_buffer_pool_))[p_buffer];
			// fill the meta information of image message
			// get acquisition time
			const guint64 t = arv_buffer_get_system_timestamp(p_buffer);
			msg_ptr->header.stamp.fromNSec(t);
			// get frame sequence number
			msg_ptr->header.seq = arv_buffer_get_frame_id(p_buffer);
			// fill other stream properties
			msg_ptr->header.frame_id = p_can->config_.frame_id;
			msg_ptr->width = p_can->roi_.width;
			msg_ptr->height = p_can->roi_.height;
			msg_ptr->encoding = p_can->sensor_.sz_pixel_format;
			msg_ptr->step = msg_ptr->width * p_can->sensor_.n_bytes_pixel;

			// get current CameraInfo data
			if (!p_can->camera_info_) p_can->camera_info_.reset(new sensor_msgs::CameraInfo);
			(*p_can->camera_info_) = p_can->p_camera_info_manager_->getCameraInfo();
			p_can->camera_info_->header = msg_ptr->header;
			p_can->camera_info_->width = p_can->roi_.width;
			p_can->camera_info_->height = p_can->roi_.height;

			p_can->publisher_.publish(msg_ptr, p_can->camera_info_);

		}
		else {
			ROS_WARN ("Frame error: %s", szBufferStatusFromInt[arv_buffer_get_status (p_buffer)]);
			arv_stream_push_buffer (p_stream, p_buffer);
		}
	}
}

void CameraAravisNodelet::controlLostCallback (ArvDevice *p_gv_device, gpointer can_instance)
{
	CameraAravisNodelet* p_can = (CameraAravisNodelet*) can_instance;
	ROS_ERROR ("Control to aravis device lost.");
	nodelet::NodeletUnload unload_service;
	unload_service.request.name = p_can->getName();
	if(false == ros::service::call(ros::this_node::getName() + "/unload_nodelet", unload_service)) {
		ros::shutdown();
	}
}

void CameraAravisNodelet::softwareTriggerThread()
{
	software_trigger_active_ = true;
	std::chrono::system_clock::time_point next_time = std::chrono::system_clock::now();
	while (software_trigger_active_) {
		next_time += std::chrono::milliseconds(size_t(std::round(1000.0 / config_.softwaretriggerrate)));
		if (publisher_.getNumSubscribers()) {
			arv_device_execute_command (p_device_, "TriggerSoftware");
		}
		if (next_time > std::chrono::system_clock::now()) {
			std::this_thread::sleep_until(next_time);
		}
		else {
			ROS_WARN("Camera Aravis: Missed a software trigger event.");
			next_time = std::chrono::system_clock::now();
		}
	}
}

// Get the child and the child's sibling, where <p___> indicates an indirection.
NODEEX CameraAravisNodelet::getGcFirstChild(ArvGc *p_genicam, NODEEX nodeex)
{
	const char *sz_name = NULL;

	if (nodeex.p_node)
	{
		nodeex.p_node = arv_dom_node_get_first_child(nodeex.p_node);
		if (nodeex.p_node)
		{
			nodeex.sz_name = arv_dom_node_get_node_name(nodeex.p_node);
			nodeex.p_node_sibling = arv_dom_node_get_next_sibling(nodeex.p_node);

			// Do the indirection.
			if (nodeex.sz_name[0]=='p' && strcmp("pInvalidator", nodeex.sz_name))
			{
				sz_name = arv_dom_node_get_node_value(arv_dom_node_get_first_child(nodeex.p_node));
				nodeex.p_node  = (ArvDomNode *)arv_gc_get_node(p_genicam, sz_name);
				nodeex.sz_tag = arv_dom_node_get_node_name(nodeex.p_node);
			}
			else
			{
				nodeex.sz_tag = nodeex.sz_name;
			}
		}
		else
			nodeex.p_node_sibling = NULL;
	}
	else
	{
		nodeex.sz_name = NULL;
		nodeex.sz_tag = NULL;
		nodeex.p_node_sibling = NULL;
	}

	//ROS_INFO("GFC name=%s, node=%p, sib=%p", szNameChild, nodeex.pNode, nodeex.pNodeSibling);
	return nodeex;
}

// Get the sibling and the sibling's sibling, where <p___> indicates an indirection.
NODEEX CameraAravisNodelet::getGcNextSibling(ArvGc *p_genicam, NODEEX nodeex)
{
	const char *sz_name=NULL;

	// Go to the sibling.
	nodeex.p_node = nodeex.p_node_sibling;
	if (nodeex.p_node)
	{
		nodeex.sz_name = arv_dom_node_get_node_name(nodeex.p_node);
		nodeex.p_node_sibling = arv_dom_node_get_next_sibling(nodeex.p_node);

		// Do the indirection.
		if (nodeex.sz_name[0]=='p' && strcmp("pInvalidator", nodeex.sz_name))
		{
			sz_name = arv_dom_node_get_node_value(arv_dom_node_get_first_child(nodeex.p_node));
			nodeex.p_node = (ArvDomNode *)arv_gc_get_node(p_genicam, sz_name);
			nodeex.sz_tag = nodeex.p_node ? arv_dom_node_get_node_name(nodeex.p_node) : NULL;
		}
		else
		{
			nodeex.sz_tag = nodeex.sz_name;
		}
	}
	else
	{
		nodeex.sz_name = NULL;
		nodeex.sz_tag = NULL;
		nodeex.p_node_sibling = NULL;
	}

	//ROS_INFO("GNS name=%s, node=%p, sib=%p", nodeex.szName, nodeex.pNode, nodeex.pNodeSibling);
	return nodeex;
}

// Walk the DOM tree, i.e. the tree represented by the XML file in the camera, and that contains all the various features, parameters, etc.
void CameraAravisNodelet::printDOMTree(ArvGc *p_genicam, NODEEX nodeex, const int n_indent)
{
	char	   *sz_indent 			= NULL;
	const char *sz_feature			= NULL;
	const char *sz_dom_name			= NULL;
	const char *sz_feature_value	= NULL;

	sz_indent = new char[n_indent+1];
	memset(sz_indent,' ',n_indent);
	sz_indent[n_indent] = 0;

	nodeex = getGcFirstChild(p_genicam, nodeex);
	if (nodeex.p_node)
	{
		do
		{
			if (ARV_IS_GC_FEATURE_NODE((ArvGcFeatureNode *)nodeex.p_node))
			{
				sz_dom_name = arv_dom_node_get_node_name(nodeex.p_node);
				sz_feature = arv_gc_feature_node_get_name((ArvGcFeatureNode *)nodeex.p_node);
				sz_feature_value = arv_gc_feature_node_get_value_as_string((ArvGcFeatureNode *)nodeex.p_node, NULL);
				if (sz_feature && sz_feature_value && sz_feature_value[0])
					ROS_INFO("FeatureName: %s%s, %s=%s", sz_indent, sz_dom_name, sz_feature, sz_feature_value);
			}
			printDOMTree(p_genicam, nodeex, n_indent+4);

			// Go to the next sibling.
			nodeex = getGcNextSibling(p_genicam, nodeex);

		} while (nodeex.p_node && nodeex.p_node_sibling);
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
	XmlRpc::XmlRpcValue	 			 xml_rpc_params;
	XmlRpc::XmlRpcValue::iterator	 iter;
	ArvGcNode						*p_gc_node;
	GError							*error = NULL;


	p_h_node_.getParam(this->getName(), xml_rpc_params);


	if (xml_rpc_params.getType() == XmlRpc::XmlRpcValue::TypeStruct)
	{
		for (iter=xml_rpc_params.begin(); iter!=xml_rpc_params.end(); iter++)
		{
			std::string		key = iter->first;

			p_gc_node = arv_device_get_feature (p_device_, key.c_str());
			if (p_gc_node && arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (p_gc_node), &error))
			{
				//				unsigned long	typeValue = arv_gc_feature_node_get_value_type((ArvGcFeatureNode *)pGcNode);
				//				ROS_INFO("%s cameratype=%lu, rosparamtype=%d", key.c_str(), typeValue, static_cast<int>(iter->second.getType()));

				// We'd like to check the value types too, but typeValue is often given as G_TYPE_INVALID, so ignore it.
				switch (iter->second.getType())
				{
				case XmlRpc::XmlRpcValue::TypeBoolean://if ((iter->second.getType()==XmlRpc::XmlRpcValue::TypeBoolean))// && (typeValue==G_TYPE_INT64))
				{
					int			value = (bool)iter->second;
					arv_device_set_integer_feature_value(p_device_, key.c_str(), value);
					ROS_INFO("Read parameter (bool) %s: %d", key.c_str(), value);
				}
				break;

				case XmlRpc::XmlRpcValue::TypeInt: //if ((iter->second.getType()==XmlRpc::XmlRpcValue::TypeInt))// && (typeValue==G_TYPE_INT64))
				{
					int			value = (int)iter->second;
					arv_device_set_integer_feature_value(p_device_, key.c_str(), value);
					ROS_INFO("Read parameter (int) %s: %d", key.c_str(), value);
				}
				break;

				case XmlRpc::XmlRpcValue::TypeDouble: //if ((iter->second.getType()==XmlRpc::XmlRpcValue::TypeDouble))// && (typeValue==G_TYPE_DOUBLE))
				{
					double		value = (double)iter->second;
					arv_device_set_float_feature_value(p_device_, key.c_str(), value);
					ROS_INFO("Read parameter (float) %s: %f", key.c_str(), value);
				}
				break;

				case XmlRpc::XmlRpcValue::TypeString: //if ((iter->second.getType()==XmlRpc::XmlRpcValue::TypeString))// && (typeValue==G_TYPE_STRING))
				{
					std::string	value = (std::string)iter->second;
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



