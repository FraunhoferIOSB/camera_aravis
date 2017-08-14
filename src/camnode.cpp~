/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */


// camera_aravis
//
// This is a ROS node that operates GenICam-based cameras via the Aravis library.
// Commonly available camera features are supported through the dynamic_reconfigure user-interface and GUI,
// and for those features not in the GUI but that are specific to a camera, they can be set in the
// camera by setting the appropriate parameter at startup.  This code reads those parameters, and
// if any of them match a camera feature, then the camera is written to.
//
// For example, if a camera has a feature called "IRFormat" that is an integer 0, 1, or 2, you can do
// rosparam set camnode/IRFormat 2
// and this driver will write it to the camera at startup.  Note that the datatype of the parameter
// must be correct for the camera feature (e.g. bool, int, double, string, etc), so for example you should use
// rosparam set camnode/GainAuto true
// and NOT
// rosparam set camnode/GainAuto 1
//


#include <arv.h>

#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include <glib.h>

#include <ros/ros.h>
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

#include "XmlRpc.h"

//#define TUNING	// Allows tuning the gains for the timestamp controller.  Publishes output on topic /dt, and receives gains on params /kp, /ki, /kd


#define CLIP(x,lo,hi)	MIN(MAX((lo),(x)),(hi))
#define THROW_ERROR(m) throw std::string((m))

#define TRIGGERSOURCE_SOFTWARE	0
#define TRIGGERSOURCE_LINE1		1
#define TRIGGERSOURCE_LINE2		2

#define ARV_PIXEL_FORMAT_BIT_PER_PIXEL(pixel_format)  (((pixel_format) >> 16) & 0xff)
#define ARV_PIXEL_FORMAT_BYTE_PER_PIXEL(pixel_format) ((((pixel_format) >> 16) & 0xff) >> 3)
typedef camera_aravis::CameraAravisConfig Config;

static gboolean SoftwareTrigger_callback (void *);

typedef struct
{
	const char *szName;
	const char *szTag;
	ArvDomNode *pNode;
	ArvDomNode *pNodeSibling;
} NODEEX;

// Global variables -------------------
struct global_s
{
	gboolean 								bCancel;
	image_transport::CameraPublisher 		publisher;
	camera_info_manager::CameraInfoManager *pCameraInfoManager;
	sensor_msgs::CameraInfo 				camerainfo;
	gint 									width, height; // buffer->width and buffer->height not working, so I used a global.
	Config 									config;
	Config 									configMin;
	Config 									configMax;
	int                                     idSoftwareTriggerTimer;
	
	int										isImplementedAcquisitionFrameRate;
	int										isImplementedAcquisitionFrameRateEnable;
	int										isImplementedGain;
	int										isImplementedExposureTimeAbs;
	int										isImplementedExposureAuto;
	int										isImplementedGainAuto;
	int										isImplementedFocusPos;
	int										isImplementedTriggerSelector;
	int										isImplementedTriggerSource;
	int										isImplementedTriggerMode;
	int										isImplementedAcquisitionMode;
	int										isImplementedMtu;

    int         							xRoi;
	int         							yRoi;
	int         							widthRoi;
	int										widthRoiMin;
	int										widthRoiMax;
	int         							heightRoi;
	int										heightRoiMin;
	int										heightRoiMax;

	int                                     widthSensor;
	int                                     heightSensor;

	const char                             *pszPixelformat;
	unsigned								nBytesPixel;
	ros::NodeHandle 					   *phNode;
	ArvCamera 							   *pCamera;
	ArvDevice 							   *pDevice;
	int										mtu;
	int										Acquire;
	const char							   *keyAcquisitionFrameRate;
#ifdef TUNING			
	ros::Publisher 							*ppubInt64;
#endif

} global;


typedef struct 
{
    GMainLoop  *main_loop;
    int         nBuffers;	// Counter for Hz calculation.
} ApplicationData;
// ------------------------------------


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


static void set_cancel (int signal)
{
    global.bCancel = TRUE;
}

ArvGvStream *CreateStream(void)
{
	gboolean 		bAutoBuffer = FALSE;
	gboolean 		bPacketResend = TRUE;
	unsigned int 	timeoutPacket = 40; // milliseconds
	unsigned int 	timeoutFrameRetention = 200;

	
	ArvGvStream *pStream = (ArvGvStream *)arv_device_create_stream (global.pDevice, NULL, NULL);
	if (pStream)
	{
		ArvBuffer	*pBuffer;
		gint 		 nbytesPayload;


		if (!ARV_IS_GV_STREAM (pStream))
			ROS_WARN("Stream is not a GV_STREAM");

		if (bAutoBuffer)
			g_object_set (pStream,
					      "socket-buffer",
						  ARV_GV_STREAM_SOCKET_BUFFER_AUTO,
						  "socket-buffer-size", 0,
						  NULL);
		if (!bPacketResend)
			g_object_set (pStream,
					      "packet-resend",
						  bPacketResend ? ARV_GV_STREAM_PACKET_RESEND_ALWAYS : ARV_GV_STREAM_PACKET_RESEND_NEVER,
						  NULL);
		g_object_set (pStream,
				          "packet-timeout",
						  (unsigned) timeoutPacket * 1000,
						  "frame-retention", (unsigned) timeoutFrameRetention * 1000,
					  NULL);
	
		// Load up some buffers.
		nbytesPayload = arv_camera_get_payload (global.pCamera);
		for (int i=0; i<50; i++)
		{
			pBuffer = arv_buffer_new (nbytesPayload, NULL);
			arv_stream_push_buffer ((ArvStream *)pStream, pBuffer);
		}
	}
	return pStream;
} // CreateStream()



void RosReconfigure_callback(Config &config, uint32_t level)
{
    int             changedAcquire;
    int             changedAcquisitionFrameRate;
    int             changedExposureAuto;
    int             changedGainAuto;
    int             changedExposureTimeAbs;
    int             changedGain;
    int             changedAcquisitionMode;
    int             changedTriggerMode;
    int             changedTriggerSource;
    int             changedSoftwarerate;
    int             changedFrameid;
    int             changedFocusPos;
    int             changedMtu;
    
    
    std::string tf_prefix = tf::getPrefixParam(*global.phNode);
    ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
    
    if (config.frame_id == "")
        config.frame_id = "camera";

    
    // Find what the user changed.
    changedAcquire    			= (global.config.Acquire != config.Acquire);
    changedAcquisitionFrameRate = (global.config.AcquisitionFrameRate != config.AcquisitionFrameRate);
    changedExposureAuto 		= (global.config.ExposureAuto != config.ExposureAuto);
    changedExposureTimeAbs  	= (global.config.ExposureTimeAbs != config.ExposureTimeAbs);
    changedGainAuto     		= (global.config.GainAuto != config.GainAuto);
    changedGain         		= (global.config.Gain != config.Gain);
    changedAcquisitionMode 		= (global.config.AcquisitionMode != config.AcquisitionMode);
    changedTriggerMode  		= (global.config.TriggerMode != config.TriggerMode);
    changedTriggerSource		= (global.config.TriggerSource != config.TriggerSource);
    changedSoftwarerate  		= (global.config.softwaretriggerrate != config.softwaretriggerrate);
    changedFrameid      		= (global.config.frame_id != config.frame_id);
    changedFocusPos     		= (global.config.FocusPos != config.FocusPos);
    changedMtu          		= (global.config.mtu != config.mtu);


    // Limit params to legal values.
    config.AcquisitionFrameRate = CLIP(config.AcquisitionFrameRate, global.configMin.AcquisitionFrameRate, 	global.configMax.AcquisitionFrameRate);
    config.ExposureTimeAbs   	= CLIP(config.ExposureTimeAbs,  	global.configMin.ExposureTimeAbs,  		global.configMax.ExposureTimeAbs);
    config.Gain          		= CLIP(config.Gain,         		global.configMin.Gain,         			global.configMax.Gain);
    config.FocusPos       		= CLIP(config.FocusPos,      		global.configMin.FocusPos,      		global.configMax.FocusPos);
    config.frame_id   			= tf::resolve(tf_prefix, config.frame_id);


    // Adjust other controls dependent on what the user changed.
    if (changedExposureTimeAbs || changedGainAuto || ((changedAcquisitionFrameRate || changedGain || changedFrameid
					|| changedAcquisitionMode || changedTriggerSource || changedSoftwarerate) && config.ExposureAuto=="Once"))
    	config.ExposureAuto = "Off";

    if (changedGain || changedExposureAuto || ((changedAcquisitionFrameRate || changedExposureTimeAbs || changedFrameid
					|| changedAcquisitionMode || changedTriggerSource || changedSoftwarerate) && config.GainAuto=="Once"))
    	config.GainAuto = "Off";

    if (changedAcquisitionFrameRate)
    	config.TriggerMode = "Off";


    // Find what changed for any reason.
    changedAcquire    			= (global.config.Acquire != config.Acquire);
    changedAcquisitionFrameRate = (global.config.AcquisitionFrameRate != config.AcquisitionFrameRate);
    changedExposureAuto 		= (global.config.ExposureAuto != config.ExposureAuto);
    changedExposureTimeAbs     	= (global.config.ExposureTimeAbs != config.ExposureTimeAbs);
    changedGainAuto     		= (global.config.GainAuto != config.GainAuto);
    changedGain            		= (global.config.Gain != config.Gain);
    changedAcquisitionMode 		= (global.config.AcquisitionMode != config.AcquisitionMode);
    changedTriggerMode  		= (global.config.TriggerMode != config.TriggerMode);
    changedTriggerSource		= (global.config.TriggerSource != config.TriggerSource);
    changedSoftwarerate  		= (global.config.softwaretriggerrate != config.softwaretriggerrate);
    changedFrameid      		= (global.config.frame_id != config.frame_id);
    changedFocusPos     		= (global.config.FocusPos != config.FocusPos);
    changedMtu          		= (global.config.mtu != config.mtu);

    
    // Set params into the camera.
    if (changedExposureTimeAbs)
    {
    	if (global.isImplementedExposureTimeAbs)
		{
			ROS_INFO ("Set ExposureTimeAbs = %f", config.ExposureTimeAbs);
			arv_device_set_float_feature_value (global.pDevice, "ExposureTimeAbs", config.ExposureTimeAbs);
		}
    	else
    		ROS_INFO ("Camera does not support ExposureTimeAbs.");
    }

    if (changedGain)
    {
    	if (global.isImplementedGain)
		{
			ROS_INFO ("Set gain = %f", config.Gain);
			//arv_device_set_integer_feature_value (global.pDevice, "GainRaw", config.GainRaw);
			arv_camera_set_gain (global.pCamera, config.Gain);
		}
    	else
    		ROS_INFO ("Camera does not support Gain or GainRaw.");
    }

    if (changedExposureAuto)
    {
    	if (global.isImplementedExposureAuto && global.isImplementedExposureTimeAbs)
		{
			ROS_INFO ("Set ExposureAuto = %s", config.ExposureAuto.c_str());
			arv_device_set_string_feature_value (global.pDevice, "ExposureAuto", config.ExposureAuto.c_str());
			if (config.ExposureAuto=="Once")
			{
				ros::Duration(2.0).sleep();
				config.ExposureTimeAbs = arv_device_get_float_feature_value (global.pDevice, "ExposureTimeAbs");
				ROS_INFO ("Get ExposureTimeAbs = %f", config.ExposureTimeAbs);
				config.ExposureAuto = "Off";
			}
		}
    	else
    		ROS_INFO ("Camera does not support ExposureAuto.");
    }
    if (changedGainAuto)
    {
    	if (global.isImplementedGainAuto && global.isImplementedGain)
		{
			ROS_INFO ("Set GainAuto = %s", config.GainAuto.c_str());
			arv_device_set_string_feature_value (global.pDevice, "GainAuto", config.GainAuto.c_str());
			if (config.GainAuto=="Once")
			{
				ros::Duration(2.0).sleep();
				//config.GainRaw = arv_device_get_integer_feature_value (global.pDevice, "GainRaw");
				config.Gain = arv_camera_get_gain (global.pCamera);
				ROS_INFO ("Get Gain = %f", config.Gain);
				config.GainAuto = "Off";
			}
		}
    	else
    		ROS_INFO ("Camera does not support GainAuto.");
    }

    if (changedAcquisitionFrameRate)
    {
    	if (global.isImplementedAcquisitionFrameRate)
		{
			ROS_INFO ("Set %s = %f", global.keyAcquisitionFrameRate, config.AcquisitionFrameRate);
			arv_device_set_float_feature_value (global.pDevice, global.keyAcquisitionFrameRate, config.AcquisitionFrameRate);
		}
    	else
    		ROS_INFO ("Camera does not support AcquisitionFrameRate.");
    }

    if (changedTriggerMode)
    {
    	if (global.isImplementedTriggerMode)
		{
			ROS_INFO ("Set TriggerMode = %s", config.TriggerMode.c_str());
			arv_device_set_string_feature_value (global.pDevice, "TriggerMode", config.TriggerMode.c_str());
		}
    	else
    		ROS_INFO ("Camera does not support TriggerMode.");
    }

    if (changedTriggerSource)
    {
    	if (global.isImplementedTriggerSource)
		{
			ROS_INFO ("Set TriggerSource = %s", config.TriggerSource.c_str());
			arv_device_set_string_feature_value (global.pDevice, "TriggerSource", config.TriggerSource.c_str());
		}
    	else
    		ROS_INFO ("Camera does not support TriggerSource.");
    }

    if ((changedTriggerMode || changedTriggerSource || changedSoftwarerate) && config.TriggerMode=="On" && config.TriggerSource=="Software")
    {
    	if (global.isImplementedAcquisitionFrameRate)
		{
			// The software rate is limited by the camera's internal framerate.  Bump up the camera's internal framerate if necessary.
			config.AcquisitionFrameRate = global.configMax.AcquisitionFrameRate;
			ROS_INFO ("Set %s = %f", global.keyAcquisitionFrameRate, config.AcquisitionFrameRate);
			arv_device_set_float_feature_value (global.pDevice, global.keyAcquisitionFrameRate, config.AcquisitionFrameRate);
		}
    }

    if (changedTriggerSource || changedSoftwarerate)
    {
    	// Recreate the software trigger callback.
		if (global.idSoftwareTriggerTimer)
		{
			g_source_remove(global.idSoftwareTriggerTimer);
			global.idSoftwareTriggerTimer = 0;
		}
    	if (!strcmp(config.TriggerSource.c_str(),"Software"))
    	{
        	ROS_INFO ("Set softwaretriggerrate = %f", 1000.0/ceil(1000.0 / config.softwaretriggerrate));

    		// Turn on software timer callback.
    		global.idSoftwareTriggerTimer = g_timeout_add ((guint)ceil(1000.0 / config.softwaretriggerrate), SoftwareTrigger_callback, global.pCamera);
    	}
    }
    if (changedFocusPos)
    {
    	if (global.isImplementedFocusPos)
		{
			ROS_INFO ("Set FocusPos = %d", config.FocusPos);
			arv_device_set_integer_feature_value(global.pDevice, "FocusPos", config.FocusPos);
			ros::Duration(1.0).sleep();
			config.FocusPos = arv_device_get_integer_feature_value(global.pDevice, "FocusPos");
			ROS_INFO ("Get FocusPos = %d", config.FocusPos);
		}
    	else
    		ROS_INFO ("Camera does not support FocusPos.");
    }
    if (changedMtu)
    {
    	if (global.isImplementedMtu)
		{
			ROS_INFO ("Set mtu = %d", config.mtu);
			arv_device_set_integer_feature_value(global.pDevice, "GevSCPSPacketSize", config.mtu);
			ros::Duration(1.0).sleep();
			config.mtu = arv_device_get_integer_feature_value(global.pDevice, "GevSCPSPacketSize");
			ROS_INFO ("Get mtu = %d", config.mtu);
		}
    	else
    		ROS_INFO ("Camera does not support mtu (i.e. GevSCPSPacketSize).");
    }

    if (changedAcquisitionMode)
    {
    	if (global.isImplementedAcquisitionMode)
		{
			ROS_INFO ("Set AcquisitionMode = %s", config.AcquisitionMode.c_str());
			arv_device_set_string_feature_value (global.pDevice, "AcquisitionMode", config.AcquisitionMode.c_str());

			ROS_INFO("AcquisitionStop");
			arv_device_execute_command (global.pDevice, "AcquisitionStop");
			ROS_INFO("AcquisitionStart");
			arv_device_execute_command (global.pDevice, "AcquisitionStart");
		}
    	else
    		ROS_INFO ("Camera does not support AcquisitionMode.");
    }

    if (changedAcquire)
    {
    	if (config.Acquire)
    	{
    		ROS_INFO("AcquisitionStart");
			arv_device_execute_command (global.pDevice, "AcquisitionStart");
    	}
    	else
    	{
    		ROS_INFO("AcquisitionStop");
			arv_device_execute_command (global.pDevice, "AcquisitionStop");
    	}
    }



    global.config = config;

} // RosReconfigure_callback()


static void NewBuffer_callback (ArvStream *pStream, ApplicationData *pApplicationdata)
{
	static uint64_t  cm = 0L;	// Camera time prev
	uint64_t  		 cn = 0L;	// Camera time now

#ifdef TUNING			
	static uint64_t  rm = 0L;	// ROS time prev
#endif
	uint64_t  		 rn = 0L;	// ROS time now

	static uint64_t	 tm = 0L;	// Calculated image time prev
	uint64_t		 tn = 0L;	// Calculated image time now
		
	static int64_t   em = 0L;	// Error prev.
	int64_t  		 en = 0L;	// Error now between calculated image time and ROS time.
	int64_t  		 de = 0L;	// derivative.
	int64_t  		 ie = 0L;	// integral.
	int64_t			 u = 0L;	// Output of controller.
	
	int64_t			 kp1 = 0L;		// Fractional gains in integer form.
	int64_t			 kp2 = 1024L;
	int64_t			 kd1 = 0L;
	int64_t			 kd2 = 1024L;
	int64_t			 ki1 = -1L;		// A gentle pull toward zero.
	int64_t			 ki2 = 1024L;

	static uint32_t	 iFrame = 0;	// Frame counter.
    
	ArvBuffer		*pBuffer;

	
#ifdef TUNING			
	std_msgs::Int64  msgInt64;
	int 			 kp = 0;
	int 			 kd = 0;
	int 			 ki = 0;
    
	if (global.phNode->hasParam(ros::this_node::getName()+"/kp"))
	{
		global.phNode->getParam(ros::this_node::getName()+"/kp", kp);
		kp1 = kp;
	}
	
	if (global.phNode->hasParam(ros::this_node::getName()+"/kd"))
	{
		global.phNode->getParam(ros::this_node::getName()+"/kd", kd);
		kd1 = kd;
	}
	
	if (global.phNode->hasParam(ros::this_node::getName()+"/ki"))
	{
		global.phNode->getParam(ros::this_node::getName()+"/ki", ki);
		ki1 = ki;
	}
#endif
	
    pBuffer = arv_stream_try_pop_buffer (pStream);
    if (pBuffer != NULL) 
    {
        if (arv_buffer_get_status (pBuffer) == ARV_BUFFER_STATUS_SUCCESS) 
        {
			sensor_msgs::Image msg;
			
			size_t buffer_size;
			char *buffer_data = (char *) arv_buffer_get_data (pBuffer, &buffer_size);
			
			pApplicationdata->nBuffers++;
			std::vector<uint8_t> this_data(buffer_size);
			memcpy(&this_data[0], buffer_data, buffer_size);


			// Camera/ROS Timestamp coordination.
			cn				= (uint64_t)arv_buffer_get_timestamp (pBuffer);	// Camera now
			rn	 			= ros::Time::now().toNSec();					// ROS now
			
			if (iFrame < 10)
			{
				cm = cn;
				tm  = rn;
			}
			
			// Control the error between the computed image timestamp and the ROS timestamp.
			en = (int64_t)tm + (int64_t)cn - (int64_t)cm - (int64_t)rn; // i.e. tn-rn, but calced from prior values.
			de = en-em;
			ie += en;
			u = kp1*(en/kp2) + ki1*(ie/ki2) + kd1*(de/kd2);  // kp<0, ki<0, kd>0
			
			// Compute the new timestamp.
			tn = (uint64_t)((int64_t)tm + (int64_t)cn-(int64_t)cm + u);

#ifdef TUNING			
			ROS_WARN("en=%16ld, ie=%16ld, de=%16ld, u=%16ld + %16ld + %16ld = %16ld", en, ie, de, kp1*(en/kp2), ki1*(ie/ki2), kd1*(de/kd2), u);
			ROS_WARN("cn=%16lu, rn=%16lu, cn-cm=%8ld, rn-rm=%8ld, tn-tm=%8ld, tn-rn=%ld", cn, rn, cn-cm, rn-rm, (int64_t)tn-(int64_t)tm, tn-rn);
			msgInt64.data = tn-rn; //cn-cm+tn-tm; //
			global.ppubInt64->publish(msgInt64);
			rm = rn;
#endif
			
			// Save prior values.
			cm = cn;
			tm = tn;
			em = en;
			
			// Construct the image message.
			msg.header.stamp.fromNSec(tn);
			msg.header.seq = arv_buffer_get_frame_id (pBuffer);
			msg.header.frame_id = global.config.frame_id;
			msg.width = global.widthRoi;
			msg.height = global.heightRoi;
			msg.encoding = global.pszPixelformat;
			msg.step = msg.width * global.nBytesPixel;
			msg.data = this_data;

			// get current CameraInfo data
			global.camerainfo = global.pCameraInfoManager->getCameraInfo();
			global.camerainfo.header.stamp = msg.header.stamp;
			global.camerainfo.header.seq = msg.header.seq;
			global.camerainfo.header.frame_id = msg.header.frame_id;
			global.camerainfo.width = global.widthRoi;
			global.camerainfo.height = global.heightRoi;

			global.publisher.publish(msg, global.camerainfo);
				
        }
        else
        	ROS_WARN ("Frame error: %s", szBufferStatusFromInt[arv_buffer_get_status (pBuffer)]);
        	
        arv_stream_push_buffer (pStream, pBuffer);
        iFrame++;
    }
} // NewBuffer_callback()


static void ControlLost_callback (ArvGvDevice *pGvDevice)
{
    ROS_ERROR ("Control lost.");

    global.bCancel = TRUE;
}

static gboolean SoftwareTrigger_callback (void *pCamera)
{
	arv_device_execute_command (global.pDevice, "TriggerSoftware");

    return TRUE;
}


// PeriodicTask_callback()
// Check for termination, and spin for ROS.
static gboolean PeriodicTask_callback (void *applicationdata)
{
    ApplicationData *pData = (ApplicationData*)applicationdata;

    //  ROS_INFO ("Frame rate = %d Hz", pData->nBuffers);
    pData->nBuffers = 0;

    if (global.bCancel)
    {
        g_main_loop_quit (pData->main_loop);
        return FALSE;
    }

    ros::spinOnce();

    return TRUE;
} // PeriodicTask_callback()


// Get the child and the child's sibling, where <p___> indicates an indirection.
NODEEX GetGcFirstChild(ArvGc *pGenicam, NODEEX nodeex)
{
	const char *szName=0;

	if (nodeex.pNode)
	{
		nodeex.pNode = arv_dom_node_get_first_child(nodeex.pNode);
		if (nodeex.pNode)
		{
			nodeex.szName = arv_dom_node_get_node_name(nodeex.pNode);
			nodeex.pNodeSibling = arv_dom_node_get_next_sibling(nodeex.pNode);

			// Do the indirection.
			if (nodeex.szName[0]=='p' && strcmp("pInvalidator", nodeex.szName))
			{
				szName = arv_dom_node_get_node_value(arv_dom_node_get_first_child(nodeex.pNode));
				nodeex.pNode  = (ArvDomNode *)arv_gc_get_node(pGenicam, szName);
				nodeex.szTag = arv_dom_node_get_node_name(nodeex.pNode);
			}
			else
			{
				nodeex.szTag = nodeex.szName;
			}
		}
		else
			nodeex.pNodeSibling = NULL;
	}
	else
	{
		nodeex.szName = NULL;
		nodeex.szTag = NULL;
		nodeex.pNodeSibling = NULL;
	}
	
	//ROS_INFO("GFC name=%s, node=%p, sib=%p", szNameChild, nodeex.pNode, nodeex.pNodeSibling);

	
	return nodeex;
} // GetGcFirstChild()


// Get the sibling and the sibling's sibling, where <p___> indicates an indirection.
NODEEX GetGcNextSibling(ArvGc *pGenicam, NODEEX nodeex)
{
	const char *szName=0;

	// Go to the sibling.
	nodeex.pNode = nodeex.pNodeSibling;
	if (nodeex.pNode)
	{
		nodeex.szName = arv_dom_node_get_node_name(nodeex.pNode);
		nodeex.pNodeSibling = arv_dom_node_get_next_sibling(nodeex.pNode);

		// Do the indirection.
		if (nodeex.szName[0]=='p' && strcmp("pInvalidator", nodeex.szName))
		{
			szName = arv_dom_node_get_node_value(arv_dom_node_get_first_child(nodeex.pNode));
			nodeex.pNode = (ArvDomNode *)arv_gc_get_node(pGenicam, szName);
			nodeex.szTag = nodeex.pNode ? arv_dom_node_get_node_name(nodeex.pNode) : NULL;
		}
		else
		{
			nodeex.szTag = nodeex.szName;
		}
	}
	else 
	{
		nodeex.szName = NULL;
		nodeex.szTag = NULL;
		nodeex.pNodeSibling = NULL;
	}

	//ROS_INFO("GNS name=%s, node=%p, sib=%p", nodeex.szName, nodeex.pNode, nodeex.pNodeSibling);

	
	return nodeex;
} // GetGcNextSibling()


// Walk the DOM tree, i.e. the tree represented by the XML file in the camera, and that contains all the various features, parameters, etc.
void PrintDOMTree(ArvGc *pGenicam, NODEEX nodeex, int nIndent)
{
	char		*szIndent=0;
	const char *szFeature=0;
	const char *szDomName=0;
	const char *szFeatureValue=0;
	
	szIndent = new char[nIndent+1];
	memset(szIndent,' ',nIndent);
	szIndent[nIndent]=0;

	nodeex = GetGcFirstChild(pGenicam, nodeex);
	if (nodeex.pNode)
	{
		do
		{
			if (ARV_IS_GC_FEATURE_NODE((ArvGcFeatureNode *)nodeex.pNode))
			{
				szDomName = arv_dom_node_get_node_name(nodeex.pNode);
				szFeature = arv_gc_feature_node_get_name((ArvGcFeatureNode *)nodeex.pNode);
				szFeatureValue = arv_gc_feature_node_get_value_as_string((ArvGcFeatureNode *)nodeex.pNode, NULL);
				if (szFeature && szFeatureValue && szFeatureValue[0])
					ROS_INFO("FeatureName: %s%s, %s=%s", szIndent, szDomName, szFeature, szFeatureValue);
			}
			PrintDOMTree(pGenicam, nodeex, nIndent+4);
			
			// Go to the next sibling.
			nodeex = GetGcNextSibling(pGenicam, nodeex);

		} while (nodeex.pNode && nodeex.pNodeSibling);
	}
} //PrintDOMTree()


// WriteCameraFeaturesFromRosparam()
// Read ROS parameters from this node's namespace, and see if each parameter has a similarly named & typed feature in the camera.  Then set the
// camera feature to that value.  For example, if the parameter camnode/Gain is set to 123.0, then we'll write 123.0 to the Gain feature
// in the camera.
//
// Note that the datatype of the parameter *must* match the datatype of the camera feature, and this can be determined by
// looking at the camera's XML file.  Camera enum's are string parameters, camera bools are false/true parameters (not 0/1),
// integers are integers, doubles are doubles, etc.
//
void WriteCameraFeaturesFromRosparam(void)
{
	XmlRpc::XmlRpcValue	 			 xmlrpcParams;
	XmlRpc::XmlRpcValue::iterator	 iter;
    ArvGcNode						*pGcNode;
	GError							*error=NULL;


	global.phNode->getParam (ros::this_node::getName(), xmlrpcParams);


	if (xmlrpcParams.getType() == XmlRpc::XmlRpcValue::TypeStruct)
	{
		for (iter=xmlrpcParams.begin(); iter!=xmlrpcParams.end(); iter++)
		{
			std::string		key = iter->first;

			pGcNode = arv_device_get_feature (global.pDevice, key.c_str());
			if (pGcNode && arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (pGcNode), &error))
			{
//				unsigned long	typeValue = arv_gc_feature_node_get_value_type((ArvGcFeatureNode *)pGcNode);
//				ROS_INFO("%s cameratype=%lu, rosparamtype=%d", key.c_str(), typeValue, static_cast<int>(iter->second.getType()));

				// We'd like to check the value types too, but typeValue is often given as G_TYPE_INVALID, so ignore it.
				switch (iter->second.getType())
				{
					case XmlRpc::XmlRpcValue::TypeBoolean://if ((iter->second.getType()==XmlRpc::XmlRpcValue::TypeBoolean))// && (typeValue==G_TYPE_INT64))
					{
						int			value = (bool)iter->second;
						arv_device_set_integer_feature_value(global.pDevice, key.c_str(), value);
						ROS_INFO("Read parameter (bool) %s: %d", key.c_str(), value);
					}
					break;

					case XmlRpc::XmlRpcValue::TypeInt: //if ((iter->second.getType()==XmlRpc::XmlRpcValue::TypeInt))// && (typeValue==G_TYPE_INT64))
					{
						int			value = (int)iter->second;
						arv_device_set_integer_feature_value(global.pDevice, key.c_str(), value);
						ROS_INFO("Read parameter (int) %s: %d", key.c_str(), value);
					}
					break;

					case XmlRpc::XmlRpcValue::TypeDouble: //if ((iter->second.getType()==XmlRpc::XmlRpcValue::TypeDouble))// && (typeValue==G_TYPE_DOUBLE))
					{
						double		value = (double)iter->second;
						arv_device_set_float_feature_value(global.pDevice, key.c_str(), value);
						ROS_INFO("Read parameter (float) %s: %f", key.c_str(), value);
					}
					break;

					case XmlRpc::XmlRpcValue::TypeString: //if ((iter->second.getType()==XmlRpc::XmlRpcValue::TypeString))// && (typeValue==G_TYPE_STRING))
					{
						std::string	value = (std::string)iter->second;
						arv_device_set_string_feature_value(global.pDevice, key.c_str(), value.c_str());
						ROS_INFO("Read parameter (string) %s: %s", key.c_str(), value.c_str());
					}
					break;

					case XmlRpc::XmlRpcValue::TypeInvalid:
					case XmlRpc::XmlRpcValue::TypeDateTime:
					case XmlRpc::XmlRpcValue::TypeBase64:
					case XmlRpc::XmlRpcValue::TypeArray:
					case XmlRpc::XmlRpcValue::TypeStruct:
					default:
						ROS_WARN("Unhandled rosparam type in WriteCameraFeaturesFromRosparam()");
				}
			}
		}
	}
} // WriteCameraFeaturesFromRosparam()



int main(int argc, char** argv) 
{
    char   		*pszGuid = NULL;
    char    	 szGuid[512];
    int			 nInterfaces = 0;
    int			 nDevices = 0;
    int 		 i = 0;
	const char	*pkeyAcquisitionFrameRate[2] = {"AcquisitionFrameRate", "AcquisitionFrameRateAbs"};
    ArvGcNode	*pGcNode;
	GError		*error=NULL;


    
    
    global.bCancel = FALSE;
    global.config = global.config.__getDefault__();
    global.idSoftwareTriggerTimer = 0;

    ros::init(argc, argv, "camera");
    global.phNode = new ros::NodeHandle();


    //g_type_init ();

    // Print out some useful info.
    ROS_INFO ("Attached cameras:");
    arv_update_device_list();
    nInterfaces = arv_get_n_interfaces();
    ROS_INFO ("# Interfaces: %d", nInterfaces);

    nDevices = arv_get_n_devices();
    ROS_INFO ("# Devices: %d", nDevices);
    for (i=0; i<nDevices; i++)
    	ROS_INFO ("Device%d: %s", i, arv_get_device_id(i));
    
    if (nDevices>0)
    {
		// Get the camera guid from either the command-line or as a parameter.
    	if (argc==2)
    	{
    		strcpy(szGuid, argv[1]);
    		pszGuid = szGuid;
    	}
    	else
    	{
    		if (global.phNode->hasParam(ros::this_node::getName()+"/guid"))
    		{
    			std::string		stGuid;
    			
    			global.phNode->getParam(ros::this_node::getName()+"/guid", stGuid);
    			strcpy (szGuid, stGuid.c_str());
        		pszGuid = szGuid;
    		}
    		else
    			pszGuid = NULL;
    	}
    	
    	// Open the camera, and set it up.
    	ROS_INFO("Opening: %s", pszGuid ? pszGuid : "(any)");
		while (TRUE)
		{
			global.pCamera = arv_camera_new(pszGuid);
			if (global.pCamera)
				break;
			else
			{
				ROS_WARN ("Could not open camera %s.  Retrying...", pszGuid);
				ros::Duration(1.0).sleep();
			    ros::spinOnce();
			}
		}

		global.pDevice = arv_camera_get_device(global.pCamera);
		ROS_INFO("Opened: %s-%s", arv_device_get_string_feature_value (global.pDevice, "DeviceVendorName"), arv_device_get_string_feature_value (global.pDevice, "DeviceID"));


		// See if some basic camera features exist.
		pGcNode = arv_device_get_feature (global.pDevice, "AcquisitionMode");
		global.isImplementedAcquisitionMode = ARV_GC_FEATURE_NODE (pGcNode) ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (pGcNode), &error) : FALSE;

		pGcNode = arv_device_get_feature (global.pDevice, "GainRaw");
		global.isImplementedGain = ARV_GC_FEATURE_NODE (pGcNode) ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (pGcNode), &error) : FALSE;
		pGcNode = arv_device_get_feature (global.pDevice, "Gain");
		global.isImplementedGain |= ARV_GC_FEATURE_NODE (pGcNode) ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (pGcNode), &error) : FALSE;

		pGcNode = arv_device_get_feature (global.pDevice, "ExposureTimeAbs");
		global.isImplementedExposureTimeAbs = ARV_GC_FEATURE_NODE (pGcNode) ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (pGcNode), &error) : FALSE;

		pGcNode = arv_device_get_feature (global.pDevice, "ExposureAuto");
		global.isImplementedExposureAuto = ARV_GC_FEATURE_NODE (pGcNode) ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (pGcNode), &error) : FALSE;

		pGcNode = arv_device_get_feature (global.pDevice, "GainAuto");
		global.isImplementedGainAuto = ARV_GC_FEATURE_NODE (pGcNode) ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (pGcNode), &error) : FALSE;

		pGcNode = arv_device_get_feature (global.pDevice, "TriggerSelector");
		global.isImplementedTriggerSelector = ARV_GC_FEATURE_NODE (pGcNode) ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (pGcNode), &error) : FALSE;

		pGcNode = arv_device_get_feature (global.pDevice, "TriggerSource");
		global.isImplementedTriggerSource = ARV_GC_FEATURE_NODE (pGcNode) ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (pGcNode), &error) : FALSE;

		pGcNode = arv_device_get_feature (global.pDevice, "TriggerMode");
		global.isImplementedTriggerMode = ARV_GC_FEATURE_NODE (pGcNode) ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (pGcNode), &error) : FALSE;

		pGcNode = arv_device_get_feature (global.pDevice, "FocusPos");
		global.isImplementedFocusPos = ARV_GC_FEATURE_NODE (pGcNode) ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (pGcNode), &error) : FALSE;

		pGcNode = arv_device_get_feature (global.pDevice, "GevSCPSPacketSize");
		global.isImplementedMtu = ARV_GC_FEATURE_NODE (pGcNode) ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (pGcNode), &error) : FALSE;

		pGcNode = arv_device_get_feature (global.pDevice, "AcquisitionFrameRateEnable");
		global.isImplementedAcquisitionFrameRateEnable = ARV_GC_FEATURE_NODE (pGcNode) ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (pGcNode), &error) : FALSE;

		// Find the key name for framerate.
		global.keyAcquisitionFrameRate = NULL;
		for (i=0; i<2; i++)
		{
			pGcNode = arv_device_get_feature (global.pDevice, pkeyAcquisitionFrameRate[i]);
			global.isImplementedAcquisitionFrameRate = pGcNode ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (pGcNode), &error) : FALSE;
			if (global.isImplementedAcquisitionFrameRate)
			{
				global.keyAcquisitionFrameRate = pkeyAcquisitionFrameRate[i];
				break;
			}
		}


		// Get parameter bounds.
		arv_camera_get_exposure_time_bounds	(global.pCamera, &global.configMin.ExposureTimeAbs, &global.configMax.ExposureTimeAbs);
		arv_camera_get_gain_bounds			(global.pCamera, &global.configMin.Gain, &global.configMax.Gain);
		arv_camera_get_sensor_size			(global.pCamera, &global.widthSensor, &global.heightSensor);
		arv_camera_get_width_bounds			(global.pCamera, &global.widthRoiMin, &global.widthRoiMax);
		arv_camera_get_height_bounds		(global.pCamera, &global.heightRoiMin, &global.heightRoiMax);

		if (global.isImplementedFocusPos)
		{
			gint64 focusMin64, focusMax64;
			arv_device_get_integer_feature_bounds (global.pDevice, "FocusPos", &focusMin64, &focusMax64);
			global.configMin.FocusPos = focusMin64;
			global.configMax.FocusPos = focusMax64;
		}
		else
		{
			global.configMin.FocusPos = 0;
			global.configMax.FocusPos = 0;
		}

		global.configMin.AcquisitionFrameRate =    0.0;
		global.configMax.AcquisitionFrameRate = 1000.0;


		// Initial camera settings.
		if (global.isImplementedExposureTimeAbs)
			arv_device_set_float_feature_value(global.pDevice, "ExposureTimeAbs", global.config.ExposureTimeAbs);
		if (global.isImplementedGain)
			arv_camera_set_gain(global.pCamera, global.config.Gain);
			//arv_device_set_integer_feature_value(global.pDevice, "GainRaw", global.config.GainRaw);
		if (global.isImplementedAcquisitionFrameRateEnable)
			arv_device_set_integer_feature_value(global.pDevice, "AcquisitionFrameRateEnable", 1);
		if (global.isImplementedAcquisitionFrameRate)
			arv_device_set_float_feature_value(global.pDevice, global.keyAcquisitionFrameRate, global.config.AcquisitionFrameRate);


		// Set up the triggering.
		if (global.isImplementedTriggerMode)
		{
			if (global.isImplementedTriggerSelector && global.isImplementedTriggerMode)
			{
				arv_device_set_string_feature_value(global.pDevice, "TriggerSelector", "AcquisitionStart");
				arv_device_set_string_feature_value(global.pDevice, "TriggerMode", "Off");
				arv_device_set_string_feature_value(global.pDevice, "TriggerSelector", "FrameStart");
				arv_device_set_string_feature_value(global.pDevice, "TriggerMode", "Off");
			}
		}


		WriteCameraFeaturesFromRosparam ();


#ifdef TUNING			
		ros::Publisher pubInt64 = global.phNode->advertise<std_msgs::Int64>(ros::this_node::getName()+"/dt", 100);
		global.ppubInt64 = &pubInt64;
#endif
    	
		// Start the camerainfo manager.
		global.pCameraInfoManager = new camera_info_manager::CameraInfoManager(ros::NodeHandle(ros::this_node::getName()), arv_device_get_string_feature_value (global.pDevice, "DeviceID"));

		// Start the dynamic_reconfigure server.
		dynamic_reconfigure::Server<Config> 				reconfigureServer;
		dynamic_reconfigure::Server<Config>::CallbackType 	reconfigureCallback;

		reconfigureCallback = boost::bind(&RosReconfigure_callback, _1, _2);
		reconfigureServer.setCallback(reconfigureCallback);
		ros::Duration(2.0).sleep();


		// Get parameter current values.
		global.xRoi=0; global.yRoi=0; global.widthRoi=0; global.heightRoi=0;
		arv_camera_get_region (global.pCamera, &global.xRoi, &global.yRoi, &global.widthRoi, &global.heightRoi);
		global.config.ExposureTimeAbs 	= global.isImplementedExposureTimeAbs ? arv_device_get_float_feature_value (global.pDevice, "ExposureTimeAbs") : 0;
		global.config.Gain      		= global.isImplementedGain ? arv_camera_get_gain (global.pCamera) : 0.0;
		global.pszPixelformat   		= g_string_ascii_down(g_string_new(arv_device_get_string_feature_value(global.pDevice, "PixelFormat")))->str;
		global.nBytesPixel      		= ARV_PIXEL_FORMAT_BYTE_PER_PIXEL(arv_device_get_integer_feature_value(global.pDevice, "PixelFormat"));
		global.config.FocusPos  		= global.isImplementedFocusPos ? arv_device_get_integer_feature_value (global.pDevice, "FocusPos") : 0;
		
		
		// Print information.
		ROS_INFO ("    Using Camera Configuration:");
		ROS_INFO ("    ---------------------------");
		ROS_INFO ("    Vendor name          = %s", arv_device_get_string_feature_value (global.pDevice, "DeviceVendorName"));
		ROS_INFO ("    Model name           = %s", arv_device_get_string_feature_value (global.pDevice, "DeviceModelName"));
		ROS_INFO ("    Device id            = %s", arv_device_get_string_feature_value (global.pDevice, "DeviceID"));
		ROS_INFO ("    Sensor width         = %d", global.widthSensor);
		ROS_INFO ("    Sensor height        = %d", global.heightSensor);
		ROS_INFO ("    ROI x,y,w,h          = %d, %d, %d, %d", global.xRoi, global.yRoi, global.widthRoi, global.heightRoi);
		ROS_INFO ("    Pixel format         = %s", global.pszPixelformat);
		ROS_INFO ("    BytesPerPixel        = %d", global.nBytesPixel);
		ROS_INFO ("    Acquisition Mode     = %s", global.isImplementedAcquisitionMode ? arv_device_get_string_feature_value (global.pDevice, "AcquisitionMode") : "(not implemented in camera)");
		ROS_INFO ("    Trigger Mode         = %s", global.isImplementedTriggerMode ? arv_device_get_string_feature_value (global.pDevice, "TriggerMode") : "(not implemented in camera)");
		ROS_INFO ("    Trigger Source       = %s", global.isImplementedTriggerSource ? arv_device_get_string_feature_value(global.pDevice, "TriggerSource") : "(not implemented in camera)");
		ROS_INFO ("    Can set FrameRate:     %s", global.isImplementedAcquisitionFrameRate ? "True" : "False");
		if (global.isImplementedAcquisitionFrameRate)
		{
			global.config.AcquisitionFrameRate = arv_device_get_float_feature_value (global.pDevice, global.keyAcquisitionFrameRate);
			ROS_INFO ("    AcquisitionFrameRate = %g hz", global.config.AcquisitionFrameRate);
		}

		ROS_INFO ("    Can set Exposure:      %s", global.isImplementedExposureTimeAbs ? "True" : "False");
		if (global.isImplementedExposureTimeAbs)
		{
			ROS_INFO ("    Can set ExposureAuto:  %s", global.isImplementedExposureAuto ? "True" : "False");
			ROS_INFO ("    Exposure             = %g us in range [%g,%g]", global.config.ExposureTimeAbs, global.configMin.ExposureTimeAbs, global.configMax.ExposureTimeAbs);
		}

		ROS_INFO ("    Can set Gain:          %s", global.isImplementedGain ? "True" : "False");
		if (global.isImplementedGain)
		{
			ROS_INFO ("    Can set GainAuto:      %s", global.isImplementedGainAuto ? "True" : "False");
			ROS_INFO ("    Gain                 = %f %% in range [%f,%f]", global.config.Gain, global.configMin.Gain, global.configMax.Gain);
		}

		ROS_INFO ("    Can set FocusPos:      %s", global.isImplementedFocusPos ? "True" : "False");

		if (global.isImplementedMtu)
			ROS_INFO ("    Network mtu          = %lu", arv_device_get_integer_feature_value(global.pDevice, "GevSCPSPacketSize"));

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
			

		ArvGvStream *pStream = NULL;
		while (TRUE)
		{
			pStream = CreateStream();
			if (pStream)
				break;
			else
			{
				ROS_WARN("Could not create image stream for %s.  Retrying...", pszGuid);
				ros::Duration(1.0).sleep();
			    ros::spinOnce();
			}
		}


		ApplicationData applicationdata;
		applicationdata.nBuffers=0;
		applicationdata.main_loop = 0;

		// Set up image_raw.
		image_transport::ImageTransport		*pTransport = new image_transport::ImageTransport(*global.phNode);
		global.publisher = pTransport->advertiseCamera(ros::this_node::getName()+"/image_raw", 1);

		// Connect signals with callbacks.
		g_signal_connect (pStream,        "new-buffer",   G_CALLBACK (NewBuffer_callback),   &applicationdata);
		g_signal_connect (global.pDevice, "control-lost", G_CALLBACK (ControlLost_callback), NULL);
		g_timeout_add_seconds (1, PeriodicTask_callback, &applicationdata);
		arv_stream_set_emit_signals ((ArvStream *)pStream, TRUE);


		void (*pSigintHandlerOld)(int);
		pSigintHandlerOld = signal (SIGINT, set_cancel);

		arv_device_execute_command (global.pDevice, "AcquisitionStart");

		applicationdata.main_loop = g_main_loop_new (NULL, FALSE);
		g_main_loop_run (applicationdata.main_loop);

		if (global.idSoftwareTriggerTimer)
		{
			g_source_remove(global.idSoftwareTriggerTimer);
			global.idSoftwareTriggerTimer = 0;
		}

		signal (SIGINT, pSigintHandlerOld);

		g_main_loop_unref (applicationdata.main_loop);

		guint64 n_completed_buffers;
		guint64 n_failures;
		guint64 n_underruns;
		guint64 n_resent;
		guint64 n_missing;
		arv_stream_get_statistics ((ArvStream *)pStream, &n_completed_buffers, &n_failures, &n_underruns);
		ROS_INFO ("Completed buffers = %Lu", (unsigned long long) n_completed_buffers);
		ROS_INFO ("Failures          = %Lu", (unsigned long long) n_failures);
		ROS_INFO ("Underruns         = %Lu", (unsigned long long) n_underruns);
		arv_gv_stream_get_statistics (pStream, &n_resent, &n_missing);
		ROS_INFO ("Resent buffers    = %Lu", (unsigned long long) n_resent);
		ROS_INFO ("Missing           = %Lu", (unsigned long long) n_missing);

		arv_device_execute_command (global.pDevice, "AcquisitionStop");

		g_object_unref (pStream);

    }
    else
    	ROS_ERROR ("No cameras detected.");
    
    delete global.phNode;
    
    return 0;
} // main()

