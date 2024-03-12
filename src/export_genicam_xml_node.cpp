/****************************************************************************
 *
 * camera_aravis
 *
 * Copyright © 2024 Fraunhofer IOSB and contributors
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

extern "C" {
#include <arv.h>
}

#include <fstream>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "export_genicam_xml");
    
    /// private node handle
    ros::NodeHandle pnh("~");

    /// GUID of the camera for which the xml is to be exported.
    std::string guid = pnh.param<std::string>("guid", "");

    /// Path string of output xml file.
    std::string xmlFileStr = pnh.param<std::string>("xml_file", "");

    //--- Print out some useful info.
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
        return -1;
    }

    //--- Open the camera.
    ArvCamera *p_camera = nullptr;
    while (!p_camera)
    {
        if (guid.empty())
        {
            ROS_INFO("Opening: (any)");
            p_camera = arv_camera_new(NULL);
        }
        else
        {
            ROS_INFO("Opening: %s", guid.c_str());
            p_camera = arv_camera_new(guid.c_str());
        }
        ros::Duration(1.0).sleep();
    }

    //--- get device
    ArvDevice *p_device = arv_camera_get_device(p_camera);
    const char* vendor_name = arv_camera_get_vendor_name(p_camera);
    const char* model_name  = arv_camera_get_model_name(p_camera);
    const char* device_id = arv_camera_get_device_id(p_camera);
    const char* device_sn = arv_device_get_string_feature_value(p_device, "DeviceSerialNumber");
    ROS_INFO("Successfully Opened: %s-%s-%s", vendor_name, model_name, 
            (device_sn) ? device_sn : device_id);

    //--- if xmlFileStr is empty construct relative path from guid

    /// Path of output xml file.
    boost::filesystem::path xmlFilePath;
    if(xmlFileStr.empty())
    {
        std::string tmpFileName = "";
        if(guid.empty())
        {
            tmpFileName += std::string(vendor_name);
            tmpFileName += "-" + std::string(model_name);
            tmpFileName += "-" + std::string((device_sn) ? device_sn : device_id);
        }
        else
        {
            tmpFileName = guid;
        }

        xmlFilePath = boost::filesystem::path(tmpFileName + ".xml");
    }
    else
    {
        xmlFilePath = boost::filesystem::path(xmlFileStr);
    }

    //--- make path absolute
    xmlFilePath = boost::filesystem::absolute(xmlFilePath);

    //--- print warning if file already exists
    if(boost::filesystem::exists(xmlFilePath))
        ROS_WARN("Output file already exists and will be overwritten. Path: %s", 
                 boost::filesystem::canonical(xmlFilePath).c_str());

    //--- make parent directory if not existing
    if(!xmlFilePath.parent_path().empty())
        boost::filesystem::create_directories(xmlFilePath.parent_path());

    //--- extract and save XML
    size_t xmlSize = 0;
    const char* p_xmldata = arv_device_get_genicam_xml(p_device, &xmlSize);
    std::ofstream fout;
    fout.open(xmlFilePath.c_str(), std::ios::binary | std::ios::out);
    fout.write(p_xmldata, xmlSize);
    fout.close();

    ROS_INFO("Written GenICam XML to file: %s", boost::filesystem::canonical(xmlFilePath).c_str());

    //--- release camera
    g_object_unref(p_camera);

    return 0;
}