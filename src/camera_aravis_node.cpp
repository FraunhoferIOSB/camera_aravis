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

// This is a ROS node that operates GenICam-based cameras (e.g. USB3Vision compatible) via the Aravis library.
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

#include <camera_aravis/camera_aravis_nodelet.h>

#include <nodelet/loader.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "camera_aravis");
	nodelet::Loader manager(ros::NodeHandle("~"));
	nodelet::M_string remap(ros::names::getRemappings());
	nodelet::V_string nargv;
	manager.load(ros::this_node::getName(), "camera_aravis/CameraAravisNodelet", remap, nargv);
	ros::spin();

    return 0;
}
