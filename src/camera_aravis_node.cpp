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
 *     http://www.gnu.org/licenses/old-licenses/gpl-2.0.html
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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


#include <nodelet/loader.h>
#include "../include/camera_aravis/camera_aravis_nodelet.h"


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

