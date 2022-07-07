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

#ifndef CAMERA_ARAVIS_CONVERTION_UTILS
#define CAMERA_ARAVIS_CONVERTION_UTILS

#include <functional>

#include <boost/bind.hpp>
#include <boost/bind/placeholders.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

namespace camera_aravis
{

// Conversions from integers to Arv types.
static const char *szBufferStatusFromInt[] = {"ARV_BUFFER_STATUS_SUCCESS", "ARV_BUFFER_STATUS_CLEARED",
                                       "ARV_BUFFER_STATUS_TIMEOUT", "ARV_BUFFER_STATUS_MISSING_PACKETS",
                                       "ARV_BUFFER_STATUS_WRONG_PACKET_ID", "ARV_BUFFER_STATUS_SIZE_MISMATCH",
                                       "ARV_BUFFER_STATUS_FILLING", "ARV_BUFFER_STATUS_ABORTED"};

// Conversion functions from Genicam to ROS formats
using ConversionFunction = std::function<void(sensor_msgs::ImagePtr& in, sensor_msgs::ImagePtr& out)>;

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

} // end namespace camera_aravis

#endif /* CAMERA_ARAVIS_CONVERTION_UTILS */
