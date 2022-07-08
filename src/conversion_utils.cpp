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

#include <camera_aravis/conversion_utils.h>

#include <ros/ros.h>

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

} // end namespace camera_aravis
