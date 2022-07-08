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

#ifndef CAMERA_ARAVIS_CAMERA_BUFFER_POOL
#define CAMERA_ARAVIS_CAMERA_BUFFER_POOL

extern "C" {
#include <arv.h>
}

#include <ros/ros.h>
#include <boost/weak_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/bind.hpp>
#include <boost/bind/placeholders.hpp>

#include <sensor_msgs/Image.h>

#include <mutex>
#include <map>
#include <stack>

namespace camera_aravis
{

class CameraBufferPool : public boost::enable_shared_from_this<CameraBufferPool>
{
public:
  typedef boost::shared_ptr<CameraBufferPool> Ptr;
  typedef boost::weak_ptr<CameraBufferPool> WPtr;

  // Note: If the CameraBufferPool is destroyed, buffers will be deallocated. Therefor, make sure
  // that the CameraBufferPool stays alive longer than the given stream object.
  //
  // stream: 			weakly managed pointer to the stream. Used to register all allocated buffers
  // payload_size_bytes:	size of a single buffer
  // n_preallocated_buffers:	number of initially allocated and registered buffers
  CameraBufferPool(ArvStream *stream, size_t payload_size_bytes, size_t n_preallocated_buffers = 2);
  virtual ~CameraBufferPool();

  // Get an image whose lifespan is administrated by this pool (but not registered to the camera).
  sensor_msgs::ImagePtr getRecyclableImg();

  // Get the image message which wraps around the given ArvBuffer.
  //
  // If this buffer is not administrated by this CameraBufferPool,
  // a new image message is allocated and the contents of the buffer
  // are copied to it.
  sensor_msgs::ImagePtr operator[](ArvBuffer *buffer);

  inline size_t getAllocatedSize() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return n_buffers_;
  }

  inline size_t getUsedSize() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return used_buffers_.size();
  }

  inline size_t getPayloadSize() const
  {
    return payload_size_bytes_;
  }

  // Allocate new buffers which are wrapped by an image message and
  // push them to the internal aravis stream.
  void allocateBuffers(size_t n = 1);

protected:
  // Custom deleter for aravis buffer wrapping image messages, which
  // either pushes the buffer back to the aravis stream cleans it up
  // when the CameraBufferPool is gone.
  static void reclaim(const WPtr &self, sensor_msgs::Image *p_img);

  // Push the buffer inside the given image back to the aravis stream,
  // remember the corresponding image message.
  void push(sensor_msgs::Image *p_img);

  ArvStream *stream_ = NULL;
  size_t payload_size_bytes_ = 0;
  size_t n_buffers_ = 0;

  std::map<const uint8_t*, sensor_msgs::ImagePtr> available_img_buffers_;
  std::map<sensor_msgs::Image*, ArvBuffer*> used_buffers_;
  std::stack<sensor_msgs::ImagePtr> dangling_imgs_;
  mutable std::mutex mutex_;
  Ptr self_;
};

} /* namespace camera_aravis */

#endif /* CAMERA_ARAVIS_CAMERA_BUFFER_POOL */
