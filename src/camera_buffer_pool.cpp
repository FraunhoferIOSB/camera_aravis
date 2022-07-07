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

#include <camera_aravis/camera_buffer_pool.h>

namespace camera_aravis
{

CameraBufferPool::CameraBufferPool(ArvStream *stream, size_t payload_size_bytes, size_t n_preallocated_buffers) :
    stream_(stream), payload_size_bytes_(payload_size_bytes), n_buffers_(0),
    self_(this, [](CameraBufferPool *p) {})
{
  allocateBuffers(n_preallocated_buffers);
}

CameraBufferPool::~CameraBufferPool()
{
}

sensor_msgs::ImagePtr CameraBufferPool::getRecyclableImg()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (dangling_imgs_.empty()) {
    return sensor_msgs::ImagePtr(new sensor_msgs::Image, boost::bind(&CameraBufferPool::reclaim, this->weak_from_this(), boost::placeholders::_1));
  }
  else {
    sensor_msgs::ImagePtr img_ptr = dangling_imgs_.top();
    dangling_imgs_.pop();
    return img_ptr;
  }
}

sensor_msgs::ImagePtr CameraBufferPool::operator[](ArvBuffer *buffer)
{
  std::lock_guard<std::mutex> lock(mutex_);
  sensor_msgs::ImagePtr img_ptr;
  if (buffer) {
    // get address and size
    size_t buffer_size;
    const uint8_t *buffer_data = (const uint8_t*)arv_buffer_get_data(buffer, &buffer_size);

    // find corresponding ImagePtr wrapper
    std::map<const uint8_t*, sensor_msgs::ImagePtr>::iterator iter = available_img_buffers_.find(buffer_data);
    if (iter != available_img_buffers_.end())
    {
      img_ptr = iter->second;
      used_buffers_.emplace(img_ptr.get(), buffer);
      available_img_buffers_.erase(iter);
    }
    else
    {
      ROS_WARN("Could not find available image in pool corresponding to buffer.");
      img_ptr.reset(new sensor_msgs::Image);
      img_ptr->data.resize(buffer_size);
      memcpy(img_ptr->data.data(), buffer_data, buffer_size);
    }
  }

  return img_ptr;
}

void CameraBufferPool::allocateBuffers(size_t n)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (ARV_IS_STREAM(stream_))
  {
    for (size_t i = 0; i < n; ++i)
    {
      sensor_msgs::Image *p_img = new sensor_msgs::Image;
      p_img->data.resize(payload_size_bytes_);
      ArvBuffer *buffer = arv_buffer_new(payload_size_bytes_, p_img->data.data());
      sensor_msgs::ImagePtr img_ptr(
          p_img, boost::bind(&CameraBufferPool::reclaim, this->weak_from_this(), boost::placeholders::_1));
      available_img_buffers_.emplace(p_img->data.data(), img_ptr);
      arv_stream_push_buffer(stream_, buffer);
      ++n_buffers_;
    }
    ROS_INFO_STREAM("Allocated " << n << " image buffers of size " << payload_size_bytes_);
  }
  else
  {
    ROS_ERROR("Error: Stream not valid. Failed to allocate buffers.");
  }
}

void CameraBufferPool::reclaim(const WPtr &self, sensor_msgs::Image *p_img)
{
  Ptr s = self.lock();
  if (s)
  {
    s->push(p_img);
  }
  else
  {
    delete p_img;
  }
}

void CameraBufferPool::push(sensor_msgs::Image *p_img)
{
  std::lock_guard<std::mutex> lock(mutex_);

  std::map<sensor_msgs::Image*, ArvBuffer*>::iterator iter = used_buffers_.find(p_img);

  if (iter != used_buffers_.end())
  {
    if (ARV_IS_STREAM(stream_))
    {
      sensor_msgs::ImagePtr img_ptr(
          p_img, boost::bind(&CameraBufferPool::reclaim, this->weak_from_this(), boost::placeholders::_1));
      available_img_buffers_.emplace(p_img->data.data(), img_ptr);
      arv_stream_push_buffer(stream_, iter->second);
    }
    else
    {
      // the camera stream is gone, so should its buffers
      delete p_img;
    }
    used_buffers_.erase(iter);
  }
  else
  {
    // this image was not an aravis registered buffer
    dangling_imgs_.emplace(p_img, boost::bind(&CameraBufferPool::reclaim, this->weak_from_this(), boost::placeholders::_1));
  }
}

} /* namespace camera_aravis */
