/*
Copyright 2011 - 2017 Occam Robotics Inc - All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
* Neither the name of Occam Vision Group, Occam Robotics Inc, nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL OCCAM ROBOTICS INC BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "image_collect.h"
#include <algorithm>
#include <iostream>
#include <assert.h>
#undef min
#undef max

#define DEBUG_SYNC

////////////////////////////////////////////////////////////////////////
// ImageCollector

void ImageCollector::addStream(int index,OccamDeviceBase* device) {
  DeviceInfo& di = devices[index];
  di.device = device;
}

int ImageCollector::read(std::shared_ptr<OccamImage>* imgout, int N) {
  bool any_read = false;
  for (auto it=devices.begin();it!=devices.end();++it) {
    if (bool(it->second.last_image))
      continue;
    OccamImage* img0 = 0;
    int r = it->second.device->readImage(&img0, 0);
    if (r != OCCAM_API_SUCCESS && r != OCCAM_API_DATA_NOT_AVAILABLE)
      return r;
    if (r == OCCAM_API_DATA_NOT_AVAILABLE)
      continue;
    it->second.last_image = std::shared_ptr<OccamImage>(img0,occamFreeImage);
  }

  int highest_index = -1;
  int lag = -1;
  for (auto it=devices.begin();it!=devices.end();++it) {
    if (!bool(it->second.last_image))
      continue;
    if (highest_index>=0)
      lag = std::max(lag,abs(int(highest_index-it->second.last_image->index)));
    if (highest_index<0||it->second.last_image->index>highest_index)
      highest_index = it->second.last_image->index;
  }

  const int max_lag = 10;
  if (lag >= max_lag) {
    for (auto it=devices.begin();it!=devices.end();++it) {
      if (!bool(it->second.last_image))
	continue;
#ifdef DEBUG_SYNC
      std::cerr<<"drop "<<it->first<<" "<<it->second.last_image->index<<" because we are lagging by "<<lag<<std::endl;
#endif // DEBUG_SYNC
      it->second.last_image.reset();
    }
  }

  if (highest_index>=0) {
    for (auto it=devices.begin();it!=devices.end();++it) {
      if (!bool(it->second.last_image))
	continue;
      if (it->second.last_image->index == highest_index)
	continue;
#ifdef DEBUG_SYNC
      std::cerr<<"drop "<<it->first<<" "<<it->second.last_image->index<<" (less than "<<highest_index<<")"<<std::endl;
#endif // DEBUG_SYNC
      it->second.last_image.reset();
    }
  }

  bool all_valid = true;
  for (auto it=devices.begin();it!=devices.end();++it) {
    if (!bool(it->second.last_image)) {
      all_valid = false;
      break;
    }
  }

  if (!all_valid)
    return OCCAM_API_DATA_NOT_AVAILABLE;

  if (N != devices.size())
    return OCCAM_API_INVALID_COUNT;

  int imgout_index = 0;
  for (auto it=devices.begin();it!=devices.end();++it) {
    imgout[imgout_index++] = it->second.last_image;
    it->second.last_image.reset();
  }

  return OCCAM_API_SUCCESS;
}
