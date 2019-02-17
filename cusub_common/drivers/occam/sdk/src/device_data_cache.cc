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

#include "indigo.h"
#include "device_data_cache.h"

static DeviceDataCacheCallback devicedatacache_cb = 0;
static void* devicedatacache_cb_data = 0;

bool deviceDataCacheLoad(const char* cid, int data_type,
			 const void* hash, int hash_len,
			 void* data, int data_len) {
  if (!devicedatacache_cb)
    return false;

  int r = devicedatacache_cb
    (cid, OCCAM_DEVICEDATACACHE_LOAD,
     data_type,
     hash, hash_len,
     data, data_len,
     devicedatacache_cb_data);
  return r == OCCAM_API_SUCCESS;
}

void deviceDataCacheStore(const char* cid, int data_type,
			  const void* hash, int hash_len,
			  const void* data, int data_len) {
  if (!devicedatacache_cb)
    return;
  devicedatacache_cb
    (cid, OCCAM_DEVICEDATACACHE_STORE,
     data_type,
     hash, hash_len,
     (void*)data, data_len,
     devicedatacache_cb_data);
}

int setDeviceDataCache(DeviceDataCacheCallback cb, void* cb_data) {
  devicedatacache_cb = cb;
  devicedatacache_cb_data = cb_data;
  return OCCAM_API_SUCCESS;
}

