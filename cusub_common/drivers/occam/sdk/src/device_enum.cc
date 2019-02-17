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

#include "device_enum.h"
#include "device_iface.h"
#include <vector>
#include <string>
#include <sstream>
#include <algorithm>
#include <string.h>
#include <libusb.h>
 //#define __USB200_H__
#ifdef HAVE_CYUSB
#include <CyAPI.h>
#endif // HAVE_CYUSB
#include <mutex>
#include <map>
#include <assert.h>
#include <iostream>

//#define DEBUG_DEVICE_ENUM

static std::mutex* device_enum_lock = 0;
static std::map<std::string, std::string>* device_enum_cache = 0;

//////////////////////////////////////////////////////////////////////////////////
// CachedEnumerate

CachedEnumerate::~CachedEnumerate() {
  std::unique_lock<std::mutex> g(*device_enum_lock);
  device_enum_cache->erase(loc);
}

void CachedEnumerate::cacheEnumeration(const std::string& _loc, const std::string& cid) {
  std::unique_lock<std::mutex> g(*device_enum_lock);
  assert(loc.empty());
  loc = _loc;
  device_enum_cache->insert(std::make_pair(loc, cid));
}

//////////////////////////////////////////////////////////////////////////////////
//

static bool libusb_enumerateDevices(std::vector<std::string>& cids) {
  libusb_device** device_list;
  ssize_t num_devices = libusb_get_device_list(0, &device_list);

  for (int j=0;j<num_devices;++j) {
    libusb_device* dev = device_list[j];
    struct libusb_device_descriptor desc;
    int r = libusb_get_device_descriptor(dev,&desc);
    if (r)
      return false;

    if (desc.idVendor != 0x285e)
      continue;

    std::stringstream loc;
    loc<<"libusb:"<<int(libusb_get_bus_number(dev))<<":"<<int(libusb_get_device_address(dev));

    {
      std::unique_lock<std::mutex> g(*device_enum_lock);
      auto it = device_enum_cache->find(loc.str());
      if (it != device_enum_cache->end()) {
	cids.push_back(it->second);
	continue;
      }
    }

    libusb_device_handle* handle;
    r = libusb_open(dev, &handle);
    if (r) {
      std::cerr<<"failed to open device during enum, r = "<<r<<std::endl;
      continue;
    }

    char model[1024] = {0};
    char serial[1024] = {0};
    r = libusb_get_string_descriptor_ascii(handle, desc.iProduct, (unsigned char*)model, sizeof(model));
    if (r<0) {
      std::cerr<<"failed to read model from handle "<<handle<<", r = "<<r<<std::endl;
      libusb_close(handle);
      continue;
    }
    r = libusb_get_string_descriptor_ascii(handle, desc.iSerialNumber, (unsigned char*)serial, sizeof(serial));
    if (r<0) {
      std::cerr<<"failed to read serial from handle "<<handle<<"r = "<<r<<std::endl;
      libusb_close(handle);
      continue;
    }

    std::stringstream cid;
    cid<<model<<":"<<serial;
    cids.push_back(cid.str());

    libusb_close(handle);
  }

  libusb_free_device_list(device_list,1);

  return true;
}

#ifdef HAVE_LIBUSBK
static bool libusbk_enumerateDevices(std::vector<std::string>& cids) {
  KLST_HANDLE lstdevices = 0;
  if (!LstK_Init(&lstdevices, KLST_FLAG(0)))
    return false;

  UINT device_count;
  LstK_Count(lstdevices, &device_count);
  if (device_count) {
    KLST_DEVINFO_HANDLE info_handle;
    while (LstK_MoveNext(lstdevices, &info_handle)) {
      if (info_handle->Common.Vid != 0x285e ||
	  info_handle->Common.Pid != 0x6da5)
	continue;
      std::string model = info_handle->DeviceDesc;
      std::string serial = info_handle->SerialNumber;
      for (int j=0;j<serial.size();++j)
	serial[j] = tolower(serial[j]);

      std::stringstream cid;
      cid<<model<<":"<<serial;
      cids.push_back(cid.str());
    }
  }
  LstK_Free(lstdevices);

  return true;
}
#endif // HAVE_LIBUSBK

#ifdef HAVE_CYUSB
static std::string narrow(const std::wstring& str) {
  std::ostringstream stm ;
  const std::ctype<char>& ctfacet = 
    std::use_facet< std::ctype<char> >(stm.getloc());
  for(size_t i=0;i<str.size();++i)
    stm << char(ctfacet.narrow(std::ctype<char>::_Elem(str[i]),0));
  return stm.str();
}
static bool cyusb_enumerateDevices(std::vector<std::string>& cids) {
  CCyUSBDevice* USBDevicej = new CCyUSBDevice(0,CYUSBDRV_GUID,true);
  int n = USBDevicej->DeviceCount();
  for (int j=0;j<n;j++) {
    USBDevicej->Open(j);
    if (USBDevicej->VendorID != 0x285e)
      continue;
    std::stringstream loc;
    loc<<"cyusb:"<<USBDevicej->DevPath;
    {
      std::unique_lock<std::mutex> g(*device_enum_lock);
      auto it = device_enum_cache->find(loc.str());
      if (it != device_enum_cache->end()) {
	cids.push_back(it->second);
	continue;
      }
    }

    std::stringstream cidj;
    std::string model = narrow(std::wstring(USBDevicej->Product));
    std::string serial = narrow(std::wstring(USBDevicej->SerialNumber));
    cidj<<model<<":"<<serial;
    cids.push_back(cidj.str());
  }
  delete USBDevicej;
  return true;
}
#endif // HAVE_CYUSB

bool libusb_resetDevices() {
  libusb_device** device_list;
  ssize_t num_devices = libusb_get_device_list(0, &device_list);
  for (int j=0;j<num_devices;++j) {
    libusb_device* dev = device_list[j];
    struct libusb_device_descriptor desc;
    int r = libusb_get_device_descriptor(dev,&desc);
    if (r)
      return false;
    if (desc.idVendor != 0x285e)
      continue;
    libusb_device_handle* handle;
    r = libusb_open(dev, &handle);
    if (r) {
      std::cerr<<"failed to open device during reset, r = "<<r<<std::endl;
      continue;
    }
    r = libusb_reset_device(handle);
    if (r) {
      std::cerr<<"libusb_reset_device failed, r = "<<r<<std::endl;
    }
    libusb_close(handle);
  }

  libusb_free_device_list(device_list,1);
  return true;
}

static int resetDevices() {
  if (!libusb_resetDevices())
    return OCCAM_API_GENERIC_ERROR;
  return OCCAM_API_SUCCESS;
}

int enumerateDevices(std::vector<std::string>& cids, bool common_serials) {
  if (!libusb_enumerateDevices(cids))
    return OCCAM_API_ERROR_ENUMERATING_DEVICES;
#ifdef HAVE_LIBUSBK
  if (!libusbk_enumerateDevices(cids))
    return OCCAM_API_ERROR_ENUMERATING_DEVICES;
#endif // HAVE_LIBUSBK
#ifdef HAVE_CYUSB
  if (!cyusb_enumerateDevices(cids))
    return OCCAM_API_ERROR_ENUMERATING_DEVICES;
#endif // HAVE_CYUSB

  if (common_serials) {
    for (std::string& cid : cids) {
      std::string::size_type p0 = cid.find_first_of(":");
      if (p0 == std::string::npos)
	continue;
      std::string model(cid.begin(),cid.begin()+p0);
      std::string serial(cid.begin()+p0+1,cid.end());
      serial = commonSerial(model, serial);
      std::stringstream sout;
      sout<<model<<":"<<serial;
      cid = sout.str();
    }
  }
  std::sort(cids.begin(),cids.end());
  cids.erase(std::unique(cids.begin(),cids.end()),cids.end());

  return OCCAM_API_SUCCESS;
}

int occamEnumerateDeviceList(int timeout_ms, OccamDeviceList** device_list) {
  std::vector<std::string> ret_devices;
  int r = enumerateDevices(ret_devices);
  if (r != OCCAM_API_SUCCESS)
    return r;

#ifdef DEBUG_DEVICE_ENUM
  std::cerr<<"devices:";
  for (int j=0;j<ret_devices.size();++j)
    std::cerr<<" "<<ret_devices[j].first;
  std::cerr<<std::endl;
#endif // DEBUG_DEVICE_ENUM

  *device_list = new OccamDeviceList;
  (*device_list)->entries = new OccamDeviceListEntry[ret_devices.size()];
  (*device_list)->entry_count = ret_devices.size();
  for (int j=0;j<ret_devices.size();++j)
    (*device_list)->entries[j].cid = strdup(ret_devices[j].c_str());

  return OCCAM_API_SUCCESS;
}

int occamFreeDeviceList(OccamDeviceList* device_list) {
  for (int j=0;j<device_list->entry_count;++j)
    free(device_list->entries[j].cid);
  delete [] device_list->entries;
  delete device_list;
  return OCCAM_API_SUCCESS;
}

void initDeviceEnum() {
  if (!device_enum_lock)
    device_enum_lock = new std::mutex;
  if (!device_enum_cache)
    device_enum_cache = new std::map<std::string, std::string>;
// #ifndef _WIN32
//   resetDevices();
// #endif // _WIN32
}

void shutdownDeviceEnum() {
  delete device_enum_cache;
  delete device_enum_lock;
}
