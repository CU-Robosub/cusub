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
#include "device_iface.h"
#include "device_enum.h"
#include "gl_utils.h"
#include <map>
#include <string>
#include <functional>
#include <algorithm>
#include <assert.h>
#include <libusb.h>
#include <string.h>
#undef min
#undef max

extern void init_omni5u3mt9v022();
extern void init_omnis5u3mt9v022();
extern void init_tgmt9v022();
extern void init_bu3ar0330();
struct DeviceModelInfo {
  std::function<OccamDeviceBase*(const std::string&)> ctor_fn;
  int prefix_len;
};
static std::map<std::string, DeviceModelInfo>* device_ctor_map = 0;

extern void init_bm_stereo();
extern void init_planar_rectify();
extern void init_debayer_filter();
extern void init_image_filter();
extern void init_undistort_filter();
extern void init_offset_blend_filter();
extern void init_cylinder_blend_filter();

int occamInitialize() {
#ifdef OCCAM_OPENGL_SUPPORT
  occamInitGL();
#endif // OCCAM_OPENGL_SUPPORT

  int r = libusb_init(NULL);
  //  libusb_set_debug(0, 3);

  initDeviceEnum();

  if (!device_ctor_map) {
    device_ctor_map = new std::map<std::string, DeviceModelInfo>;

#ifdef DEVICE_OMNI5U3MT9V022
    init_omni5u3mt9v022();
#endif
#ifdef DEVICE_OMNIS5U3MT9V022
    init_omnis5u3mt9v022();
#endif
#ifdef DEVICE_TGMT9V02
    init_tgmt9v022();
#endif
#ifdef DEVICE_BU3AR0330
    init_bu3ar0330();
#endif // DEVICE_BU3AR0330
  }

  init_bm_stereo();
  init_planar_rectify();
  init_debayer_filter();
  init_image_filter();
  init_undistort_filter();
  init_offset_blend_filter();
  init_cylinder_blend_filter();

  return r ? OCCAM_API_NOT_INITIALIZED : OCCAM_API_SUCCESS;
}

int occamShutdown() {
  libusb_exit(0);
  delete device_ctor_map;
  device_ctor_map = 0;
  shutdownDeviceEnum();

#ifdef OCCAM_OPENGL_SUPPORT
  occamShutdownGL();
#endif // OCCAM_OPENGL_SUPPORT

  return OCCAM_API_SUCCESS;
}

void* occamAlloc(int size) {
  return malloc(size);
}

void occamFree(void* ptr) {
  return free(ptr);
}

void registerDevice(const std::string& model,
		    std::function<OccamDeviceBase*(const std::string&)> ctor_fn,
		    int prefix_len) {
  assert(device_ctor_map);
  DeviceModelInfo model_info;
  model_info.ctor_fn = ctor_fn;
  model_info.prefix_len = prefix_len;
  device_ctor_map->insert(std::make_pair(model,model_info));
}

std::string commonSerial(const std::string& model, const std::string& serial0) {
  auto fi = device_ctor_map->find(model);
  if (fi == device_ctor_map->end())
    return serial0;
  std::string serial1 = serial0;
  int prefix_len = std::min(int(serial1.size()),fi->second.prefix_len);
  serial1.erase(serial1.end()-prefix_len,serial1.end());
  serial1.insert(serial1.end(),prefix_len,'0');
  return serial1;
}

int occamOpenDevice(const char* cid, OccamDevice** device) {
  std::string model(cid);
  std::string::size_type p0 = model.find_first_of(":");
  if (p0 == std::string::npos)
    return OCCAM_API_INVALID_PARAMETER;
  model.erase(model.begin()+p0,model.end());
  auto fi = device_ctor_map->find(model);
  if (fi == device_ctor_map->end())
    return OCCAM_API_INVALID_PARAMETER;
  *device = fi->second.ctor_fn(cid);
  if (!*device)
    return OCCAM_API_ERROR_ENUMERATING_DEVICES;
  return OCCAM_API_SUCCESS;
}

int occamCloseDevice(OccamDevice* device) {
  delete (OccamDeviceBase*)device;
  return OCCAM_API_SUCCESS;
}

int occamEnumerateParamList(OccamDevice* device, OccamParamList** param_list) {
  return ((OccamDeviceBase*)device)->enumerateParamList(param_list);
}

int occamFreeParamList(OccamParamList* param_list) {
  for (int j=0;j<param_list->param_count;++j)
    free(param_list->params[j].name);
  delete [] param_list->params;
  delete param_list;
  return OCCAM_API_SUCCESS;
}

int occamSetDeviceValuei(OccamDevice* device, OccamParam id, int value) {
  return ((OccamDeviceBase*)device)->setDeviceValuei(id, value);
}

int occamSetDeviceValuer(OccamDevice* device, OccamParam id, double value) {
  return ((OccamDeviceBase*)device)->setDeviceValuer(id, value);
}

int occamSetDeviceValues(OccamDevice* device, OccamParam id, const char* value) {
  return ((OccamDeviceBase*)device)->setDeviceValues(id, value);
}

int occamSetDeviceValueiv(OccamDevice* device, OccamParam id, const int* values, int value_count) {
  return ((OccamDeviceBase*)device)->setDeviceValueiv(id, values, value_count);
}

int occamSetDeviceValuerv(OccamDevice* device, OccamParam id, const double* values, int value_count) {
  return ((OccamDeviceBase*)device)->setDeviceValuerv(id, values, value_count);
}

int occamSetDeviceValuesv(OccamDevice* device, OccamParam id, char** values, int value_count) {
  return ((OccamDeviceBase*)device)->setDeviceValuesv(id, values, value_count);
}

int occamGetDeviceValuei(OccamDevice* device, OccamParam id, int* value) {
  return ((OccamDeviceBase*)device)->getDeviceValuei(id, value);
}

int occamGetDeviceValuer(OccamDevice* device, OccamParam id, double* value) {
  return ((OccamDeviceBase*)device)->getDeviceValuer(id, value);
}

int occamGetDeviceValues(OccamDevice* device, OccamParam id, char** value) {
  return ((OccamDeviceBase*)device)->getDeviceValues(id, value);
}

int occamGetDeviceValuep(OccamDevice* device, OccamParam id, void** value) {
  return ((OccamDeviceBase*)device)->getDeviceValuep(id, value);
}

int occamGetDeviceValueiv(OccamDevice* device, OccamParam id, int* values, int value_count) {
  return ((OccamDeviceBase*)device)->getDeviceValueiv(id, values, value_count);
}

int occamGetDeviceValuerv(OccamDevice* device, OccamParam id, double* values, int value_count) {
  return ((OccamDeviceBase*)device)->getDeviceValuerv(id, values, value_count);
}

int occamGetDeviceValuesv(OccamDevice* device, OccamParam id, char** values, int value_count) {
  return ((OccamDeviceBase*)device)->getDeviceValuesv(id, values, value_count);
}

int occamGetDeviceValuepv(OccamDevice* device, OccamParam id, void** values, int value_count) {
  return ((OccamDeviceBase*)device)->getDeviceValuepv(id, values, value_count);
}

int occamGetDeviceValueCount(OccamDevice* device, OccamParam id, int* value_count) {
  return ((OccamDeviceBase*)device)->getDeviceValueCount(id, value_count);
}

int occamResetDeviceValue(OccamDevice* device, OccamParam id) {
  return ((OccamDeviceBase*)device)->resetDeviceValue(id);
}

int occamDeviceReadData(OccamDevice* device, int req_count, const OccamDataName* req,
			OccamDataType* ret_types, void** ret_data, int block) {
  return ((OccamDeviceBase*)device)->readData(req_count, req, ret_types, ret_data, block);
}

int occamDeviceAvailableData(OccamDevice* device, int* req_count, OccamDataName** req, OccamDataType** types) {
  return ((OccamDeviceBase*)device)->availableData(device, req_count, req, types);
}

int occamFreeDeviceData(int req_count, OccamDataType* types, void** data) {
  for (int j=0;j<req_count;++j) {
    switch (types[j]) {
    case OCCAM_MARKERS:
      occamFreeMarkers((OccamMarkers*)data[j]);
      break;
    case OCCAM_IMAGE:
      occamFreeImage((OccamImage*)data[j]);
      break;
    case OCCAM_POINT_CLOUD:
      occamFreePointCloud((OccamPointCloud*)data[j]);
      break;
    }
  }
  occamFree(types);
  occamFree(data);
  return OCCAM_API_SUCCESS;
}

int occamWriteRegister(OccamDevice* device, uint32_t addr, uint32_t value) {
  return ((OccamDeviceBase*)device)->writeRegister(addr, value);
}

int occamReadRegister(OccamDevice* device, uint32_t addr, uint32_t* value) {
  return ((OccamDeviceBase*)device)->readRegister(addr, value);
}

int occamWriteStorage(OccamDevice* device, uint32_t target, uint32_t addr, uint32_t len, const uint8_t* data) {
  return ((OccamDeviceBase*)device)->writeStorage(target, addr, len, data);
}

int occamReadStorage(OccamDevice* device, uint32_t target, uint32_t addr, uint32_t len, uint8_t* data) {
  return ((OccamDeviceBase*)device)->readStorage(target, addr, len, data);
}

int occamSaveSettings(OccamDevice* device) {
  return ((OccamDeviceBase*)device)->saveSettings();
}

int occamResetDevice(OccamDevice* device) {
  return ((OccamDeviceBase*)device)->reset();
}

int occamGetErrorString(OccamError error, char* str, int max_len) {
  const char* str0 = 0;
  switch (error) {
  case OCCAM_API_SUCCESS: str0 = "SUCCESS"; break;
  case OCCAM_API_GENERIC_ERROR: str0 = "GENERIC_ERROR"; break;
  case OCCAM_API_NOT_INITIALIZED: str0 = "NOT_INITIALIZED"; break;
  case OCCAM_API_ALREADY_INITIALIZED: str0 = "ALREADY_INITIALIZED"; break;
  case OCCAM_API_ERROR_ENUMERATING_DEVICES: str0 = "ERROR_ENUMERATING_DEVICES"; break;
  case OCCAM_API_NOT_SUPPORTED: str0 = "NOT_SUPPORTED"; break;
  case OCCAM_API_UNSUPPORTED_DATA: str0 = "UNSUPPORTED_DATA"; break;
  case OCCAM_API_INVALID_PARAMETER: str0 = "INVALID_PARAMETER"; break;
  case OCCAM_API_INVALID_TYPE: str0 = "INVALID_TYPE"; break;
  case OCCAM_API_INVALID_COUNT: str0 = "INVALID_COUNT"; break;
  case OCCAM_API_INVALID_FORMAT: str0 = "INVALID_FORMAT"; break;
  case OCCAM_API_DATA_NOT_AVAILABLE: str0 = "DATA_NOT_AVAILABLE"; break;
  case OCCAM_API_WRITE_ERROR: str0 = "WRITE_ERROR"; break;
  case OCCAM_API_READ_ERROR: str0 = "READ_ERROR"; break;
  case OCCAM_API_FIELD_NOT_FOUND: str0 = "FIELD_NOT_FOUND"; break;
  case OCCAM_API_MODULE_ALREADY_LOADED: str0 = "MODULE_ALREADY_LOADED"; break;
  case OCCAM_API_MODULE_NOT_FOUND: str0 = "MODULE_NOT_FOUND"; break;
  case OCCAM_API_MODULE_FAILED_TO_LOAD: str0 = "MODULE_FAILED_TO_LOAD"; break;
  case OCCAM_API_MODULE_FAILED_TO_ENUMERATE: str0 = "MODULE_FAILED_TO_ENUMERATE"; break;
  case OCCAM_API_INVALID_MODULE_PATH: str0 = "INVALID_MODULE_PATH"; break;
  case OCCAM_API_INTERFACE_NOT_FOUND: str0 = "INTERFACE_NOT_FOUND"; break;
  case OCCAM_API_PARTIAL_DEVICE: str0 = "PARTIAL_DEVICE"; break;
  }
  if (!str0)
    return OCCAM_API_INVALID_PARAMETER;
  strncpy(str,str0,max_len);
  return OCCAM_API_SUCCESS;
}

