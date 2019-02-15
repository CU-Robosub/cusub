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
#include "omni_libusb.h"
#include "device_data_cache.h"
#include "crc_utils.h"
#include <assert.h>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <list>
#include <memory>
#include <string.h>
#include <algorithm>
#undef min
#undef max

//#define DEBUG_DATA_RATES
#define DEBUG_SYNC

//////////////////////////////////////////////////////////////////////////////////
// OmniDevice

void OmniDevice::newDataAvailable(std::shared_ptr<USBBuffer>& buf) {
  if (xfer_rate.increment(buf->size())) {
#ifdef DEBUG_DATA_RATES
    std::cerr<<"OmniDevice["<<this<<"]: fx3 xfer rate: "<<xfer_rate.rateMBs()<<" MB/s"<<std::endl;
#endif // DEBUG_DATA_RATES
  }

  stream.push(buf);
  stream.flush(buffers);
  newDataAvailable(stream);
}

void OmniDevice::newDataAvailable(USBParseFIFO& stream) {
  std::unique_lock<std::mutex> g(regs_lock);

  if (!image_fr)
    initFrame();

  bool underrun = false;
  while (stream.remaining()>0&&!underrun) {
    switch (state) {
    case 0: {
      if (stream.consume32() == 0x4819bd4b)
	state = 1;
      break;
    }
    case 1: {
      addr16 = stream.consume32();
      addr16_noflags = (addr16&((1<<22)-1));
      addr16_noflags %= sensor_framebuf_size;

      uint32_t addr8 = addr16_noflags*2;
      int page_index = addr8 / 512;
      md[page_index&0xf] = addr16>>24;

      if (page_index == 0)
	state = 0;
      else {
	state = 2;
	int new_pixel_addr8 = page_index * 512;
	if (new_pixel_addr8 < pixel_addr8) {

	  if (have_metadata) {
	    image_fr->index = 0;
	    for (int j=0;j<4;++j) {
	      image_fr->index <<= 8;
	      image_fr->index += uint32_t(md[15-j]);
	    }
	    image_fr->time_ns = 0;
	    for (int j=0;j<8;++j) {
	      image_fr->time_ns <<= 8;
	      image_fr->time_ns += uint64_t(md[11-j]);
	    }
	    image_fr->time_ns = (image_fr->time_ns * timescale_p) / timescale_q;
	    //	    std::cerr<<"OmniDevice["<<this<<"] metadata: index "<<image_fr->index<<", time_ns = "<<image_fr->time_ns<<std::endl;
	  } else {
	    const int hz = sensor_assumed_fps;
	    image_fr->index = frame_count++;
	    image_fr->time_ns = uint64_t(image_fr->index) * uint64_t(1000 * 1000 * 1000) / uint64_t(hz);
	  }

	  emitFrame(image_fr);
	  initFrame();
	}
	pixel_addr8 = new_pixel_addr8;
      }
      break;
    }
    case 2: {
      if (pixel_addr8<0 || pixel_addr8>=sensor_framebuf_size*2) {
	state = 0;
	break;
      }
      if (stream.remaining()<512) {
	underrun = true;
	break;
      }
      stream.consume(image_fr->data[0]+pixel_addr8,512);

      state = 0;
      break;
    }
    default: {
      state = 0;
      break;
    }
    }
  }
}

void OmniDevice::initFrame() {
  image_fr = new OccamImage;
  memset(image_fr,0,sizeof(*image_fr));
  image_fr->cid = strdup(cid().c_str());
  memset(image_fr->timescale,0,sizeof(image_fr->timescale));
  image_fr->refcnt = 1;
  image_fr->backend = OCCAM_CPU;
  image_fr->format = OCCAM_GRAY8;
  image_fr->width = sensor_width;
  image_fr->height = sensor_subframe_height*sensor_count;
  memset(image_fr->step,0,sizeof(image_fr->step));
  memset(image_fr->data,0,sizeof(image_fr->data));
  image_fr->step[0] = sensor_width;
  image_fr->data[0] = new uint8_t[sensor_framebuf_size*2];
}

void OmniDevice::emitFrame(OccamImage* image_fr) {
  //  std::cerr<<"OmniDevice::emitFrame: "<<this<<", index = "<<image_fr->index<<std::endl;

  if (frame_rate.increment()) {
#ifdef DEBUG_DATA_RATES
    std::cerr<<"OmniDevice["<<this<<"]: FPS: "<<frame_rate.rate()<<std::endl;
#endif // DEBUG_DATA_RATES
  }

  queued_frames.push_back(image_fr);
  ++queued_frames_count;
  while (queued_frames_count > 10) {
#ifdef DEBUG_SYNC
    //    std::cerr<<"base driver drop frame"<<std::endl;
#endif // DEBUG_SYNC
    --queued_frames_count;
    assert(!queued_frames.empty());
    occamFreeImage(*queued_frames.begin());
    queued_frames.pop_front();
  }
}

bool OmniDevice::readDeviceData(int data_type, int offset, void* data, int data_len) {
  int r;

  uint32_t crc0;
  r = readStorage(0xc6,offset+data_len,sizeof(uint32_t),(uint8_t*)&crc0);
  if (r != OCCAM_API_SUCCESS) {
    std::cerr<<"readDeviceData (1) fail, r = "<<r<<std::endl;
    return false;
  }

  //  std::cerr<<"read crc "<<crc0<<", data_len = "<<data_len<<std::endl;

  if (deviceDataCacheLoad(cid().c_str(), data_type, &crc0, sizeof(crc0), data, data_len))
    return true;

  for (int j=0,k;j<data_len;j+=k) {
    k = std::min(data_len-j,512);
    r = readStorage(0xc6,offset+j,k,((uint8_t*)data)+j);
    if (r != OCCAM_API_SUCCESS) {
      std::cerr<<"readDeviceData (2) fail, r = "<<r<<std::endl;
      return false;
    }
  }

  uint32_t crc1 = crc32(0,data,data_len);
  if (crc1 != crc0) {
    std::cerr<<"readDeviceData fail, crc mismatch, "<<crc1<<" != "<<crc0<<std::endl;
    return false;
  }

  deviceDataCacheStore(cid().c_str(), data_type, &crc1, sizeof(crc1), data, data_len);

  return true;
}

bool OmniDevice::writeDeviceData(int data_type, int offset, const void* data, int data_len) {
  int r;

  uint32_t crc0 = crc32(0,data,data_len);

  r = writeStorage(0xc5,offset,data_len,(const uint8_t*)data);
  if (r != OCCAM_API_SUCCESS)
    return false;

  r = writeStorage(0xc5,offset+data_len,sizeof(uint32_t),(const uint8_t*)&crc0);
  if (r != OCCAM_API_SUCCESS)
    return false;

  return true;
}

#ifdef HAVE_CYUSB
static std::string narrow(const std::wstring& str) {
  std::ostringstream stm ;
  const std::ctype<char>& ctfacet = 
    std::use_facet< std::ctype<char> >(stm.getloc());
  for(size_t i=0;i<str.size();++i)
    stm << char(ctfacet.narrow(std::ctype<char>::_Elem(str[i]),0));
  return stm.str();
}
#endif // HAVE_CYUSB

OmniDevice::OmniDevice(const std::string& cid,
		       int _max_buffers)
  : OccamDeviceBase(cid),
    handle(0),
#ifdef HAVE_CYUSB
    USBDevice(0),
#endif // HAVE_CYUSB
    buffer_size(256*1024),
    buffer_count(0),
    max_buffers(_max_buffers),
    image_fr(0),
    pixels_pos(0),
    frame_count(0),
    queued_frames_count(0),
    state(0),
    pixel_addr8(0) {

  auto get_wire_fps = [this](){
    return int(frame_rate.rate());
  };
  auto get_wire_bps = [this](){
    return int(xfer_rate.rateMBs()*1048576);
  };
  registerParami(OCCAM_WIRE_FPS, "wire_fps", OCCAM_NOT_STORED, 0, 0, get_wire_fps);
  registerParami(OCCAM_WIRE_BPS, "wire_bps", OCCAM_NOT_STORED, 0, 0, get_wire_bps);

  timescale_p = 384615;
  timescale_q = 10000;
  have_metadata = true;
  sensor_count = 5;
  sensor_width = 752;
  sensor_height = 480;
  sensor_subframe_height = 480;
  sensor_framebuf_size = 5*752*480/2;
  sensor_assumed_fps = 60;

  // prefer libusb
  {
    libusb_device** device_list;
    ssize_t num_devices = libusb_get_device_list(0, &device_list);

    for (int j=0;j<num_devices;++j) {
      libusb_device* dev = device_list[j];
      struct libusb_device_descriptor desc;
      int r = libusb_get_device_descriptor(dev,&desc);

      if (desc.idVendor != 0x285e)
	continue;

      libusb_device_handle* handlej;
      r = libusb_open(dev, &handlej);
      if (r)
	continue;

      char model[1024];
      char serial[1024];
      r = libusb_get_string_descriptor_ascii(handlej, desc.iProduct, (unsigned char*)model, sizeof(model));
      r = libusb_get_string_descriptor_ascii(handlej, desc.iSerialNumber, (unsigned char*)serial, sizeof(serial));

      std::stringstream cidj;
      cidj<<model<<":"<<serial;
      if (cidj.str() == cid) {
	handle = handlej;

	std::stringstream loc;
	loc<<"libusb:"<<int(libusb_get_bus_number(dev))<<":"<<int(libusb_get_device_address(dev));
	cached_enum.cacheEnumeration(loc.str(), cid);

	break;
      }

      libusb_close(handlej);
    }

    libusb_free_device_list(device_list,1);

    //    std::cerr<<"OmniDevice ctor["<<this<<"]: handle "<<handle<<std::endl;

    if (handle) {
      int r;
#ifndef _WIN32
      r = libusb_reset_device(handle);
      //      std::cerr<<"libusb_reset_device = "<<r<<std::endl;
      if (r) {
	std::cerr<<"libusb_reset_device failed, r = "<<r<<std::endl;
      }
#endif // _WIN32
      r = libusb_claim_interface(handle, 0);
      //      std::cerr<<"libusb_claim_interface = "<<r<<std::endl;
      if (r) {
	std::cerr<<"libusb_claim_interface failed, r = "<<r<<std::endl;
      }
      r = libusb_set_interface_alt_setting(handle, 0, 0);
      //      std::cerr<<"libusb_set_interface_alt_setting = "<<r<<std::endl;
      if (r) {
	std::cerr<<"libusb_set_interface_alt_setting failed, r = "<<r<<std::endl;
      }
    }
  }

#ifdef HAVE_CYUSB
  // fall back to cyusb
  if (!handle) {
    CCyUSBDevice* USBDevicej = new CCyUSBDevice(0,CYUSBDRV_GUID,true);
    int n = USBDevicej->DeviceCount();
    for (int j=0;j<n;j++) {
      USBDevicej->Open(j);
      if (USBDevicej->VendorID != 0x285e)
	continue;
      std::stringstream cidj;
      cidj<<narrow(std::wstring(USBDevicej->Product))<<":"<<narrow(std::wstring(USBDevicej->SerialNumber));
      if (cidj.str() == cid) {
	USBDevice = USBDevicej;
	USBDevicej = 0;

	std::stringstream loc;
	loc<<"cyusb:"<<USBDevice->DevPath;
	//	std::cerr<<"omni cyusb enum cache: "<<loc.str()<<" --> "<<cid<<std::endl;
	cached_enum.cacheEnumeration(loc.str(), cid);

	break;
      }
    }
    delete USBDevicej;

    if (USBDevice) {
      USBDevice->SetAltIntfc(0);

      CCyUSBEndPoint* endpoint = USBDevice->EndPointOf((UCHAR)0x81);
      int ppx = 128;
      buffer_size = ppx * endpoint->MaxPktSize;
      int max_buffer_size = 0x400000;
      if (buffer_size > max_buffer_size) {
	ppx = max_buffer_size / (endpoint->MaxPktSize);
	if((ppx%8)!=0)
	  ppx -= (ppx%8);
      }
      ppx = (ppx / 8) * 8;
      buffer_size = ppx * endpoint->MaxPktSize;

      endpoint->SetXferSize(buffer_size);
    }
  }
#endif // HAVE_CYUSB

  uint32_t firmware_major = 1;
  uint32_t firmware_minor = 0;
  uint32_t firmware_patch = 0;
  readRegister(0xcc0a, &firmware_major);
  readRegister(0xcc0b, &firmware_minor);
  readRegister(0xcc0c, &firmware_patch);
  //  std::cerr<<"OmniDevice "<<this<<": firmware version = "<<firmware_major<<"."<<firmware_minor<<"."<<firmware_patch<<std::endl;
  firmware_version = (firmware_major<<16) + (firmware_minor<<8) + firmware_patch;

  registerParami(OCCAM_FIRMWARE_VERSION_A,"firmware_version_a",OCCAM_NOT_STORED,0,0,
		 [=](){return firmware_major;});
  registerParami(OCCAM_FIRMWARE_VERSION_B,"firmware_version_b",OCCAM_NOT_STORED,0,0,
		 [=](){return firmware_minor;});
  registerParami(OCCAM_FIRMWARE_VERSION_C,"firmware_version_c",OCCAM_NOT_STORED,0,0,
		 [=](){return firmware_patch;});
  registerParami(OCCAM_FIRMWARE_VERSION,"firmware_version",OCCAM_NOT_STORED,0,0,
		 [=](){return (firmware_major<<16)|(firmware_minor<<8)|firmware_patch;});
}

OmniDevice::~OmniDevice() {
#ifdef HAVE_CYUSB
  USBDevice->EndPointOf(0x81)->Abort();
  while (!buffers.empty()) {
    auto b = *buffers.begin();
    if (!b->isPending())
      buffers.erase(buffers.begin());
  }
#endif // HAVE_CYUSB
  
  if (handle) {
    std::list<std::shared_ptr<USBBuffer> > term_buffers;
    for (auto it=buffers.begin();it!=buffers.end();++it)
      (*it)->cancel();
  }

  for (OccamImage* img : queued_frames)
    occamFreeImage(img);
  if (image_fr)
    occamFreeImage(image_fr);

#ifdef HAVE_CYUSB
  if (USBDevice) {
    delete USBDevice;
  }
#endif // HAVE_CYUSB

  if (handle) {
    int r = libusb_release_interface(handle,0);
    if (r)
      std::cerr<<"libusb_release_interface failure, r = "<<r<<std::endl;
    libusb_close(handle);
  }
}

bool OmniDevice::isOpen() const {
  return !!handle;
}

bool OmniDevice::minimumFirmware(int x,int y,int z) const {
  int firmware_version_x_y_z = (x<<16) + (y<<8) + z;
  return firmware_version >= firmware_version_x_y_z;
}

int OmniDevice::writeRegister(uint32_t addr, uint32_t value) {
  if ((addr>>8)==0xdf) {
    std::unique_lock<std::mutex> g(regs_lock);
    if (addr == 0xdf00)
      sensor_count = value;
    else if (addr == 0xdf01)
      sensor_width = value;
    else if (addr == 0xdf02)
      sensor_height = value;
    else if (addr == 0xdf03)
      sensor_subframe_height = value;
    else if (addr == 0xdf04)
      sensor_framebuf_size = value;
    else if (addr == 0xdf05)
      sensor_assumed_fps = value;
    else if (addr == 0xdf06)
      have_metadata = value?true:false;
    else
      return OCCAM_API_WRITE_ERROR;
    return OCCAM_API_SUCCESS;
  }

  //  std::cerr<<"OmniDevice::writeRegister["<<this<<"]: reg "<<addr<<" (0x"<<std::hex<<addr<<std::dec<<") = "<<value<<" (0x"<<std::hex<<value<<std::dec<<")"<<std::endl;

#ifdef HAVE_CYUSB
  if (USBDevice) {
    uint64_t index_value;
    uint32_t* index_value32 = (uint32_t*)&index_value;
    index_value32[0] = addr;
    index_value32[1] = value;
    LONG len = sizeof(index_value);
    USBDevice->ControlEndPt->Target = TGT_DEVICE;
    USBDevice->ControlEndPt->ReqType = REQ_VENDOR;
    USBDevice->ControlEndPt->ReqCode = 0xc0;
    USBDevice->ControlEndPt->Direction = DIR_TO_DEVICE;
    USBDevice->ControlEndPt->Value = 0;
    USBDevice->ControlEndPt->Index = 0;
    if (!USBDevice->ControlEndPt->XferData((UCHAR*)&index_value, len, 0))
      return OCCAM_API_WRITE_ERROR;
    return OCCAM_API_SUCCESS;
  }
#endif // HAVE_CYUSB

  if (!handle)
    return OCCAM_API_WRITE_ERROR;

  uint64_t index_value;
  uint32_t* index_value32 = (uint32_t*)&index_value;
  index_value32[0] = addr;
  index_value32[1] = value;

  int r = libusb_control_transfer
    (handle, LIBUSB_RECIPIENT_DEVICE|LIBUSB_REQUEST_TYPE_VENDOR|LIBUSB_ENDPOINT_OUT,
     0xc0, 0, 0, (unsigned char*)&index_value, sizeof(index_value), 1000);
  if (r != 8)
    return OCCAM_API_WRITE_ERROR;

  return OCCAM_API_SUCCESS;
}

int OmniDevice::readRegister(uint32_t addr, uint32_t* value) {
  if ((addr>>8)==0xdf) {
    std::unique_lock<std::mutex> g(regs_lock);
    if (addr == 0xdf00)
      *value = sensor_count;
    else if (addr == 0xdf01)
      *value = sensor_width;
    else if (addr == 0xdf02)
      *value = sensor_height;
    else if (addr == 0xdf03)
      *value = sensor_subframe_height;
    else if (addr == 0xdf04)
      *value = sensor_framebuf_size;
    else if (addr == 0xdf05)
      *value = sensor_assumed_fps;
    else
      return OCCAM_API_READ_ERROR;
    return OCCAM_API_SUCCESS;
  }

  int r;
  if ((r = writeRegister(0xcc00, addr)) != OCCAM_API_SUCCESS)
    return r;

#ifdef HAVE_CYUSB
  if (USBDevice) {
    LONG len = sizeof(value);
    USBDevice->ControlEndPt->Target = TGT_DEVICE;
    USBDevice->ControlEndPt->ReqType = REQ_VENDOR;
    USBDevice->ControlEndPt->ReqCode = 0xc1;
    USBDevice->ControlEndPt->Direction = DIR_FROM_DEVICE;
    USBDevice->ControlEndPt->Value = 0;
    USBDevice->ControlEndPt->Index = 0;
    if (!USBDevice->ControlEndPt->XferData((UCHAR*)value, len, 0))
      return OCCAM_API_READ_ERROR;

    //    std::cerr<<"read reg "<<addr<<" (0x"<<std::hex<<addr<<std::dec<<") = "<<(*value)<<" (0x"<<std::hex<<(*value)<<std::dec<<")"<<std::endl;

    return OCCAM_API_SUCCESS;
  }
#endif // HAVE_CYUSB

  if (!handle)
    return OCCAM_API_READ_ERROR;

  r = libusb_control_transfer
    (handle, LIBUSB_RECIPIENT_DEVICE|LIBUSB_REQUEST_TYPE_VENDOR|LIBUSB_ENDPOINT_IN,
     0xc1, 0, 0, (unsigned char*)value, sizeof(value), 1000);
  if (r != 4)
    return OCCAM_API_READ_ERROR;

  //  std::cerr<<"OmniDevice::readRegister["<<this<<"]: reg "<<addr<<" (0x"<<std::hex<<addr<<std::dec<<") = "<<(*value)<<" (0x"<<std::hex<<(*value)<<std::dec<<")"<<std::endl;

  return OCCAM_API_SUCCESS;
}

int OmniDevice::writeStorage(uint32_t target, uint32_t addr, uint32_t len, const uint8_t* data) {
#ifdef HAVE_CYUSB
  if (USBDevice) {
    uint16_t value, index;
    if (target == 0xc5 || target == 0xc6) {
      value = 0;
      index = addr;
    } else if (target == 0xc2 || target == 0xc4) {
      value = addr & 0xffff;
      index = (addr >> 16) & 0xffff;
    }

    LONG xfer_len = len;

    USBDevice->ControlEndPt->Target = TGT_DEVICE;
    USBDevice->ControlEndPt->ReqType = REQ_VENDOR;
    USBDevice->ControlEndPt->ReqCode = target;
    USBDevice->ControlEndPt->Direction = DIR_TO_DEVICE;
    USBDevice->ControlEndPt->Value = value;
    USBDevice->ControlEndPt->Index = index;
    if (!USBDevice->ControlEndPt->XferData((PUCHAR)data, xfer_len, 0))
      return OCCAM_API_READ_ERROR;
    return OCCAM_API_SUCCESS;
  }
#endif // HAVE_CYUSB

  if (!handle)
    return OCCAM_API_WRITE_ERROR;

  uint16_t value, index;
  if (target == 0xc5 || target == 0xc6) {
    value = 0;
    index = addr;
  } else if (target == 0xc2 || target == 0xc4) {
    value = addr & 0xffff;
    index = (addr >> 16) & 0xffff;
  }
  int r = libusb_control_transfer
    (handle, LIBUSB_RECIPIENT_DEVICE|LIBUSB_REQUEST_TYPE_VENDOR|LIBUSB_ENDPOINT_OUT,
     target, value, index, (unsigned char*)data, len, 1000);
  if (r != len)
    return OCCAM_API_WRITE_ERROR;
  return OCCAM_API_SUCCESS;
}

int OmniDevice::readStorage(uint32_t target, uint32_t addr, uint32_t len, uint8_t* data) {
#ifdef HAVE_CYUSB
  if (USBDevice) {
    uint16_t value, index;
    if (target == 0xc5 || target == 0xc6) {
      value = 0;
      index = addr;
    } else if (target == 0xc2 || target == 0xc4) {
      value = addr & 0xffff;
      index = (addr >> 16) & 0xffff;
    }

    LONG xfer_len = len;
    USBDevice->ControlEndPt->Target = TGT_DEVICE;
    USBDevice->ControlEndPt->ReqType = REQ_VENDOR;
    USBDevice->ControlEndPt->ReqCode = target;
    USBDevice->ControlEndPt->Direction = DIR_FROM_DEVICE;
    USBDevice->ControlEndPt->Value = value;
    USBDevice->ControlEndPt->Index = index;
    if (!USBDevice->ControlEndPt->XferData((PUCHAR)data, xfer_len, 0))
      return OCCAM_API_READ_ERROR;
    return OCCAM_API_SUCCESS;
  }
#endif // HAVE_CYUSB

  //  std::cerr<<"handle = "<<std::hex<<handle<<", target = "<<target<<std::dec<<", addr = "<<addr<<", len = "<<len<<std::endl;

  if (!handle)
    return OCCAM_API_READ_ERROR;

  uint16_t value, index;
  if (target == 0xc5 || target == 0xc6) {
    value = 0;
    index = addr;
  } else if (target == 0xc2 || target == 0xc4) {
    value = addr & 0xffff;
    index = (addr >> 16) & 0xffff;
  }

  int r = libusb_control_transfer
    (handle, LIBUSB_RECIPIENT_DEVICE|LIBUSB_REQUEST_TYPE_VENDOR|LIBUSB_ENDPOINT_IN,
     target, value, index, data, len, 1000);
  //  std::cerr<<"readStorage, libusb_control_transfer, r = "<<r<<", len = "<<len<<std::endl;
  if (r != len)
    return OCCAM_API_READ_ERROR;
  return OCCAM_API_SUCCESS;
}

const uint32_t SETTINGS_STORAGE_MAGIC = 0x58e0a35d;

int OmniDevice::writeStorage(OccamStorageClass storage_class, const std::vector<uint8_t>& data) {
  if (storage_class != OCCAM_SETTINGS)
    return OCCAM_API_NOT_SUPPORTED;

  int offset = 4096;

  uint32_t header[] = {SETTINGS_STORAGE_MAGIC,uint32_t(data.size())};
  if (!writeDeviceData(100, offset, header, sizeof(header)))
    return OCCAM_API_WRITE_ERROR;

  offset += sizeof(uint32_t)*3;
  if (!writeDeviceData(101, offset, &data[0], data.size()))
    return OCCAM_API_WRITE_ERROR;

  return OCCAM_API_SUCCESS;
}

int OmniDevice::readStorage(OccamStorageClass storage_class, std::vector<uint8_t>& data) {
  if (storage_class != OCCAM_SETTINGS)
    return OCCAM_API_NOT_SUPPORTED;

  int offset = 4096;

  uint32_t header[] = {0,0};
  if (!readDeviceData(100, offset, header, sizeof(header)))
    return OCCAM_API_READ_ERROR;

  if (header[0] != SETTINGS_STORAGE_MAGIC)
    return OCCAM_API_READ_ERROR;

  uint32_t data_size = header[1];
  if (data_size > 16*1024)
    return OCCAM_API_READ_ERROR;
  data.resize(data_size);

  offset += sizeof(uint32_t)*3;
  if (!readDeviceData(101, offset, &data[0], data.size()))
    return OCCAM_API_READ_ERROR;

  return OCCAM_API_SUCCESS;
}

int OmniDevice::reset() {
  writeRegister(0xcc20, 1);
  return OCCAM_API_SUCCESS;
}

int OmniDevice::readImage(OccamImage** image, int block) {
  //    std::cerr<<"OmniDevice["<<this<<"]: read image"<<std::endl;

#ifdef HAVE_CYUSB
  if (!handle && !USBDevice)
    return OCCAM_API_READ_ERROR;
#else // HAVE_CYUSB
  if (!handle)
    return OCCAM_API_READ_ERROR;
#endif // HAVE_CYUSB

  if (handle) {
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    int completed = 0;
    int r = libusb_handle_events_timeout_completed(0, &tv, &completed);
    if (r)
      return OCCAM_API_READ_ERROR;
  }

  for (;;) {
    if (!update())
      return OCCAM_API_READ_ERROR;

    if (!queued_frames.empty()) {
      *image = *queued_frames.begin();
      queued_frames.pop_front();
      assert(queued_frames_count>0);
      --queued_frames_count;
      return OCCAM_API_SUCCESS;
    }

    if (!block)
      return OCCAM_API_DATA_NOT_AVAILABLE;

#ifdef HAVE_CYUSB
    if (USBDevice)
      Sleep(1);
#endif // HAVE_CYUSB

    if (handle) {
      struct timeval tv;
      tv.tv_sec = 0;
      tv.tv_usec = 1;
      int completed = 0;
      int r = libusb_handle_events_timeout_completed(0, &tv, &completed);
      if (r)
	return OCCAM_API_READ_ERROR;
    }
  }
}

bool OmniDevice::update() {
  while (buffer_count < max_buffers) {
#ifdef HAVE_CYUSB
    if (USBDevice) {
      auto b0 = std::make_shared<USBBuffer>(USBDevice,buffer_size);
      buffers.push_back(b0);
      ++buffer_count;
      b0->submit();
    }
#endif // HAVE_CYUSB
    if (handle) {
      auto b0 = std::make_shared<USBBuffer>(handle,buffer_size);
      buffers.push_back(b0);
      ++buffer_count;
      b0->submit();
    }
  }
  if (buffers.empty())
    return true;
  if ((*buffers.begin())->isFailed())
    return false;
  //  std::cerr<<"update "<<this<<", queued_frames_count = "<<queued_frames_count<<", first pending = "<<(*buffers.begin())->isPending()<<", buffers = [";;
  //    for (auto it=buffers.begin();it!=buffers.end();++it) {
  //      std::cerr<<(*it)->isPending()<<" ";
  //    }
  //    std::cerr<<"]"<<std::endl;
  int max_buffers = 100;
  while (!buffers.empty() && !(*buffers.begin())->isPending() && --max_buffers>0) {
    std::shared_ptr<USBBuffer> buf = *buffers.begin();
    buffers.pop_front();
    if (buf->isTimedOut()) {
      buf->submit();
      buffers.push_back(buf);
      continue;
    }
    newDataAvailable(buf);
  }
  return true;
}
