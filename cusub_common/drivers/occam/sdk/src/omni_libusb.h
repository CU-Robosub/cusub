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

#pragma once

#include "device_iface.h"
#include "device_enum.h"
#include "rate_utils.h"
#include <list>
#include <memory>
#include "libusb_utils.h"
#ifdef HAVE_CYUSB
#include <CyAPI.h>
#endif // HAVE_CYUSB

class OmniDevice : public OccamDeviceBase {
  libusb_device_handle* handle;
#ifdef HAVE_CYUSB
  CCyUSBDevice* USBDevice;
  CCyUSBEndPoint* endpoint;
#endif // HAVE_CYUSB
  std::mutex regs_lock;
  CachedEnumerate cached_enum;
  USBParseFIFO stream;
  std::list<std::shared_ptr<USBBuffer> > buffers;
  int buffer_size;
  int buffer_count;
  int max_buffers;
  DataRateCounter xfer_rate;
  FrameCounter frame_rate;
  int pixels_pos;
  int pixels_size;
  int frame_count;

  int sensor_count;
  int sensor_width;
  int sensor_height;
  int sensor_subframe_height;
  int sensor_framebuf_size;
  int sensor_assumed_fps;

  int queued_frames_count;
  std::list<OccamImage*> queued_frames;

  int state;
  uint32_t addr16;
  uint32_t addr16_noflags;
  int sensor_index;
  int pixel_addr8;
  uint8_t md[16];

  uint32_t firmware_version;
  int timescale_p;
  int timescale_q;
  bool have_metadata;

protected:
  OccamImage* image_fr;
  virtual void newDataAvailable(std::shared_ptr<USBBuffer>& buf);
  virtual void newDataAvailable(USBParseFIFO& stream);
  void initFrame();
  void emitFrame(OccamImage* image_fr);

public:
  //  OmniDevice(const std::string& cid, int max_buffers = 8);//32);
  OmniDevice(const std::string& cid, int max_buffers = 32);
  ~OmniDevice();
  bool isOpen() const;

  bool minimumFirmware(int x,int y,int z) const;

  virtual int writeRegister(uint32_t addr, uint32_t value);
  virtual int readRegister(uint32_t addr, uint32_t* value);

  virtual int writeStorage(uint32_t target, uint32_t addr, uint32_t len, const uint8_t* data);
  virtual int readStorage(uint32_t target, uint32_t addr, uint32_t len, uint8_t* data);
  virtual int writeStorage(OccamStorageClass storage_class, const std::vector<uint8_t>& data);
  virtual int readStorage(OccamStorageClass storage_class, std::vector<uint8_t>& data);
  bool readDeviceData(int data_type, int offset, void* data, int data_len);
  bool writeDeviceData(int data_type, int offset, const void* data, int data_len);
  virtual int reset();

  virtual int readImage(OccamImage** image, int block);

  bool update();
};


// Local Variables:
// mode: c++
// End:
