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

#include <vector>
#include <list>
#include <memory>
#include <libusb.h>
#ifdef HAVE_CYUSB
#include <CyAPI.h>
#endif // HAVE_CYUSB

class USBBuffer {
  struct libusb_transfer* xfer;
#ifdef HAVE_CYUSB
  CCyUSBEndPoint* endpoint;
  CCyIsoPktInfo* pkt_info;
  OVERLAPPED inovlap;
  PUCHAR context;
#endif // HAVE_CYUSB
  std::vector<uint8_t> _data;
  int _data_size;
  bool pending;
  bool failed;
  bool canceled;
  bool timed_out;
  static void LIBUSB_CALL __transfer_done(struct libusb_transfer* xfer0);
  void transfer_done(struct libusb_transfer* xfer0);
public:
  USBBuffer(libusb_device_handle* handle,
	    int size = 256*1024,
	    int ep = 0x81);
#ifdef HAVE_CYUSB
  USBBuffer(CCyUSBDevice* handle,
	    int size = 256*1024,
	    int ep = 0x81);
#endif // HAVE_CYUSB
  ~USBBuffer();
  void submit();
  void cancel();
  bool isPending();
  bool isFailed();
  bool isCanceled();
  bool isTimedOut();
  const uint8_t* data() const;
  uint8_t* data();
  int size() const;
};

class USBParseFIFO {
  struct BufferInfo {
    std::shared_ptr<USBBuffer> buf;
    int pos8;
    int remaining8;
  };
  std::vector<BufferInfo> bufs;
public:
  int remaining();

  void consume(void* dst, int n);
  uint32_t consume32();

  void push(std::shared_ptr<USBBuffer> buf);
  void clear();
  void flush(std::list<std::shared_ptr<USBBuffer> >& buffers);
};

// Local Variables:
// mode: c++
// End:
