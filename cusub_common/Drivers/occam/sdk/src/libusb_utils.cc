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

#include "libusb_utils.h"
#include <assert.h>
#include <iostream>
#include <algorithm>
#undef min
#undef max

//////////////////////////////////////////////////////////////////////////////////
// USBBuffer

void USBBuffer::__transfer_done(struct libusb_transfer* xfer0) {
  ((USBBuffer*)xfer0->user_data)->transfer_done(xfer0);
}

void USBBuffer::transfer_done(struct libusb_transfer* xfer0) {
  assert(xfer == xfer0);
  pending = false;

  const char* status_str = "??";
  switch (xfer->status) {
  case LIBUSB_TRANSFER_COMPLETED: status_str = "COMPLETED"; break;
  case LIBUSB_TRANSFER_ERROR: status_str = "ERROR"; break;
  case LIBUSB_TRANSFER_TIMED_OUT: status_str = "TIMED_OUT"; break;
  case LIBUSB_TRANSFER_CANCELLED: status_str = "CANCELLED"; break;
  case LIBUSB_TRANSFER_STALL: status_str = "STALL"; break;
  case LIBUSB_TRANSFER_NO_DEVICE: status_str = "NO_DEVICE"; break;
  case LIBUSB_TRANSFER_OVERFLOW: status_str = "OVERFLOW"; break;
  }

    static int count = 0;
  ++count;
  //  std::cerr<<"transfer "<<xfer<<" done, status = "<<status_str<<", count = "<<count<<std::endl; // * debug logic

  if (xfer->status == LIBUSB_TRANSFER_TIMED_OUT) {
    //    std::cerr<<"transfer "<<xfer<<" timed out, status = "<<status_str<<std::endl;
    timed_out = true;
    return;
  }
  if (xfer->status == LIBUSB_TRANSFER_CANCELLED) {
    //    std::cerr<<"transfer "<<xfer<<" canceled, status = "<<status_str<<std::endl;
    canceled = true;
    return;
  }
  if (xfer->status != LIBUSB_TRANSFER_COMPLETED) {
    failed = true;
    //    std::cerr<<"transfer "<<xfer<<" done, status = "<<status_str<<std::endl;
    return;
  }
}

USBBuffer::USBBuffer(libusb_device_handle* handle,
		     int size,
		     int ep)
  : xfer(0),
#ifdef HAVE_CYUSB
    endpoint(0),
#endif // HAVE_CYUSB
    pending(false),
    failed(false),
    canceled(false),
    timed_out(false) {
  _data.resize(size);
  _data_size = size;

  xfer = libusb_alloc_transfer(0);
  if (!xfer)
    return;
  libusb_fill_bulk_transfer(xfer, handle, ep,
			    &_data[0], _data.size(),
			    __transfer_done, this, 2000);
  //  std::cerr<<"alloc xfer "<<xfer<<std::endl;
}

#ifdef HAVE_CYUSB
USBBuffer::USBBuffer(CCyUSBDevice* handle,
		     int size,
		     int ep)
 : xfer(0),
   endpoint(0),
   pending(false),
   failed(false),
   canceled(false),
   timed_out(false) {

  endpoint = handle->EndPointOf((UCHAR)ep);
  _data.resize(size);
  inovlap.hEvent = CreateEvent(NULL, false, false, NULL);
}
#endif // HAVE_CYUSB

USBBuffer::~USBBuffer() {
  //  std::cerr<<"destroy xfer "<<xfer<<std::endl;
#ifdef HAVE_CYUSB
  if (endpoint) {
    CloseHandle(inovlap.hEvent);
  }
#endif // HAVE_CYUSB
  if (xfer)
    libusb_free_transfer(xfer);
}

void USBBuffer::submit() {
  //  std::cerr<<"submit xfer "<<xfer<<std::endl;
  assert(!pending);
  pending = true;
  failed = false;
  canceled = false;
  timed_out = false;

#ifdef HAVE_CYUSB
  if (endpoint) {
    context = endpoint->BeginDataXfer(&_data[0], _data.size(), &inovlap);
    if (endpoint->NtStatus || endpoint->UsbdStatus) {
      failed = true;
      return;
    }
  }
#endif // HAVE_CYUSB
  if (xfer) {
    int r = libusb_submit_transfer(xfer);
  }
}

void USBBuffer::cancel() {
  if (xfer && pending) {
    //    std::cerr<<"cancel xfer "<<xfer<<std::endl;
    int r = libusb_cancel_transfer(xfer);
    if (r) {
      //      std::cerr<<"cancel xfer "<<xfer<<" failed, r = "<<r<<std::endl;
      canceled = true;
    }
  } else
    canceled = true;
}

bool USBBuffer::isPending() {
#ifdef HAVE_CYUSB
  if (pending && endpoint) {
    if (endpoint->WaitForXfer(&inovlap, 0)) {
      pending = false;
      LONG len = _data.size();
      if (!endpoint->FinishDataXfer(&_data[0], len, &inovlap, context))
	failed = true;
      else
	_data_size = len;
    } else {
      if (endpoint->LastError != ERROR_IO_PENDING) {
	endpoint->Abort();
	failed = true;
	pending = false;
      }
    }
  }
#endif // HAVE_CYUSB

  return pending;
}

bool USBBuffer::isFailed() {
  return failed;
}

bool USBBuffer::isCanceled() {
  return canceled;
}

bool USBBuffer::isTimedOut() {
  return timed_out;
}

const uint8_t* USBBuffer::data() const {
  return &_data[0];
}

uint8_t* USBBuffer::data() {
  return &_data[0];
}

int USBBuffer::size() const {
  return _data_size;
}

//////////////////////////////////////////////////////////////////////////////////
// USBParseFIFO

int USBParseFIFO::remaining() {
  int remaining8 = 0;
  for (BufferInfo& bi : bufs)
    remaining8 += bi.remaining8;
  return remaining8;
}

void USBParseFIFO::consume(void* dst, int n) {
  uint8_t* dst8 = (uint8_t*)dst;
  for (BufferInfo& bi : bufs) {
    int cplen = (std::min)(bi.remaining8,n);
    std::copy(bi.buf->data()+bi.pos8,
	      bi.buf->data()+bi.pos8+cplen,
	      (uint8_t*)dst8);
    bi.pos8 += cplen;
    bi.remaining8 -= cplen;
    dst8 += cplen;
    n -= cplen;
    if (!n)
      return;
  }
  assert(0);
}

uint32_t USBParseFIFO::consume32() {
  uint32_t v;
  consume(&v, sizeof(uint32_t));
  return v;
}

void USBParseFIFO::push(std::shared_ptr<USBBuffer> buf) {
  if (buf->size()&3) {
    assert(0);
    return;
  }
  BufferInfo& bi = *bufs.insert(bufs.end(),BufferInfo());
  bi.buf = buf;
  bi.pos8 = 0;
  bi.remaining8 = buf->size();
}

void USBParseFIFO::clear() {
  for (BufferInfo& bi : bufs)
    bi.remaining8 = 0;
}

void USBParseFIFO::flush(std::list<std::shared_ptr<USBBuffer> >& buffers) {
  while (!bufs.empty() && !bufs.begin()->remaining8) {
    bufs.begin()->buf->submit();
    buffers.push_back(bufs.begin()->buf);
    bufs.erase(bufs.begin());
  }
}
