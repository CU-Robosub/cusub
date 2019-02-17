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

#include "rate_utils.h"

//////////////////////////////////////////////////////////////////////////////////
// FrameCounter

FrameCounter::FrameCounter()
  : last_time(0), count(0), last_rate(0) {
}

bool FrameCounter::increment() {
  bool r = false;
  time_t now = time(0);
  if (now != last_time) {
    last_rate = count;
    last_time = now;
    count = 0;
    r = true;
  }
  ++count;
  return r;
}

int FrameCounter::rate() const {
  return last_rate;
}

//////////////////////////////////////////////////////////////////////////////////
// DataRateCounter

DataRateCounter::DataRateCounter()
  : last_time(0),
    last_byte_count(0),
    total_bytes(0) {
}

bool DataRateCounter::increment(int bytes) {
  total_bytes += bytes;
  bool r = false;
  byte_count += bytes;
  time_t now = time(0);
  if (last_time != now) {
    last_byte_count = byte_count;
    last_time = now;
    byte_count = 0;
    r = true;
  }
  return r;
}

double DataRateCounter::rateMBs() const {
  return last_byte_count / 1024. / 1024.;
}

double DataRateCounter::totalMB() const {
  return double(total_bytes) / 1024 / 1024;
}

uint64_t DataRateCounter::totalBytes() const {
  return total_bytes;
}
