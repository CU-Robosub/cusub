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
#include <map>
#include <stdint.h>

struct SerializeBufferError {
};

class SerializeBuffer : public std::vector<uint8_t> {
  int rdpos;
public:
  SerializeBuffer();
  SerializeBuffer(const std::vector<uint8_t>& x);
  void rewind();
  void clear();
  void write(const void* data, int len);
  void read(void* data, int len);
  const void* data() const;
  int remaining() const;

  SerializeBuffer& operator= (const std::vector<uint8_t>& rhs);
};

SerializeBuffer& operator<< (SerializeBuffer& sb, bool v);
SerializeBuffer& operator>> (SerializeBuffer& sb, bool& v);

SerializeBuffer& operator<< (SerializeBuffer& sb, int32_t v);
SerializeBuffer& operator>> (SerializeBuffer& sb, int32_t& v);

SerializeBuffer& operator<< (SerializeBuffer& sb, uint32_t v);
SerializeBuffer& operator>> (SerializeBuffer& sb, uint32_t& v);

SerializeBuffer& operator<< (SerializeBuffer& sb, uint8_t v);
SerializeBuffer& operator>> (SerializeBuffer& sb, uint8_t& v);

SerializeBuffer& operator<< (SerializeBuffer& sb, int8_t v);
SerializeBuffer& operator>> (SerializeBuffer& sb, int8_t& v);

SerializeBuffer& operator<< (SerializeBuffer& sb, int16_t v);
SerializeBuffer& operator>> (SerializeBuffer& sb, int16_t& v);

SerializeBuffer& operator<< (SerializeBuffer& sb, uint16_t v);
SerializeBuffer& operator>> (SerializeBuffer& sb, uint16_t& v);

SerializeBuffer& operator<< (SerializeBuffer& sb, int64_t v);
SerializeBuffer& operator>> (SerializeBuffer& sb, int64_t& v);

SerializeBuffer& operator<< (SerializeBuffer& sb, uint64_t v);
SerializeBuffer& operator>> (SerializeBuffer& sb, uint64_t& v);

SerializeBuffer& operator<< (SerializeBuffer& sb, float v);
SerializeBuffer& operator>> (SerializeBuffer& sb, float& v);

SerializeBuffer& operator<< (SerializeBuffer& sb, double v);
SerializeBuffer& operator>> (SerializeBuffer& sb, double& v);

SerializeBuffer& operator<< (SerializeBuffer& sb, const std::string& v);
SerializeBuffer& operator>> (SerializeBuffer& sb, std::string& v);

template <class T>
SerializeBuffer& operator<< (SerializeBuffer& sb, const std::vector<T>& v);
template <class T>
SerializeBuffer& operator>> (SerializeBuffer& sb, std::vector<T>& v);

template <class T>
SerializeBuffer& operator<< (SerializeBuffer& sb, const std::list<T>& v);
template <class T>
SerializeBuffer& operator>> (SerializeBuffer& sb, std::list<T>& v);

template <class T, class S>
SerializeBuffer& operator<< (SerializeBuffer& sb, const std::map<T,S>& v);
template <class T, class S>
SerializeBuffer& operator>> (SerializeBuffer& sb, std::map<T,S>& v);

#define GENERIC_SERIALIZER(T)						\
  inline SerializeBuffer& operator<< (SerializeBuffer& sb, const T& t){	\
    sb.write( &t, sizeof(T) );						\
    return sb;								\
  }									\
  inline SerializeBuffer& operator>> (SerializeBuffer& sb, T& t){	\
    sb.read( &t, sizeof(T) );						\
    return sb;								\
  }									

#include "serialize_utils.tcc"

// Local Variables:
// mode: c++
// End:
