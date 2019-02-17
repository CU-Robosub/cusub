/*
Copyright 2011 - 2015 Occam Robotics Inc - All rights reserved.

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

#include <assert.h>

inline SerializeBuffer::SerializeBuffer()
  : rdpos(0) {
}

inline SerializeBuffer::SerializeBuffer(const std::vector<uint8_t>& x)
  : std::vector<uint8_t>(x), rdpos(0) {
}

inline void SerializeBuffer::rewind() {
  rdpos = 0;
}

inline void SerializeBuffer::clear() {
  std::vector<uint8_t>::clear();
  rewind();
}

inline void SerializeBuffer::write(const void* data, int len) {
  insert(end(), (uint8_t*)data, (uint8_t*)data + len);
}

inline void SerializeBuffer::read(void* data, int len) {
  if (rdpos + len > (int)size())
    throw SerializeBufferError();
  std::copy(begin()+rdpos,begin()+rdpos+len,(uint8_t*)data);
  rdpos += len;
}

inline const void* SerializeBuffer::data() const {
  return size() ? &*begin() : 0;
}

inline int SerializeBuffer::remaining() const {
  return size() - rdpos;
}

inline SerializeBuffer& SerializeBuffer::operator= (const std::vector<uint8_t>& rhs) {
  rdpos = 0;
  std::vector<uint8_t>::operator= (rhs);
  return *this;
}

inline SerializeBuffer& operator<< (SerializeBuffer& sb, bool v) {
  uint8_t v_ = v?1:0;
  sb.write(&v_, sizeof(v_));
  return sb;
}

inline SerializeBuffer& operator>> (SerializeBuffer& sb, bool& v) {
  uint8_t v_;
  sb.read(&v_, sizeof(v_));
  v = v_?true:false;
  return sb;
}

inline SerializeBuffer& operator<< (SerializeBuffer& sb, int32_t v) {
  sb.write(&v, sizeof(v));
  return sb;
}

inline SerializeBuffer& operator>> (SerializeBuffer& sb, int32_t& v) {
  sb.read(&v, sizeof(v));
  return sb;
}

inline SerializeBuffer& operator<< (SerializeBuffer& sb, uint32_t v) {
  sb.write(&v, sizeof(v));
  return sb;
}

inline SerializeBuffer& operator>> (SerializeBuffer& sb, uint32_t& v) {
  sb.read(&v, sizeof(v));
  return sb;
}

inline SerializeBuffer& operator<< (SerializeBuffer& sb, int8_t v) {
  sb.write(&v, sizeof(v));
  return sb;
}

inline SerializeBuffer& operator>> (SerializeBuffer& sb, int8_t& v) {
  sb.read(&v, sizeof(v));
  return sb;
}

inline SerializeBuffer& operator<< (SerializeBuffer& sb, uint8_t v) {
  sb.write(&v, sizeof(v));
  return sb;
}

inline SerializeBuffer& operator>> (SerializeBuffer& sb, uint8_t& v) {
  sb.read(&v, sizeof(v));
  return sb;
}

inline SerializeBuffer& operator<< (SerializeBuffer& sb, int16_t v) {
  sb.write(&v, sizeof(v));
  return sb;
}

inline SerializeBuffer& operator>> (SerializeBuffer& sb, int16_t& v) {
  sb.read(&v, sizeof(v));
  return sb;
}

inline SerializeBuffer& operator<< (SerializeBuffer& sb, uint16_t v) {
  sb.write(&v, sizeof(v));
  return sb;
}

inline SerializeBuffer& operator>> (SerializeBuffer& sb, uint16_t& v) {
  sb.read(&v, sizeof(v));
  return sb;
}

inline SerializeBuffer& operator<< (SerializeBuffer& sb, int64_t v) {
  sb.write(&v, sizeof(v));
  return sb;
}

inline SerializeBuffer& operator>> (SerializeBuffer& sb, int64_t& v) {
  sb.read(&v, sizeof(v));
  return sb;
}

inline SerializeBuffer& operator<< (SerializeBuffer& sb, uint64_t v) {
  sb.write(&v, sizeof(v));
  return sb;
}

inline SerializeBuffer& operator>> (SerializeBuffer& sb, uint64_t& v) {
  sb.read(&v, sizeof(v));
  return sb;
}

inline SerializeBuffer& operator<< (SerializeBuffer& sb, float v) {
  sb.write(&v, sizeof(v));
  return sb;
}

inline SerializeBuffer& operator>> (SerializeBuffer& sb, float& v) {
  sb.read(&v, sizeof(v));
  return sb;
}

inline SerializeBuffer& operator<< (SerializeBuffer& sb, double v) {
  sb.write(&v, sizeof(v));
  return sb;
}

inline SerializeBuffer& operator>> (SerializeBuffer& sb, double& v) {
  sb.read(&v, sizeof(v));
  return sb;
}

inline SerializeBuffer& operator<< (SerializeBuffer& sb, const std::string& v) {
  int32_t s = v.size();
  sb<<s;
  if (s)
    sb.write(&v[0], v.size());
  return sb;
}

inline SerializeBuffer& operator>> (SerializeBuffer& sb, std::string& v) {
  int32_t s;
  sb>>s;
  if (s) {
    v.resize(s);
    sb.read(&v[0], v.size());
  } else
    v = std::string();
  return sb;
}

template <class T>
SerializeBuffer& operator<< (SerializeBuffer& sb, const std::vector<T>& v) {
  int32_t s = v.size();
  sb<<s;
  for (int j=0;j<v.size();++j)
    sb<<v[j];
  return sb;
}

template <class T>
SerializeBuffer& operator>> (SerializeBuffer& sb, std::vector<T>& v) {
  int32_t s;
  sb>>s;
  v.resize(s);
  for (int j=0;j<s;++j)
    sb>>v[j];
  return sb;
}

template <class T>
SerializeBuffer& operator<< (SerializeBuffer& sb, const std::list<T>& v) {
  int32_t s = v.size();
  sb<<s;
  for (typename std::list<T>::const_iterator it=v.begin();it!=v.end();++it)
    sb<<*it;
  return sb;
}

template <class T>
SerializeBuffer& operator>> (SerializeBuffer& sb, std::list<T>& v) {
  int32_t s;
  sb>>s;
  v.clear();
  for (int j=0;j<s;++j) {
    T tmp;
    sb>>tmp;
    v.push_back(tmp);
  }
  return sb;
}

template <class T, class S>
SerializeBuffer& operator<< (SerializeBuffer& sb, const std::map<T,S>& v) {
  int32_t s = v.size();
  sb<<s;
  for (auto it=v.begin();it!=v.end();++it)
    sb<<it->first<<it->second;
  return sb;
}

template <class T, class S>
SerializeBuffer& operator>> (SerializeBuffer& sb, std::map<T,S>& v) {
  int32_t s;
  sb>>s;
  v.clear();
  for (int j=0;j<s;++j) {
    T t0;
    S s0;
    sb>>t0;
    sb>>s0;
    v.insert(std::make_pair(t0,s0));
  }
  return sb;
}

// Local Variables:
// mode: c++
// End:
