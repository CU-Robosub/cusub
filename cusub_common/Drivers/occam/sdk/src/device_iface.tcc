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

//////////////////////////////////////////////////////////////////////////////////
// template <class T> Deferred_

template <class T>
void Deferred_<T>::Rep::generateTyped() {
  if (value_valid || !bool(gen_fn))
    return;
  assert(!value_valid);
  value = gen_fn();
  value_valid = true;
}

template <>
inline void Deferred_<std::shared_ptr<OccamMarkers> >::Rep::copy(void** ret_data) {
  occamCopyMarkers(&*value, (OccamMarkers**)ret_data, 0);
}

template <>
inline void Deferred_<std::shared_ptr<OccamImage> >::Rep::copy(void** ret_data) {
  occamCopyImage(&*value, (OccamImage**)ret_data, 0);
}

template <>
inline void Deferred_<std::shared_ptr<OccamPointCloud> >::Rep::copy(void** ret_data) {
  occamCopyPointCloud(&*value, (OccamPointCloud**)ret_data, 0);
}

template <>
inline OccamDataType Deferred_<std::shared_ptr<OccamMarkers> >::Rep::dataType() const {
  return OCCAM_MARKERS;
}

template <>
inline OccamDataType Deferred_<std::shared_ptr<OccamImage> >::Rep::dataType() const {
  return OCCAM_IMAGE;
}

template <>
inline OccamDataType Deferred_<std::shared_ptr<OccamPointCloud> >::Rep::dataType() const {
  return OCCAM_POINT_CLOUD;
}

template <class T>
Deferred_<T>::Deferred_() {
}

template <class T>
Deferred_<T>::Deferred_(const T& value) {
  Rep* r = new Rep;
  r->value = value;
  r->value_valid = true;
  init(r);
}

template <class T>
Deferred_<T>::Deferred_(std::function<T()> gen_fn, int num_deps, const Deferred* const* deps) {
  Rep* r = new Rep;
  r->gen_fn = gen_fn;
  r->value_valid = false;
  init(r, num_deps, deps);
}

template <class T>
Deferred_<T>::Deferred_(std::function<T()> gen_fn, const Deferred& dep0) {
  Rep* r = new Rep;
  r->gen_fn = gen_fn;
  r->value_valid = false;
  Deferred* deps[] = {(Deferred*)&dep0};
  init(r, 1, deps);
}

template <class T>
Deferred_<T>::Deferred_(std::function<T()> gen_fn, const Deferred& dep0, const Deferred& dep1) {
  Rep* r = new Rep;
  r->gen_fn = gen_fn;
  r->value_valid = false;
  Deferred* deps[] = {(Deferred*)&dep0,(Deferred*)&dep1};
  init(r, 2, deps);
}

template <class T>
const T& Deferred_<T>::value() const {
  return static_cast<Rep*>(Deferred::rep)->value;
}

template <class T>
const T& Deferred_<T>::operator* () const {
  return value();
}

template <class T>
const T* Deferred_<T>::operator-> () const {
  return &value();
}

// Local Variables:
// mode: c++
// End:
