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

#include "indigo.h"
#include <atomic>
#include <vector>
#include <functional>
#include <string.h>
#include <string>

class OccamModule : public IOccamInterfaceBase {
  std::atomic<int> refcnt;
  std::vector<std::pair<OccamModuleInterfaceType,void*> > ifaces;
  static int _getInterface(void* handle,OccamModuleInterfaceType iface_type,void** ret_iface);
  static int _retain(void* handle);
  static int _release(void* handle);
protected:
  void init(OccamModuleInterfaceType iface_type, void* iface);
public:
  OccamModule();
  virtual ~OccamModule();
  OccamModule(const OccamModule& x) = delete;
  OccamModule& operator= (const OccamModule& rhs) = delete;
  void* iface(OccamModuleInterfaceType iface_type);
};

template <class T>
class OccamModuleFactory : public IOccamModuleInfo {
  std::string module_name;
  std::string module_pretty_name;
  OccamModuleClass module_class;
  int module_priority;
  int module_version;
  static int _construct(void* handle,const char* keys,void** ret_handle) {
    OccamModuleFactory<T>* self = (OccamModuleFactory<T>*)handle;
    *ret_handle = static_cast<IOccamInterfaceBase*>(new T());
    return OCCAM_API_SUCCESS;
  }
  static int _getName(void* handle,char** ret_name) {
    OccamModuleFactory<T>* self = (OccamModuleFactory<T>*)handle;
    *ret_name = (char*)occamAlloc(self->module_name.size()+1);
    strcpy(*ret_name,self->module_name.c_str());
    return OCCAM_API_SUCCESS;
  }
  static int _getPrettyName(void* handle,char** ret_name) {
    OccamModuleFactory<T>* self = (OccamModuleFactory<T>*)handle;
    *ret_name = (char*)occamAlloc(self->module_pretty_name.size()+1);
    strcpy(*ret_name,self->module_pretty_name.c_str());
    return OCCAM_API_SUCCESS;
  }
  static int _getClass(void* handle,OccamModuleClass* ret_class) {
    OccamModuleFactory<T>* self = (OccamModuleFactory<T>*)handle;
    *ret_class = self->module_class;
    return OCCAM_API_SUCCESS;
  }
  static int _getPriority(void* handle,int* ret_priority) {
    OccamModuleFactory<T>* self = (OccamModuleFactory<T>*)handle;
    *ret_priority = self->module_priority;
    return OCCAM_API_SUCCESS;
  }
  static int _getVersion(void* handle,int* ret_version) {
    OccamModuleFactory<T>* self = (OccamModuleFactory<T>*)handle;
    *ret_version = self->module_version;
    return OCCAM_API_SUCCESS;
  }
public:
  OccamModuleFactory(const std::string& _module_name, 
		     const std::string& _module_pretty_name, 
		     OccamModuleClass _module_class,
		     int _module_priority,
		     int _module_version)
    : module_name(_module_name), 
      module_pretty_name(_module_pretty_name),
      module_class(_module_class),
      module_priority(_module_priority),
      module_version(_module_version) {
    construct = _construct;
    getName = _getName;
    getPrettyName = _getPrettyName;
    getClass = _getClass;
    getPriority = _getPriority;
    getVersion = _getVersion;
  }
  void registerModule() {
    occamRegisterModule(this);
  }
  void unregisterModule() {
    occamUnregisterModule(this);
  }
};

class OccamParameters : public virtual OccamModule, public IOccamParameters {
  struct ParamInfo {
    OccamParam id;
    std::string name;
    OccamStorageClass storage_class;
    OccamParamType type;
    double min_value;
    double max_value;
    bool valid_default_value;
    bool default_bool_value;
    int default_int_value;
    double default_real_value;
    std::string default_string_value;
    int count;
    bool read_only;
    std::function<bool()> getb_fn;
    std::function<void(bool)> setb_fn;
    std::function<int()> geti_fn;
    std::function<void(int)> seti_fn;
    std::function<double()> getr_fn;
    std::function<void(double)> setr_fn;
    std::function<std::string()> gets_fn;
    std::function<void*()> getp_fn;
    std::function<void(const std::string&)> sets_fn;
    std::function<void(int*)> getiv_fn;
    std::function<void(const int*)> setiv_fn;
    std::function<void(double*)> getrv_fn;
    std::function<void(const double*)> setrv_fn;
    std::function<void(std::string*)> getsv_fn;
    std::function<void(const std::string*)> setsv_fn;
    std::function<void(void**)> getpv_fn;
  };
  std::vector<ParamInfo> _params;
  ParamInfo* getParam(OccamParam id);

  static int _enumerateParameters(void* handle,OccamParamList** param_list);
  static int _setValuei(void* handle,OccamParam id,int value);
  static int _setValuer(void* handle,OccamParam id,double value);
  static int _setValues(void* handle,OccamParam id,const char* value);
  static int _setValueiv(void* handle,OccamParam id,const int* values,int value_count);
  static int _setValuerv(void* handle,OccamParam id,const double* values,int value_count);
  static int _setValuesv(void* handle,OccamParam id,char** values,int value_count);
  static int _getValuei(void* handle,OccamParam id,int* value);
  static int _getValuer(void* handle,OccamParam id,double* value);
  static int _getValues(void* handle,OccamParam id,char** value);
  static int _getValuep(void* handle,OccamParam id,void** value);
  static int _getValueiv(void* handle,OccamParam id,int* values,int value_count);
  static int _getValuerv(void* handle,OccamParam id,double* values,int value_count);
  static int _getValuesv(void* handle,OccamParam id,char** values,int value_count);
  static int _getValuepv(void* handle,OccamParam id,void** values,int value_count);
  static int _getValueCount(void* handle,OccamParam id,int* value_count);
  static int _setDefaultValueb(void* handle,OccamParam id,int value);
  static int _setDefaultValuei(void* handle,OccamParam id,int value);
  static int _setDefaultValuer(void* handle,OccamParam id,double value);
  static int _setDefaultValues(void* handle,OccamParam id,const char* value);
  static int _resetValue(void* handle,OccamParam id);

protected:
  virtual int enumerateParameters(OccamParamList** param_list);
  virtual int setValuei(OccamParam id,int value);
  virtual int setValuer(OccamParam id,double value);
  virtual int setValues(OccamParam id,const char* value);
  virtual int setValueiv(OccamParam id,const int* values,int value_count);
  virtual int setValuerv(OccamParam id,const double* values,int value_count);
  virtual int setValuesv(OccamParam id,char** values,int value_count);
  virtual int getValuei(OccamParam id,int* value);
  virtual int getValuer(OccamParam id,double* value);
  virtual int getValues(OccamParam id,char** value);
  virtual int getValuep(OccamParam id,void** value);
  virtual int getValueiv(OccamParam id,int* values,int value_count);
  virtual int getValuerv(OccamParam id,double* values,int value_count);
  virtual int getValuesv(OccamParam id,char** values,int value_count);
  virtual int getValuepv(OccamParam id,void** values,int value_count);
  virtual int getValueCount(OccamParam id,int* value_count);
  virtual int setDefaultValueb(OccamParam id,bool value);
  virtual int setDefaultValuei(OccamParam id,int value);
  virtual int setDefaultValuer(OccamParam id,double value);
  virtual int setDefaultValues(OccamParam id,const char* value);
  virtual int resetValue(OccamParam id);
public:
  OccamParameters();
  virtual ~OccamParameters();

  void registerParamb(OccamParam id,
		      const std::string& name,
		      OccamStorageClass storage_class,
		      std::function<bool()> getb_fn,
		      std::function<void(bool)> setb_fn = std::function<void(bool)>());
  void registerParami(OccamParam id,
		      const std::string& name,
		      OccamStorageClass storage_class,
		      int min_value,
		      int max_value,
		      std::function<int()> geti_fn,
		      std::function<void(int)> seti_fn = std::function<void(int)>());
  void registerParamr(OccamParam id,
		      const std::string& name,
		      OccamStorageClass storage_class,
		      double min_value,
		      double max_value,
		      std::function<double()> getr_fn,
		      std::function<void(double)> setr_fn = std::function<void(double)>());
  void registerParams(OccamParam id,
		      const std::string& name,
		      OccamStorageClass storage_class,
		      std::function<std::string()> gets_fn,
		      std::function<void(const std::string&)> sets_fn = std::function<void(const std::string&)>());
  void registerParamiv(OccamParam id,
		       const std::string& name,
		       OccamStorageClass storage_class,
		       int min_value,
		       int max_value,
		       int count,
		       std::function<void(int*)> getiv_fn,
		       std::function<void(const int*)> setiv_fn = std::function<void(const int*)>());
  void registerParamrv(OccamParam id,
		       const std::string& name,
		       OccamStorageClass storage_class,
		       double min_value,
		       double max_value,
		       int count,
		       std::function<void(double*)> getrv_fn,
		       std::function<void(const double*)> setrv_fn = std::function<void(const double*)>());
  void registerParamsv(OccamParam id,
		       const std::string& name,
		       OccamStorageClass storage_class,
		       int count,
		       std::function<void(std::string*)> getsv_fn,
		       std::function<void(const std::string*)> setsv_fn = std::function<void(const std::string*)>());
  void unregisterParam(OccamParam id);
  void setAllowedValues(OccamParam id, const std::vector<std::pair<std::string,int> >& values);
};

class OccamStereo : public virtual OccamModule, public IOccamStereo {
  static int _configure(void* handle,int N,int width,int height,
			const double* const* D,const double* const* K,
			const double* const* R,const double* const* T);
  static int _compute(void* handle,int index,const OccamImage* img0,const OccamImage* img1,
		      OccamImage** disp);

protected:
  virtual int configure(int N,int width,int height,
			const double* const* D,const double* const* K,
			const double* const* R,const double* const* T) = 0;
  virtual int compute(int index,const OccamImage* img0,const OccamImage* img1,
		      OccamImage** disp) = 0;
public:
  OccamStereo();
  virtual ~OccamStereo();
};

class OccamStereoRectify : public virtual OccamModule, public IOccamStereoRectify {
  static int _configure(void* handle,int N,int width,int height,
			const double* const* D,const double* const* K,
			const double* const* R,const double* const* T,
			int transposed);
  static int _rectify(void* handle,int index,const OccamImage* img0,OccamImage** img1);
  static int _unrectify(void* handle,int index,const OccamImage* img0,OccamImage** img1);
  static int _generateCloud(void* handle,int N,const int* indices,int transform,
			    const OccamImage* const* img0,const OccamImage* const* disp0,
			    OccamPointCloud** cloud1);

protected:
  virtual int configure(int N,int width,int height,
			const double* const* D,const double* const* K,
			const double* const* R,const double* const* T,
			int transposed) = 0;
  virtual int rectify(int index,const OccamImage* img0,OccamImage** img1) = 0;
  virtual int unrectify(int index,const OccamImage* img0,OccamImage** img1) = 0;
  virtual int generateCloud(int N,const int* indices,int transform,
			    const OccamImage* const* img0,const OccamImage* const* disp0,
			    OccamPointCloud** cloud1) = 0;
public:
  OccamStereoRectify();
  virtual ~OccamStereoRectify();
};

class OccamImageFilter : public virtual OccamModule, public IOccamImageFilter {
  static int _compute(void* handle,const OccamImage* img0,OccamImage** img1);

protected:
  virtual int compute(const OccamImage* img0,OccamImage** img1) = 0;
public:
  OccamImageFilter();
  virtual ~OccamImageFilter();
};

class OccamUndistortFilter : public virtual OccamModule, public IOccamUndistortFilter {
  static int _configure(void* handle,int N,const int* si_x,const int* si_y,
			const int* si_width,const int* si_height,
			const double* const* D,const double* const* K0,const double* const* K1);
  static int _compute(void* handle,const OccamImage* img0,OccamImage** img1);
  static int _undistortPoints(void* handle,int N,const int* sensor_indices,
			      const float* x0,const float* y0,float* x1,float* y1);

protected:
  virtual int configure(int N,const int* si_x,const int* si_y,
			const int* si_width,const int* si_height,
			const double* const* D,const double* const* K0,const double* const* K1) = 0;
  virtual int compute(const OccamImage* img0,OccamImage** img1) = 0;
  virtual int undistortPoints(int N,const int* sensor_indices,
			      const float* x0,const float* y0,float* x1,float* y1) = 0;
public:
  OccamUndistortFilter();
  virtual ~OccamUndistortFilter();
};

class OccamBlendFilter : public virtual OccamModule, public IOccamBlendFilter {
  static int _configure(void* handle,int N,
			const int* sensor_width,const int* sensor_height,
			const double* const* D,const double* const* K,
			const double* const* R,const double* const* T);
  static int _compute(void* handle,const OccamImage* const* img0,OccamImage** img1);

protected:
  virtual int configure(int N,
			const int* sensor_width,const int* sensor_height,
			const double* const* D,const double* const* K,
			const double* const* R,const double* const* T) = 0;
  virtual int compute(const OccamImage* const* img0,OccamImage** img1) = 0;
public:
  OccamBlendFilter();
  virtual ~OccamBlendFilter();
};

class OccamDebugData : public virtual OccamModule, public IOccamDebugData {
  static int _setp(void* handle,int index,void* v);
  static int _seti(void* handle,int index,int v);
  static int _setr(void* handle,int index,double v);
  static int _getp(void* handle,int index,void** v);
  static int _geti(void* handle,int index,int* v);
  static int _getr(void* handle,int index,double* v);

protected:
  virtual int setp(int index,void* v) = 0;
  virtual int seti(int index,int v) = 0;
  virtual int setr(int index,double v) = 0;
  virtual int getp(int index,void** v) = 0;
  virtual int geti(int index,int* v) = 0;
  virtual int getr(int index,double* v) = 0;
public:
  OccamDebugData();
  virtual ~OccamDebugData();
};

// Local Variables:
// mode: c++
// End:

