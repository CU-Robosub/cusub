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

#include "module_utils.h"
#include "device_iface.h"
#include <algorithm>

//////////////////////////////////////////////////////////////////////////////////
// OccamModule

OccamModule::OccamModule()
  : refcnt(1) {
  init(IOCCAMINTERFACEBASE,this);
  IOccamInterfaceBase::getInterface = _getInterface;
  IOccamInterfaceBase::retain = _retain;
  IOccamInterfaceBase::release = _release;
}

OccamModule::~OccamModule() {
}

int OccamModule::_getInterface(void* handle,OccamModuleInterfaceType iface_type,void** ret_iface) {
  IOccamInterfaceBase* iface_base = (IOccamInterfaceBase*)handle;
  OccamModule* self = static_cast<OccamModule*>(iface_base);
  for (int j=0;j<self->ifaces.size();++j)
    if (self->ifaces[j].first == iface_type) {
      *ret_iface = self->ifaces[j].second;
      return OCCAM_API_SUCCESS;
    }
  return OCCAM_API_INTERFACE_NOT_FOUND;
}

int OccamModule::_retain(void* handle) {
  IOccamInterfaceBase* iface_base = (IOccamInterfaceBase*)handle;
  OccamModule* self = static_cast<OccamModule*>(iface_base);
  ++self->refcnt;
  return OCCAM_API_SUCCESS;
}

int OccamModule::_release(void* handle) {
  IOccamInterfaceBase* iface_base = (IOccamInterfaceBase*)handle;
  OccamModule* self = static_cast<OccamModule*>(iface_base);
  if (--self->refcnt<=0)
    delete self;
  return OCCAM_API_SUCCESS;
}

void OccamModule::init(OccamModuleInterfaceType iface_type, void* iface) {
  ifaces.push_back(std::make_pair(iface_type, iface));
}

void* OccamModule::iface(OccamModuleInterfaceType iface_type) {
  for (int j=0;j<ifaces.size();++j)
    if (ifaces[j].first == iface_type)
      return ifaces[j].second;
  return 0;
}

template <class T, class B>
static T& moduleGetSelf(void* handle, OccamModuleInterfaceType iface_type) {
  IOccamInterfaceBase* iface_base = (IOccamInterfaceBase*)handle;
  OccamModule* module = static_cast<OccamModule*>(iface_base);
  return *static_cast<T*>((B*)module->iface(iface_type));
}

//////////////////////////////////////////////////////////////////////////////////
// OccamParameters

OccamParameters::ParamInfo* OccamParameters::getParam(OccamParam id) {
  for (ParamInfo& pi : _params)
    if (pi.id == id)
      return &pi;
  return 0;
}

int OccamParameters::enumerateParameters(OccamParamList** param_list) {
  *param_list = new OccamParamList;
  (*param_list)->params = new OccamParamEntry[_params.size()];
  (*param_list)->param_count = _params.size();
  for (int j=0;j<_params.size();++j) {
    ParamInfo& p0 = _params[j];
    OccamParamEntry& p1 = (*param_list)->params[j];
    memset(&p1,0,sizeof(OccamParamEntry));
    p1.id = p0.id;
    p1.name = strdup(p0.name.c_str());
    p1.storage_class = p0.storage_class;
    p1.type = p0.type;
    p1.min_value = p0.min_value;
    p1.max_value = p0.max_value;
    p1.count = p0.count;
    p1.read_only = p0.read_only;
  }
  return OCCAM_API_SUCCESS;
}

int OccamParameters::setValuei(OccamParam id,int value) {
  ParamInfo* pi = getParam(id);
  if (!pi)
    return OCCAM_API_INVALID_PARAMETER;
  if (bool(pi->seti_fn)) {
    try {
      pi->seti_fn(value);
      return OCCAM_API_SUCCESS;
    } catch (RegisterProgramError&) {
      return OCCAM_API_WRITE_ERROR;
    }
  } else if (bool(pi->setb_fn)) {
    try {
      pi->setb_fn(value?true:false);
      return OCCAM_API_SUCCESS;
    } catch (RegisterProgramError&) {
      return OCCAM_API_WRITE_ERROR;
    }
  }
  return OCCAM_API_INVALID_TYPE;
}

int OccamParameters::setValuer(OccamParam id,double value) {
  ParamInfo* pi = getParam(id);
  if (!pi)
    return OCCAM_API_INVALID_PARAMETER;
  if (!bool(pi->setr_fn))
    return OCCAM_API_INVALID_TYPE;
  try {
    pi->setr_fn(value);
    return OCCAM_API_SUCCESS;
  } catch (RegisterProgramError&) {
    return OCCAM_API_WRITE_ERROR;
  }
}

int OccamParameters::setValues(OccamParam id,const char* value) {
  ParamInfo* pi = getParam(id);
  if (!pi)
    return OCCAM_API_INVALID_PARAMETER;
  if (!bool(pi->sets_fn))
    return OCCAM_API_INVALID_TYPE;
  try {
    pi->sets_fn(value);
    return OCCAM_API_SUCCESS;
  } catch (RegisterProgramError&) {
    return OCCAM_API_WRITE_ERROR;
  }
}

int OccamParameters::setValueiv(OccamParam id,const int* values,int value_count) {
  ParamInfo* pi = getParam(id);
  if (!pi)
    return OCCAM_API_INVALID_PARAMETER;
  if (!bool(pi->setiv_fn))
    return OCCAM_API_INVALID_TYPE;
  if (value_count != pi->count)
    return OCCAM_API_INVALID_COUNT;
  try {
    pi->setiv_fn(values);
  } catch (RegisterProgramError&) {
    return OCCAM_API_GENERIC_ERROR;
  }
  return OCCAM_API_SUCCESS;
}

int OccamParameters::setValuerv(OccamParam id,const double* values,int value_count) {
  ParamInfo* pi = getParam(id);
  if (!pi)
    return OCCAM_API_INVALID_PARAMETER;
  if (!bool(pi->setrv_fn))
    return OCCAM_API_INVALID_TYPE;
  if (value_count != pi->count)
    return OCCAM_API_INVALID_COUNT;
  try {
    pi->setrv_fn(values);
  } catch (RegisterProgramError&) {
    return OCCAM_API_GENERIC_ERROR;
  }
  return OCCAM_API_SUCCESS;
}

int OccamParameters::setValuesv(OccamParam id,char** values,int value_count) {
  ParamInfo* pi = getParam(id);
  if (!pi)
    return OCCAM_API_INVALID_PARAMETER;
  if (!bool(pi->setsv_fn))
    return OCCAM_API_INVALID_TYPE;
  if (value_count != pi->count)
    return OCCAM_API_INVALID_COUNT;
  try {
    std::vector<std::string> tmp(value_count);
    for (int j=0;j<value_count;++j)
      tmp[j] = values[j];
    pi->setsv_fn(&tmp[0]);
  } catch (RegisterProgramError&) {
    return OCCAM_API_GENERIC_ERROR;
  }
  return OCCAM_API_SUCCESS;
}

int OccamParameters::getValuei(OccamParam id,int* value) {
  ParamInfo* pi = getParam(id);
  if (!pi)
    return OCCAM_API_INVALID_PARAMETER;
  if (bool(pi->geti_fn)) {
    try {
      *value = pi->geti_fn();
      return OCCAM_API_SUCCESS;
    } catch (RegisterProgramError&) {
      return OCCAM_API_READ_ERROR;
    }
  } else if (bool(pi->getb_fn)) {
    try {
      *value = pi->getb_fn()?1:0;
      return OCCAM_API_SUCCESS;
    } catch (RegisterProgramError&) {
      return OCCAM_API_READ_ERROR;
    }
  }
  return OCCAM_API_INVALID_TYPE;
}

int OccamParameters::getValuer(OccamParam id,double* value) {
  ParamInfo* pi = getParam(id);
  if (!pi)
    return OCCAM_API_INVALID_PARAMETER;
  if (!bool(pi->getr_fn))
    return OCCAM_API_INVALID_TYPE;
  try {
    *value = pi->getr_fn();
  } catch (RegisterProgramError&) {
    return OCCAM_API_READ_ERROR;
  }
  return OCCAM_API_SUCCESS;
}

int OccamParameters::getValues(OccamParam id,char** value) {
  ParamInfo* pi = getParam(id);
  if (!pi)
    return OCCAM_API_INVALID_PARAMETER;
  if (!bool(pi->gets_fn))
    return OCCAM_API_INVALID_TYPE;
  try {
    *value = strdup(pi->gets_fn().c_str());
  } catch (RegisterProgramError&) {
    return OCCAM_API_READ_ERROR;
  }
  return OCCAM_API_SUCCESS;
}

int OccamParameters::getValuep(OccamParam id,void** value) {
  ParamInfo* pi = getParam(id);
  if (!pi)
    return OCCAM_API_INVALID_PARAMETER;
  if (!bool(pi->gets_fn))
    return OCCAM_API_INVALID_TYPE;
  try {
    *value = pi->getp_fn();
  } catch (RegisterProgramError&) {
    return OCCAM_API_READ_ERROR;
  }
  return OCCAM_API_SUCCESS;
}

int OccamParameters::getValueiv(OccamParam id,int* values,int value_count) {
  ParamInfo* pi = getParam(id);
  if (!pi)
    return OCCAM_API_INVALID_PARAMETER;
  if (!bool(pi->getiv_fn))
    return OCCAM_API_INVALID_TYPE;
  if (value_count != pi->count)
    return OCCAM_API_INVALID_COUNT;
  try {
    pi->getiv_fn(values);
  } catch (RegisterProgramError&) {
    return OCCAM_API_GENERIC_ERROR;
  }
  return OCCAM_API_SUCCESS;
}

int OccamParameters::getValuerv(OccamParam id,double* values,int value_count) {
  ParamInfo* pi = getParam(id);
  if (!pi)
    return OCCAM_API_INVALID_PARAMETER;
  if (!bool(pi->getrv_fn))
    return OCCAM_API_INVALID_TYPE;
  if (value_count != pi->count)
    return OCCAM_API_INVALID_COUNT;
  try {
    pi->getrv_fn(values);
  } catch (RegisterProgramError&) {
    return OCCAM_API_GENERIC_ERROR;
  }
  return OCCAM_API_SUCCESS;
}

int OccamParameters::getValuesv(OccamParam id,char** values,int value_count) {
  ParamInfo* pi = getParam(id);
  if (!pi)
    return OCCAM_API_INVALID_PARAMETER;
  if (!bool(pi->getsv_fn))
    return OCCAM_API_INVALID_TYPE;
  if (value_count != pi->count)
    return OCCAM_API_INVALID_COUNT;
  try {
    std::vector<std::string> tmp(value_count);;
    pi->getsv_fn(&tmp[0]);
    for (int j=0;j<value_count;++j)
      values[j] = strdup(tmp[j].c_str());
  } catch (RegisterProgramError&) {
    return OCCAM_API_GENERIC_ERROR;
  }
  return OCCAM_API_SUCCESS;
}

int OccamParameters::getValuepv(OccamParam id,void** values,int value_count) {
  ParamInfo* pi = getParam(id);
  if (!pi)
    return OCCAM_API_INVALID_PARAMETER;
  if (!bool(pi->getsv_fn))
    return OCCAM_API_INVALID_TYPE;
  if (value_count != pi->count)
    return OCCAM_API_INVALID_COUNT;
  try {
    pi->getpv_fn(values);
  } catch (RegisterProgramError&) {
    return OCCAM_API_GENERIC_ERROR;
  }
  return OCCAM_API_SUCCESS;
}

int OccamParameters::getValueCount(OccamParam id,int* value_count) {
  ParamInfo* pi = getParam(id);
  if (!pi)
    return OCCAM_API_INVALID_PARAMETER;
  *value_count = pi->count;
  return OCCAM_API_SUCCESS;
}

int OccamParameters::setDefaultValueb(OccamParam id,bool value) {
  ParamInfo* pi = getParam(id);
  if (!pi)
    return OCCAM_API_INVALID_PARAMETER;
  pi->default_bool_value = value;
  pi->valid_default_value = true;
  if (bool(pi->setb_fn))
    pi->setb_fn(value);
  return OCCAM_API_SUCCESS;
}

int OccamParameters::setDefaultValuei(OccamParam id,int value) {
  ParamInfo* pi = getParam(id);
  if (!pi)
    return OCCAM_API_INVALID_PARAMETER;
  pi->default_int_value = value;
  pi->valid_default_value = true;
  if (bool(pi->seti_fn))
    pi->seti_fn(value);
  return OCCAM_API_SUCCESS;
}

int OccamParameters::setDefaultValuer(OccamParam id,double value) {
  ParamInfo* pi = getParam(id);
  if (!pi)
    return OCCAM_API_INVALID_PARAMETER;
  pi->default_real_value = value;
  pi->valid_default_value = true;
  if (bool(pi->setr_fn))
    pi->setr_fn(value);
  return OCCAM_API_SUCCESS;
}

int OccamParameters::setDefaultValues(OccamParam id,const char* value) {
  ParamInfo* pi = getParam(id);
  if (!pi)
    return OCCAM_API_INVALID_PARAMETER;
  pi->default_string_value = value;
  pi->valid_default_value = true;
  if (bool(pi->sets_fn))
    pi->sets_fn(value);
  return OCCAM_API_SUCCESS;
}

int OccamParameters::resetValue(OccamParam id) {
  ParamInfo* pi = getParam(id);
  if (!pi)
    return OCCAM_API_INVALID_PARAMETER;
  if (pi->valid_default_value) {
    if (pi->type == OCCAM_PARAM_BOOL && bool(pi->setb_fn))
      pi->setb_fn(pi->default_bool_value);
    else if (pi->type == OCCAM_PARAM_INT && bool(pi->seti_fn))
      pi->seti_fn(pi->default_int_value);
    else if (pi->type == OCCAM_PARAM_REAL && bool(pi->setr_fn))
      pi->setr_fn(pi->default_real_value);
    else if (pi->type == OCCAM_PARAM_STRING && bool(pi->sets_fn))
      pi->sets_fn(pi->default_string_value);
  }
  return OCCAM_API_SUCCESS;
}

int OccamParameters::_enumerateParameters(void* handle,OccamParamList** param_list) {
  OccamParameters& self = moduleGetSelf<OccamParameters,IOccamParameters>(handle,IOCCAMPARAMETERS);
  return self.enumerateParameters(param_list);
}

int OccamParameters::_setValuei(void* handle,OccamParam id,int value) {
  OccamParameters& self = moduleGetSelf<OccamParameters,IOccamParameters>(handle,IOCCAMPARAMETERS);
  return self.setValuei(id,value);
}

int OccamParameters::_setValuer(void* handle,OccamParam id,double value) {
  OccamParameters& self = moduleGetSelf<OccamParameters,IOccamParameters>(handle,IOCCAMPARAMETERS);
  return self.setValuer(id,value);
}

int OccamParameters::_setValues(void* handle,OccamParam id,const char* value) {
  OccamParameters& self = moduleGetSelf<OccamParameters,IOccamParameters>(handle,IOCCAMPARAMETERS);
  return self.setValues(id,value);
}

int OccamParameters::_setValueiv(void* handle,OccamParam id,const int* values,int value_count) {
  OccamParameters& self = moduleGetSelf<OccamParameters,IOccamParameters>(handle,IOCCAMPARAMETERS);
  return self.setValueiv(id,values,value_count);
}

int OccamParameters::_setValuerv(void* handle,OccamParam id,const double* values,int value_count) {
  OccamParameters& self = moduleGetSelf<OccamParameters,IOccamParameters>(handle,IOCCAMPARAMETERS);
  return self.setValuerv(id,values,value_count);
}

int OccamParameters::_setValuesv(void* handle,OccamParam id,char** values,int value_count) {
  OccamParameters& self = moduleGetSelf<OccamParameters,IOccamParameters>(handle,IOCCAMPARAMETERS);
  return self.setValuesv(id,values,value_count);
}

int OccamParameters::_getValuei(void* handle,OccamParam id,int* value) {
  OccamParameters& self = moduleGetSelf<OccamParameters,IOccamParameters>(handle,IOCCAMPARAMETERS);
  return self.getValuei(id,value);
}

int OccamParameters::_getValuer(void* handle,OccamParam id,double* value) {
  OccamParameters& self = moduleGetSelf<OccamParameters,IOccamParameters>(handle,IOCCAMPARAMETERS);
  return self.getValuer(id,value);
}

int OccamParameters::_getValues(void* handle,OccamParam id,char** value) {
  OccamParameters& self = moduleGetSelf<OccamParameters,IOccamParameters>(handle,IOCCAMPARAMETERS);
  return self.getValues(id,value);
}

int OccamParameters::_getValuep(void* handle,OccamParam id,void** value) {
  OccamParameters& self = moduleGetSelf<OccamParameters,IOccamParameters>(handle,IOCCAMPARAMETERS);
  return self.getValuep(id,value);
}

int OccamParameters::_getValueiv(void* handle,OccamParam id,int* values,int value_count) {
  OccamParameters& self = moduleGetSelf<OccamParameters,IOccamParameters>(handle,IOCCAMPARAMETERS);
  return self.getValueiv(id,values,value_count);
}

int OccamParameters::_getValuerv(void* handle,OccamParam id,double* values,int value_count) {
  OccamParameters& self = moduleGetSelf<OccamParameters,IOccamParameters>(handle,IOCCAMPARAMETERS);
  return self.getValuerv(id,values,value_count);
}

int OccamParameters::_getValuesv(void* handle,OccamParam id,char** values,int value_count) {
  OccamParameters& self = moduleGetSelf<OccamParameters,IOccamParameters>(handle,IOCCAMPARAMETERS);
  return self.getValuesv(id,values,value_count);
}

int OccamParameters::_getValuepv(void* handle,OccamParam id,void** values,int value_count) {
  OccamParameters& self = moduleGetSelf<OccamParameters,IOccamParameters>(handle,IOCCAMPARAMETERS);
  return self.getValuepv(id,values,value_count);
}

int OccamParameters::_getValueCount(void* handle,OccamParam id,int* value_count) {
  OccamParameters& self = moduleGetSelf<OccamParameters,IOccamParameters>(handle,IOCCAMPARAMETERS);
  return self.getValueCount(id,value_count);
}

int OccamParameters::_setDefaultValueb(void* handle,OccamParam id,int value) {
  OccamParameters& self = moduleGetSelf<OccamParameters,IOccamParameters>(handle,IOCCAMPARAMETERS);
  return self.setDefaultValueb(id,value?true:false);
}

int OccamParameters::_setDefaultValuei(void* handle,OccamParam id,int value) {
  OccamParameters& self = moduleGetSelf<OccamParameters,IOccamParameters>(handle,IOCCAMPARAMETERS);
  return self.setDefaultValuei(id,value);
}

int OccamParameters::_setDefaultValuer(void* handle,OccamParam id,double value) {
  OccamParameters& self = moduleGetSelf<OccamParameters,IOccamParameters>(handle,IOCCAMPARAMETERS);
  return self.setDefaultValuer(id,value);
}

int OccamParameters::_setDefaultValues(void* handle,OccamParam id,const char* value) {
  OccamParameters& self = moduleGetSelf<OccamParameters,IOccamParameters>(handle,IOCCAMPARAMETERS);
  return self.setDefaultValues(id,value);
}

int OccamParameters::_resetValue(void* handle,OccamParam id) {
  OccamParameters& self = moduleGetSelf<OccamParameters,IOccamParameters>(handle,IOCCAMPARAMETERS);
  return self.resetValue(id);
}

OccamParameters::OccamParameters() {
  init(IOCCAMPARAMETERS,static_cast<IOccamParameters*>(this));
  IOccamParameters::enumerateParameters = _enumerateParameters;
  IOccamParameters::setValuei = _setValuei;
  IOccamParameters::setValuer = _setValuer;
  IOccamParameters::setValues = _setValues;
  IOccamParameters::setValueiv = _setValueiv;
  IOccamParameters::setValuerv = _setValuerv;
  IOccamParameters::setValuesv = _setValuesv;
  IOccamParameters::getValuei = _getValuei;
  IOccamParameters::getValuer = _getValuer;
  IOccamParameters::getValues = _getValues;
  IOccamParameters::getValuep = _getValuep;
  IOccamParameters::getValueiv = _getValueiv;
  IOccamParameters::getValuerv = _getValuerv;
  IOccamParameters::getValuesv = _getValuesv;
  IOccamParameters::getValuepv = _getValuepv;
  IOccamParameters::getValueCount = _getValueCount;
  IOccamParameters::setDefaultValueb = _setDefaultValueb;
  IOccamParameters::setDefaultValuei = _setDefaultValuei;
  IOccamParameters::setDefaultValuer = _setDefaultValuer;
  IOccamParameters::setDefaultValues = _setDefaultValues;
  IOccamParameters::resetValue = _resetValue;
}

OccamParameters::~OccamParameters() {
}

void OccamParameters::registerParamb(OccamParam id,
				     const std::string& name,
				     OccamStorageClass storage_class,
				     std::function<bool()> getb_fn,
				     std::function<void(bool)> setb_fn) {
  ParamInfo& pi = *_params.emplace(_params.end());
  pi.id = id;
  pi.name = name;
  pi.storage_class = storage_class;
  pi.type = OCCAM_PARAM_BOOL;
  pi.min_value = 0;
  pi.max_value = 1;
  pi.valid_default_value = false;
  pi.count = 0;
  pi.getb_fn = getb_fn;
  pi.setb_fn = setb_fn;
  pi.read_only = !bool(setb_fn);
}

void OccamParameters::registerParami(OccamParam id,
				     const std::string& name,
				     OccamStorageClass storage_class,
				     int min_value,
				     int max_value,
				     std::function<int()> geti_fn,
				     std::function<void(int)> seti_fn) {
  ParamInfo& pi = *_params.emplace(_params.end());
  pi.id = id;
  pi.name = name;
  pi.storage_class = storage_class;
  pi.type = OCCAM_PARAM_INT;
  pi.min_value = double(min_value);
  pi.max_value = double(max_value);
  pi.valid_default_value = false;
  pi.count = 0;
  pi.geti_fn = geti_fn;
  pi.seti_fn = seti_fn;
  pi.read_only = !bool(seti_fn);
}

void OccamParameters::registerParamr(OccamParam id,
				     const std::string& name,
				     OccamStorageClass storage_class,
				     double min_value,
				     double max_value,
				     std::function<double()> getr_fn,
				     std::function<void(double)> setr_fn) {
  ParamInfo& pi = *_params.emplace(_params.end());
  pi.id = id;
  pi.name = name;
  pi.storage_class = storage_class;
  pi.type = OCCAM_PARAM_REAL;
  pi.min_value = min_value;
  pi.max_value = max_value;
  pi.valid_default_value = false;
  pi.count = 0;
  pi.getr_fn = getr_fn;
  pi.setr_fn = setr_fn;
  pi.read_only = !bool(setr_fn);
}

void OccamParameters::registerParams(OccamParam id,
				     const std::string& name,
				     OccamStorageClass storage_class,
				     std::function<std::string()> gets_fn,
				     std::function<void(const std::string&)> sets_fn) {
  ParamInfo& pi = *_params.emplace(_params.end());
  pi.id = id;
  pi.name = name;
  pi.storage_class = storage_class;
  pi.type = OCCAM_PARAM_STRING;
  pi.min_value = 0;
  pi.max_value = 0;
  pi.valid_default_value = false;
  pi.count = 0;
  pi.gets_fn = gets_fn;
  pi.sets_fn = sets_fn;
  pi.read_only = !bool(sets_fn);
}

void OccamParameters::registerParamiv(OccamParam id,
				      const std::string& name,
				      OccamStorageClass storage_class,
				      int min_value,
				      int max_value,
				      int count,
				      std::function<void(int*)> getiv_fn,
				      std::function<void(const int*)> setiv_fn) {
  ParamInfo& pi = *_params.emplace(_params.end());
  pi.id = id;
  pi.name = name;
  pi.storage_class = storage_class;
  pi.type = OCCAM_PARAM_INT_ARRAY;
  pi.min_value = double(min_value);
  pi.max_value = double(max_value);
  pi.valid_default_value = false;
  pi.count = count;
  pi.getiv_fn = getiv_fn;
  pi.setiv_fn = setiv_fn;
  pi.read_only = !bool(setiv_fn);
}

void OccamParameters::registerParamrv(OccamParam id,
				      const std::string& name,
				      OccamStorageClass storage_class,
				      double min_value,
				      double max_value,
				      int count,
				      std::function<void(double*)> getrv_fn,
				      std::function<void(const double*)> setrv_fn) {
  ParamInfo& pi = *_params.emplace(_params.end());
  pi.id = id;
  pi.name = name;
  pi.storage_class = storage_class;
  pi.type = OCCAM_PARAM_REAL_ARRAY;
  pi.min_value = min_value;
  pi.max_value = max_value;
  pi.valid_default_value = false;
  pi.count = count;
  pi.getrv_fn = getrv_fn;
  pi.setrv_fn = setrv_fn;
  pi.read_only = !bool(setrv_fn);
}

void OccamParameters::registerParamsv(OccamParam id,
				      const std::string& name,
				      OccamStorageClass storage_class,
				      int count,
				      std::function<void(std::string*)> getsv_fn,
				      std::function<void(const std::string*)> setsv_fn) {
  ParamInfo& pi = *_params.emplace(_params.end());
  pi.id = id;
  pi.name = name;
  pi.storage_class = storage_class;
  pi.type = OCCAM_PARAM_STRING_ARRAY;
  pi.min_value = 0;
  pi.max_value = 0;
  pi.valid_default_value = false;
  pi.count = count;
  pi.getsv_fn = getsv_fn;
  pi.setsv_fn = setsv_fn;
  pi.read_only = !bool(setsv_fn);
}

void OccamParameters::unregisterParam(OccamParam id) {
  auto it = std::remove_if(_params.begin(),_params.end(),[id](const ParamInfo& pi){
      return pi.id == id;
    });
  _params.erase(it,_params.end());
}

void OccamParameters::setAllowedValues(OccamParam id, const std::vector<std::pair<std::string,int> >& values) {
  ParamInfo* pi = getParam(id);
  if (!pi)
    return;
  assert(pi->type == OCCAM_PARAM_INT);
  pi->count = values.size();
  pi->getiv_fn = [values](int* ret_values){
    for (int j=0;j<values.size();++j)
      ret_values[j] = values[j].second;
  };
  pi->getsv_fn = [values](std::string* ret_values){
    for (int j=0;j<values.size();++j)
      ret_values[j] = values[j].first;
  };
}

//////////////////////////////////////////////////////////////////////////////////
// OccamStereo

int OccamStereo::_configure(void* handle,int N,int width,int height,
			    const double* const* D,const double* const* K,
			    const double* const* R,const double* const* T) {
  OccamStereo& self = moduleGetSelf<OccamStereo,IOccamStereo>(handle,IOCCAMSTEREO);
  return self.configure(N,width,height,D,K,R,T);
}

int OccamStereo::_compute(void* handle,int index,const OccamImage* img0,const OccamImage* img1,
			  OccamImage** disp) {
  OccamStereo& self = moduleGetSelf<OccamStereo,IOccamStereo>(handle,IOCCAMSTEREO);
  return self.compute(index,img0,img1,disp);
}

OccamStereo::OccamStereo() {
  init(IOCCAMSTEREO,static_cast<IOccamStereo*>(this));
  IOccamStereo::configure = _configure;
  IOccamStereo::compute = _compute;
}

OccamStereo::~OccamStereo() {
}

//////////////////////////////////////////////////////////////////////////////////
// OccamStereoRectify

int OccamStereoRectify::_configure(void* handle,int N,int width,int height,
				   const double* const* D,const double* const* K,
				   const double* const* R,const double* const* T,
				   int transposed) {
  OccamStereoRectify& self = moduleGetSelf<OccamStereoRectify,IOccamStereoRectify>(handle,IOCCAMSTEREORECTIFY);
  return self.configure(N,width,height,D,K,R,T,transposed);
}

int OccamStereoRectify::_rectify(void* handle,int index,const OccamImage* img0,OccamImage** img1) {
  OccamStereoRectify& self = moduleGetSelf<OccamStereoRectify,IOccamStereoRectify>(handle,IOCCAMSTEREORECTIFY);
  return self.rectify(index,img0,img1);
}

int OccamStereoRectify::_unrectify(void* handle,int index,const OccamImage* img0,OccamImage** img1) {
  OccamStereoRectify& self = moduleGetSelf<OccamStereoRectify,IOccamStereoRectify>(handle,IOCCAMSTEREORECTIFY);
  return self.unrectify(index,img0,img1);
}

int OccamStereoRectify::_generateCloud(void* handle,int N,const int* indices,int transform,
				       const OccamImage* const* img0,const OccamImage* const* disp0,
				       OccamPointCloud** cloud1) {
  OccamStereoRectify& self = moduleGetSelf<OccamStereoRectify,IOccamStereoRectify>(handle,IOCCAMSTEREORECTIFY);
  return self.generateCloud(N,indices,transform,img0,disp0,cloud1);
}

OccamStereoRectify::OccamStereoRectify() {
  init(IOCCAMSTEREORECTIFY,static_cast<IOccamStereoRectify*>(this));
  IOccamStereoRectify::configure = _configure;
  IOccamStereoRectify::rectify = _rectify;
  IOccamStereoRectify::unrectify = _unrectify;
  IOccamStereoRectify::generateCloud = _generateCloud;
}

OccamStereoRectify::~OccamStereoRectify() {
}

//////////////////////////////////////////////////////////////////////////////////
// OccamImageFilter

int OccamImageFilter::_compute(void* handle,const OccamImage* img0,OccamImage** img1) {
  OccamImageFilter& self = moduleGetSelf<OccamImageFilter,IOccamImageFilter>(handle,IOCCAMIMAGEFILTER);
  return self.compute(img0,img1);
}

OccamImageFilter::OccamImageFilter() {
  init(IOCCAMIMAGEFILTER,static_cast<IOccamImageFilter*>(this));
  IOccamImageFilter::compute = _compute;
}

OccamImageFilter::~OccamImageFilter() {
}

//////////////////////////////////////////////////////////////////////////////////
// OccamUndistortFilter

int OccamUndistortFilter::_configure(void* handle,int N,const int* si_x,const int* si_y,
				     const int* si_width,const int* si_height,
				     const double* const* D,const double* const* K0,const double* const* K1) {
  OccamUndistortFilter& self = moduleGetSelf<OccamUndistortFilter,IOccamUndistortFilter>(handle,IOCCAMUNDISTORTFILTER);
  return self.configure(N,si_x,si_y,si_width,si_height,D,K0,K1);
}

int OccamUndistortFilter::_compute(void* handle,const OccamImage* img0,OccamImage** img1) {
  OccamUndistortFilter& self = moduleGetSelf<OccamUndistortFilter,IOccamUndistortFilter>(handle,IOCCAMUNDISTORTFILTER);
  return self.compute(img0,img1);
}

int OccamUndistortFilter::_undistortPoints(void* handle,int N,const int* sensor_indices,
					   const float* x0,const float* y0,float* x1,float* y1) {
  OccamUndistortFilter& self = moduleGetSelf<OccamUndistortFilter,IOccamUndistortFilter>(handle,IOCCAMUNDISTORTFILTER);
  return self.undistortPoints(N,sensor_indices,x0,y0,x1,y1);
}

OccamUndistortFilter::OccamUndistortFilter() {
  init(IOCCAMUNDISTORTFILTER,static_cast<IOccamUndistortFilter*>(this));
  IOccamUndistortFilter::configure = _configure;
  IOccamUndistortFilter::compute = _compute;
  IOccamUndistortFilter::undistortPoints = _undistortPoints;
}

OccamUndistortFilter::~OccamUndistortFilter() {
}

//////////////////////////////////////////////////////////////////////////////////
// OccamBlendFilter

int OccamBlendFilter::_configure(void* handle,int N,
				 const int* sensor_width,const int* sensor_height,
				 const double* const* D,const double* const* K,
				 const double* const* R,const double* const* T) {
  OccamBlendFilter& self = moduleGetSelf<OccamBlendFilter,IOccamBlendFilter>(handle,IOCCAMBLENDFILTER);
  return self.configure(N,sensor_width,sensor_height,D,K,R,T);
}

int OccamBlendFilter::_compute(void* handle,const OccamImage* const* img0,OccamImage** img1) {
  OccamBlendFilter& self = moduleGetSelf<OccamBlendFilter,IOccamBlendFilter>(handle,IOCCAMBLENDFILTER);
  return self.compute(img0,img1);
}

OccamBlendFilter::OccamBlendFilter() {
  init(IOCCAMBLENDFILTER,static_cast<IOccamBlendFilter*>(this));
  IOccamBlendFilter::configure = _configure;
  IOccamBlendFilter::compute = _compute;
}

OccamBlendFilter::~OccamBlendFilter() {
}

//////////////////////////////////////////////////////////////////////////////////
// OccamDebugData

int OccamDebugData::_setp(void* handle,int index,void* v) {
  OccamDebugData& self = moduleGetSelf<OccamDebugData,IOccamDebugData>(handle,IOCCAMDEBUGDATA);
  return self.setp(index,v);
}

int OccamDebugData::_seti(void* handle,int index,int v) {
  OccamDebugData& self = moduleGetSelf<OccamDebugData,IOccamDebugData>(handle,IOCCAMDEBUGDATA);
  return self.seti(index,v);
}

int OccamDebugData::_setr(void* handle,int index,double v) {
  OccamDebugData& self = moduleGetSelf<OccamDebugData,IOccamDebugData>(handle,IOCCAMDEBUGDATA);
  return self.setr(index,v);
}

int OccamDebugData::_getp(void* handle,int index,void** v) {
  OccamDebugData& self = moduleGetSelf<OccamDebugData,IOccamDebugData>(handle,IOCCAMDEBUGDATA);
  return self.getp(index,v);
}

int OccamDebugData::_geti(void* handle,int index,int* v) {
  OccamDebugData& self = moduleGetSelf<OccamDebugData,IOccamDebugData>(handle,IOCCAMDEBUGDATA);
  return self.geti(index,v);
}

int OccamDebugData::_getr(void* handle,int index,double* v) {
  OccamDebugData& self = moduleGetSelf<OccamDebugData,IOccamDebugData>(handle,IOCCAMDEBUGDATA);
  return self.getr(index,v);
}

OccamDebugData::OccamDebugData() {
  init(IOCCAMDEBUGDATA,static_cast<IOccamDebugData*>(this));
  IOccamDebugData::setp = _setp;
  IOccamDebugData::seti = _seti;
  IOccamDebugData::setr = _setr;
  IOccamDebugData::getp = _getp;
  IOccamDebugData::geti = _geti;
  IOccamDebugData::getr = _getr;
}

OccamDebugData::~OccamDebugData() {
}
