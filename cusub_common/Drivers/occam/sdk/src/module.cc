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
#include <set>
#include <string>
#include <assert.h>
#include <string.h>
#ifdef _WIN32
#include <windows.h>
#endif // _WIN32

struct module_info_cmp {
  bool operator() (IOccamModuleInfo* lhs, IOccamModuleInfo* rhs) const {
    OccamModuleClass lhs_class;
    int lhs_priority;
    int lhs_version;
    lhs->getClass(lhs, &lhs_class);
    lhs->getPriority(lhs, &lhs_priority);
    lhs->getVersion(lhs, &lhs_version);

    OccamModuleClass rhs_class;
    int rhs_priority;
    int rhs_version;
    rhs->getClass(rhs, &rhs_class);
    rhs->getPriority(rhs, &rhs_priority);
    rhs->getVersion(rhs, &rhs_version);

    if (lhs_class != rhs_class)
      return lhs_class < rhs_class;
    if (lhs_priority != rhs_priority)
      return rhs_priority < lhs_priority;
    return rhs_version < lhs_version;
  }
};
static std::set<IOccamModuleInfo*,module_info_cmp>* loaded_modules = 0;
static void init_loaded_modules() {
  if (!loaded_modules)
    loaded_modules = new std::set<IOccamModuleInfo*,module_info_cmp>;
}

static std::string* module_keys = 0;
static void init_module_keys() {
  if (!module_keys)
    module_keys = new std::string;
}

int occamRegisterModule(IOccamModuleInfo* module_info) {
  init_loaded_modules();
  if (loaded_modules->find(module_info) != loaded_modules->end())
    return OCCAM_API_MODULE_ALREADY_LOADED;
  loaded_modules->insert(module_info);
  return OCCAM_API_SUCCESS;
}

int occamUnregisterModule(IOccamModuleInfo* module_info) {
  init_loaded_modules();
  if (loaded_modules->find(module_info) == loaded_modules->end())
    return OCCAM_API_MODULE_NOT_FOUND;
  loaded_modules->erase(module_info);
  return OCCAM_API_SUCCESS;
}

int occamEnumerateModules(OccamModuleClass module_class, IOccamModuleInfo*** ret_module_info) {
  init_loaded_modules();
  int count = 0;
  for (IOccamModuleInfo* module_info : *loaded_modules) {
    OccamModuleClass module_class0;
    module_info->getClass(module_info,&module_class0);
    if (module_class0 == module_class)
      ++count;
  }
  *ret_module_info = (IOccamModuleInfo**)occamAlloc(sizeof(IOccamModuleInfo*)*(count+1));
  int index = 0;
  for (IOccamModuleInfo* module_info : *loaded_modules) {
    OccamModuleClass module_class0;
    module_info->getClass(module_info,&module_class0);
    if (module_class0 != module_class)
      continue;
    (*ret_module_info)[index++] = module_info;
    assert(index<=count);
  }
  (*ret_module_info)[count] = 0;
  return OCCAM_API_SUCCESS;
}

int occamLoadModules(const char* path) {
#ifdef _WIN32
  DWORD file_attr = GetFileAttributes(path);
  if (file_attr & FILE_ATTRIBUTE_DIRECTORY) {
    WIN32_FIND_DATA file;
    HANDLE find_handle = NULL;

    char path0[2048];

    if((find_handle = FindFirstFile(path0, &file)) == INVALID_HANDLE_VALUE)
      return OCCAM_API_INVALID_MODULE_PATH;

    do {
      if(strcmp(file.cFileName, ".") != 0 && strcmp(file.cFileName, "..") != 0)
	occamLoadModules(path0);
    } while(FindNextFile(find_handle, &file));
  } else {
    HMODULE library_handle = LoadLibrary(path);
    if (!library_handle)
      return OCCAM_API_MODULE_FAILED_TO_LOAD;
    typedef int (*occamRegisterModule_pfn)(IOccamModuleInfo*);
    typedef int (*occamInitModule)(occamRegisterModule_pfn);
    occamInitModule init_pfn = (occamInitModule)GetProcAddress(library_handle,"occamInitModule");
    if (!init_pfn)
      return OCCAM_API_MODULE_FAILED_TO_LOAD;
    return init_pfn(occamRegisterModule);
  }
#else // _WIN32
  assert(0);
#endif // _WIN32
  return OCCAM_API_GENERIC_ERROR;
}

int occamAddModuleKeys(const char* keys) {
  init_module_keys();
  (*module_keys) += ":";
  (*module_keys) += keys;
  return OCCAM_API_SUCCESS;
}

int occamConstructModule(OccamModuleClass module_class,
			 const char* name,
			 void** ret_handle) {
  init_loaded_modules();
  init_module_keys();
  for (IOccamModuleInfo* module_info : *loaded_modules) {
    OccamModuleClass cur_module_class;
    module_info->getClass(module_info, &cur_module_class);
    if (name) {
      char* name0=0;
      module_info->getName(module_info,&name0);
      if (!name0)
	continue;
      if (strcmp(name0,name)) {
	if (name0)
	  occamFree(name0);
	continue;
      }
      if (name0)
	occamFree(name0);
    }
    if (module_class == cur_module_class)
      return module_info->construct(module_info, module_keys->c_str(), ret_handle);
  }
  return OCCAM_API_MODULE_NOT_FOUND;
}

int occamGetInterface(void* handle,
		      OccamModuleInterfaceType iface_type,
		      void** ret_iface) {
  IOccamInterfaceBase* module_base = (IOccamInterfaceBase*)handle;
  return module_base->getInterface(handle, iface_type, ret_iface);
}

int occamRetainModule(void* handle) {
  int r;
  IOccamInterfaceBase* module_base = 0;
  if (OCCAM_API_SUCCESS != (r = occamGetInterface(handle, IOCCAMINTERFACEBASE, (void**)&module_base)))
    return r;
  return module_base->retain(handle);
}

int occamReleaseModule(void* handle) {
  int r;
  IOccamInterfaceBase* module_base = 0;
  if (OCCAM_API_SUCCESS != (r = occamGetInterface(handle, IOCCAMINTERFACEBASE, (void**)&module_base)))
    return r;
  return module_base->release(handle);
}
