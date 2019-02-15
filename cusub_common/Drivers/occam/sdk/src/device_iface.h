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
#include "rate_utils.h"
#include <functional>
#include <vector>
#include <list>
#include <string>
#include <atomic>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>

class OccamDeviceBase;

class RegisterProgramError : public std::exception {
  std::string msg;
  int error;
public:
  RegisterProgramError(const std::string& msg, int error = OCCAM_API_GENERIC_ERROR);
  virtual ~RegisterProgramError() throw();
  virtual const char* what() const throw();
  int errorCode() const;
};

class RegisterProgramResponse {
public:
  struct ResultInfo {
    uint32_t reg_index;
    uint32_t reg_value;
  };
private:
  std::vector<ResultInfo> results;
public:
  void add(uint32_t reg_index, uint32_t reg_value);
  void assign(int i, uint32_t reg_index, uint32_t reg_value);
  void clear();
  int size() const;
  uint32_t regIndex(int i) const;
  uint32_t regValue(int i) const;
  uint32_t operator[] (int i) const;
  std::string toString() const;
};

class RegisterProgram {
public:
  enum CommandType {
    CommandType_Read,
    CommandType_Write,
    CommandType_ReadWrite,
    CommandType_Sleep
  };
  struct CommandInfo {
    CommandType type;
    uint32_t reg_index;
    uint32_t reg_mask;
    uint32_t reg_value;
    uint64_t delay_us;
    bool late_bound;
  };
private:
  std::vector<CommandInfo> commands;
  template <class T>
  const char* parseNumber(const char* cmd_s, T& number, bool* late_bound = 0);
  const char* parseNextChar(const char* cmd_s, char& c, const char* expect_hint);
  const char* parseDelim(const char* cmd_s, char delim, bool optional);
  const char* parseCommandType(const char* cmd_s, CommandType& type);
  const char* parseCommand(const char* cmd_s, CommandInfo& cmd);
public:
  RegisterProgram();
  RegisterProgram(const std::string& reg_cmd);
  void add(const std::string& reg_cmd);
  void addRead(uint32_t reg_index);
  void addWrite(uint32_t reg_index, uint32_t reg_value);
  void addSleep(uint64_t delay_us);
  void bind(int index, uint32_t value);
  RegisterProgramResponse execute(OccamDeviceBase& dev);
  int size() const;
  const CommandInfo& operator[] (int index) const;
  CommandInfo& operator[] (int index);
};

class Deferred {
  friend class DeferredEvaluator;
  friend class DeviceOutput;
protected:
  struct RepBase {
    std::atomic<int> refcnt;
    std::vector<RepBase*> deps;
    std::vector<RepBase*> depees;
    std::function<void(Deferred::RepBase*)> queue_fn;
    int* frame_dep_count;
    RepBase();
    virtual ~RepBase();
    virtual void generateTyped() = 0;
    virtual void copy(void** ret_data) = 0;
    virtual OccamDataType dataType() const = 0;
    void generate(std::mutex& lock);
    void initQueue(std::function<void(Deferred::RepBase*)> _queue_fn, int* _frame_dep_count);
    void retain();
    void release();
  };
  RepBase* rep;
  void init(RepBase* new_rep, int num_deps = 0, const Deferred* const* deps = 0);
public:
  Deferred();
  Deferred(const Deferred& x);
  Deferred& operator= (const Deferred& rhs);
  ~Deferred();
  void initQueue(std::function<void(Deferred::RepBase*)> queue_fn, int* frame_dep_count);
  void copy(void** ret_data);
  OccamDataType dataType() const;
};

template <class T>
class Deferred_ : public Deferred {
  struct Rep : public RepBase {
    T value;
    bool value_valid;
    std::function<T()> gen_fn;
    virtual void generateTyped();
    virtual void copy(void** ret_data);
    virtual OccamDataType dataType() const;
  };
public:
  Deferred_();
  Deferred_(const T& value);
  Deferred_(std::function<T()> gen_fn, int num_deps, const Deferred* const* deps);
  Deferred_(std::function<T()> gen_fn, const Deferred& dep0);
  Deferred_(std::function<T()> gen_fn, const Deferred& dep0, const Deferred& dep1);
  const T& value() const;
  const T& operator* () const;
  const T* operator-> () const;
};

typedef Deferred_<std::shared_ptr<OccamMarkers> > DeferredMarkers;
typedef Deferred_<std::shared_ptr<OccamImage> > DeferredImage;
typedef Deferred_<std::shared_ptr<OccamPointCloud> > DeferredPointCloud;

class DeviceOutput {
  struct Rep {
    std::vector<std::pair<OccamDataName, Deferred> > data;
    int dep_count;
  };
  std::shared_ptr<Rep> rep;
public:
  DeviceOutput();
  void set(OccamDataName name, Deferred value);
  void queue(std::function<void(Deferred::RepBase*)> queue_fn);
  int depCount() const;
  bool pack(int req_count, const OccamDataName* req);
  bool unpack(int req_count, const OccamDataName* req, OccamDataType* ret_types, void** ret_data);
};

class DeferredEvaluator {
  std::list<DeviceOutput> pending_frames;
  std::list<DeviceOutput> reaping_frames;
  std::list<Deferred::RepBase*> pending_tasks;
  int pending_frame_count;
  int reaping_frame_count;
  int max_pending_frames;
  int max_reaping_frames;
  FrameCounter push_fps;
  FrameCounter pop_fps;

  std::vector<std::thread> threads;
  bool shutdown;
  std::mutex lock;
  std::condition_variable cond;
  void threadproc();
public:
  DeferredEvaluator();
  ~DeferredEvaluator();

  void push(const DeviceOutput& out);
  bool pop(DeviceOutput& out);

  int emitFPS() const;
  int maxPendingFrames() const;
  void setMaxPendingFrames(int value);
  int maxReapingFrames() const;
  void setMaxReapingFrames(int value);
};

class OccamDeviceBase {
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
  struct ModuleInfo {
    std::shared_ptr<void> handle;
    std::string name;
    std::string pretty_name;
    int module_index;
  };
  struct ModuleGroupInfo {
    OccamParam id;
    int selected_index;
    std::vector<ModuleInfo> modules;
  };
  std::vector<ParamInfo> _params;
  std::vector<std::shared_ptr<void> > _modules;
  std::vector<ModuleGroupInfo> _module_groups;
  std::string _cid;
  std::string _model;
  std::string _serial;
  DeferredEvaluator _deferred_eval;
  ParamInfo* getParam(OccamParam id);
protected:
  virtual int readData(DeviceOutput& out);
  virtual void availableData(std::vector<std::pair<OccamDataName,OccamDataType> >& available_data);
public:
  OccamDeviceBase(const std::string& cid);
  const std::string& cid() const;
  const std::string& model() const;
  const std::string& serial() const;
  virtual ~OccamDeviceBase();
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
  void setMinMaxValues(OccamParam id, int min_value, int max_value);
  void setMinMaxValues(OccamParam id, double min_value, double max_value);

  void addConfigurableModule(OccamParam id,
			     const std::string& name,
			     OccamModuleClass module_class);
  std::shared_ptr<void> module(OccamParam id);

  int enumerateParamList(OccamParamList** param_list);
  int setDeviceValueb(OccamParam id, bool value);
  int setDeviceValuei(OccamParam id, int value);
  int setDeviceValuer(OccamParam id, double value);
  int setDeviceValues(OccamParam id, const char* value);
  int setDeviceValueiv(OccamParam id, const int* values, int values_count);
  int setDeviceValuerv(OccamParam id, const double* values, int values_count);
  int setDeviceValuesv(OccamParam id, char** values, int values_count);
  int getDeviceValueb(OccamParam id, bool* value);
  int getDeviceValuei(OccamParam id, int* value);
  int getDeviceValuer(OccamParam id, double* value);
  int getDeviceValues(OccamParam id, char** value);
  int getDeviceValuep(OccamParam id, void** value);
  int getDeviceValueiv(OccamParam id, int* values, int value_count);
  int getDeviceValuerv(OccamParam id, double* values, int value_count);
  int getDeviceValuesv(OccamParam id, char** values, int value_count);
  int getDeviceValuepv(OccamParam id, void** values, int value_count);
  int getDeviceValueCount(OccamParam id, int* value_count);
  int setDefaultDeviceValueb(OccamParam id, bool value);
  int setDefaultDeviceValuei(OccamParam id, int value);
  int setDefaultDeviceValuer(OccamParam id, double value);
  int setDefaultDeviceValues(OccamParam id, const char* value);
  int resetDeviceValue(OccamParam id);

  virtual int writeRegister(uint32_t addr, uint32_t value);
  virtual int writeRegister(uint32_t addr, uint32_t value, uint32_t mask);
  virtual int readRegister(uint32_t addr, uint32_t* value);
  RegisterProgramResponse program(const char* prog_str, uint32_t v0=0, uint32_t v1=0, uint32_t v2=0,
				  uint32_t v3=0, uint32_t v4=0, uint32_t v5=0, uint32_t v6=0);

  virtual int writeStorage(uint32_t target, uint32_t addr, uint32_t len, const uint8_t* data);
  virtual int readStorage(uint32_t target, uint32_t addr, uint32_t len, uint8_t* data);
  virtual int writeStorage(OccamStorageClass storage_class, const std::vector<uint8_t>& data);
  virtual int readStorage(OccamStorageClass storage_class, std::vector<uint8_t>& data);
  virtual int saveSettings();
  virtual int loadSettings();
  virtual int reset();

  int readData(int req_count, const OccamDataName* req, OccamDataType* ret_types, void** ret_data, int block);
  int availableData(OccamDevice* device, int* req_count, OccamDataName** req, OccamDataType** types);

  virtual int readImage(OccamImage** image, int block);
};

class OccamMetaDeviceBase : public OccamDeviceBase {
  time_t last_update_time;
  int default_device_index;
  std::string cid_prefix;
  std::vector<std::shared_ptr<OccamDeviceBase> > devices;
public:
  OccamMetaDeviceBase(const std::string& meta_cid, int prefix_len);
  virtual ~OccamMetaDeviceBase();

  void updateDevices();
  void removeDevice(OccamDeviceBase* dev);
  virtual OccamDeviceBase* addDevice(const std::string& cid) = 0;
  virtual void initDevices() = 0;
  virtual void clearDevices() = 0;

  virtual int writeRegister(uint32_t addr, uint32_t value);
  virtual int readRegister(uint32_t addr, uint32_t* value);
  virtual int writeStorage(uint32_t target, uint32_t addr, uint32_t len, const uint8_t* data);
  virtual int readStorage(uint32_t target, uint32_t addr, uint32_t len, uint8_t* data);
  virtual int writeStorage(OccamStorageClass storage_class, const std::vector<uint8_t>& data);
  virtual int readStorage(OccamStorageClass storage_class, std::vector<uint8_t>& data);
  virtual int reset();
};

void registerDevice(const std::string& model,
		    std::function<OccamDeviceBase*(const std::string&)> ctor_fn,
		    int prefix_len = 0);
std::string commonSerial(const std::string& model, const std::string& serial);
template <class T>
void registerDevice(const std::string& model, int prefix_len = 0) {
  registerDevice(model, [](const std::string& cid){
      return static_cast<OccamDeviceBase*>(new T(cid));
    }, prefix_len);
}

#include "device_iface.tcc"

// Local Variables:
// mode: c++
// End:
