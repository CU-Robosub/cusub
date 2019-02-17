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

#include "device_iface.h"
#include "device_enum.h"
#include "serialize_utils.h"
#include <sstream>
#include <algorithm>
#include <assert.h>
#include <string.h>
#ifdef _WIN32
#include <windows.h>
#else // _WIN32
#include <unistd.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <sys/types.h>
#endif // _WIN32
#include <iostream>
#undef min
#undef max

//#define DEBUG_DATA_RATES
//#define DEBUG_SYNC

////////////////////////////////////////////////////////////////////////
// RegisterProgramError

RegisterProgramError::RegisterProgramError(const std::string& _msg, int _error)
  : msg(_msg),
    error(_error) {
}

RegisterProgramError::~RegisterProgramError() throw() {
}

const char* RegisterProgramError::what() const throw() {
  return msg.c_str();
}

int RegisterProgramError::errorCode() const {
  return error;
}

////////////////////////////////////////////////////////////////////////
// RegisterProgramResponse

void RegisterProgramResponse::add(uint32_t reg_index, uint32_t reg_value){ 
  ResultInfo& res = *results.insert(results.end(),ResultInfo());
  res.reg_index = reg_index;
  res.reg_value = reg_value;
}

void RegisterProgramResponse::assign(int i, uint32_t reg_index, uint32_t reg_value) {
  assert(i>=0&&i<results.size());
  ResultInfo& res = results[i];
  res.reg_index = reg_index;
  res.reg_value = reg_value;
}

void RegisterProgramResponse::clear() {
  results.clear();
}

int RegisterProgramResponse::size() const {
  return results.size();
}

uint32_t RegisterProgramResponse::regIndex(int i) const {
  return results[i].reg_index;
}

uint32_t RegisterProgramResponse::regValue(int i) const {
  return results[i].reg_value;
}

uint32_t RegisterProgramResponse::operator[] (int i) const {
  return results[i].reg_value;
}

std::string RegisterProgramResponse::toString() const {
  std::stringstream sout;
  for (int j=0;j<results.size();++j) {
    const ResultInfo& r = results[j];
    sout<<(j?", ":"")<<std::hex<<"0x"<<r.reg_index<<"=0x"<<r.reg_value<<std::dec;
  }
  return sout.str();
}

////////////////////////////////////////////////////////////////////////
// RegisterProgram

template <class T>
const char* RegisterProgram::parseNumber(const char* cmd_s, T& number, bool* late_bound) {
  int number_type = 0;
  int digit_count = 0;
  number = 0;
  int state = 0;
  const char* chars[] = 
    {"0123456789",
     "0123456789abcdef",
     "01"
    };
  int base[] = {10,16,2};
  if (late_bound)
    *late_bound = false;
  for (;;) {
    if (!*cmd_s) {
      std::stringstream sout;
      sout<<"Syntax error, expected number";
      throw RegisterProgramError(sout.str());
    }
    else if (*cmd_s == ' ' || *cmd_s == '\t' || *cmd_s == '\r' || *cmd_s == '\n')
      ++cmd_s;
    else {
      break;
    }
  }
  for (bool done=false;!done;) {
    switch (state) {
    case 0: {
      assert(*cmd_s);
      if (*cmd_s == '?' && late_bound) {
	++cmd_s;
	*late_bound = true;
	done = true;
	break;
      } else if (*cmd_s == '0') {
	++cmd_s;
	state = 1;
      } else if (*cmd_s == 'b') {
	number_type = 2;
	++cmd_s;
	state = 2;
      } else
	state = 2;
      break;
    }
    case 1: {
      if (*cmd_s != 'x') {
	++digit_count;
	state = 2;
	break;
      }
      number_type = 1;
      ++cmd_s;
      state = 2;
      break;
    }
    case 2: {
      bool valid_char = false;
      int digit_value = 0;
      for (digit_value=0;chars[number_type][digit_value];++digit_value)
	if (*cmd_s == chars[number_type][digit_value]) {
	  valid_char = true;
	  break;
	}
      if (!valid_char && digit_count == 0) {
	std::stringstream sout;
	sout<<"Syntax error, expected at least one digit: "<<chars[number_type];
	throw RegisterProgramError(sout.str());
      }
      if (!valid_char) {
	done = true;
	break;
      }
      number = number * base[number_type];
      number += digit_value;
      ++digit_count;
      ++cmd_s;
      break;
    }
    }
  }
  return cmd_s;
}

template const char* RegisterProgram::parseNumber<uint32_t>(const char* cmd_s, uint32_t& number, bool* late_bound);
template const char* RegisterProgram::parseNumber<uint64_t>(const char* cmd_s, uint64_t& number, bool* late_bound);

const char* RegisterProgram::parseNextChar(const char* cmd_s, char& c, const char* expect_hint) {
  for (;;) {
    if (!*cmd_s)
      return 0;
    else if (*cmd_s == '#') {
      while (*cmd_s && *cmd_s != '\r' && *cmd_s != '\n')
	++cmd_s;
    } else if (*cmd_s == ' ' || *cmd_s == '\t' || *cmd_s == '\r' || *cmd_s == '\n')
      ++cmd_s;
    else {
      c = *cmd_s;
      ++cmd_s;
      break;
    }
  }
  return cmd_s;
}

const char* RegisterProgram::parseDelim(const char* cmd_s, char delim, bool optional) {
  for (;;) {
    if (!*cmd_s && !optional) {
      std::stringstream sout;
      sout<<"Syntax error, expected delimeter: "<<delim;
      throw RegisterProgramError(sout.str());
    }
    else if (!*cmd_s)
      break;
    if (*cmd_s == ' ' || *cmd_s == '\t' || *cmd_s == '\r' || *cmd_s == '\n')
      ++cmd_s;
    else if (*cmd_s == delim) {
      ++cmd_s;
      break;
    }
    else if (*cmd_s != delim) {
      std::stringstream sout;
      sout<<"Syntax error, expected delimeter: "<<delim<<", but got: "<<*cmd_s;
      throw RegisterProgramError(sout.str());
    }
  }
  return cmd_s;
}

const char* RegisterProgram::parseCommandType(const char* cmd_s, CommandType& type) {
  char c;
  for (;;) {
    cmd_s = parseNextChar(cmd_s, c, "r w W s ;");
    if (!cmd_s)
      return 0;
    if (c != ';')
      break;
  }
  if (c == 'r')
    type = CommandType_Read;
  else if (c == 'w')
    type = CommandType_Write;
  else if (c == 'W')
    type = CommandType_ReadWrite;
  else if (c == 's')
    type = CommandType_Sleep;
  else {
    std::stringstream sout;
    sout<<"Unknown command type: "<<c;
    throw RegisterProgramError(sout.str());
  }
  return cmd_s;
}

const char* RegisterProgram::parseCommand(const char* cmd_s, CommandInfo& cmd) {
  cmd_s = parseCommandType(cmd_s, cmd.type);
  if (!cmd_s)
    return 0;
  cmd.late_bound = false;
  if (cmd.type == CommandType_Read) {
    cmd_s = parseNumber(cmd_s, cmd.reg_index);
  } else if (cmd.type == CommandType_Write) {
    cmd_s = parseNumber(cmd_s, cmd.reg_index);
    cmd_s = parseDelim(cmd_s,'=',false);
    cmd_s = parseNumber(cmd_s, cmd.reg_value, &cmd.late_bound);
  } else if (cmd.type == CommandType_ReadWrite) {
    cmd_s = parseNumber(cmd_s, cmd.reg_index);
    cmd_s = parseDelim(cmd_s,'(',false);
    cmd_s = parseNumber(cmd_s, cmd.reg_mask);
    cmd_s = parseDelim(cmd_s,')',false);
    cmd_s = parseDelim(cmd_s,'=',false);
    cmd_s = parseNumber(cmd_s, cmd.reg_value, &cmd.late_bound);
  } else if (cmd.type == CommandType_Sleep) {
    cmd_s = parseNumber(cmd_s, cmd.delay_us);
  }

  cmd_s = parseDelim(cmd_s,';',true);
  return cmd_s;
}

RegisterProgram::RegisterProgram() {
}

RegisterProgram::RegisterProgram(const std::string& reg_cmd) {
  add(reg_cmd);
}

void RegisterProgram::add(const std::string& reg_cmd) {
  if (reg_cmd.empty())
    return;
  const char* cmd_s = reg_cmd.c_str();
  for (;;) {
    for (int j=0;
	 *cmd_s==' '&&*cmd_s=='\t'&&
	   *cmd_s=='r'&&*cmd_s=='\n';++cmd_s);
    if (!*cmd_s)
      break;
    CommandInfo cmd;
    cmd_s = parseCommand(cmd_s, cmd);
    if (!cmd_s)
      break;
    commands.push_back(cmd);
  }
}

void RegisterProgram::addRead(uint32_t reg_index) {
  CommandInfo& cmd = *commands.insert(commands.end(),CommandInfo());
  cmd.type = CommandType_Read;
  cmd.reg_index = reg_index;
}

void RegisterProgram::addWrite(uint32_t reg_index, uint32_t reg_value) {
  CommandInfo& cmd = *commands.insert(commands.end(),CommandInfo());
  cmd.type = CommandType_Write;
  cmd.reg_index = reg_index;
  cmd.reg_value = reg_value;
}

void RegisterProgram::addSleep(uint64_t delay_us) {
  CommandInfo& cmd = *commands.insert(commands.end(),CommandInfo());
  cmd.type = CommandType_Sleep;
  cmd.delay_us = delay_us;
}

void RegisterProgram::bind(int index, uint32_t value) {
  for (CommandInfo& cmd : commands) {
    if (!cmd.late_bound)
      continue;
    if (!index) {
      cmd.reg_value = value;
      break;
    }
    --index;
  }
}

RegisterProgramResponse RegisterProgram::execute(OccamDeviceBase& dev) {
  RegisterProgramResponse resp;
  for (const CommandInfo& cmd : commands) {
    int r = OCCAM_API_SUCCESS;
    switch (cmd.type) {
    case CommandType_Read: {
      uint32_t read_value = 0;
      r = dev.readRegister(cmd.reg_index, &read_value);
      resp.add(cmd.reg_index, read_value);
      break;
    }
    case CommandType_Write: {
      r = dev.writeRegister(cmd.reg_index, cmd.reg_value);
      break;
    }
    case CommandType_ReadWrite: {
      r = dev.writeRegister(cmd.reg_index, cmd.reg_value, cmd.reg_mask);
      break;
    }
    case CommandType_Sleep: {
#ifdef _WIN32
      Sleep(DWORD(cmd.delay_us/1000));
#else // _WIN32
      usleep(cmd.delay_us);
#endif // _WIN32
      break;
    }
    }

    if (r != OCCAM_API_SUCCESS) {
      std::stringstream sout;
      sout<<"Register program error: ";
      switch (cmd.type) {
      case CommandType_Read: {
	sout<<"read index "<<cmd.reg_index<<" ("<<std::hex<<cmd.reg_index<<std::dec<<")";
	break;
      }
      case CommandType_Write: {
	sout<<"write index "<<cmd.reg_index<<" ("<<std::hex<<cmd.reg_index<<std::dec<<") value "<<cmd.reg_value<<" ("<<std::hex<<cmd.reg_value<<std::dec<<")";
	break;
      }
      case CommandType_ReadWrite: {
	sout<<"read-write index "<<cmd.reg_index<<" ("<<std::hex<<cmd.reg_index<<std::dec<<") value "<<cmd.reg_value<<" ("<<std::hex<<cmd.reg_value<<std::dec<<") mask "<<cmd.reg_mask<<" ("<<std::hex<<cmd.reg_mask<<std::dec<<")";
	break;
      }
      case CommandType_Sleep: {
	sout<<"sleep "<<cmd.delay_us;
	break;
      }
      }
      throw RegisterProgramError(sout.str(), r);
    }
  }
  return resp;
}

int RegisterProgram::size() const {
  return commands.size();
}

const RegisterProgram::CommandInfo& RegisterProgram::operator[] (int index) const {
  return commands[index];
}

RegisterProgram::CommandInfo& RegisterProgram::operator[] (int index) {
  return commands[index];
}

//////////////////////////////////////////////////////////////////////////////////
// Deferred

Deferred::RepBase::RepBase()
  : refcnt(1) {
}

Deferred::RepBase::~RepBase() {
}

void Deferred::RepBase::generate(std::mutex& lock) {
  assert(deps.empty());
  generateTyped();
  std::unique_lock<std::mutex> g(lock);
  --*frame_dep_count;
  for (RepBase* r : depees) {
    auto it = std::find(r->deps.begin(),r->deps.end(),this);
    assert(it != r->deps.end());
    r->deps.erase(it);
    if (r->deps.empty())
      r->queue_fn(r);
  }
}

void Deferred::RepBase::initQueue(std::function<void(Deferred::RepBase*)> _queue_fn, int* _frame_dep_count) {
  if (bool(queue_fn))
    return;
  queue_fn = _queue_fn;
  frame_dep_count = _frame_dep_count;
  ++*frame_dep_count;
  if (deps.empty()) {
    queue_fn(this);
  } else {
    for (RepBase* r : deps) {
      r->depees.push_back(this);
      r->initQueue(_queue_fn, _frame_dep_count);
    }
  }
}

void Deferred::RepBase::retain() {
  ++refcnt;
}

void Deferred::RepBase::release() {
  if (--refcnt<=0)
    delete this;
}

void Deferred::init(RepBase* new_rep, int num_deps, const Deferred* const* deps) {
  rep = new_rep;
  for (int j=0;j<num_deps;++j) {
    rep->deps.push_back(deps[j]->rep);
  }
}

Deferred::Deferred()
  : rep(0) {
}

Deferred::Deferred(const Deferred& x) {
  rep = x.rep;
  if (rep)
    rep->retain();
}

Deferred& Deferred::operator= (const Deferred& rhs) {
  if (rhs.rep)
    rhs.rep->retain();
  if (rep)
    rep->release();
  rep = rhs.rep;
  return *this;
}

Deferred::~Deferred() {
  if (rep)
    rep->release();
}

void Deferred::initQueue(std::function<void(Deferred::RepBase*)> queue_fn, int* frame_dep_count) {
  rep->initQueue(queue_fn, frame_dep_count);
}

void Deferred::copy(void** ret_data) {
  rep->copy(ret_data);
}

OccamDataType Deferred::dataType() const {
  return rep->dataType();
}

//////////////////////////////////////////////////////////////////////////////////
// DeviceOutput

DeviceOutput::DeviceOutput() {
  rep = std::make_shared<Rep>();
  rep->dep_count = 0;
}

void DeviceOutput::set(OccamDataName name, Deferred value) {
  rep->data.push_back(std::make_pair(name, value));
}

void DeviceOutput::queue(std::function<void(Deferred::RepBase*)> queue_fn) {
  auto cmp0 = [](const std::pair<OccamDataName, Deferred>& lhs,
		 const std::pair<OccamDataName, Deferred>& rhs){
    return lhs.first < rhs.first;
  };
  std::sort(rep->data.begin(),rep->data.end(),cmp0);
  for (int j=0;j<rep->data.size();++j)
    rep->data[j].second.initQueue(queue_fn, &rep->dep_count);
}

int DeviceOutput::depCount() const {
  return rep->dep_count;
}

bool DeviceOutput::pack(int req_count, const OccamDataName* req) {
  auto cmp0 = [](const std::pair<OccamDataName, Deferred>& lhs,
		 const std::pair<OccamDataName, Deferred>& rhs){
    return lhs.first < rhs.first;
  };
  std::sort(rep->data.begin(),rep->data.end(),cmp0);

  std::vector<std::pair<OccamDataName, Deferred> > data0;
  data0.reserve(rep->data.size());

  for (int j=0;j<req_count;++j) {
    auto it = std::lower_bound
      (rep->data.begin(),rep->data.end(),req[j],
       [](const std::pair<OccamDataName, Deferred>& lhs, OccamDataName req0){
	return lhs.first < req0;
      });
    if (it == rep->data.end() || it->first != req[j])
      return false;

    data0.push_back(*it);
  }
  rep->data.swap(data0);

  return true;
}

bool DeviceOutput::unpack(int req_count, const OccamDataName* req, OccamDataType* ret_types, void** ret_data) {
  for (int j=0;j<req_count;++j) {
    auto it = std::lower_bound
      (rep->data.begin(),rep->data.end(),req[j],
       [](const std::pair<OccamDataName, Deferred>& lhs, OccamDataName req0){
	return lhs.first < req0;
      });
    if (it == rep->data.end() || it->first != req[j])
      return false;
  }

  for (int j=0;j<req_count;++j) {
    auto it = std::lower_bound
      (rep->data.begin(),rep->data.end(),req[j],
       [](const std::pair<OccamDataName, Deferred>& lhs, OccamDataName req0){
	return lhs.first < req0;
      });
    if (ret_types)
      ret_types[j] = it->second.dataType();
    it->second.copy(&ret_data[j]);
  }

  return true;
}

//////////////////////////////////////////////////////////////////////////////////
// DeferredEvaluator

void DeferredEvaluator::threadproc() {
#ifdef _WIN32
  SetThreadPriority(GetCurrentThread(),THREAD_PRIORITY_BELOW_NORMAL);
#else // _WIN32
  //  struct sched_param param;
  //  memset(&param,0,sizeof(param));
  //  int r = sched_setscheduler(getpid(), SCHED_IDLE, &param);
  //  setpriority(PRIO_PROCESS,syscall(SYS_gettid),15);
#endif // _WIN32


  for (;;) {
    {
      Deferred::RepBase* task;
      {
	std::unique_lock<std::mutex> g(lock);
	if (shutdown)
	  break;
	if (pending_tasks.empty()) {
	  if (pending_frames.empty() ||
	      reaping_frame_count >= max_reaping_frames) {
	    cond.wait(g);
	    continue;
	  }
	  DeviceOutput out = *pending_frames.begin();
	  pending_frames.pop_front();
	  --pending_frame_count;
	  out.queue([&](Deferred::RepBase* task) {
	      pending_tasks.push_back(task);
	      cond.notify_one();
	    });
	  reaping_frames.push_back(out);
	  ++reaping_frame_count;
	  continue;
	}
	task = *pending_tasks.begin();
	pending_tasks.pop_front();
      }
      task->generate(lock);
    }
  }
}

DeferredEvaluator::DeferredEvaluator()
 : pending_frame_count(0),
   reaping_frame_count(0),
   max_pending_frames(1),
   max_reaping_frames(1),
   shutdown(false) {
  int nthreads = std::thread::hardware_concurrency();
  if (nthreads<1)
    nthreads = 8;
  //        nthreads = 1; // * debug logic
  for (int j=0;j<nthreads;++j)
    threads.push_back(std::thread([this](){
	  this->threadproc();
	}));
}

DeferredEvaluator::~DeferredEvaluator() {
  {
    std::unique_lock<std::mutex> g(lock);
    shutdown = true;
    cond.notify_all();
  }
  for (std::thread& th : threads)
    th.join();
}

void DeferredEvaluator::push(const DeviceOutput& out) {
  push_fps.increment();
  std::unique_lock<std::mutex> g(lock);
  pending_frames.push_back(out);
  ++pending_frame_count;
  while (pending_frame_count > max_pending_frames) {
    pending_frames.pop_front();
    --pending_frame_count;
  }
  cond.notify_one();
}

bool DeferredEvaluator::pop(DeviceOutput& out) {
  std::unique_lock<std::mutex> g(lock);
  if (reaping_frames.empty())
    return false;
  if (reaping_frames.begin()->depCount()>0)
    return false;
  out = *reaping_frames.begin();
  reaping_frames.pop_front();
  --reaping_frame_count;
  cond.notify_one();
  if (pop_fps.increment()) {
#ifdef DEBUG_DATA_RATES
    std::cerr<<"deferred eval fps: push "<<push_fps.rate()<<", pop "<<pop_fps.rate()<<std::endl;
#endif // DEBUG_DATA_RATES
  }
  return true;
}

int DeferredEvaluator::emitFPS() const {
  return pop_fps.rate();
}

int DeferredEvaluator::maxPendingFrames() const {
  return max_pending_frames;
}

void DeferredEvaluator::setMaxPendingFrames(int value) {
  max_pending_frames = value;
}

int DeferredEvaluator::maxReapingFrames() const {
  return max_reaping_frames;
}

void DeferredEvaluator::setMaxReapingFrames(int value) {
  max_reaping_frames = value;
}

//////////////////////////////////////////////////////////////////////////////////
// OccamDeviceBase

OccamDeviceBase::OccamDeviceBase(const std::string& cid)
  : _cid(cid) {
  std::string::size_type p0 = _cid.find_first_of(":");
  if (p0 != std::string::npos) {
    _model.assign(_cid.begin(),_cid.begin()+p0);
    _serial.assign(_cid.begin()+p0+1,_cid.end());
  }

  registerParams(OCCAM_CID,"cid",OCCAM_NOT_STORED,
		 std::bind(&OccamDeviceBase::cid,this));
  registerParams(OCCAM_MODEL,"model",OCCAM_NOT_STORED,
		 std::bind(&OccamDeviceBase::model,this));
  registerParams(OCCAM_SERIAL,"serial",OCCAM_NOT_STORED,
		 std::bind(&OccamDeviceBase::serial,this));

  auto get_emit_fps = [this](){
    return this->_deferred_eval.emitFPS();
  };
  registerParami(OCCAM_EMIT_FPS, "emit_fps", OCCAM_NOT_STORED, 0, 0, get_emit_fps);

  auto get_pending_frames = [this](){
    return this->_deferred_eval.maxPendingFrames();
  };
  auto set_pending_frames = [this](int value){
    return this->_deferred_eval.setMaxPendingFrames(value);
  };
  auto get_reaping_frames = [this](){
    return this->_deferred_eval.maxReapingFrames();
  };
  auto set_reaping_frames = [this](int value){
    return this->_deferred_eval.setMaxReapingFrames(value);
  };
  registerParami(OCCAM_MAX_DEFERRED_PENDING_FRAMES, "deferred_pending_frames", OCCAM_SETTINGS, 1, 4,
		 get_pending_frames,set_pending_frames);
  setDefaultDeviceValuei(OCCAM_MAX_DEFERRED_PENDING_FRAMES,1);
  registerParami(OCCAM_MAX_DEFERRED_REAPING_FRAMES, "deferred_reaping_frames", OCCAM_SETTINGS, 1, 4,
		 get_reaping_frames,set_reaping_frames);
  setDefaultDeviceValuei(OCCAM_MAX_DEFERRED_REAPING_FRAMES,1);
}

OccamDeviceBase::~OccamDeviceBase() {
}

const std::string& OccamDeviceBase::cid() const {
  return _cid;
}

const std::string& OccamDeviceBase::model() const {
  return _model;
}

const std::string& OccamDeviceBase::serial() const {
  return _serial;
}

OccamDeviceBase::ParamInfo* OccamDeviceBase::getParam(OccamParam id) {
  for (ParamInfo& pi : _params)
    if (pi.id == id)
      return &pi;
  return 0;
}

int OccamDeviceBase::readData(DeviceOutput& out) {
  return OCCAM_API_NOT_SUPPORTED;
}

void OccamDeviceBase::availableData(std::vector<std::pair<OccamDataName,OccamDataType> >& available_data) {
}

void OccamDeviceBase::registerParamb(OccamParam id,
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

void OccamDeviceBase::registerParami(OccamParam id,
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

void OccamDeviceBase::registerParamr(OccamParam id,
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

void OccamDeviceBase::registerParams(OccamParam id,
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

void OccamDeviceBase::registerParamiv(OccamParam id,
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

void OccamDeviceBase::registerParamrv(OccamParam id,
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

void OccamDeviceBase::registerParamsv(OccamParam id,
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

void OccamDeviceBase::unregisterParam(OccamParam id) {
  auto it = std::remove_if(_params.begin(),_params.end(),[id](const ParamInfo& pi){
      return pi.id == id;
    });
  _params.erase(it,_params.end());
}

void OccamDeviceBase::setAllowedValues(OccamParam id, const std::vector<std::pair<std::string,int> >& values) {
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

void OccamDeviceBase::setMinMaxValues(OccamParam id, int min_value, int max_value) {
  return setMinMaxValues(id,double(min_value),double(max_value));
}

void OccamDeviceBase::setMinMaxValues(OccamParam id, double min_value, double max_value) {
  ParamInfo* pi = getParam(id);
  if (!pi)
    return;
  pi->min_value = min_value;
  pi->max_value = max_value;
}

void OccamDeviceBase::addConfigurableModule(OccamParam id, const std::string& name,
					    OccamModuleClass module_class) {
  IOccamModuleInfo** module_info = 0;
  occamEnumerateModules(module_class, &module_info);
  if (!*module_info)
    return;

  int mg_index = _module_groups.size();
  ModuleGroupInfo& mg = *_module_groups.emplace(_module_groups.end());
  mg.id = id;
  mg.selected_index = -1;
  for (int j=0;module_info[j];++j) {
    char* namep = 0;
    module_info[j]->getName(module_info[j],&namep);
    if (!namep)
      continue;
    std::string name = namep;
    occamFree(namep);
    char* pretty_namep = 0;
    module_info[j]->getPrettyName(module_info[j],&pretty_namep);
    if (!namep)
      continue;
    std::string pretty_name = pretty_namep;
    occamFree(pretty_namep);
    void* handlep = 0;
    occamConstructModule(module_class, name.c_str(), &handlep);
    if (!handlep)
      continue;
    auto handle = std::shared_ptr<void>(handlep, occamReleaseModule);
    _modules.push_back(handle);
    mg.selected_index = 0;
    ModuleInfo& mi = *mg.modules.emplace(mg.modules.end());
    mi.handle = handle;
    mi.name = name;
    mi.pretty_name = pretty_name;
    mi.module_index = _modules.size();
  }
  occamFree(module_info);

  if (mg.selected_index<0) {
    _module_groups.pop_back();
    return;
  }

  ParamInfo& pi0 = *_params.emplace(_params.end());
  pi0.id = id;
  pi0.name = name;
  pi0.type = OCCAM_PARAM_STRING;
  pi0.min_value = 0;
  pi0.max_value = 0;
  pi0.count = mg.modules.size();
  pi0.gets_fn = [this,mg_index](){
    const ModuleGroupInfo& mg = this->_module_groups[mg_index];
    const ModuleInfo& mi = mg.modules[mg.selected_index];
    return mi.name;
  };
  pi0.geti_fn = [this,mg_index](){
    const ModuleGroupInfo& mg = this->_module_groups[mg_index];
    const ModuleInfo& mi = mg.modules[mg.selected_index];
    return mi.module_index;
  };
  pi0.getiv_fn = [this,mg_index](int* module_indices){
    const ModuleGroupInfo& mg = this->_module_groups[mg_index];
    for (const ModuleInfo& mi : mg.modules)
      *module_indices++ = mi.module_index;
  };
  pi0.getsv_fn = [this,mg_index](std::string* names){
    const ModuleGroupInfo& mg = this->_module_groups[mg_index];
    for (const ModuleInfo& mi : mg.modules)
      *names++ = mi.pretty_name;
  };
  pi0.sets_fn = [this,mg_index](const std::string& new_name){
    ModuleGroupInfo& mg = this->_module_groups[mg_index];
    for (int j=0;j<mg.modules.size();++j) {
      const ModuleInfo& mi = mg.modules[j];
      if (mi.name == new_name || mi.pretty_name == new_name) {
	mg.selected_index = j;
	break;
      }
    }
    throw RegisterProgramError("Module "+new_name+" not found", OCCAM_API_MODULE_NOT_FOUND);
  };
  pi0.seti_fn = [this,mg_index](int new_module_index){
    ModuleGroupInfo& mg = this->_module_groups[mg_index];
    for (int j=0;j<mg.modules.size();++j) {
      const ModuleInfo& mi = mg.modules[j];
      if (mi.module_index == new_module_index) {
	mg.selected_index = j;
	break;
      }
    }
    std::stringstream sout;
    sout<<"Module "<<new_module_index<<" not found";
    throw RegisterProgramError(sout.str(), OCCAM_API_MODULE_NOT_FOUND);
  };
  pi0.getp_fn = [this,mg_index](){
    ModuleGroupInfo& mg = this->_module_groups[mg_index];
    ModuleInfo& mi = mg.modules[mg.selected_index];
    void* handle = mi.handle.get();
    occamRetainModule(handle);
    return handle;
  };
  pi0.getpv_fn = [this,mg_index](void** handles){
    ModuleGroupInfo& mg = this->_module_groups[mg_index];
    for (const ModuleInfo& mi : mg.modules) {
      void* handle = mi.handle.get();
      occamRetainModule(handle);
      *handles++ = handle;
    }
  };
  pi0.read_only = false;
}

std::shared_ptr<void> OccamDeviceBase::module(OccamParam id) {
  void* handle = 0;
  getDeviceValuep(id, &handle);
  return std::shared_ptr<void>(handle,occamReleaseModule);
}

int OccamDeviceBase::enumerateParamList(OccamParamList** param_list) {
  *param_list = new OccamParamList;

  std::vector<OccamParamEntry> ret_params;

  for (int j=0;j<_params.size();++j) {
    ParamInfo& p0 = _params[j];
    OccamParamEntry& p1 = *ret_params.emplace(ret_params.end());
    memset(&p1,0,sizeof(OccamParamEntry));
    p1.id = p0.id;
    p1.module_index = 0;
    p1.name = strdup(p0.name.c_str());
    p1.storage_class = p0.storage_class;
    p1.type = p0.type;
    p1.min_value = p0.min_value;
    p1.max_value = p0.max_value;
    p1.count = p0.count;
    p1.read_only = p0.read_only;
  }

  int module_index = 1;
  for (int module_index=1;module_index<=_modules.size();++module_index) {
    std::shared_ptr<void> handle = _modules[module_index-1];
    IOccamParameters* param_iface;
    int r;
    r = occamGetInterface(handle.get(), IOCCAMPARAMETERS, (void**)&param_iface);
    if (r != OCCAM_API_SUCCESS)
      continue;
    OccamParamList* module_params;
    r = param_iface->enumerateParameters(handle.get(),&module_params);
    if (r != OCCAM_API_SUCCESS)
      continue;
    for (int j=0;j<module_params->param_count;++j) {
      auto it = ret_params.emplace(ret_params.end(),module_params->params[j]);
      it->name = strdup(it->name);
      it->module_index = module_index;
    }
    occamFreeParamList(module_params);
  }

  (*param_list)->params = new OccamParamEntry[ret_params.size()];
  (*param_list)->param_count = ret_params.size();
  std::copy(ret_params.begin(),ret_params.end(),(*param_list)->params);

  return OCCAM_API_SUCCESS;
}

int OccamDeviceBase::setDeviceValueb(OccamParam id, bool value) {
  return setDeviceValuei(id,value?1:0);
}

int OccamDeviceBase::setDeviceValuei(OccamParam id, int value) {
  int module_index = OCCAM_MODULE_INDEX(id);
  id = OCCAM_PARAM_ID(id);
  if (module_index>0&&module_index<=_modules.size()) {
    IOccamParameters* param_iface;
    int r;
    void* handle = _modules[module_index-1].get();
    r = occamGetInterface(handle, IOCCAMPARAMETERS, (void**)&param_iface);
    if (r != OCCAM_API_SUCCESS)
      return OCCAM_API_NOT_SUPPORTED;
    return param_iface->setValuei(handle,id,value);
  }

  ParamInfo* pi = getParam(id);
  if (!pi) {
    for (ParamInfo& p : _params) {
      int module_index = 0;
      if (bool(p.getp_fn)&&bool(p.geti_fn))
	module_index = p.geti_fn();
      if (module_index>0&&module_index<=_modules.size()) {
	IOccamParameters* param_iface;
	int r;
	void* handle = _modules[module_index-1].get();
	r = occamGetInterface(handle, IOCCAMPARAMETERS, (void**)&param_iface);
	if (r != OCCAM_API_SUCCESS)
	  continue;
	r = param_iface->setValuei(handle,id,value);
	if (r != OCCAM_API_INVALID_PARAMETER)
	  return r;
      }
    }
    return OCCAM_API_INVALID_PARAMETER;
  }
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

int OccamDeviceBase::setDeviceValuer(OccamParam id, double value) {
  int module_index = OCCAM_MODULE_INDEX(id);
  id = OCCAM_PARAM_ID(id);
  if (module_index>0&&module_index<=_modules.size()) {
    IOccamParameters* param_iface;
    int r;
    void* handle = _modules[module_index-1].get();
    r = occamGetInterface(handle, IOCCAMPARAMETERS, (void**)&param_iface);
    if (r != OCCAM_API_SUCCESS)
      return OCCAM_API_NOT_SUPPORTED;
    return param_iface->setValuer(handle,id,value);
  }

  ParamInfo* pi = getParam(id);
  if (!pi) {
    for (ParamInfo& p : _params) {
      int module_index = 0;
      if (bool(p.getp_fn)&&bool(p.geti_fn))
	module_index = p.geti_fn();
      if (module_index>0&&module_index<=_modules.size()) {
	IOccamParameters* param_iface;
	int r;
	void* handle = _modules[module_index-1].get();
	r = occamGetInterface(handle, IOCCAMPARAMETERS, (void**)&param_iface);
	if (r != OCCAM_API_SUCCESS)
	  continue;
	r = param_iface->setValuer(handle,id,value);
	if (r != OCCAM_API_INVALID_PARAMETER)
	  return r;
      }
    }
    return OCCAM_API_INVALID_PARAMETER;
  }
  if (!bool(pi->setr_fn))
    return OCCAM_API_INVALID_TYPE;
  try {
    pi->setr_fn(value);
    return OCCAM_API_SUCCESS;
  } catch (RegisterProgramError&) {
    return OCCAM_API_WRITE_ERROR;
  }
}

int OccamDeviceBase::setDeviceValues(OccamParam id, const char* value) {
  int module_index = OCCAM_MODULE_INDEX(id);
  id = OCCAM_PARAM_ID(id);
  if (module_index>0&&module_index<=_modules.size()) {
    IOccamParameters* param_iface;
    int r;
    void* handle = _modules[module_index-1].get();
    r = occamGetInterface(handle, IOCCAMPARAMETERS, (void**)&param_iface);
    if (r != OCCAM_API_SUCCESS)
      return OCCAM_API_NOT_SUPPORTED;
    return param_iface->setValues(handle,id,value);
  }

  ParamInfo* pi = getParam(id);
  if (!pi) {
    for (ParamInfo& p : _params) {
      int module_index = 0;
      if (bool(p.getp_fn)&&bool(p.geti_fn))
	module_index = p.geti_fn();
      if (module_index>0&&module_index<=_modules.size()) {
	IOccamParameters* param_iface;
	int r;
	void* handle = _modules[module_index-1].get();
	r = occamGetInterface(handle, IOCCAMPARAMETERS, (void**)&param_iface);
	if (r != OCCAM_API_SUCCESS)
	  continue;
	r = param_iface->setValues(handle,id,value);
	if (r != OCCAM_API_INVALID_PARAMETER)
	  return r;
      }
    }
    return OCCAM_API_INVALID_PARAMETER;
  }
  if (!bool(pi->sets_fn))
    return OCCAM_API_INVALID_TYPE;
  try {
    pi->sets_fn(value);
    return OCCAM_API_SUCCESS;
  } catch (RegisterProgramError&) {
    return OCCAM_API_WRITE_ERROR;
  }
}

int OccamDeviceBase::setDeviceValueiv(OccamParam id, const int* values, int value_count) {
  int module_index = OCCAM_MODULE_INDEX(id);
  id = OCCAM_PARAM_ID(id);
  if (module_index>0&&module_index<=_modules.size()) {
    IOccamParameters* param_iface;
    int r;
    void* handle = _modules[module_index-1].get();
    r = occamGetInterface(handle, IOCCAMPARAMETERS, (void**)&param_iface);
    if (r != OCCAM_API_SUCCESS)
      return OCCAM_API_NOT_SUPPORTED;
    return param_iface->setValueiv(handle,id,values,value_count);
  }

  ParamInfo* pi = getParam(id);
  if (!pi) {
    for (ParamInfo& p : _params) {
      int module_index = 0;
      if (bool(p.getp_fn)&&bool(p.geti_fn))
	module_index = p.geti_fn();
      if (module_index>0&&module_index<=_modules.size()) {
	IOccamParameters* param_iface;
	int r;
	void* handle = _modules[module_index-1].get();
	r = occamGetInterface(handle, IOCCAMPARAMETERS, (void**)&param_iface);
	if (r != OCCAM_API_SUCCESS)
	  continue;
	r = param_iface->setValueiv(handle,id,values,value_count);
	if (r != OCCAM_API_INVALID_PARAMETER)
	  return r;
      }
    }
    return OCCAM_API_INVALID_PARAMETER;
  }
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

int OccamDeviceBase::setDeviceValuerv(OccamParam id, const double* values, int value_count) {
  int module_index = OCCAM_MODULE_INDEX(id);
  id = OCCAM_PARAM_ID(id);
  if (module_index>0&&module_index<=_modules.size()) {
    IOccamParameters* param_iface;
    int r;
    void* handle = _modules[module_index-1].get();
    r = occamGetInterface(handle, IOCCAMPARAMETERS, (void**)&param_iface);
    if (r != OCCAM_API_SUCCESS)
      return OCCAM_API_NOT_SUPPORTED;
    return param_iface->setValuerv(handle,id,values,value_count);
  }

  ParamInfo* pi = getParam(id);
  if (!pi) {
    for (ParamInfo& p : _params) {
      int module_index = 0;
      if (bool(p.getp_fn)&&bool(p.geti_fn))
	module_index = p.geti_fn();
      if (module_index>0&&module_index<=_modules.size()) {
	IOccamParameters* param_iface;
	int r;
	void* handle = _modules[module_index-1].get();
	r = occamGetInterface(handle, IOCCAMPARAMETERS, (void**)&param_iface);
	if (r != OCCAM_API_SUCCESS)
	  continue;
	r = param_iface->setValuerv(handle,id,values,value_count);
	if (r != OCCAM_API_INVALID_PARAMETER)
	  return r;
      }
    }
    return OCCAM_API_INVALID_PARAMETER;
  }
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

int OccamDeviceBase::setDeviceValuesv(OccamParam id, char** values, int value_count) {
  int module_index = OCCAM_MODULE_INDEX(id);
  id = OCCAM_PARAM_ID(id);
  if (module_index>0&&module_index<=_modules.size()) {
    IOccamParameters* param_iface;
    int r;
    void* handle = _modules[module_index-1].get();
    r = occamGetInterface(handle, IOCCAMPARAMETERS, (void**)&param_iface);
    if (r != OCCAM_API_SUCCESS)
      return OCCAM_API_NOT_SUPPORTED;
    return param_iface->setValuesv(handle,id,values,value_count);
  }

  ParamInfo* pi = getParam(id);
  if (!pi) {
    for (ParamInfo& p : _params) {
      int module_index = 0;
      if (bool(p.getp_fn)&&bool(p.geti_fn))
	module_index = p.geti_fn();
      if (module_index>0&&module_index<=_modules.size()) {
	IOccamParameters* param_iface;
	int r;
	void* handle = _modules[module_index-1].get();
	r = occamGetInterface(handle, IOCCAMPARAMETERS, (void**)&param_iface);
	if (r != OCCAM_API_SUCCESS)
	  continue;
	r = param_iface->setValuesv(handle,id,values,value_count);
	if (r != OCCAM_API_INVALID_PARAMETER)
	  return r;
      }
    }
    return OCCAM_API_INVALID_PARAMETER;
  }
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

int OccamDeviceBase::getDeviceValueCount(OccamParam id, int* value_count) {
  int module_index = OCCAM_MODULE_INDEX(id);
  id = OCCAM_PARAM_ID(id);
  if (module_index>0&&module_index<=_modules.size()) {
    IOccamParameters* param_iface;
    int r;
    void* handle = _modules[module_index-1].get();
    r = occamGetInterface(handle, IOCCAMPARAMETERS, (void**)&param_iface);
    if (r != OCCAM_API_SUCCESS)
      return OCCAM_API_NOT_SUPPORTED;
    return param_iface->getValueCount(handle,id,value_count);
  }

  ParamInfo* pi = getParam(id);
  if (!pi) {
    for (ParamInfo& p : _params) {
      int module_index = 0;
      if (bool(p.getp_fn)&&bool(p.geti_fn))
	module_index = p.geti_fn();
      if (module_index>0&&module_index<=_modules.size()) {
	IOccamParameters* param_iface;
	int r;
	void* handle = _modules[module_index-1].get();
	r = occamGetInterface(handle, IOCCAMPARAMETERS, (void**)&param_iface);
	if (r != OCCAM_API_SUCCESS)
	  continue;
	r = param_iface->getValueCount(handle,id,value_count);
	if (r != OCCAM_API_INVALID_PARAMETER)
	  return r;
      }
    }
    return OCCAM_API_INVALID_PARAMETER;
  }
  *value_count = pi->count;
  return OCCAM_API_SUCCESS;
}

int OccamDeviceBase::setDefaultDeviceValueb(OccamParam id,bool value) {
  for (auto handle : _modules) {
    IOccamParameters* param_iface;
    int r;
    r = occamGetInterface(handle.get(), IOCCAMPARAMETERS, (void**)&param_iface);
    if (r != OCCAM_API_SUCCESS)
      continue;
    param_iface->setDefaultValueb(handle.get(),id,value);
  }
  ParamInfo* pi = getParam(id);
  if (!pi)
    return OCCAM_API_INVALID_PARAMETER;
  pi->default_bool_value = value;
  pi->valid_default_value = true;
  try {
    if (bool(pi->setb_fn))
      pi->setb_fn(value);
  } catch (RegisterProgramError&) {
  }
  return OCCAM_API_SUCCESS;
}

int OccamDeviceBase::setDefaultDeviceValuei(OccamParam id,int value) {
  for (auto handle : _modules) {
    IOccamParameters* param_iface;
    int r;
    r = occamGetInterface(handle.get(), IOCCAMPARAMETERS, (void**)&param_iface);
    if (r != OCCAM_API_SUCCESS)
      continue;
    param_iface->setDefaultValuei(handle.get(),id,value);
  }
  ParamInfo* pi = getParam(id);
  if (!pi)
    return OCCAM_API_INVALID_PARAMETER;
  pi->default_int_value = value;
  pi->valid_default_value = true;
  try {
    if (bool(pi->seti_fn))
      pi->seti_fn(value);
  } catch (RegisterProgramError&) {
  }
  return OCCAM_API_SUCCESS;
}

int OccamDeviceBase::setDefaultDeviceValuer(OccamParam id,double value) {
  for (auto handle : _modules) {
    IOccamParameters* param_iface;
    int r;
    r = occamGetInterface(handle.get(), IOCCAMPARAMETERS, (void**)&param_iface);
    if (r != OCCAM_API_SUCCESS)
      continue;
    param_iface->setDefaultValuer(handle.get(),id,value);
  }
  ParamInfo* pi = getParam(id);
  if (!pi)
    return OCCAM_API_INVALID_PARAMETER;
  pi->default_real_value = value;
  pi->valid_default_value = true;
  try {
    if (bool(pi->setr_fn))
      pi->setr_fn(value);
  } catch (RegisterProgramError&) {
  }
  return OCCAM_API_SUCCESS;
}

int OccamDeviceBase::setDefaultDeviceValues(OccamParam id,const char* value) {
  for (auto handle : _modules) {
    IOccamParameters* param_iface;
    int r;
    r = occamGetInterface(handle.get(), IOCCAMPARAMETERS, (void**)&param_iface);
    if (r != OCCAM_API_SUCCESS)
      continue;
    param_iface->setDefaultValues(handle.get(),id,value);
  }
  ParamInfo* pi = getParam(id);
  if (!pi)
    return OCCAM_API_INVALID_PARAMETER;
  pi->default_string_value = value;
  pi->valid_default_value = true;
  try {
    if (bool(pi->sets_fn))
      pi->sets_fn(value);
  } catch (RegisterProgramError&) {
  }
  return OCCAM_API_SUCCESS;
}

int OccamDeviceBase::resetDeviceValue(OccamParam id) {
  int module_index = OCCAM_MODULE_INDEX(id);
  id = OCCAM_PARAM_ID(id);
  if (module_index>0&&module_index<=_modules.size()) {
    IOccamParameters* param_iface;
    int r;
    void* handle = _modules[module_index-1].get();
    r = occamGetInterface(handle, IOCCAMPARAMETERS, (void**)&param_iface);
    if (r != OCCAM_API_SUCCESS)
      return OCCAM_API_NOT_SUPPORTED;
    return param_iface->resetValue(handle,id);
  }

  for (auto handle : _modules) {
    IOccamParameters* param_iface;
    int r;
    r = occamGetInterface(handle.get(), IOCCAMPARAMETERS, (void**)&param_iface);
    if (r != OCCAM_API_SUCCESS)
      continue;
    param_iface->resetValue(handle.get(),id);
  }

  ParamInfo* pi = getParam(id);
  if (!pi)
    return OCCAM_API_INVALID_PARAMETER;
  if (pi->valid_default_value) {
    try {
      if (pi->type == OCCAM_PARAM_BOOL && bool(pi->setb_fn))
	pi->setb_fn(pi->default_bool_value);
      else if (pi->type == OCCAM_PARAM_INT && bool(pi->seti_fn))
	pi->seti_fn(pi->default_int_value);
      else if (pi->type == OCCAM_PARAM_REAL && bool(pi->setr_fn))
	pi->setr_fn(pi->default_real_value);
      else if (pi->type == OCCAM_PARAM_STRING && bool(pi->sets_fn))
	pi->sets_fn(pi->default_string_value);
    } catch (RegisterProgramError&) {
    }
  }
  return OCCAM_API_SUCCESS;
}

int OccamDeviceBase::getDeviceValueb(OccamParam id, bool* value) {
  int value0 = 0;
  int r = getDeviceValuei(id,&value0);
  *value = value0?true:false;
  return r;
}

int OccamDeviceBase::getDeviceValuei(OccamParam id, int* value) {
  int module_index = OCCAM_MODULE_INDEX(id);
  id = OCCAM_PARAM_ID(id);
  if (module_index>0&&module_index<=_modules.size()) {
    IOccamParameters* param_iface;
    int r;
    void* handle = _modules[module_index-1].get();
    r = occamGetInterface(handle, IOCCAMPARAMETERS, (void**)&param_iface);
    if (r != OCCAM_API_SUCCESS)
      return OCCAM_API_NOT_SUPPORTED;
    return param_iface->getValuei(handle,id,value);
  }

  ParamInfo* pi = getParam(id);
  if (!pi) {
    for (ParamInfo& p : _params) {
      int module_index = 0;
      if (bool(p.getp_fn)&&bool(p.geti_fn))
	module_index = p.geti_fn();
      if (module_index>0&&module_index<=_modules.size()) {
	IOccamParameters* param_iface;
	int r;
	void* handle = _modules[module_index-1].get();
	r = occamGetInterface(handle, IOCCAMPARAMETERS, (void**)&param_iface);
	if (r != OCCAM_API_SUCCESS)
	  continue;
	r = param_iface->getValuei(handle,id,value);
	if (r != OCCAM_API_INVALID_PARAMETER)
	  return r;
      }
    }
    return OCCAM_API_INVALID_PARAMETER;
  }
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

int OccamDeviceBase::getDeviceValuer(OccamParam id, double* value) {
  int module_index = OCCAM_MODULE_INDEX(id);
  id = OCCAM_PARAM_ID(id);
  if (module_index>0&&module_index<=_modules.size()) {
    IOccamParameters* param_iface;
    int r;
    void* handle = _modules[module_index-1].get();
    r = occamGetInterface(handle, IOCCAMPARAMETERS, (void**)&param_iface);
    if (r != OCCAM_API_SUCCESS)
      return OCCAM_API_NOT_SUPPORTED;
    return param_iface->getValuer(handle,id,value);
  }

  ParamInfo* pi = getParam(id);
  if (!pi) {
    for (ParamInfo& p : _params) {
      int module_index = 0;
      if (bool(p.getp_fn)&&bool(p.geti_fn))
	module_index = p.geti_fn();
      if (module_index>0&&module_index<=_modules.size()) {
	IOccamParameters* param_iface;
	int r;
	void* handle = _modules[module_index-1].get();
	r = occamGetInterface(handle, IOCCAMPARAMETERS, (void**)&param_iface);
	if (r != OCCAM_API_SUCCESS)
	  continue;
	r = param_iface->getValuer(handle,id,value);
	if (r != OCCAM_API_INVALID_PARAMETER)
	  return r;
      }
    }
    return OCCAM_API_INVALID_PARAMETER;
  }
  if (!bool(pi->getr_fn))
    return OCCAM_API_INVALID_TYPE;
  try {
    *value = pi->getr_fn();
  } catch (RegisterProgramError&) {
    return OCCAM_API_READ_ERROR;
  }
  return OCCAM_API_SUCCESS;
}

int OccamDeviceBase::getDeviceValues(OccamParam id, char** value) {
  int module_index = OCCAM_MODULE_INDEX(id);
  id = OCCAM_PARAM_ID(id);
  if (module_index>0&&module_index<=_modules.size()) {
    IOccamParameters* param_iface;
    int r;
    void* handle = _modules[module_index-1].get();
    r = occamGetInterface(handle, IOCCAMPARAMETERS, (void**)&param_iface);
    if (r != OCCAM_API_SUCCESS)
      return OCCAM_API_NOT_SUPPORTED;
    return param_iface->getValues(handle,id,value);
  }

  ParamInfo* pi = getParam(id);
  if (!pi) {
    for (ParamInfo& p : _params) {
      int module_index = 0;
      if (bool(p.getp_fn)&&bool(p.geti_fn))
	module_index = p.geti_fn();
      if (module_index>0&&module_index<=_modules.size()) {
	IOccamParameters* param_iface;
	int r;
	void* handle = _modules[module_index-1].get();
	r = occamGetInterface(handle, IOCCAMPARAMETERS, (void**)&param_iface);
	if (r != OCCAM_API_SUCCESS)
	  continue;
	r = param_iface->getValues(handle,id,value);
	if (r != OCCAM_API_INVALID_PARAMETER)
	  return r;
      }
    }
    return OCCAM_API_INVALID_PARAMETER;
  }
  if (!bool(pi->gets_fn))
    return OCCAM_API_INVALID_TYPE;
  try {
    *value = strdup(pi->gets_fn().c_str());
  } catch (RegisterProgramError&) {
    return OCCAM_API_READ_ERROR;
  }
  return OCCAM_API_SUCCESS;
}

int OccamDeviceBase::getDeviceValuep(OccamParam id, void** value) {
  int module_index = OCCAM_MODULE_INDEX(id);
  id = OCCAM_PARAM_ID(id);
  if (module_index>0&&module_index<=_modules.size()) {
    IOccamParameters* param_iface;
    int r;
    void* handle = _modules[module_index-1].get();
    r = occamGetInterface(handle, IOCCAMPARAMETERS, (void**)&param_iface);
    if (r != OCCAM_API_SUCCESS)
      return OCCAM_API_NOT_SUPPORTED;
    return param_iface->getValuep(handle,id,value);
  }

  ParamInfo* pi = getParam(id);
  if (!pi) {
    for (ParamInfo& p : _params) {
      int module_index = 0;
      if (bool(p.getp_fn)&&bool(p.geti_fn))
	module_index = p.geti_fn();
      if (module_index>0&&module_index<=_modules.size()) {
	IOccamParameters* param_iface;
	int r;
	void* handle = _modules[module_index-1].get();
	r = occamGetInterface(handle, IOCCAMPARAMETERS, (void**)&param_iface);
	if (r != OCCAM_API_SUCCESS)
	  continue;
	r = param_iface->getValuep(handle,id,value);
	if (r != OCCAM_API_INVALID_PARAMETER)
	  return r;
      }
    }
    return OCCAM_API_INVALID_PARAMETER;
  }
  if (!bool(pi->getp_fn))
    return OCCAM_API_INVALID_TYPE;
  try {
    *value = pi->getp_fn();
  } catch (RegisterProgramError&) {
    return OCCAM_API_READ_ERROR;
  }
  return OCCAM_API_SUCCESS;
}

int OccamDeviceBase::getDeviceValueiv(OccamParam id, int* values, int value_count) {
  int module_index = OCCAM_MODULE_INDEX(id);
  id = OCCAM_PARAM_ID(id);
  if (module_index>0&&module_index<=_modules.size()) {
    IOccamParameters* param_iface;
    int r;
    void* handle = _modules[module_index-1].get();
    r = occamGetInterface(handle, IOCCAMPARAMETERS, (void**)&param_iface);
    if (r != OCCAM_API_SUCCESS)
      return OCCAM_API_NOT_SUPPORTED;
    return param_iface->getValueiv(handle,id,values,value_count);
  }

  ParamInfo* pi = getParam(id);
  if (!pi) {
    for (ParamInfo& p : _params) {
      int module_index = 0;
      if (bool(p.getp_fn)&&bool(p.geti_fn))
	module_index = p.geti_fn();
      if (module_index>0&&module_index<=_modules.size()) {
	IOccamParameters* param_iface;
	int r;
	void* handle = _modules[module_index-1].get();
	r = occamGetInterface(handle, IOCCAMPARAMETERS, (void**)&param_iface);
	if (r != OCCAM_API_SUCCESS)
	  continue;
	r = param_iface->getValueiv(handle,id,values,value_count);
	if (r != OCCAM_API_INVALID_PARAMETER)
	  return r;
      }
    }
    return OCCAM_API_INVALID_PARAMETER;
  }
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

int OccamDeviceBase::getDeviceValuerv(OccamParam id, double* values, int value_count) {
  int module_index = OCCAM_MODULE_INDEX(id);
  id = OCCAM_PARAM_ID(id);
  if (module_index>0&&module_index<=_modules.size()) {
    IOccamParameters* param_iface;
    int r;
    void* handle = _modules[module_index-1].get();
    r = occamGetInterface(handle, IOCCAMPARAMETERS, (void**)&param_iface);
    if (r != OCCAM_API_SUCCESS)
      return OCCAM_API_NOT_SUPPORTED;
    return param_iface->getValuerv(handle,id,values,value_count);
  }

  ParamInfo* pi = getParam(id);
  if (!pi) {
    for (ParamInfo& p : _params) {
      int module_index = 0;
      if (bool(p.getp_fn)&&bool(p.geti_fn))
	module_index = p.geti_fn();
      if (module_index>0&&module_index<=_modules.size()) {
	IOccamParameters* param_iface;
	int r;
	void* handle = _modules[module_index-1].get();
	r = occamGetInterface(handle, IOCCAMPARAMETERS, (void**)&param_iface);
	if (r != OCCAM_API_SUCCESS)
	  continue;
	r = param_iface->getValuerv(handle,id,values,value_count);
	if (r != OCCAM_API_INVALID_PARAMETER)
	  return r;
      }
    }
    return OCCAM_API_INVALID_PARAMETER;
  }
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

int OccamDeviceBase::getDeviceValuesv(OccamParam id, char** values, int value_count) {
  int module_index = OCCAM_MODULE_INDEX(id);
  id = OCCAM_PARAM_ID(id);
  if (module_index>0&&module_index<=_modules.size()) {
    IOccamParameters* param_iface;
    int r;
    void* handle = _modules[module_index-1].get();
    r = occamGetInterface(handle, IOCCAMPARAMETERS, (void**)&param_iface);
    if (r != OCCAM_API_SUCCESS)
      return OCCAM_API_NOT_SUPPORTED;
    return param_iface->getValuesv(handle,id,values,value_count);
  }

  ParamInfo* pi = getParam(id);
  if (!pi) {
    for (ParamInfo& p : _params) {
      int module_index = 0;
      if (bool(p.getp_fn)&&bool(p.geti_fn))
	module_index = p.geti_fn();
      if (module_index>0&&module_index<=_modules.size()) {
	IOccamParameters* param_iface;
	int r;
	void* handle = _modules[module_index-1].get();
	r = occamGetInterface(handle, IOCCAMPARAMETERS, (void**)&param_iface);
	if (r != OCCAM_API_SUCCESS)
	  continue;
	r = param_iface->getValuesv(handle,id,values,value_count);
	if (r != OCCAM_API_INVALID_PARAMETER)
	  return r;
      }
    }
    return OCCAM_API_INVALID_PARAMETER;
  }
  if (!bool(pi->getsv_fn))
    return OCCAM_API_INVALID_TYPE;
  if (value_count != pi->count)
    return OCCAM_API_INVALID_COUNT;
  try {
    std::vector<std::string> tmp(value_count);
    pi->getsv_fn(&tmp[0]);
    for (int j=0;j<value_count;++j)
      values[j] = strdup(tmp[j].c_str());
  } catch (RegisterProgramError&) {
    return OCCAM_API_GENERIC_ERROR;
  }
  return OCCAM_API_SUCCESS;
}

int OccamDeviceBase::getDeviceValuepv(OccamParam id, void** values, int value_count) {
  int module_index = OCCAM_MODULE_INDEX(id);
  id = OCCAM_PARAM_ID(id);
  if (module_index>0&&module_index<=_modules.size()) {
    IOccamParameters* param_iface;
    int r;
    void* handle = _modules[module_index-1].get();
    r = occamGetInterface(handle, IOCCAMPARAMETERS, (void**)&param_iface);
    if (r != OCCAM_API_SUCCESS)
      return OCCAM_API_NOT_SUPPORTED;
    return param_iface->getValuepv(handle,id,values,value_count);
  }

  ParamInfo* pi = getParam(id);
  if (!pi) {
    for (ParamInfo& p : _params) {
      int module_index = 0;
      if (bool(p.getp_fn)&&bool(p.geti_fn))
	module_index = p.geti_fn();
      if (module_index>0&&module_index<=_modules.size()) {
	IOccamParameters* param_iface;
	int r;
	void* handle = _modules[module_index-1].get();
	r = occamGetInterface(handle, IOCCAMPARAMETERS, (void**)&param_iface);
	if (r != OCCAM_API_SUCCESS)
	  continue;
	r = param_iface->getValuepv(handle,id,values,value_count);
	if (r != OCCAM_API_INVALID_PARAMETER)
	  return r;
      }
    }
    return OCCAM_API_INVALID_PARAMETER;
  }
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

int OccamDeviceBase::writeRegister(uint32_t addr, uint32_t value) {
  return OCCAM_API_NOT_SUPPORTED;
}

int OccamDeviceBase::writeRegister(uint32_t addr, uint32_t write_value, uint32_t mask) {
  uint32_t old_value;
  int r = readRegister(addr, &old_value);
  uint32_t new_value = (old_value & ~mask) | write_value;
  return writeRegister(addr, new_value);
}

int OccamDeviceBase::readRegister(uint32_t addr, uint32_t* value) {
  return OCCAM_API_NOT_SUPPORTED;
}

RegisterProgramResponse OccamDeviceBase::program
(const char* prog_str, uint32_t v0, uint32_t v1, uint32_t v2,
 uint32_t v3, uint32_t v4, uint32_t v5, uint32_t v6) {
  RegisterProgram prog(prog_str);
  prog.bind(0,v0);
  prog.bind(1,v1);
  prog.bind(2,v2);
  prog.bind(3,v3);
  prog.bind(4,v4);
  prog.bind(5,v5);
  prog.bind(6,v6);
  return prog.execute(*this);
}

int OccamDeviceBase::writeStorage(uint32_t target, uint32_t addr, uint32_t len, const uint8_t* data) {
  return OCCAM_API_NOT_SUPPORTED;
}

int OccamDeviceBase::readStorage(uint32_t target, uint32_t addr, uint32_t len, uint8_t* data) {
  return OCCAM_API_NOT_SUPPORTED;
}

int OccamDeviceBase::writeStorage(OccamStorageClass storage_class, const std::vector<uint8_t>& data) {
  return OCCAM_API_NOT_SUPPORTED;
}

int OccamDeviceBase::readStorage(OccamStorageClass storage_class, std::vector<uint8_t>& data) {
  return OCCAM_API_NOT_SUPPORTED;
}

const uint32_t SETTINGS_MAGIC = 0x56d8c097;

int OccamDeviceBase::saveSettings() {
  SerializeBuffer sb;
  uint32_t magic = SETTINGS_MAGIC;
  sb<<magic;

  uint32_t module_group_count = _module_groups.size();
  sb<<module_group_count;
  for (int j=0;j<module_group_count;++j) {
    const ModuleGroupInfo& mg = _module_groups[j];
    if (mg.modules.empty())
      continue;
    uint8_t module_count = uint8_t(mg.modules.size());
    uint32_t selector_id = mg.id;
    const std::string& selected_name = mg.modules[mg.selected_index].name;
    sb<<module_count<<selector_id<<selected_name;
    for (int k=0;k<module_count;++k) {
      const ModuleInfo& mi = mg.modules[k];
      uint8_t module_index = mi.module_index;
      const std::string& module_name = mi.name;
      sb<<module_index<<module_name;
    }
  }

  OccamParamList* param_list = 0;
  int r = enumerateParamList(&param_list);
  if (r != OCCAM_API_SUCCESS)
    return r;

  uint32_t serialized_param_count = 0;
  for (int j=0;j<param_list->param_count;++j) {
    OccamParamEntry& p = param_list->params[j];
    if (p.type != OCCAM_PARAM_BOOL &&
	p.type != OCCAM_PARAM_INT)
      continue;
    if (p.storage_class != OCCAM_SETTINGS)
      continue;
    ++serialized_param_count;
  }
  sb<<serialized_param_count;

  for (int j=0;j<param_list->param_count;++j) {
    OccamParamEntry& p = param_list->params[j];
    if (p.type != OCCAM_PARAM_BOOL &&
	p.type != OCCAM_PARAM_INT)
      continue;
    if (p.storage_class != OCCAM_SETTINGS)
      continue;
    uint32_t param_id = (uint32_t)OCCAM_MAKE_PARAM(p.id,p.module_index);
    uint8_t param_type = p.type;
    sb<<param_id<<param_type;
    if (p.type == OCCAM_PARAM_BOOL) {
      bool value0 = false;
      getDeviceValueb((OccamParam)param_id,&value0);
      uint8_t value = value0?1:0;
      sb<<value;
    } else if (p.type == OCCAM_PARAM_INT) {
      int value0 = 0;
      getDeviceValuei((OccamParam)param_id, &value0);
      uint32_t value = value0;
      sb<<value;
    }
  }

  occamFreeParamList(param_list);

  return writeStorage(OCCAM_SETTINGS, sb);
}

int OccamDeviceBase::loadSettings() {
  SerializeBuffer sb;
  int r = readStorage(OCCAM_SETTINGS, sb);
  if (r != OCCAM_API_SUCCESS)
    return r;

  try {
    uint32_t magic = 0;
    sb>>magic;
    if (magic != SETTINGS_MAGIC)
      return OCCAM_API_READ_ERROR;

    std::map<int,int> module_index_map;
    module_index_map.insert(std::make_pair(0,0));

    uint32_t module_group_count = 0;
    sb>>module_group_count;
    for (int j=0;j<module_group_count;++j) {
      uint8_t module_count = 0;
      uint32_t selector_id = 0;
      std::string selected_name;
      sb>>module_count>>selector_id>>selected_name;

      ModuleGroupInfo* mg = 0;
      for (int k=0;k<_module_groups.size();++k) {
	ModuleGroupInfo& mg0 = _module_groups[k];
	if (mg0.id == selector_id) {
	  mg = &mg0;
	  break;
	}
      }
      if (mg) {
	for (int k=0;k<mg->modules.size();++k)
	  if (mg->modules[k].name == selected_name) {
	    mg->selected_index = k;
	    break;
	  }
      }

      for (int k=0;k<module_count;++k) {
	uint8_t module_index = 0;
	std::string module_name;
	sb>>module_index>>module_name;
	if (!mg)
	  continue;
	for (int l=0;l<mg->modules.size();++l)
	  if (mg->modules[l].name == module_name) {
	    module_index_map.insert(std::make_pair(module_index,mg->modules[l].module_index));
	    break;
	  }
      }
    }

    uint32_t serialized_param_count = 0;
    sb>>serialized_param_count;
    for (int j=0;j<serialized_param_count;++j) {
      uint32_t param_id;
      uint8_t param_type;
      sb>>param_id>>param_type;
      OccamParam id = OCCAM_PARAM_ID(param_id);
      int module_index0 = OCCAM_MODULE_INDEX(param_id);
      auto it = module_index_map.find(module_index0);
      int module_index1 = it == module_index_map.end() ? -1 : it->second;
      if (param_type == OCCAM_PARAM_BOOL) {
	uint8_t value = 0;
	sb>>value;
	if (module_index1>=0)
	  setDeviceValueb(OCCAM_MAKE_PARAM(id,module_index1),value?true:false);
      } else if (param_type == OCCAM_PARAM_INT) {
	uint32_t value = 0;
	sb>>value;
	if (module_index1>=0)
	  setDeviceValuei(OCCAM_MAKE_PARAM(id,module_index1),value);
      }
    }


  } catch (SerializeBufferError&) {
    return OCCAM_API_READ_ERROR;
  }

  return OCCAM_API_SUCCESS;
}

int OccamDeviceBase::reset() {
  return OCCAM_API_NOT_SUPPORTED;
}

int OccamDeviceBase::readData(int req_count, const OccamDataName* req, OccamDataType* ret_types, void** ret_data, int block) {
  for (;;) {
    const int max_injest = 100;
    int injest_count;
    for (injest_count=0;injest_count<max_injest;++injest_count) {
      DeviceOutput out;

      int r = readData(out);
      if (r != OCCAM_API_SUCCESS && r != OCCAM_API_DATA_NOT_AVAILABLE)
	return r;
      if (r == OCCAM_API_SUCCESS) {
	if (!out.pack(req_count, req))
	  return OCCAM_API_UNSUPPORTED_DATA;
	_deferred_eval.push(out);
	continue;
      }
      break;
    }
#ifdef DEBUG_SYNC
    if (injest_count>0)
      std::cerr<<"injested "<<injest_count<<std::endl;
#endif // DEBUG_SYNC

    DeviceOutput out;
    if (_deferred_eval.pop(out)) {
      if (out.unpack(req_count, req, ret_types, ret_data))
	return OCCAM_API_SUCCESS;
      else
	return OCCAM_API_UNSUPPORTED_DATA;
    }
    else if (block) {
#ifdef _WIN32
      Sleep(1);
#else // _WIN32
      usleep(1000);
#endif // _WIN32
    } else
      return OCCAM_API_DATA_NOT_AVAILABLE;
  }
}

int OccamDeviceBase::availableData(OccamDevice* device, int* req_count, OccamDataName** req, OccamDataType** types) {
  std::vector<std::pair<OccamDataName,OccamDataType> > available_data;
  availableData(available_data);
  *req_count = available_data.size();
  if (req)
    *req = (OccamDataName*)occamAlloc(sizeof(OccamDataName)*available_data.size());
  if (types)
    *types = (OccamDataType*)occamAlloc(sizeof(OccamDataType)*available_data.size());
  for (int j=0;j<available_data.size();++j) {
    if (req)
      (*req)[j] = available_data[j].first;
    if (types)
      (*types)[j] = available_data[j].second;
  }
  return OCCAM_API_SUCCESS;
}

int OccamDeviceBase::readImage(OccamImage** image, int block) {
  return OCCAM_API_NOT_SUPPORTED;
}

//////////////////////////////////////////////////////////////////////////////////
// OccamMetaDeviceBase

OccamMetaDeviceBase::OccamMetaDeviceBase(const std::string& meta_cid, int prefix_len)
  : OccamDeviceBase(meta_cid),
    default_device_index(0) {
  cid_prefix = meta_cid;
  cid_prefix.erase(cid_prefix.begin()+std::max(0,int(cid_prefix.size()-prefix_len)),
		   cid_prefix.end());

  using namespace std::placeholders;
  registerParams(OCCAM_PAIRED_DEVICES,"paired_devices",OCCAM_NOT_STORED,
		 [this](){
		   std::stringstream sout;
		   for (int j=0;j<devices.size();++j)
		     sout<<(j?",":"")<<devices[j]->serial();
		   return sout.str();
		 });

  auto get_version_comp = [=](OccamParam id) {
    int version = 0;
    for (auto it=devices.begin();it!=devices.end();++it) {
      int versionj = 255;
      (*it)->getDeviceValuei(id,&versionj);
      version = std::max(version,versionj);
    }
    return version;
  };
  auto get_version_combined = [=](){
    int firmware_major = get_version_comp(OCCAM_FIRMWARE_VERSION_A);
    int firmware_minor = get_version_comp(OCCAM_FIRMWARE_VERSION_B);
    int firmware_patch = get_version_comp(OCCAM_FIRMWARE_VERSION_C);
    return (firmware_major<<16)|(firmware_minor<<8)|firmware_patch;
  };
  registerParami(OCCAM_FIRMWARE_VERSION_A,"firmware_version_a",OCCAM_NOT_STORED,0,0,
		 std::bind(get_version_comp,OCCAM_FIRMWARE_VERSION_A));
  registerParami(OCCAM_FIRMWARE_VERSION_B,"firmware_version_a",OCCAM_NOT_STORED,0,0,
		 std::bind(get_version_comp,OCCAM_FIRMWARE_VERSION_B));
  registerParami(OCCAM_FIRMWARE_VERSION_C,"firmware_version_a",OCCAM_NOT_STORED,0,0,
		 std::bind(get_version_comp,OCCAM_FIRMWARE_VERSION_C));
  registerParami(OCCAM_FIRMWARE_VERSION,"firmware_version",OCCAM_NOT_STORED,0,0,
		 get_version_combined);
}

OccamMetaDeviceBase::~OccamMetaDeviceBase() {
}

void OccamMetaDeviceBase::updateDevices() {
  time_t now = time(0);
  if (now == last_update_time)
    return;
  last_update_time = now;

  std::vector<std::string> enum_cids;
  int r = enumerateDevices(enum_cids, false);
  if (r != OCCAM_API_SUCCESS)
    return;
  auto cmp0 = [&](const std::string& cid0) {
    return cid0.find(cid_prefix) != 0;
  };
  auto it = std::remove_if(enum_cids.begin(),enum_cids.end(),cmp0);
  enum_cids.erase(it,enum_cids.end());
  std::sort(enum_cids.begin(),enum_cids.end());

  std::vector<std::string> existing_cids;
  existing_cids.reserve(devices.size());
  for (int j=0;j<devices.size();++j)
    existing_cids.push_back(devices[j]->cid());
  std::sort(existing_cids.begin(),existing_cids.end());

  bool init_required = false;

  int i,j;
  for (i=0,j=0;i<enum_cids.size()&&j<existing_cids.size();) {
    const std::string& cidi = enum_cids[i];
    const std::string& cidj = existing_cids[j];
    if (cidi < cidj) {
      ++i;
      OccamDeviceBase* dev = addDevice(cidi);
      init_required = true;
      if (dev)
	devices.push_back(std::shared_ptr<OccamDeviceBase>(dev));
    } else if (cidj < cidi) {
      ++j;
      init_required = true;
    } else {
      ++i;
      ++j;
    }
  }
  for (;i<enum_cids.size();++i) {
    const std::string& cidi = enum_cids[i];
    OccamDeviceBase* dev = addDevice(cidi);
    init_required = true;
    if (dev)
      devices.push_back(std::shared_ptr<OccamDeviceBase>(dev));
  }

  if (init_required)
    initDevices();
}

void OccamMetaDeviceBase::removeDevice(OccamDeviceBase* dev) {
  auto cmp0 = [dev](const std::shared_ptr<OccamDeviceBase>& dev0) {
    return dev0.get() == dev;
  };
  auto it = std::find_if(devices.begin(),devices.end(),cmp0);
  if (it == devices.end())
    return;
  delete dev;
  devices.erase(it);
}

int OccamMetaDeviceBase::writeRegister(uint32_t addr, uint32_t value) {
  if (addr == 0xde00) {
    default_device_index = value;
    return OCCAM_API_SUCCESS;
  }

  if (devices.empty())
    return OCCAM_API_NOT_INITIALIZED;
  int device_index = addr>>24;
  addr &= (1<<24)-1;
  if (!device_index)
    device_index = default_device_index;
  if (device_index == 0) {
    for (int j=0;j<devices.size();++j)
      devices[j]->writeRegister(addr, value);
    return OCCAM_API_SUCCESS;
  } else if (device_index<=devices.size())
    return devices[device_index-1]->writeRegister(addr, value);
  return OCCAM_API_NOT_INITIALIZED;
}

int OccamMetaDeviceBase::readRegister(uint32_t addr, uint32_t* value) {
  if (addr == 0xde00) {
    *value = default_device_index;
    return OCCAM_API_SUCCESS;
  }

  if (devices.empty())
    return OCCAM_API_NOT_INITIALIZED;
  int device_index = addr>>24;
  addr &= (1<<24)-1;
  if (!device_index)
    device_index = default_device_index;
  if (device_index>0)
    --device_index;
  if (device_index<devices.size())
    return devices[device_index]->readRegister(addr, value);
  return OCCAM_API_NOT_INITIALIZED;
}

int OccamMetaDeviceBase::writeStorage(uint32_t target, uint32_t addr, uint32_t len, const uint8_t* data) {
  if (devices.empty())
    return OCCAM_API_NOT_INITIALIZED;
  int device_index = target>>24;
  target &= (1<<24)-1;
  if (device_index == 0) {
    for (int j=0;j<devices.size();++j)
      devices[j]->writeStorage(target, addr, len, data);
    return OCCAM_API_SUCCESS;
  } else if (device_index<=devices.size())
    return devices[device_index-1]->writeStorage(target, addr, len, data);
  return OCCAM_API_NOT_INITIALIZED;
}

int OccamMetaDeviceBase::readStorage(uint32_t target, uint32_t addr, uint32_t len, uint8_t* data) {
  if (devices.empty())
    return OCCAM_API_NOT_INITIALIZED;
  int device_index = target>>24;
  target &= (1<<24)-1;
  if (device_index == 0) {
    for (int j=0;j<devices.size();++j)
      devices[j]->readStorage(target, addr, len, data);
    return OCCAM_API_SUCCESS;
  } else if (device_index<=devices.size())
    return devices[device_index-1]->readStorage(target, addr, len, data);
  return OCCAM_API_NOT_INITIALIZED;
}

int OccamMetaDeviceBase::writeStorage(OccamStorageClass storage_class, const std::vector<uint8_t>& data) {
  if (devices.empty())
    return OCCAM_API_NOT_INITIALIZED;
  int r = OCCAM_API_SUCCESS;
  for (int j=0;j<devices.size();++j) {
    int r0 = devices[j]->writeStorage(storage_class, data);
    if (r0 != OCCAM_API_SUCCESS)
      r = r0;
  }
  return r;
}

int OccamMetaDeviceBase::readStorage(OccamStorageClass storage_class, std::vector<uint8_t>& data) {
  if (devices.empty())
    return OCCAM_API_NOT_INITIALIZED;
  int r = OCCAM_API_READ_ERROR;
  for (int j=0;j<devices.size();++j) {
    r = devices[j]->readStorage(storage_class, data);
    if (r == OCCAM_API_SUCCESS)
      break;
  }
  return r;
}

int OccamMetaDeviceBase::reset() {
  if (devices.empty())
    return OCCAM_API_NOT_INITIALIZED;
  for (int j=0;j<devices.size();++j)
    devices[j]->reset();
  return OCCAM_API_SUCCESS;
}
