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

#include "system.h"
#include <string.h>
#include <assert.h>

// taken from OpenCV b5cdc03b8143c9a1645e4b99026f073c1c490f81
/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009, Willow Garage Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

#if defined WIN32 || defined _WIN32 || defined WINCE

#ifndef _WIN32_WINNT           // This is needed for the declaration of TryEnterCriticalSection in winbase.h with Visual Studio 2005 (and older?)
  #define _WIN32_WINNT 0x0400  // http://msdn.microsoft.com/en-us/library/ms686857(VS.85).aspx
#endif
#include <windows.h>
#if (_WIN32_WINNT >= 0x0602)
  #include <synchapi.h>
#endif
#undef small
#undef min
#undef max
#undef abs
#include <tchar.h>
#if defined _MSC_VER
  #if _MSC_VER >= 1400
    #include <intrin.h>
  #elif defined _M_IX86
    static void __cpuid(int* cpuid_data, int)
    {
        __asm
        {
            push ebx
            push edi
            mov edi, cpuid_data
            mov eax, 1
            cpuid
            mov [edi], eax
            mov [edi + 4], ebx
            mov [edi + 8], ecx
            mov [edi + 12], edx
            pop edi
            pop ebx
        }
    }
  #endif
#endif
#endif

struct OccamHWFeatures {
  enum { MAX_FEATURE = OCCAM_HARDWARE_MAX_FEATURE };

  OccamHWFeatures() {
    memset( have, 0, sizeof(have) );
    x86_family = 0;
  }

  static OccamHWFeatures initialize() {
    OccamHWFeatures f;
    int cpuid_data[4] = { 0, 0, 0, 0 };

#if defined _MSC_VER && (defined _M_IX86 || defined _M_X64)
    __cpuid(cpuid_data, 1);
#elif defined __GNUC__ && (defined __i386__ || defined __x86_64__)
#ifdef __x86_64__
    asm __volatile__
      (
       "movl $1, %%eax\n\t"
       "cpuid\n\t"
       :[eax]"=a"(cpuid_data[0]),[ebx]"=b"(cpuid_data[1]),[ecx]"=c"(cpuid_data[2]),[edx]"=d"(cpuid_data[3])
       :
       : "cc"
       );
#else
    asm volatile
      (
       "pushl %%ebx\n\t"
       "movl $1,%%eax\n\t"
       "cpuid\n\t"
       "popl %%ebx\n\t"
       : "=a"(cpuid_data[0]), "=c"(cpuid_data[2]), "=d"(cpuid_data[3])
       :
       : "cc"
       );
#endif
#endif

    f.x86_family = (cpuid_data[0] >> 8) & 15;
    if (f.x86_family >= 6) {
      f.have[OCCAM_CPU_MMX] = (cpuid_data[3] & (1 << 23)) != 0;
      f.have[OCCAM_CPU_SSE] = (cpuid_data[3] & (1<<25)) != 0;
      f.have[OCCAM_CPU_SSE2] = (cpuid_data[3] & (1<<26)) != 0;
      f.have[OCCAM_CPU_SSE3] = (cpuid_data[2] & (1<<0)) != 0;
      f.have[OCCAM_CPU_SSSE3] = (cpuid_data[2] & (1<<9)) != 0;
      f.have[OCCAM_CPU_SSE4_1] = (cpuid_data[2] & (1<<19)) != 0;
      f.have[OCCAM_CPU_SSE4_2] = (cpuid_data[2] & (1<<20)) != 0;
      f.have[OCCAM_CPU_POPCNT] = (cpuid_data[2] & (1<<23)) != 0;
      f.have[OCCAM_CPU_AVX] = (((cpuid_data[2] & (1<<28)) != 0)&&((cpuid_data[2] & (1<<27)) != 0));//OS uses XSAVE_XRSTORE and CPU support AVX
    }

    return f;
  }

  int x86_family;
  bool have[MAX_FEATURE+1];
};

static OccamHWFeatures  featuresEnabled = OccamHWFeatures::initialize(), featuresDisabled = OccamHWFeatures();
static OccamHWFeatures* currentFeatures = &featuresEnabled;

bool occamHardwareSupport(int feature) {
  assert(0 <= feature && feature <= OCCAM_HARDWARE_MAX_FEATURE);
  return currentFeatures->have[feature];
}
