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
#include <stdio.h>
#include <stdlib.h>

static void reportError(int error_code) {
  fprintf(stderr,"Occam API Error: %i\n",error_code);
  abort();
}

int main(int argc, const char** argv) {
  int r;
  int i;
  int j;
  int dev_index = argc>=2?atoi(argv[1]):0;
  OccamDeviceList* device_list;
  OccamDevice* device;

  if ((r = occamInitialize()) != OCCAM_API_SUCCESS)
    reportError(r);

  if ((r = occamEnumerateDeviceList(2000, &device_list)) != OCCAM_API_SUCCESS)
    reportError(r);
  printf("%i devices found\n", device_list->entry_count);
  for (i=0;i<device_list->entry_count;++i) {
    printf("device[%i]: cid = %s\n",
	   i,device_list->entries[i].cid);
  }
  if (dev_index<0 || dev_index >= device_list->entry_count) {
    fprintf(stderr,"device index %i out of range\n",dev_index);
    return 1;
  }

  if ((r = occamOpenDevice(device_list->entries[dev_index].cid, &device)) != OCCAM_API_SUCCESS)
    reportError(r);

  int sensor_count = 0;
  occamGetDeviceValuei(device, OCCAM_SENSOR_COUNT, &sensor_count);
  OccamDataName* req = (OccamDataName*)occamAlloc(sensor_count*sizeof(OccamDataName));
  OccamImage** images = (OccamImage**)occamAlloc(sensor_count*sizeof(OccamImage*));
  for (i=0;i<sensor_count;++i)
    req[i] = OCCAM_RAW_IMAGE0+i;

  for (i=0;i<10;++i) {
    if ((r = occamDeviceReadData(device, sensor_count, req, 0, (void**)images, 1)) != OCCAM_API_SUCCESS)
      reportError(r);

    for (j=0;j<sensor_count;++j) {
      printf("read image %i.%i: time_ns = %llu, index = %i, format = %i, width = %i, height = %i\n",
	     i, j, (long long unsigned int)images[j]->time_ns, images[j]->index,
	     images[j]->format, images[j]->width, images[j]->height);
      occamFreeImage(images[j]);
    }
  }

  occamFree(images);
  occamFree(req);
  occamCloseDevice(device);
  occamFreeDeviceList(device_list);
  occamShutdown();

  return 0;
}
