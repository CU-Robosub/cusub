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

#ifdef _WIN32
#include <windows.h>
#endif // _WIN32
#include "indigo.h"
#include "system.h"
#include <stdlib.h>
#include <string.h>

OCCAM_API int occamFreeMarkers(OccamMarkers* markers) {
  if (!markers)
    return OCCAM_API_SUCCESS;

  int refcnt1 = OCCAM_XADD(&markers->refcnt, -1);

  if (refcnt1 == 1) {
    if (markers->cid)
      free(markers->cid);
    if (markers->fields)
      free(markers->fields);
    if (markers->markers)
      free(markers->markers);
  }

  return OCCAM_API_SUCCESS;
}

OCCAM_API int occamCopyMarkers(const OccamMarkers* markers, OccamMarkers** new_markers, int deep_copy) {

  if (deep_copy) {
    *new_markers = new OccamMarkers(*markers);
    (*new_markers)->refcnt = 1;
    if (markers->cid)
      (*new_markers)->cid = strdup(markers->cid);

    (*new_markers)->fields = (OccamMarkerField*)malloc(sizeof(OccamMarkerField)*(*new_markers)->field_count);
    memcpy((*new_markers)->fields,markers->fields,sizeof(OccamMarkerField)*(*new_markers)->field_count);

    int marker_size = 0;
    for (int j=0;j<(*new_markers)->field_count;++j) {
      switch ((*new_markers)->fields[j].type) {
      case OCCAM_FIELD_UINT32:
	marker_size += 4;
	break;
      case OCCAM_FIELD_FLOAT:
	marker_size += 4;
	break;
      }
    }

    (*new_markers)->markers = malloc(marker_size*(*new_markers)->marker_count);
    memcpy((*new_markers)->markers,markers->markers,marker_size*(*new_markers)->marker_count);

  } else {
    OCCAM_XADD(&((OccamMarkers*)markers)->refcnt, 1);
    *new_markers = (OccamMarkers*)markers;
  }

  return OCCAM_API_SUCCESS;
}

OCCAM_API int occamGetMarkerField(const OccamMarkers* markers, int i,
				  OccamMarkerFieldName name, OccamMarkerFieldType type,
				  void* value) {
  int field_offset = -1;
  int marker_size = 0;
  OccamMarkerFieldType type0;
  for (int j=0;j<markers->field_count;++j) {
    if (markers->fields[j].name == name) {
      field_offset = marker_size;
      type0 = markers->fields[j].type;
    }
    switch (markers->fields[j].type) {
    case OCCAM_FIELD_UINT32:
      marker_size += 4;
      break;
    case OCCAM_FIELD_FLOAT:
      marker_size += 4;
      break;
    }
  }
  if (field_offset < 0)
    return OCCAM_API_FIELD_NOT_FOUND;

  uint8_t* value0 = ((uint8_t*)markers->markers)+i*marker_size+field_offset;
  switch (type) {
  case OCCAM_FIELD_UINT32: {
    switch (type0) {
    case OCCAM_FIELD_UINT32: {
      *(uint32_t*)value = *(uint32_t*)value0;
      break;
    }
    case OCCAM_FIELD_FLOAT: {
      *(uint32_t*)value = uint32_t(*(float*)value0);
      break;
    }
    default: {
      return OCCAM_API_INVALID_TYPE;
    }
    }
    break;
  }
  case OCCAM_FIELD_FLOAT: {
    switch (type0) {
    case OCCAM_FIELD_UINT32: {
      *(float*)value = float(*(uint32_t*)value0);
      break;
    }
    case OCCAM_FIELD_FLOAT: {
      *(float*)value = *(float*)value0;
      break;
    }
    default: {
      return OCCAM_API_INVALID_TYPE;
    }
    }
    break;
  }
  default: {
    return OCCAM_API_INVALID_TYPE;
  }
  }

  return OCCAM_API_SUCCESS;
}
