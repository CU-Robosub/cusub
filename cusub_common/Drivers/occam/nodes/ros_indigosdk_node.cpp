/*
  Copyright 2011 - 2015 Occam Robotics Inc - All rights reserved.

  This software is licensed under GPLv2, as specified in the top-level
  LICENSE.txt file. Please contact us at info@occamvisiongroup.com for
  information regarding commercial licensing.
*/

#include <stdio.h>
#include <iostream>
#include <atomic>
#include <ros/ros.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <geometry_msgs/Point32.h>
//not Occam
#include <std_msgs/Float64.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include "indigo.h"

static void reportError(int error_code) {
  ROS_INFO("Occam API Error: %i", error_code);
  abort();
}

#define OCCAM_CHECK(call) { int r = call; if (r != OCCAM_API_SUCCESS) reportError(r); }

static std::string dataNameString(OccamDataName data_name) {
  switch (data_name) {
  case OCCAM_IMAGE0: return "image0";
  case OCCAM_IMAGE1: return "image1";
  case OCCAM_IMAGE2: return "image2";
  case OCCAM_IMAGE3: return "image3";
  case OCCAM_IMAGE4: return "image4";
  case OCCAM_IMAGE5: return "image5";
  case OCCAM_IMAGE6: return "image6";
  case OCCAM_IMAGE7: return "image7";
  case OCCAM_IMAGE8: return "image8";
  case OCCAM_IMAGE9: return "image9";
  case OCCAM_IMAGE10: return "image10";
  case OCCAM_IMAGE11: return "image11";
  case OCCAM_IMAGE12: return "image12";
  case OCCAM_IMAGE13: return "image13";
  case OCCAM_IMAGE14: return "image14";
  case OCCAM_RAW_IMAGE0: return "raw_image0";
  case OCCAM_RAW_IMAGE1: return "raw_image1";
  case OCCAM_RAW_IMAGE2: return "raw_image2";
  case OCCAM_RAW_IMAGE3: return "raw_image3";
  case OCCAM_RAW_IMAGE4: return "raw_image4";
  case OCCAM_RAW_IMAGE5: return "raw_image5";
  case OCCAM_RAW_IMAGE6: return "raw_image6";
  case OCCAM_RAW_IMAGE7: return "raw_image7";
  case OCCAM_RAW_IMAGE8: return "raw_image8";
  case OCCAM_RAW_IMAGE9: return "raw_image9";
  case OCCAM_RAW_IMAGE10: return "raw_image10";
  case OCCAM_RAW_IMAGE11: return "raw_image11";
  case OCCAM_RAW_IMAGE12: return "raw_image12";
  case OCCAM_RAW_IMAGE13: return "raw_image13";
  case OCCAM_RAW_IMAGE14: return "raw_image14";
  case OCCAM_IMAGE_TILES0: return "image_tiles0";
  case OCCAM_IMAGE_TILES1: return "image_tiles1";
  case OCCAM_IMAGE_TILES2: return "image_tiles2";
  case OCCAM_RAW_IMAGE_TILES0: return "raw_image_tiles0";
  case OCCAM_RAW_IMAGE_TILES1: return "raw_image_tiles1";
  case OCCAM_RAW_IMAGE_TILES2: return "raw_image_tiles2";
  case OCCAM_UNDISTORTED_IMAGE_TILES0: return "undistorted_image_tiles0";
  case OCCAM_UNDISTORTED_IMAGE_TILES1: return "undistorted_image_tiles1";
  case OCCAM_UNDISTORTED_IMAGE_TILES2: return "undistorted_image_tiles2";
  case OCCAM_UNDISTORTED_IMAGE0: return "undistorted_image0";
  case OCCAM_UNDISTORTED_IMAGE1: return "undistorted_image1";
  case OCCAM_UNDISTORTED_IMAGE2: return "undistorted_image2";
  case OCCAM_UNDISTORTED_IMAGE3: return "undistorted_image3";
  case OCCAM_UNDISTORTED_IMAGE4: return "undistorted_image4";
  case OCCAM_UNDISTORTED_IMAGE5: return "undistorted_image5";
  case OCCAM_UNDISTORTED_IMAGE6: return "undistorted_image6";
  case OCCAM_UNDISTORTED_IMAGE7: return "undistorted_image7";
  case OCCAM_UNDISTORTED_IMAGE8: return "undistorted_image8";
  case OCCAM_UNDISTORTED_IMAGE9: return "undistorted_image9";
  case OCCAM_UNDISTORTED_IMAGE10: return "undistorted_image10";
  case OCCAM_UNDISTORTED_IMAGE11: return "undistorted_image11";
  case OCCAM_UNDISTORTED_IMAGE12: return "undistorted_image12";
  case OCCAM_UNDISTORTED_IMAGE13: return "undistorted_image13";
  case OCCAM_UNDISTORTED_IMAGE14: return "undistorted_image14";
  case OCCAM_STITCHED_IMAGE0: return "stitched_image0";
  case OCCAM_STITCHED_IMAGE1: return "stitched_image1";
  case OCCAM_STITCHED_IMAGE2: return "stitched_image2";
  case OCCAM_RECTIFIED_IMAGE0: return "rectified_image0";
  case OCCAM_RECTIFIED_IMAGE1: return "rectified_image1";
  case OCCAM_RECTIFIED_IMAGE2: return "rectified_image2";
  case OCCAM_RECTIFIED_IMAGE3: return "rectified_image3";
  case OCCAM_RECTIFIED_IMAGE4: return "rectified_image4";
  case OCCAM_RECTIFIED_IMAGE5: return "rectified_image5";
  case OCCAM_RECTIFIED_IMAGE6: return "rectified_image6";
  case OCCAM_RECTIFIED_IMAGE7: return "rectified_image7";
  case OCCAM_RECTIFIED_IMAGE8: return "rectified_image8";
  case OCCAM_RECTIFIED_IMAGE9: return "rectified_image9";
  case OCCAM_DISPARITY_IMAGE0: return "disparity_image0";
  case OCCAM_DISPARITY_IMAGE1: return "disparity_image1";
  case OCCAM_DISPARITY_IMAGE2: return "disparity_image2";
  case OCCAM_DISPARITY_IMAGE3: return "disparity_image3";
  case OCCAM_DISPARITY_IMAGE4: return "disparity_image4";
  case OCCAM_TILED_DISPARITY_IMAGE: return "tiled_disparity_image";
  case OCCAM_STITCHED_DISPARITY_IMAGE: return "stitched_disparity_image";
  case OCCAM_POINT_CLOUD0: return "point_cloud0";
  case OCCAM_POINT_CLOUD1: return "point_cloud1";
  case OCCAM_POINT_CLOUD2: return "point_cloud2";
  case OCCAM_POINT_CLOUD3: return "point_cloud3";
  case OCCAM_POINT_CLOUD4: return "point_cloud4";
  case OCCAM_STITCHED_POINT_CLOUD: return "stitched_point_cloud";
  }
  return std::string();
}

class Publisher {
  OccamDataName req;
public:
  Publisher(OccamDataName _req)
    : req(_req) {
  }
  Publisher(const Publisher& x) = delete;
  Publisher& operator= (const Publisher& rhs) = delete;
  OccamDataName dataName() const {
    return req;
  }
  virtual bool isRequested() = 0;
  virtual void publish(void* data, const ros::Time& now) = 0;
};

class ImagePublisher : public Publisher {
  image_transport::Publisher pub;
  OccamDataName req;
  std::atomic<int> subscribers;
public:
  ImagePublisher(OccamDataName _req, image_transport::ImageTransport it)
    : Publisher(_req),
      req(_req) {

    std::string req_name = dataNameString(req);
    ROS_INFO("advertising %s",req_name.c_str());
    pub = it.advertise(req_name, 1);
  }
  virtual bool isRequested() {
    if (pub.getNumSubscribers()>0)
      ROS_INFO_THROTTLE(5,"subscribers of data %s = %i",dataNameString(req).c_str(),pub.getNumSubscribers());
    return pub.getNumSubscribers()>0;
  }
  virtual void publish(void* data, const ros::Time& now) {
    OccamImage* img0 = (OccamImage*)data;
    if (!img0)
      return;

    ROS_INFO_THROTTLE(1,"sending data %s...",dataNameString(req).c_str());

    const char* image_encoding = 0;
    int bpp = 1;
    switch (img0->format) {
    case OCCAM_GRAY8:
      bpp = 1;
      image_encoding = "mono8";
      break;
    case OCCAM_RGB24:
      bpp = 3;
      image_encoding = "rgb8";
      break;
    case OCCAM_SHORT1:
      bpp = 2;
      image_encoding = "8SC1";
      break;
    }

    int width = img0->width;
    int height = img0->height;

    sensor_msgs::Image img1;
    img1.header.frame_id = "occam";
    img1.header.stamp = now;
    img1.encoding = image_encoding;
    img1.height = height;
    img1.width = width;
    img1.step = width*bpp;
    img1.data.resize(img1.height*img1.step);
    img1.is_bigendian = 0;
    const uint8_t* srcp = img0->data[0];
    int src_step = img0->step[0];
    uint8_t* dstp = &img1.data[0];
    int dst_step = img1.step;
    for (int j=0;j<height;++j,dstp+=dst_step,srcp+=src_step)
      memcpy(dstp,srcp,width*bpp);

    pub.publish(img1);

    occamFreeImage(img0);
  }
};

class PointCloudPublisher : public Publisher {
  OccamDataName req;
  ros::Publisher pub;
public:
  PointCloudPublisher(OccamDataName _req, ros::NodeHandle nh)
    : Publisher(_req),
      req(_req) {

    std::string req_name = dataNameString(req);
    ROS_INFO("advertising %s",req_name.c_str());
    pub = nh.advertise<sensor_msgs::PointCloud>(nh.resolveName(req_name), 1);
  }
  virtual bool isRequested() {
    if (pub.getNumSubscribers()>0)
      ROS_INFO_THROTTLE(5,"subscribers of data %s = %i",dataNameString(req).c_str(),pub.getNumSubscribers());
    return pub.getNumSubscribers()>0;
  }
  virtual void publish(void* data, const ros::Time& now) {
    OccamPointCloud* pc0 = (OccamPointCloud*)data;
    if (!pc0)
      return;

    ROS_INFO_THROTTLE(1,"sending data %s...",dataNameString(req).c_str());

    sensor_msgs::PointCloud pc1;
    pc1.header.frame_id = "occam";
    pc1.points.resize(pc0->point_count);
    for (int j=0,k=0;j<pc0->point_count;++j,k+=3) {
      geometry_msgs::Point32& p1 = pc1.points[j];
      p1.x = pc0->xyz[k+0] / 1000.f;
      p1.y = pc0->xyz[k+1] / 1000.f;
      p1.z = pc0->xyz[k+2] / 1000.f;
    }

    if (pc0->rgb) {
      sensor_msgs::ChannelFloat32& rgb_channel = *pc1.channels.emplace(pc1.channels.end());
      rgb_channel.name = "rgb";
      rgb_channel.values.resize(pc0->point_count);
      for (int j=0,k=0;j<rgb_channel.values.size();++j,k+=3) {
	float& rgbf = rgb_channel.values[j];
	uint32_t& rgb = *(uint32_t*)&rgbf;
	uint8_t r = pc0->rgb[k+0];
	uint8_t g = pc0->rgb[k+1];
	uint8_t b = pc0->rgb[k+2];
	rgb = uint32_t(b)|(uint32_t(g)<<8)|(uint32_t(r)<<16);
      }
    }

    pub.publish(pc1);

    occamFreePointCloud(pc0);
  }
};

class OccamConfig {
  ros::NodeHandle nh;
  ros::ServiceServer set_service;
  ros::Publisher update_pub;
  ros::Publisher descr_pub;

  std::string cid;
  OccamDevice* device;
  OccamParamList* param_list;

  void initSensorConfig();
  void initImageProcessingConfig();
  void initBMConfig(int select_index);
  void initStereoConfig();
  void initCylindricalBlenderConfig(int select_index);
  void initBlendingConfig();
  void initAdvancedConfig();
  void initConfig();

  OccamParam findParamId(const std::string& name) {
    std::string::size_type p0 = name.find_first_of('-');
    std::string base_name;
    int module_index;
    if (p0 != std::string::npos) {
      base_name = std::string(name.begin(),name.begin()+p0);
      module_index = atoi(std::string(name.begin()+p0+1,name.end()).c_str());
    } else {
      base_name = name;
      module_index = 0;
    }
    
    for (int j=0;j<param_list->param_count;++j) {
      if (param_list->params[j].read_only)
	continue;
      if (param_list->params[j].name != base_name)
	continue;
      if (module_index && param_list->params[j].module_index != module_index)
	continue;
      return param_list->params[j].id;
    }

    return OccamParam(-1);
  }

  bool setConfigCallback(dynamic_reconfigure::Reconfigure::Request& req, 
			 dynamic_reconfigure::Reconfigure::Response& rsp) {

    for (dynamic_reconfigure::IntParameter& param : req.config.ints) {
      OccamParam id = findParamId(param.name);
      OCCAM_CHECK(occamSetDeviceValuei(device, id, param.value));
    }
    for (dynamic_reconfigure::BoolParameter& param : req.config.bools) {
      OccamParam id = findParamId(param.name);
      OCCAM_CHECK(occamSetDeviceValuei(device, id, param.value?1:0));
    }

    generateConfig(rsp.config);
    publishConfigDescription();
    return true;
  }

  void publishConfigDescription() {
    dynamic_reconfigure::ConfigDescription msg;
    
    dynamic_reconfigure::Group& group = *msg.groups.emplace(msg.groups.end());
    group.name = "Default";
    group.type = "";
    group.parent = 0;
    group.id = 0;

    for (int j=0;j<param_list->param_count;++j) {
      OccamParamEntry& p0 = param_list->params[j];
      if (p0.read_only)
	continue;
      OccamParam id = OCCAM_MAKE_PARAM(p0.id,p0.module_index);

      if (p0.type == OCCAM_PARAM_INT) {
	{
	  dynamic_reconfigure::ParamDescription& desc = *group.parameters.emplace(group.parameters.end());
	  desc.name = p0.name;
	  desc.type = "int";
	  desc.level = 0;
	  desc.description = p0.name;
	  desc.edit_method = "";

	  int value_count = 0;
	  if (OCCAM_API_SUCCESS == occamGetDeviceValueCount(device, id, &value_count)) {
	    char** valuesv = (char**)alloca(sizeof(char*)*value_count);
	    memset(valuesv,0,sizeof(char*)*value_count);
	    int* valueiv = (int*)alloca(sizeof(int)*value_count);
	    if (OCCAM_API_SUCCESS == occamGetDeviceValuesv(device, id, valuesv, value_count) &&
		OCCAM_API_SUCCESS == occamGetDeviceValueiv(device, id, valueiv, value_count)) {
	      std::stringstream sout;
	      sout<<"{'enum_description': '"<<p0.name<<"', 'enum': [";
	      for (int j=0;j<value_count;++j) {
		if (j)
		  sout<<", ";
		sout<<"{'description': '"<<p0.name<<"', 'cconsttype': 'int', 'value': "<<valueiv[j]<<", 'ctype': 'int', 'type': 'int', 'name': '"<<valuesv[j]<<"'}";
	      }
	      sout<<"]}";
	      desc.edit_method = sout.str();
	    }
	    for (int j=0;j<value_count;++j)
	      occamFree(valuesv[j]);
	  }
	}

	{
	  dynamic_reconfigure::IntParameter& param = *msg.min.ints.emplace(msg.min.ints.end());
	  param.name = p0.name;
	  param.value = int(p0.min_value);
	}
	{
	  dynamic_reconfigure::IntParameter& param = *msg.max.ints.emplace(msg.max.ints.end());
	  param.name = p0.name;
	  param.value = int(p0.max_value);
	}
	{
	  dynamic_reconfigure::IntParameter& param = *msg.dflt.ints.emplace(msg.dflt.ints.end());
	  param.name = p0.name;
	  int value0 = 0;
	  OCCAM_CHECK(occamGetDeviceValuei(device, id, &value0));
	  param.value = value0;
	}
      }

      else if (p0.type == OCCAM_PARAM_BOOL) {
	{
	  dynamic_reconfigure::ParamDescription& desc = *group.parameters.emplace(group.parameters.end());
	  desc.name = p0.name;
	  desc.type = "bool";
	  desc.level = 0;
	  desc.description = p0.name;
	  desc.edit_method = "";
	}

	{
	  dynamic_reconfigure::BoolParameter& param = *msg.min.bools.emplace(msg.min.bools.end());
	  param.name = p0.name;
	  param.value = false;
	}
	{
	  dynamic_reconfigure::BoolParameter& param = *msg.max.bools.emplace(msg.max.bools.end());
	  param.name = p0.name;
	  param.value = true;
	}
	{
	  dynamic_reconfigure::BoolParameter& param = *msg.dflt.bools.emplace(msg.dflt.bools.end());
	  param.name = p0.name;
	  int value0 = 0;
	  OCCAM_CHECK(occamGetDeviceValuei(device, id, &value0));
	  param.value = bool(value0);
	}
      }
      
    }

    descr_pub.publish(msg);
  }
  void generateConfig(dynamic_reconfigure::Config& msg) {

    for (int j=0;j<param_list->param_count;++j) {
      OccamParamEntry& p0 = param_list->params[j];
      if (p0.read_only)
	continue;
      OccamParam id = OCCAM_MAKE_PARAM(p0.id,p0.module_index);

      if (p0.type == OCCAM_PARAM_INT) {
	dynamic_reconfigure::IntParameter& param = *msg.ints.emplace(msg.ints.end());
	param.name = p0.name;
	int value0 = 0;
	OCCAM_CHECK(occamGetDeviceValuei(device, id, &value0));
	param.value = value0;
      }

      else if (p0.type == OCCAM_PARAM_BOOL) {
	dynamic_reconfigure::BoolParameter& param = *msg.bools.emplace(msg.bools.end());
	param.name = p0.name;
	int value0 = 0;
	OCCAM_CHECK(occamGetDeviceValuei(device, id, &value0));
	param.value = bool(value0);
      }
    }
  }

  void publishConfig() {
    dynamic_reconfigure::Config msg;
    generateConfig(msg);

    update_pub.publish(msg);
  }

public:
  OccamConfig(ros::NodeHandle _nh,const std::string& cid, OccamDevice* _device)
    : nh(_nh),
      device(_device),
      param_list(0) {
    OCCAM_CHECK(occamEnumerateParamList(device, &param_list));

    set_service = nh.advertiseService("set_parameters",
				      &OccamConfig::setConfigCallback, this);
    descr_pub = nh.advertise<dynamic_reconfigure::ConfigDescription>("parameter_descriptions", 1, true);
    update_pub = nh.advertise<dynamic_reconfigure::Config>("parameter_updates", 1, true);

    publishConfigDescription();
    publishConfig();
  }
  virtual ~OccamConfig() {
    if (param_list)
      occamFreeParamList(param_list);
  }
};

class OccamNode {
public:
  ros::NodeHandle nh;

  // not occam
  ros::Subscriber sub = nh.subscribe("/occam/set_exposure", 1000, &OccamNode::exposure_callback, this);

  std::string cid;
  OccamDevice* device;
  std::vector<std::shared_ptr<Publisher> > pubs;
  std::shared_ptr<OccamConfig> config;

  OccamNode() :
    nh("~"),
    device(0) {

    int r;

    OccamDeviceList* device_list;
    OCCAM_CHECK(occamEnumerateDeviceList(2000, &device_list));
    ROS_INFO("%i devices found", device_list->entry_count);
    int dev_index = 0;
    for (int i=0;i<device_list->entry_count;++i) {
      if (!cid.empty() && device_list->entries[i].cid == cid) {
	dev_index = i;
      }
      ROS_INFO("device[%i]: cid = %s",i,device_list->entries[i].cid);
    }
    if (dev_index<0 || dev_index >= device_list->entry_count)
      return;
    if (!cid.empty() && device_list->entries[dev_index].cid != cid) {
      ROS_INFO("Requested cid %s not found",cid.c_str());
      return;
    }

    OCCAM_CHECK(occamOpenDevice(device_list->entries[dev_index].cid, &device));
    OCCAM_CHECK(occamFreeDeviceList(device_list));

    image_transport::ImageTransport it(nh);

    int req_count;
    OccamDataName* req;
    OccamDataType* types;
    OCCAM_CHECK(occamDeviceAvailableData(device, &req_count, &req, &types));
    for (int j=0;j<req_count;++j) {
      if (types[j] == OCCAM_IMAGE)
	pubs.push_back(std::make_shared<ImagePublisher>(req[j],it));
      else if (types[j] == OCCAM_POINT_CLOUD)
	pubs.push_back(std::make_shared<PointCloudPublisher>(req[j],nh));
    }
    occamFree(req);
    occamFree(types);

    config = std::make_shared<OccamConfig>(nh,cid,device);
  }

  virtual ~OccamNode() {
    if (device)
      occamCloseDevice(device);
  }

  OccamNode(const OccamNode& x) = delete;
  OccamNode& operator= (const OccamNode& rhs) = delete;


  // not occam stuff
  void exposure_callback(const std_msgs::Float64::ConstPtr& msg) {
        int exposure = 0;

        OCCAM_CHECK(occamSetDeviceValuei(device, OCCAM_EXPOSURE, msg->data));
        OCCAM_CHECK(occamGetDeviceValuei(device, OCCAM_EXPOSURE, &exposure));
        ROS_INFO("exposure is now %d", exposure);

  }


  bool take_and_send_data() {
    int r;

    std::vector<OccamDataName> reqs;
    std::vector<std::shared_ptr<Publisher> > reqs_pubs;
    reqs.reserve(pubs.size());
    reqs_pubs.reserve(pubs.size());
    for (std::shared_ptr<Publisher> pub : pubs)
      if (pub->isRequested()) {
	reqs.push_back(pub->dataName());
	reqs_pubs.push_back(pub);
      }

    std::vector<OccamDataType> types;
    std::vector<void*> data;
    types.resize(reqs.size());
    data.resize(reqs.size());
    r = occamDeviceReadData(device, reqs.size(), &reqs[0], &types[0], &data[0], 0);
    if (r != OCCAM_API_SUCCESS && r != OCCAM_API_DATA_NOT_AVAILABLE) {
      char error_str[256] = {0};
      occamGetErrorString((OccamError)r, error_str, sizeof(error_str));
      ROS_ERROR_THROTTLE(10,"Driver returned error %s (%i)",error_str,r);
      return false;
    }
    if (r != OCCAM_API_SUCCESS)
      return false;

    ros::Time now = ros::Time::now();
    for (int j=0;j<reqs.size();++j)
      reqs_pubs[j]->publish(data[j], now);

    return true;
  }

  bool spin() {
    if (!device)
      return false;

    while (nh.ok()) {
      if (!take_and_send_data())
        usleep(1000);
      ros::spinOnce();
    }
    return true;
  }
};

int main(int argc, char **argv) {
  OCCAM_CHECK(occamInitialize());
  ros::init(argc, argv, "occam");
  OccamNode a;
  a.spin();
  exit(0);
  return 0;
}
