/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Robert Bosch LLC.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Robert Bosch nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
*********************************************************************/

#include <ros/ros.h>
#include <usb_cam/usb_cam.h>
#include <usb_cam/CameraParameterConfig.h>
#include <usb_cam/rotate.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/image_encodings.h>
#include <sstream>
#include <std_msgs/Duration.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>

namespace usb_cam {

class UsbCamNode
{
public:
  // private ROS node handle
  ros::NodeHandle node_;

  // shared image message
  sensor_msgs::Image img_;
  sensor_msgs::Image img_pad_;
  sensor_msgs::Image img_rotated_;
  image_transport::CameraPublisher image_pub_;

  // parameters
  std::string video_device_name_, io_method_name_, pixel_format_name_, camera_name_, camera_info_url_;
  //std::string start_service_name_, start_service_name_;
  bool streaming_status_;
  int image_width_, image_height_, framerate_, exposure_, brightness_, contrast_, saturation_, sharpness_, focus_,
      white_balance_, gain_;
  bool autofocus_, autoexposure_, auto_white_balance_;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;

  int padding_top_, padding_right_, padding_bottom_, padding_left_;
  int drop_, drop_per_;
  size_t drop_cnt_;

  int gamma_default_, gamma_max_, gamma_min_;

  UsbCam cam_;

  ros::ServiceServer service_start_, service_stop_;
  dynamic_reconfigure::Server<usb_cam::CameraParameterConfig> camera_parameter_server_;
  ros::Subscriber sub_exposure_absolute_, sub_gamma_;

  RotateCode rotate_code_;

  bool service_start_cap(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res )
  {
    cam_.start_capturing();
    return true;
  }


  bool service_stop_cap( std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res )
  {
    cam_.stop_capturing();
    return true;
  }

  void cameraParameterServerCallback(usb_cam::CameraParameterConfig &config, uint32_t level)
  {
    const int exposure_auto = config.exposure_auto ? 0 : 1;
    cam_.set_v4l_parameter("exposure_auto", exposure_auto);
  }

  void exposureAbsoluteCallback(const std_msgs::Duration& msg)
  {
    // V4L2_CID_EXPOSURE_ABSOLUTE is in 100 microseconds units.
    // https://linuxtv.org/downloads/v4l-dvb-apis/uapi/v4l/ext-ctrls-camera.html
    const int exposure_absolute = msg.data.toSec() * 10000;
    cam_.set_v4l_parameter("exposure_absolute", exposure_absolute);
  }

  void gammaCallback(const std_msgs::Float32& msg)
  {
    // Convert real gamma value to device gamma value
    int gamma = msg.data * gamma_default_;
    if(gamma > gamma_max_) gamma = gamma_max_;
    if(gamma < gamma_min_) gamma = gamma_min_;
    cam_.set_v4l_parameter("gamma", gamma);
  }

  UsbCamNode()
    : node_("~")
    , drop_cnt_(0)
    , camera_parameter_server_(node_)
  {
    // advertise the main image topic
    image_transport::ImageTransport it(node_);
    image_pub_ = it.advertiseCamera("image_raw", 1);

    // grab the parameters
    node_.param("video_device", video_device_name_, std::string("/dev/video0"));
    node_.param("brightness", brightness_, -1); //0-255, -1 "leave alone"
    node_.param("contrast", contrast_, -1); //0-255, -1 "leave alone"
    node_.param("saturation", saturation_, -1); //0-255, -1 "leave alone"
    node_.param("sharpness", sharpness_, -1); //0-255, -1 "leave alone"
    // possible values: mmap, read, userptr
    node_.param("io_method", io_method_name_, std::string("mmap"));
    node_.param("image_width", image_width_, 640);
    node_.param("image_height", image_height_, 480);
    node_.param("framerate", framerate_, 30);
    // possible values: yuyv, uyvy, mjpeg, yuvmono10, rgb24
    node_.param("pixel_format", pixel_format_name_, std::string("mjpeg"));
    // enable/disable autofocus
    node_.param("autofocus", autofocus_, false);
    node_.param("focus", focus_, -1); //0-255, -1 "leave alone"
    // enable/disable autoexposure
    node_.param("autoexposure", autoexposure_, true);
    node_.param("exposure", exposure_, 100);
    node_.param("gain", gain_, -1); //0-100?, -1 "leave alone"
    // enable/disable auto white balance temperature
    node_.param("auto_white_balance", auto_white_balance_, true);
    node_.param("white_balance", white_balance_, 4000);

    node_.param("padding_left", padding_left_, 0);
    node_.param("padding_top", padding_top_, 0);
    node_.param("padding_right", padding_right_, 0);
    node_.param("padding_bottom", padding_bottom_, 0);

    node_.param("drop", drop_, 0);
    node_.param("drop_per", drop_per_, 1);

    node_.param("gamma_default", gamma_default_, 100);
    node_.param("gamma_max", gamma_max_, 200);
    node_.param("gamma_min", gamma_min_, 50);

    std::string rotate_code_str;
    node_.param<std::string>("rotate_code", rotate_code_str, "");
    if(rotate_code_str == "90CW_ROT")
    {
      rotate_code_ = ROTATE_90_CW;
    }
    else if(rotate_code_str == "90CCW_ROT")
    {
      rotate_code_ = ROTATE_90_CCW;
    }
    else if(rotate_code_str == "180_ROT")
    {
      rotate_code_ = ROTATE_180;
    }
    else
    {
      if (rotate_code_str != "")
      {
        ROS_WARN("Invalild rotate code: [%s]. No rotation will be applied.", rotate_code_str.c_str());
      }
      rotate_code_ = ROTATE_NONE;
    }

    // load the camera info
    node_.param("camera_frame_id", img_.header.frame_id, std::string("head_camera"));
    node_.param("camera_name", camera_name_, std::string("head_camera"));
    node_.param("camera_info_url", camera_info_url_, std::string(""));
    cinfo_.reset(new camera_info_manager::CameraInfoManager(node_, camera_name_, camera_info_url_));

    // create Services
    service_start_ = node_.advertiseService("start_capture", &UsbCamNode::service_start_cap, this);
    service_stop_ = node_.advertiseService("stop_capture", &UsbCamNode::service_stop_cap, this);

    // camera parameter controls
    camera_parameter_server_.setCallback(boost::bind(&UsbCamNode::cameraParameterServerCallback, this, _1, _2));
    sub_exposure_absolute_ = node_.subscribe("exposure_absolute", 1, &UsbCamNode::exposureAbsoluteCallback, this);
    sub_gamma_ = node_.subscribe("gamma", 1, &UsbCamNode::gammaCallback, this);

    // check for default camera info
    if (!cinfo_->isCalibrated())
    {
      cinfo_->setCameraName(video_device_name_);
      sensor_msgs::CameraInfo camera_info;
      camera_info.header.frame_id = img_.header.frame_id;
      camera_info.width = image_width_;
      camera_info.height = image_height_;
      cinfo_->setCameraInfo(camera_info);
    }


    ROS_INFO("Starting '%s' (%s) at %dx%d via %s (%s) at %i FPS", camera_name_.c_str(), video_device_name_.c_str(),
        image_width_, image_height_, io_method_name_.c_str(), pixel_format_name_.c_str(), framerate_);

    // set the IO method
    UsbCam::io_method io_method = UsbCam::io_method_from_string(io_method_name_);
    if(io_method == UsbCam::IO_METHOD_UNKNOWN)
    {
      ROS_FATAL("Unknown IO method '%s'", io_method_name_.c_str());
      node_.shutdown();
      return;
    }

    // set the pixel format
    UsbCam::pixel_format pixel_format = UsbCam::pixel_format_from_string(pixel_format_name_);
    if (pixel_format == UsbCam::PIXEL_FORMAT_UNKNOWN)
    {
      ROS_FATAL("Unknown pixel format '%s'", pixel_format_name_.c_str());
      node_.shutdown();
      return;
    }

    // start the camera
    cam_.start(video_device_name_.c_str(), io_method, pixel_format, image_width_,
		     image_height_, framerate_);

    // set camera parameters
    if (brightness_ >= 0)
    {
      cam_.set_v4l_parameter("brightness", brightness_);
    }

    if (contrast_ >= 0)
    {
      cam_.set_v4l_parameter("contrast", contrast_);
    }

    if (saturation_ >= 0)
    {
      cam_.set_v4l_parameter("saturation", saturation_);
    }

    if (sharpness_ >= 0)
    {
      cam_.set_v4l_parameter("sharpness", sharpness_);
    }

    if (gain_ >= 0)
    {
      cam_.set_v4l_parameter("gain", gain_);
    }

    // check auto white balance
    if (auto_white_balance_)
    {
      cam_.set_v4l_parameter("white_balance_temperature_auto", 1);
    }
    else
    {
      cam_.set_v4l_parameter("white_balance_temperature_auto", 0);
      cam_.set_v4l_parameter("white_balance_temperature", white_balance_);
    }

    // check auto exposure
    if (!autoexposure_)
    {
      // turn down exposure control (from max of 3)
      cam_.set_v4l_parameter("exposure_auto", 1);
      // change the exposure level
      cam_.set_v4l_parameter("exposure_absolute", exposure_);
    }

    // check auto focus
    if (autofocus_)
    {
      cam_.set_auto_focus(1);
      cam_.set_v4l_parameter("focus_auto", 1);
    }
    else
    {
      cam_.set_v4l_parameter("focus_auto", 0);
      if (focus_ >= 0)
      {
        cam_.set_v4l_parameter("focus_absolute", focus_);
      }
    }
  }

  virtual ~UsbCamNode()
  {
  }

  bool take_and_send_image()
  {
    sensor_msgs::Image *img_ptr(&img_);
    // grab the image
    cam_.grab_image(&img_);

    ++drop_cnt_;
    if (drop_cnt_ % drop_per_ < drop_)
    {
      return true;
    }

    // grab the camera info
    sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
    ci->header.frame_id = img_ptr->header.frame_id;
    ci->header.stamp = img_ptr->header.stamp;

    if (padding_left_ != 0 || padding_top_ != 0 ||
        padding_right_ != 0 || padding_bottom_ != 0)
    {
      const int ch = sensor_msgs::image_encodings::numChannels(img_.encoding);

      img_ptr = &img_pad_;
      img_pad_.header = img_.header;
      img_pad_.encoding = img_.encoding;
      img_pad_.is_bigendian = img_.is_bigendian;
      img_pad_.height = img_.height + padding_top_ + padding_bottom_;
      img_pad_.width = img_.width + padding_left_ + padding_right_;
      img_pad_.step = img_pad_.width * ch;
      img_pad_.data.resize(img_pad_.height * img_pad_.step);

      uint8_t *pixel_ptr(img_.data.data());
      uint8_t *pixel_ptr_pad(img_pad_.data.data() + padding_top_ * img_pad_.step);
      for (size_t i = 0; i < img_.height; ++i)
      {
        memcpy(pixel_ptr_pad + padding_left_ * ch, pixel_ptr, img_.width * ch);
        pixel_ptr += img_.step;
        pixel_ptr_pad += img_pad_.step;
      }

      ci->height += padding_top_ + padding_bottom_;
      ci->width += padding_left_ + padding_right_;
      ci->K[2] += padding_left_;
      ci->K[3 + 2] += padding_top_;
      ci->P[2] += padding_left_;
      ci->P[4 + 2] += padding_top_;

      if (ci->roi.width != 0 && ci->roi.height != 0)
      {
        ci->roi.x_offset += padding_left_;
        ci->roi.y_offset += padding_top_;
      }
    }

    if (rotate_code_ != ROTATE_NONE)
    {
      const int ch = sensor_msgs::image_encodings::numChannels(img_.encoding);
      img_rotated_.header = img_ptr->header;
      img_rotated_.encoding = img_ptr->encoding;
      img_rotated_.is_bigendian = img_ptr->is_bigendian;

      update_camera_info(ci, rotate_code_);
      img_rotated_.width = ci->width;
      img_rotated_.height = ci->height;
      img_rotated_.step = img_rotated_.width * ch;
      img_rotated_.data.resize(img_rotated_.height * img_rotated_.step);
      rotate(img_ptr->data.data(), img_rotated_.data.data(), img_ptr->height, img_ptr->width, ch, rotate_code_);
      image_pub_.publish(img_rotated_, *ci);
    }
    else
    {
      image_pub_.publish(*img_ptr, *ci);
    }


    return true;
  }

  bool spin()
  {
    ros::Rate loop_rate(this->framerate_);
    while (node_.ok())
    {
      if (cam_.is_capturing()) {
        if (!take_and_send_image()) ROS_WARN("USB camera did not respond in time.");
      }
      ros::spinOnce();
      loop_rate.sleep();

    }
    return true;
  }






};

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "usb_cam");
  usb_cam::UsbCamNode a;
  a.spin();
  return EXIT_SUCCESS;
}
