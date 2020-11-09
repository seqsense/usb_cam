/*
* Software License Agreement (BSD License)
*
* Copyright (c) 2020, SEQSENSE.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
*  * Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*  * Redistributions in binary form must reproduce the above
*    copyright notice, this list of conditions and the following
*    disclaimer in the documentation and/or other materials provided
*    with the distribution.
*  * Neither the name of the SEQSENSE. nor the names of its
*    contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

#include <gtest/gtest.h>
#include <usb_cam/rotate.h>

sensor_msgs::CameraInfoPtr create_camera_info()
{
  sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo());
  ci->width = 320;
  ci->height = 240;
  ci->K = {1, 0, 140, 0, 2, 100, 0, 0, 1};
  ci->P = {1, 0, 140, 0, 0, 2, 100, 0, 0, 0, 1, 0};
  ci->roi.x_offset = 20;
  ci->roi.y_offset = 30;
  ci->roi.width = 50;
  ci->roi.height = 40;
  return ci;
}

TEST(UpdateCameraInfo, NoRotation)
{
  sensor_msgs::CameraInfoPtr ci = create_camera_info();
  usb_cam::update_camera_info(ci, usb_cam::RotateCode::ROTATE_NONE);
  // width, height
  ASSERT_EQ(ci->width, 320);
  ASSERT_EQ(ci->height, 240);
  // fx, fy
  ASSERT_EQ(ci->K[0], 1);
  ASSERT_EQ(ci->K[4], 2);
  ASSERT_EQ(ci->P[0], 1);
  ASSERT_EQ(ci->P[5], 2);
  // cx, cy
  ASSERT_EQ(ci->K[2], 140);
  ASSERT_EQ(ci->K[5], 100);
  ASSERT_EQ(ci->P[2], 140);
  ASSERT_EQ(ci->P[6], 100);
  // roi
  ASSERT_EQ(ci->roi.x_offset, 20);
  ASSERT_EQ(ci->roi.y_offset, 30);
  ASSERT_EQ(ci->roi.width, 50);
  ASSERT_EQ(ci->roi.height, 40);
}

TEST(UpdateCameraInfo, 90CWRotation)
{
  sensor_msgs::CameraInfoPtr ci = create_camera_info();
  usb_cam::update_camera_info(ci, usb_cam::RotateCode::ROTATE_90_CW);
  // width, height
  ASSERT_EQ(ci->width, 240);
  ASSERT_EQ(ci->height, 320);
  // fx, fy
  ASSERT_EQ(ci->K[0], 2);
  ASSERT_EQ(ci->K[4], 1);
  ASSERT_EQ(ci->P[0], 2);
  ASSERT_EQ(ci->P[5], 1);
  // cx, cy
  ASSERT_EQ(ci->K[2], 140);
  ASSERT_EQ(ci->K[5], 140);
  ASSERT_EQ(ci->P[2], 140);
  ASSERT_EQ(ci->P[6], 140);
  // roi
  ASSERT_EQ(ci->roi.x_offset, 210);
  ASSERT_EQ(ci->roi.y_offset, 20);
  ASSERT_EQ(ci->roi.width, 40);
  ASSERT_EQ(ci->roi.height, 50);
}

TEST(UpdateCameraInfo, 90CCWRotation)
{
  sensor_msgs::CameraInfoPtr ci = create_camera_info();
  usb_cam::update_camera_info(ci, usb_cam::RotateCode::ROTATE_90_CCW);
  // width, height
  ASSERT_EQ(ci->width, 240);
  ASSERT_EQ(ci->height, 320);
  // fx, fy
  ASSERT_EQ(ci->K[0], 2);
  ASSERT_EQ(ci->K[4], 1);
  ASSERT_EQ(ci->P[0], 2);
  ASSERT_EQ(ci->P[5], 1);
  // cx, cy
  ASSERT_EQ(ci->K[2], 100);
  ASSERT_EQ(ci->K[5], 180);
  ASSERT_EQ(ci->P[2], 100);
  ASSERT_EQ(ci->P[6], 180);
  // roi
  ASSERT_EQ(ci->roi.x_offset, 30);
  ASSERT_EQ(ci->roi.y_offset, 300);
  ASSERT_EQ(ci->roi.width, 40);
  ASSERT_EQ(ci->roi.height, 50);
}

TEST(UpdateCameraInfo, 180Rotation)
{
  sensor_msgs::CameraInfoPtr ci = create_camera_info();
  usb_cam::update_camera_info(ci, usb_cam::RotateCode::ROTATE_180);
  // width, height
  ASSERT_EQ(ci->width, 320);
  ASSERT_EQ(ci->height, 240);
  // fx, fy
  ASSERT_EQ(ci->K[0], 1);
  ASSERT_EQ(ci->K[4], 2);
  ASSERT_EQ(ci->P[0], 1);
  ASSERT_EQ(ci->P[5], 2);
  // cx, cy
  ASSERT_EQ(ci->K[2], 180);
  ASSERT_EQ(ci->K[5], 140);
  ASSERT_EQ(ci->P[2], 180);
  ASSERT_EQ(ci->P[6], 140);
  // roi
  ASSERT_EQ(ci->roi.x_offset, 300);
  ASSERT_EQ(ci->roi.y_offset, 210);
  ASSERT_EQ(ci->roi.width, 50);
  ASSERT_EQ(ci->roi.height, 40);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
