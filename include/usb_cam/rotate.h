
#ifndef USB_CAM_ROTATE_H
#define USB_CAM_ROTATE_H

#include "sensor_msgs/CameraInfo.h"

namespace usb_cam {

enum RotateCode { ROTATE_NONE, ROTATE_90_CW, ROTATE_90_CCW, ROTATE_180 };

void update_camera_info(sensor_msgs::CameraInfoPtr ci, const RotateCode rotate_code)
{
  if (rotate_code == ROTATE_NONE)
    return;

  if (rotate_code == ROTATE_180)
  {
    // update cx and cy
    ci->K[2] = ci->width - ci->K[2];
    ci->K[3 + 2] = ci->height - ci->K[3 + 2];
    ci->P[2] = ci->width - ci->P[2];
    ci->P[4 + 2] = ci->height - ci->P[4 + 2];

    // update ROI
    if (ci->roi.width != 0 && ci->roi.height != 0)
    {
      ci->roi.x_offset = ci->width - ci->roi.x_offset;
      ci->roi.y_offset = ci->height - ci->roi.y_offset;
    }
  }
  else
  {
    // swap fx and fy
    std::swap(ci->K[0], ci->K[4]);
    std::swap(ci->P[0], ci->P[5]);

    if (rotate_code == ROTATE_90_CW)
    {
      // update cx and cy
      const double tmp_kcx = ci->K[2];
      ci->K[2] = ci->height - ci->K[3 + 2];
      ci->K[3 + 2] = tmp_kcx;
      const double tmp_pcx = ci->P[2];
      ci->P[2] = ci->height - ci->P[4 + 2];
      ci->P[4 + 2] = tmp_pcx;

      // update ROI offset
      if (ci->roi.width != 0 && ci->roi.height != 0)
      {
        const double tmp_x_offset = ci->roi.x_offset;
        ci->roi.x_offset = ci->height - ci->roi.y_offset;
        ci->roi.y_offset = tmp_x_offset;
      }
    }

    if (rotate_code == ROTATE_90_CCW)
    {
      // update cx and cy
      const double tmp_kcx = ci->K[2];
      ci->K[2] = ci->K[3 + 2];
      ci->K[3 + 2] = ci->width - tmp_kcx;
      const double tmp_pcx = ci->P[2];
      ci->P[2] = ci->P[4 + 2];
      ci->P[4 + 2] = ci->width - tmp_pcx;

      // update ROI offset
      if (ci->roi.width != 0 && ci->roi.height != 0)
      {
        const double tmp_x_offset = ci->roi.x_offset;
        ci->roi.x_offset = ci->roi.y_offset;
        ci->roi.y_offset = ci->width - tmp_x_offset;
      }
    }

    // update ROI width, height
    if (ci->roi.width != 0 && ci->roi.height != 0)
    {
      const double tmp_roi_width = ci->roi.width;
      ci->roi.width = ci->roi.height;
      ci->roi.height = tmp_roi_width;
    }

    // update the canmera info width and height values
    // as width and height have been swapped.
    const double tmp_ciw = ci->width;
    ci->width = ci->height;
    ci->height = tmp_ciw;
  }
}

void rotate(uint8_t *src, uint8_t *dst, const int row, const int col, const int ch, const RotateCode rotate_code)
{
  if (rotate_code == ROTATE_90_CW)
  {
    for (int i = 0; i < row; i++)
    {
      for (int j = 0; j < col; j++)
      {
        for (int c = 0; c < ch; c++)
        {
          dst[(row * (j + 1) - 1 - i) * ch + c] = src[(col * i + j) * ch + c];
        }
      }
    }
  }
  else if (rotate_code == ROTATE_90_CCW)
  {
    for (int i = 0; i < row; i++)
    {
      for (int j = 0; j < col; j++)
      {
        for (int c = 0; c < ch; c++)
        {
          dst[(row * j + i) * ch + c] = src[(col * i + j) * ch + c];
        }
      }
    }
  }
  else if (rotate_code == ROTATE_180)
  {
    for (int i = 0; i < row; i++)
    {
      for (int j = 0; j < col; j++)
      {
        for (int c = 0; c < ch; c++)
        {
          dst[(col * (row - i) - 1 - j) * ch + c] = src[(col * i + j) * ch + c];
        }
      }
    }
  }
}
}

#endif  // USB_CAM_ROTATE_H
