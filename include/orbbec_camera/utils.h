/**************************************************************************/
/*                                                                        */
/* Copyright (c) 2013-2021 Orbbec 3D Technology, Inc                      */
/*                                                                        */
/* PROPRIETARY RIGHTS of Orbbec 3D Technology are involved in the         */
/* subject matter of this material. All manufacturing, reproduction, use, */
/* and sales rights pertaining to this subject matter are governed by the */
/* license agreement. The recipient of this software implicitly accepts   */
/* the terms of the license.                                              */
/*                                                                        */
/**************************************************************************/

#pragma once

#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/distortion_models.h"
#include "libobsensor/h/ObTypes.h"

class Utils
{
  public:
    static sensor_msgs::CameraInfo convertToCameraInfo(OBCameraIntrinsic intrinsic, OBCameraDistortion distortion)
    {
        sensor_msgs::CameraInfo info;
        info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
        info.width = intrinsic.width;
        info.height = intrinsic.height;
        info.D.resize(5, 0.0);
        info.D[0] = distortion.k1;
        info.D[1] = distortion.k2;
        info.D[2] = distortion.k3;
        info.D[3] = distortion.k4;
        info.D[4] = distortion.k5;

        info.K.assign(0.0);
        info.K[0] = intrinsic.fx;
        info.K[2] = intrinsic.cx;
        info.K[4] = intrinsic.fy;
        info.K[5] = intrinsic.cy;
        info.K[8] = 1.0;

        info.R.assign(0.0);
        info.R[0] = 1;
        info.R[4] = 1;
        info.R[8] = 1;

        info.P.assign(0.0);
        info.P[0] = info.K[0];
        info.P[2] = info.K[2];
        info.P[5] = info.K[4];
        info.P[6] = info.K[5];
        info.P[10] = 1.0;

        // int width = mColorProfile->width();
        // double scaling = (double)width / 640;
        // info.K[0] *= scaling; // fx
        // info.K[2] *= scaling; // cx
        // info.K[4] *= scaling; // fy
        // info.K[5] *= scaling; // cy
        // info.P[0] *= scaling; // fx
        // info.P[2] *= scaling; // cx
        // info.P[5] *= scaling; // fy
        // info.P[6] *= scaling; // cy

        return info;
    }
};
