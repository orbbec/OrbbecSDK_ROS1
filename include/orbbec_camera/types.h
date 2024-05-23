/*******************************************************************************
 * Copyright (c) 2023 Orbbec 3D Technology, Inc
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************/

#pragma once
#include "libobsensor/ObSensor.hpp"
#include "constants.h"
#include "utils.h"
#include "json.hpp"
#include <functional>
#include <boost/optional.hpp>
#include <eigen3/Eigen/Dense>
#include "orbbec_camera/DeviceInfo.h"
#include "orbbec_camera/Extrinsics.h"
#include "orbbec_camera/Metadata.h"
#include "orbbec_camera/GetDeviceInfo.h"
#include "orbbec_camera/GetBool.h"
#include "orbbec_camera/GetInt32.h"
#include "orbbec_camera/GetString.h"
#include "orbbec_camera/GetCameraInfo.h"
#include "orbbec_camera/SetBool.h"
#include "orbbec_camera/SetInt32.h"
#include "orbbec_camera/SetString.h"
#include "orbbec_camera/GetCameraParams.h"
#include "std_srvs/SetBool.h"
#include "std_srvs/Empty.h"
#include <boost/filesystem.hpp>
#include <atomic>
#include <mutex>

namespace orbbec_camera {

typedef std::pair<ob_stream_type, int> stream_index_pair;

const stream_index_pair COLOR{OB_STREAM_COLOR, 0};
const stream_index_pair DEPTH{OB_STREAM_DEPTH, 0};
const stream_index_pair INFRA0{OB_STREAM_IR, 0};
const stream_index_pair INFRA1{OB_STREAM_IR_LEFT, 0};
const stream_index_pair INFRA2{OB_STREAM_IR_RIGHT, 0};

const stream_index_pair GYRO{OB_STREAM_GYRO, 0};
const stream_index_pair ACCEL{OB_STREAM_ACCEL, 0};

const std::vector<stream_index_pair> IMAGE_STREAMS = {DEPTH, INFRA0, COLOR, INFRA1, INFRA2};

const std::vector<stream_index_pair> HID_STREAMS = {GYRO, ACCEL};
const std::map<std::string, OBDepthPrecisionLevel> DEPTH_PRECISION_STR2ENUM = {
    {"1mm", OB_PRECISION_1MM},    {"0.8mm", OB_PRECISION_0MM8}, {"0.4mm", OB_PRECISION_0MM4},
    {"0.2mm", OB_PRECISION_0MM2}, {"0.1mm", OB_PRECISION_0MM1},
};

const std::map<OBStreamType, OBFrameType> STREAM_TYPE_TO_FRAME_TYPE = {
    {OB_STREAM_COLOR, OB_FRAME_COLOR},
    {OB_STREAM_DEPTH, OB_FRAME_DEPTH},
    {OB_STREAM_IR, OB_FRAME_IR},
    {OB_STREAM_IR_LEFT, OB_FRAME_IR_LEFT},
    {OB_STREAM_IR_RIGHT, OB_FRAME_IR_RIGHT},
    {OB_STREAM_GYRO, OB_FRAME_GYRO},
    {OB_STREAM_ACCEL, OB_FRAME_ACCEL},
};
}  // namespace orbbec_camera
