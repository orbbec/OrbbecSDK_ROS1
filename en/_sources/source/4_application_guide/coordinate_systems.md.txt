### ROS Robot vs Camera Optical Coordination Systems

* Point Of View:
  * Imagine we are standing behind of the camera, and looking forward.
  * Always use this point of view when talking about coordinates, left vs right IRs, position of sensor, etc..

![ROS and Camera Coordinate System](../image/application_guide/image0.png)

* ROS Coordinate System: (X: Forward, Y: Left, Z: Up)
* Camera Optical Coordinate System: (X: Right, Y: Down, Z: Forward)
* All data published in our wrapper topics is optical data taken directly from our camera sensors.
* Static and dynamic TF topics publish optical CS and ROS CS to give the user the ability to transform from one CS to the other.

### Using ROS1 TF Tools

#### Viewing the TF Tree Structure

You can use the following ROS1 commands to print and visualize the TF tree published by the camera package:

**Print all TF relationships:**

```bash
rosrun tf view_frames
```

This command generates a `frames.pdf` file that shows the hierarchical relationships between all frames.

![frame.pdf](../image/application_guide/image4.png)

**View all currently published TF information:**

```bash
rostopic echo /tf
```

#### Visualizing the TF Tree in rviz

You can visualize the TF tree structure and relative positions of coordinate systems in real-time in rviz:

```bash
rviz
```

In rviz:

- Add the `TF` display plugin
- Configure the Fixed Frame to `camera_link` or `camera_depth_optical_frame`, etc.
- Select the TF frames to display

![Visualizing TF Tree](../image/application_guide/image5.png)

### ROS1 Camera TF Calculation and Publishing Mechanism

#### Core Function: `OBCameraNode::calcAndPublishStaticTransform()`

The camera node calculates and publishes static transformation relationships between all sensors through this function. Below is a detailed explanation of the ROS1 code:

#### Quaternion Initialization and Coordinate System Transformation

```cpp
tf2::Quaternion quaternion_optical, zero_rot;
zero_rot.setRPY(0.0, 0.0, 0.0);
quaternion_optical.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
tf2::Vector3 zero_trans(0, 0, 0);
```

**Explanation:**

- `quaternion_optical`: Defines the rotational transformation from the optical coordinate system to the ROS standard coordinate system (90-degree rotation). This rotation converts the camera's optical coordinate system (X-right, Y-down, Z-forward) to the ROS standard coordinate system (X-forward, Y-left, Z-up).
- `zero_trans`: Used to publish transformations that contain only rotation without translation.

#### Base Stream

```cpp
auto base_stream_profile = stream_profile_[base_stream_];
CHECK_NOTNULL(base_stream_profile.get());
```

**Explanation:**

- Selects a base stream (usually the depth stream). The transformations of all other sensors are calculated relative to this base stream.

#### Iterating Through All Streams and Calculating Relative Transformations

```cpp
for (const auto& item : stream_profile_) {
    auto stream_index = item.first;
    auto stream_profile = item.second;
    if (!stream_profile) {
        continue;
    }
    OBExtrinsic ex;
    try {
        ex = stream_profile->getExtrinsicTo(base_stream_profile);
    } catch (const ob::Error& e) {
        ROS_ERROR_STREAM("Failed to get " << stream_name_[stream_index]
                                          << " extrinsic: " << e.getMessage());
        ex = OBExtrinsic({{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 0}});
    }

    auto Q = rotationMatrixToQuaternion(ex.rot);
    Q = quaternion_optical * Q * quaternion_optical.inverse();
    Q = Q.normalize();
    tf2::Vector3 trans(ex.trans[0], ex.trans[1], ex.trans[2]);

    auto timestamp = ros::Time::now();
```

**Explanation:**

- This transformation converts the camera's native optical coordinate system to the ROS standard coordinate system.

#### Publishing TF Transformations

```cpp
if (stream_index.first != base_stream_.first) {
    if (stream_index.first == OB_STREAM_IR_RIGHT && base_stream_.first == OB_STREAM_DEPTH) {
        trans[0] = std::abs(trans[0]);  // because left and right ir calibration is error
    }
    publishStaticTF(timestamp, trans, Q, frame_id_[base_stream_], frame_id_[stream_index]);
}
publishStaticTF(timestamp, zero_trans, quaternion_optical, frame_id_[stream_index],
                optical_frame_id_[stream_index]);
ROS_INFO_STREAM("Publishing static transform from " << stream_name_[stream_index] << " to "
                                                    << stream_name_[base_stream_]);
ROS_INFO_STREAM("Translation " << trans[0] << ", " << trans[1] << ", " << trans[2]);
ROS_INFO_STREAM("Rotation " << Q.getX() << ", " << Q.getY() << ", " << Q.getZ() << ", "
                            << Q.getW());
```

**Explanation:**

- The first `publishStaticTF`: Published only when the current stream is not the base stream. It contains the transformation (translation + rotation) from the base stream to the current sensor.
- The second `publishStaticTF`: Publishes the transformation from the physical frame to the optical frame (pure rotation, no translation).
- `frame_id_[stream_index]`: The physical coordinate system frame name (e.g., `camera_depth_frame`).
- `optical_frame_id_[stream_index]`: The optical coordinate system frame name (e.g., `camera_depth_optical_frame`).
- Special handling for the X-axis offset of left and right infrared cameras, ensuring it is positive through `abs()`.

#### Special Device Handling (FEMTO Series)

```cpp
auto device_info = device_->getDeviceInfo();
CHECK_NOTNULL(device_info);
auto pid = device_info->pid();
if ((pid == FEMTO_BOLT_PID || pid == FEMTO_MEGA_PID) && enable_stream_[DEPTH] &&
    enable_stream_[COLOR]) {
    // calc depth to color
    CHECK_NOTNULL(stream_profile_[COLOR]);
    auto depth_to_color_extrinsics = base_stream_profile->getExtrinsicTo(stream_profile_[COLOR]);
    auto Q = rotationMatrixToQuaternion(depth_to_color_extrinsics.rot);
    Q = quaternion_optical * Q * quaternion_optical.inverse();
    Q = Q.normalize();
    publishStaticTF(ros::Time::now(), zero_trans, Q, camera_link_frame_id_,
                    frame_id_[base_stream_]);
} else {
    publishStaticTF(ros::Time::now(), zero_trans, zero_rot, camera_link_frame_id_,
                    frame_id_[base_stream_]);
}
```

**Explanation:**

- Identifies FEMTO series cameras (FEMTO_BOLT, FEMTO_MEGA) based on device PID.
- For FEMTO series cameras, an additional transformation relationship from depth to color sensor needs to be calculated to ensure correct coordinate system mapping across different camera models.
