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

#include <mutex>
#include <utility>

namespace orbbec_camera {

// Owned by callbacks, independently of the object they call. detach() drains an
// active callback before permitting the target to be destroyed. Never call it
// while holding a lock that a callback may acquire.
template <typename Target>
class CallbackContext {
 public:
  void attach(Target* target) {
    std::lock_guard<std::mutex> lock(mutex_);
    target_ = target;
  }

  void detach() {
    std::lock_guard<std::mutex> lock(mutex_);
    target_ = nullptr;
  }

  template <typename Callback>
  void invoke(Callback&& callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (target_) {
      std::forward<Callback>(callback)(*target_);
    }
  }

 private:
  std::mutex mutex_;
  Target* target_ = nullptr;
};

}  // namespace orbbec_camera
