// Local PicknPlace integration patch. SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cmath>
#include <cstdint>
#include <memory>

#include "libobsensor/ObSensor.hpp"

namespace orbbec_camera {

// D2C depth uses the RGB pixel grid, but may have rectified (zero) distortion.
// Inspect this output frame's profile; never substitute cached startup intrinsics.
inline const char *softwareD2CError(const std::shared_ptr<ob::Frame> &depth,
                                  const std::shared_ptr<ob::Frame> &color,
                                  bool alignment_completed) {
  if (!depth || !depth->is<ob::DepthFrame>()) {
    return "Missing usable depth output";
  }
  if (!color || !color->is<ob::ColorFrame>()) {
    return "Missing RGB alignment target";
  }
  if (!alignment_completed) {
    return "Software depth-to-color alignment did not complete";
  }
  const auto depth_video = depth->as<ob::VideoFrame>();
  const auto color_video = color->as<ob::VideoFrame>();
  const auto depth_profile = depth->getStreamProfile();
  const auto color_profile = color->getStreamProfile();
  if (!depth_profile || !color_profile || !depth_profile->is<ob::VideoStreamProfile>() ||
      !color_profile->is<ob::VideoStreamProfile>()) {
    return "Missing video calibration profile";
  }
  const auto d = depth_profile->as<ob::VideoStreamProfile>()->getIntrinsic();
  const auto c = color_profile->as<ob::VideoStreamProfile>()->getIntrinsic();
  if (depth_video->getWidth() != color_video->getWidth() ||
      depth_video->getHeight() != color_video->getHeight() ||
      d.width <= 0 || d.height <= 0 || c.width <= 0 || c.height <= 0 ||
      static_cast<uint32_t>(d.width) != depth_video->getWidth() ||
      static_cast<uint32_t>(d.height) != depth_video->getHeight() ||
      static_cast<uint32_t>(c.width) != color_video->getWidth() ||
      static_cast<uint32_t>(c.height) != color_video->getHeight()) {
    return "Aligned depth/RGB dimensions differ";
  }
  if (!std::isfinite(d.fx) || !std::isfinite(d.fy) || !std::isfinite(d.cx) ||
      !std::isfinite(d.cy) || d.fx <= 0 || d.fy <= 0 || d.cx < 0 || d.cy < 0 ||
      d.cx >= d.width || d.cy >= d.height ||
      d.fx != c.fx || d.fy != c.fy || d.cx != c.cx || d.cy != c.cy) {
    return "Aligned depth intrinsics do not match RGB";
  }
  return nullptr;
}

}  // namespace orbbec_camera
