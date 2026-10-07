// Local PicknPlace regression tests. SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "orbbec_camera/aligned_depth_guard.hpp"
#include "libobsensor/hpp/Filter.hpp"

namespace {
int checks = 0;

void require(bool value, const char *message) {
  if (!value) {
    throw std::runtime_error(message);
  }
  ++checks;
}

std::shared_ptr<ob::Frame> frame(bool color, const OBCameraIntrinsic &intrinsic,
                               int width = 424, int height = 240) {
  const auto type = color ? OB_STREAM_COLOR : OB_STREAM_DEPTH;
  const auto format = color ? OB_FORMAT_RGB : OB_FORMAT_Y16;
  ob_error *error = nullptr;
  auto profile = ob::StreamProfileFactory::create(
      ob_create_video_stream_profile(type, format, width, height, 30, &error));
  ob::Error::handle(&error);
  auto video_profile = profile->as<ob::VideoStreamProfile>();
  video_profile->setIntrinsic(intrinsic);
  OBCameraDistortion distortion{};
  distortion.model = OB_DISTORTION_BROWN_CONRADY;
  distortion.k1 = color ? .007f : 0.f;
  video_profile->setDistortion(distortion);
  auto output = ob::FrameFactory::createFrameFromStreamProfile(profile);
  std::memset(output->getData(), 0, output->getDataSize());
  if (!color) {
    auto data = reinterpret_cast<uint16_t *>(output->getData());
    std::fill(data, data + width * height, 1000);
  }
  return output;
}
}  // namespace

int main() {
  try {
    // Reproduce the real 424x240 failure: same dimensions, different raw-depth K.
    const OBCameraIntrinsic rgb_k{230.60759f, 230.59514f, 212.41443f, 119.79241f, 424, 240};
    const OBCameraIntrinsic raw_k{207.04921f, 207.04921f, 211.00626f, 122.15625f, 424, 240};
    const auto color = frame(true, rgb_k);
    const auto raw = frame(false, raw_k);
    const auto aligned = frame(false, rgb_k);
    using orbbec_camera::softwareD2CError;
    require(softwareD2CError(raw, nullptr, false), "Depth-only bundle must be rejected");
    require(softwareD2CError(raw, color, false), "Skipped/failed Align must be rejected");
    require(softwareD2CError(aligned, color, false), "Matching K alone cannot prove alignment");
    require(softwareD2CError(raw, color, true), "SDK passthrough with raw K must be rejected");
    require(!softwareD2CError(aligned, color, true), "Registered depth with zero D must pass");
    require(softwareD2CError(nullptr, color, true), "Missing output must be rejected");
    require(softwareD2CError(color, color, true), "Color cannot be used as depth");
    require(softwareD2CError(aligned, raw, true), "Depth cannot be used as RGB target");
    require(softwareD2CError(frame(false, rgb_k, 212, 120), color, true),
            "Different output dimensions must be rejected");
    auto bad = rgb_k;
    bad.width = 212;
    require(softwareD2CError(frame(false, bad), color, true),
            "Calibration dimensions must match pixels");
    for (const float invalid : {0.f, -1.f, std::numeric_limits<float>::quiet_NaN(),
                               std::numeric_limits<float>::infinity()}) {
      bad = rgb_k;
      bad.fx = invalid;
      require(softwareD2CError(frame(false, bad), frame(true, bad), true),
              "Invalid focal length must be rejected even if both profiles match");
    }
    // Exercise the actual SDK Align with synthetic images, without opening a device.
    auto depth_profile = std::const_pointer_cast<ob::StreamProfile>(raw->getStreamProfile());
    auto color_profile = std::const_pointer_cast<ob::StreamProfile>(color->getStreamProfile());
    OBExtrinsic extrinsic{};
    extrinsic.rot[0] = extrinsic.rot[4] = extrinsic.rot[8] = 1;
    extrinsic.trans[0] = 20;
    depth_profile->bindExtrinsicTo(color_profile, extrinsic);
    auto bundle = ob::FrameFactory::createFrameSet();
    bundle->pushFrame(raw);
    bundle->pushFrame(color);
    ob::Align align(OB_STREAM_COLOR);
    align.setAlignToStreamProfile(color_profile);
    auto result = align.process(bundle);
    require(result && result->is<ob::FrameSet>(), "Synthetic SDK Align must return a frameset");
    auto output = result->as<ob::FrameSet>();
    require(!softwareD2CError(output->depthFrame(), output->colorFrame(), true),
            "Real SDK-aligned synthetic depth must pass");
    require(softwareD2CError(raw, nullptr, false), "A later incomplete bundle must still fail");
    require(!softwareD2CError(output->depthFrame(), output->colorFrame(), true),
            "A subsequent valid bundle must pass without a retained failure latch");
    std::cout << checks << " registered-depth checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "Registered-depth regression failed: " << error.what() << '\n';
    return 1;
  }
}
