# MoveIt calibration pipeline attribution

This standalone implementation follows the ROS 2 logic in
[moveit/moveit_calibration](https://github.com/moveit/moveit_calibration/tree/3f9d48ebe843caf1de060bfafe78160585c7c26f),
commit `3f9d48ebe843caf1de060bfafe78160585c7c26f`:

- `moveit_calibration_plugins/handeye_calibration_solver/src/handeye_solver_opencv.cpp`:
  Tsai1989 and mounting-mode input conventions; author Andrej Orsula.
- `moveit_calibration_plugins/handeye_calibration_solver/include/moveit/handeye_calibration_solver/handeye_solver_base.h`:
  adjacent-pair AX=XB residual formula adapted in `calibration_core.py`; author Yu Yan.
- `moveit_calibration_gui/handeye_calibration_rviz_plugin/src/handeye_control_widget.cpp`:
  five-sample automatic solving and all-prior robot/board rotation gates; author Yu Yan.
- `moveit_calibration_plugins/handeye_calibration_target/src/handeye_target_charuco.cpp`:
  RGB/grayscale ChArUco pipeline and detector choices; authors Yu Yan, John Stechschulte.

This is not a vendored MoveIt ROS package or plugin. There is no MoveIt runtime
dependency, target-TF publication, robot motion, or solver selection. Modern
OpenCV 4.10 APIs replace upstream legacy calls, the existing physical legacy
board layout is explicit, and AX=XB translation/rotation values use correctly
named mm/degree fields rather than the upstream GUI's reversed pair labels.
The private OpenCV wheel's separate notices remain under `third_party/wheels/`.

## Solver and GUI source license

Software License Agreement (BSD License)

Copyright (c) 2019, Intel Corporation.
Copyright (c) 2023, University of Luxembourg
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

- Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.
- Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the following
  disclaimer in the documentation and/or other materials provided
  with the distribution.
- Neither the name of Willow Garage nor the names of its
  contributors may be used to endorse or promote products derived
  from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

## ChArUco target source license

Software License Agreement (BSD License)

Copyright (c) 2020, PickNik, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

- Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.
- Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the following
  disclaimer in the documentation and/or other materials provided
  with the distribution.
- Neither the name of PickNik, Inc., nor the names of its
  contributors may be used to endorse or promote products derived
  from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
