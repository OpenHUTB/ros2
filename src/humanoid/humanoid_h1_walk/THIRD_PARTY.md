# 第三方资源与许可

本模块的**代码**（ROS 2 封装、观测拼接、测试脚本、文档）为原创，遵循 BSD-3-Clause。

`assets/` 与 `weights/` 下的模型、网格与策略权重来自
[Unitree Robotics 的 unitree_rl_gym](https://github.com/unitreerobotics/unitree_rl_gym)，
固定到 commit `276801e46c5d433564f24658bac64f254b7d2d4b`，未做任何修改。

| 本模块路径 | 上游路径 |
|---|---|
| `assets/h1/{h1.xml,scene.xml,meshes/}` | `resources/robots/h1/` |
| `assets/h1_2/{h1_2_12dof.xml,scene.xml,meshes/}` | `resources/robots/h1_2/`（只取 `scene.xml` 引用的 12 自由度版本及其网格） |
| `weights/h1_motion.pt` | `deploy/pre_train/h1/motion.pt` |
| `weights/h1_2_motion.pt` | `deploy/pre_train/h1_2/motion.pt` |

## BSD 3-Clause License

Copyright (c) 2024, Unitree Robotics
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
