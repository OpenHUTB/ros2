# Third-party notices / 第三方资源说明

本模块的 `assets/` 目录下的 MJCF 模型文件**复制自 Gymnasium 的 MuJoCo 环境资产**，未做功能修改，
仅为避免安装时引入 gymnasium 依赖。原始文件版权归其作者所有，上游采用 MIT 许可证：

| 文件 | 上游路径 | 说明 |
|------|---------|------|
| `assets/half_cheetah.xml` | `gymnasium/envs/mujoco/assets/half_cheetah.xml` | 源自 OpenAI Gym 的 HalfCheetah 模型 |
| `assets/ant.xml` | `gymnasium/envs/mujoco/assets/ant.xml` | 源自 OpenAI Gym 的 Ant 模型（Gymnasium 修订版） |
| `assets/walker2d.xml` | `gymnasium/envs/mujoco/assets/walker2d.xml` | 源自 OpenAI Gym 的 Walker2d 模型（@kallinteris-Andreas 修订版） |

上游项目：

- Gymnasium: https://github.com/Farama-Foundation/Gymnasium
- OpenAI Gym（历史来源）: https://github.com/openai/gym

许可证（MIT）：

```
The MIT License

Copyright (c) 2016 OpenAI
Copyright (c) 2022 Farama Foundation

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
```

其余代码为本仓库原创，遵循本仓库的 BSD-3-Clause 许可证。
