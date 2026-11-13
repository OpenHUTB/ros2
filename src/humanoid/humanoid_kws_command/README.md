# humanoid_kws_command

人形机器人的**离线语音指令控制**：log-mel 前端 + 单隐层 MLP，把音频**直接**分类成命令，
**不接 ASR 引擎、不下载外部预训练权重**。模型 5,899 个参数、权重 25 KB。

| 模块 | 做法 | 依赖 | 参数 |
|---|---|---|---|
| 已有 `humanoid_voice_command` | Whisper tiny（ASR）+ 文本意图 MLP | 需下载 72 MB 权重 | ~39 M |
| 本模块 | log-mel + MLP 端到端分类 | 只要 numpy | **5.9 k** |

## 1. 结论

命令集 10 个（前进 / 后退 / 左转 / 右转 / 停止 / 加速 / 减速 / 挥手 / 蹲下 / 起立），
另有 6 个未知词统一归入 `unknown` 类，共 11 类。

| 评测 | 结果 |
|---|---|
| 留一条件交叉验证（12 折，留出变调/变速条件） | **100%**（最低 100%，最高 100%） |
| 抗噪 20 / 10 dB（训练时见过） | 100% / 100% |
| 抗噪 **5 dB（训练未见过）** | **100%** |
| 抗噪 **0 dB（训练未见过）** | **68.8%** |
| 抗噪 **−5 dB（训练未见过）** | **40.1%** |
| 拒识 FAR / FRR（干净） | 0% / 0% |

**诚实说明**：干净语料上 100% 不能算泛化证据 —— 12 条语料全部来自同一个 TTS 语音
（Microsoft Huihui），只做了音高 × 语速变化，词形与发音完全一致，模型学到的是「这个词
长什么样」而不是「不同人怎么念」。**真人语音的泛化性目前未验证**，见第 5 节。

真正有意义的是抗噪曲线：5 / 0 / −5 dB 这三档训练时完全没出现，仍能到 100% / 68.8% / 40.1%，
说明训练期噪声增强带来的是可用鲁棒性，而不是背答案。

## 2. 效果图

| 抗噪曲线 | 混淆矩阵（干净） |
|---|---|
| ![](../../docs/img/humanoid/humanoid_kws_command/snr_curve.png) | ![](../../docs/img/humanoid/humanoid_kws_command/confusion.png) |

## 3. 运行环境与复现

只需要 numpy（出图另需 matplotlib）。仓库**已附带提取好的特征** `data/features.npz`
（1152×80，329 KB，含 6 个 SNR 档），因此不需要 TTS、不需要语料就能复现训练与评测：

```bash
cd src/humanoid/humanoid_kws_command
python3 -m pip install -r requirements.txt
python3 main.py --version
python3 main.py --test     # 8 项自检
python3 main.py --train    # 留一条件 CV + 全量训练，写 weights/kws_mlp.npz
python3 main.py --eval     # 抗噪曲线 / FAR / FRR / 混淆矩阵（写 results/）
```

重新生成语料（仅 Windows，用系统自带的 System.Speech，不需要联网）：

```powershell
# 4 种音高 × 3 种语速 × 16 个词 = 192 条 wav
powershell -ExecutionPolicy Bypass -File tools/make_corpus.ps1 -OutDir corpus_wav
python3 main.py --build-features corpus_wav     # 重新生成 data/features.npz
```

ROS 2：

```bash
colcon build --packages-select humanoid_kws_command
source install/setup.bash
ros2 launch humanoid_kws_command main.launch.py \
  wav_path:=/abs/path/stop.wav model_path:=/abs/path/weights/kws_mlp.npz
ros2 topic echo /kws_command_node/command --once
ros2 topic echo /kws_command_node/base_command --once   # [线速度, 角速度, 置信度]
```

## 4. 参数

| 参数 | 位置 | 默认 | 说明 |
|---|---|---|---|
| `N_MELS` / `WIN` / `HOP` | `config.py` | 40 / 400 / 160 | 20 ms 窗? 实为 25 ms 窗 / 10 ms 帧移 |
| `HIDDEN` | `config.py` | 64 | 单隐层宽度 |
| `EPOCHS` / `LR` / `L2` | `config.py` | 300 / 0.05 / 1e-4 | SGD + 动量 0.9 |
| `TRAIN_SNRS` | `config.py` | 干净 + 20 + 10 dB | 训练时见过的噪声档 |
| `UNSEEN_SNRS` | `config.py` | 5 / 0 / −5 dB | 只用于评测 |

## 5. 能力边界

1. **真人语音未验证**。语料是单一 TTS 语音，说话人多样性只靠音高/语速模拟。要下结论必须
   接入真人录音 —— 这是本模块最大的缺口，已如实写明。
2. **不是流式识别**：给一个 WAV、判一次；没有 VAD、没有滑窗连续解码。
3. **命令集固定 11 类**，`unknown` 是训练的第六个「词」，不是开集拒识。
4. **只做指令映射**：识别结果发成 `~/base_command`（线速度/角速度），不含完整人形行走控制。
5. 干净语料上的 100% 是语料太容易造成的，不作为性能声明。

## 6. 许可

代码、前端、训练脚本均为本模块原创；语料由 Windows 自带 TTS 合成，不涉及第三方版权。
MIT。