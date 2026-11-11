# 第三方来源与许可

本模块的算法与权重均来自开源项目，未做二次训练改动（意图网络除外，见下）。

## 1. OpenAI Whisper（语音识别）

| 项 | 内容 |
|---|---|
| 仓库 | <https://github.com/openai/whisper> |
| 版本 | `openai-whisper` 20250625（pip 包） |
| 许可证 | MIT |
| 用途 | 口令音频 → 文本 |
| 引用 | Radford et al., *Robust Speech Recognition via Large-Scale Weak Supervision*, 2022 |

**权重不随仓库提供**（`tiny` 约 72 MB，超过主仓库现有最大文件一个量级）。
运行时由 `whisper.load_model('tiny')` 下载到 `~/.cache/whisper`，或用
`scripts/download_weights.py` 预先取一次：

```bash
python scripts/download_weights.py            # 下载并校验 sha256
python scripts/download_weights.py --check    # 只检查本地是否已有
```

官方发布的 `tiny` 权重（75,572,083 B）指纹，用于核对下载结果：

```
65147644a518d12f04e32d6f3b26facc3f8dd46e5390956a9424a650c0ce22b9  tiny.pt
```

权重为 Whisper 官方原版，未做任何修改。断网环境可以在一台能联网的机器上跑
`python scripts/download_weights.py --copy-to <包内 weights/whisper 目录>`，
再把该文件拷过去；`asr.py` 的查找顺序是
`HUMANOID_VOICE_WHISPER` 环境变量 → 包内 `weights/whisper/tiny.pt` → `~/.cache/whisper`。

## 2. 口令音频

`audio/*.wav` 由本模块用 Windows 中文语音合成引擎（Microsoft Huihui，
`System.Speech` / SAPI5）离线合成，语速 130 wpm，再经 `scripts/prepare_audio.py`
统一重采样为 16 kHz / 16 bit / 单声道。合成文本是通用运动口令，不含第三方版权内容。

Linux 环境可用 `espeak-ng` 或 `edge-tts` 生成同类音频，接口一致。

## 3. 仿真执行层

`launch/main.launch.py` 启动的 `mujoco_sim_node` 来自本仓库已有的
`src/humanoid/humanoid_keyboard_control` 包（MIT），本模块不重复实现仿真与运动学控制，
只通过 `/cmd_vel` 标准话题与其对接。

## 4. 意图网络（本模块自研）

`weights/intent_mlp.pt` 由 `scripts/train_intent.py` 在本机 CPU 上训练得到
（约 1 s），训练数据由口令模板自动增广生成，不含任何第三方数据集。
网络结构为字符 1/2/3-gram 哈希特征（512 维）+ 两层 MLP（128 → 64 → 9），
许可证与本仓库一致（BSD-3-Clause）。

```
bd7a0effdcf0dd457ec9be87a30c2b8b5523ae53307a7fb5f558e3c5fcd4189b  weights/intent_mlp.pt
```
