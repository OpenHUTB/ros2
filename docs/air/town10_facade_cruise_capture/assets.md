# 图像与路线资产记录

三张原始照片来自补充包 `airsim_facade_dataset_20261026/dataset_manual/images`，原始尺寸均为 640 × 360 像素。裁剪框采用左、上、右、下像素坐标，右下边界不包含在内；输出为 1000 × 800 的 RGB PNG。原图只记录外观，不附加无法追溯的拍摄位姿或深度。

| 场景 | 原始文件名 | 裁剪框 |
|---|---|---|
| 砖墙 | `rgb_1792251577_002.png` | `[25,30,615,345]` |
| 灰墙 | `rgb_1792251340_000.png` | `[40,0,600,350]` |
| 玻璃 | `rgb_1792253773_001.png` | `[50,25,620,345]` |

| 原始文件 | SHA-256 |
|---|---|
| `rgb_1792251577_002.png` | `21972d2ffd250dd8d04d4930a6b6e7a173958443fcf8f30fd846baa513ab14b2` |
| `rgb_1792251340_000.png` | `7992ca7cc28790f0357e0d96b87bd6b03533cc99b3236a16ddd6db5cf603eeb5` |
| `rgb_1792253773_001.png` | `db487dc4dcc312df031f7ba9c7b7c11ca1f18843675bc5b8f64411d540260309` |

| 输出纹理 | SHA-256 |
|---|---|
| `brick.png` | `01816b6cd7de148cbcb21fabc28c422ac6430f46153f93927b7dfb35dcf7ec03` |
| `gray.png` | `b3406dad10415c32dc265eb63bddf5b37976a95d27254837cbcc9a4eebaa8408` |
| `glass.png` | `b4d057bc8ebc730dc9d25d3e2eba6bbf9737865d1dc925fdb8ff6f2760513b54` |

精确可机读记录随包保存在 `town10_capture/assets/provenance.json`。纹理由矩形网格的一组明确UV覆盖整个平面，未恢复图中门窗、墙缝和反射物体的三维几何。所有纹理共用10 m × 8 m代理平面，该尺度是实验场景定义。

原生路线来自 `airsim_facade_impl_20261026/01_ros_pkg/airsim_facade_inspection_yolo/config/route_town10.txt`，保留36个NED航点和270°朝向。路线文件SHA-256：`6c3b0479aeeb80dc39dcd35f25df2e608e39ee83d123d6b7cd56cab3e6b4f384`。它的绝对位置需按场景出生点校准。

控制器权重采用仓库 `mujoco_facade_capture/models/facade_controller.json` 的15–32–3结构，随本模块独立安装。权重SHA-256：`03c225afff59f8036fc704688583d4efbbcae0974371e4601e77c47d7fc42504`。
