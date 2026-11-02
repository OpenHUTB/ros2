# maps

本目录存放 gmapping 建图结果。

- `map.yaml`：地图配置（图像文件名、分辨率、原点、阈值），已随包提供。
- `map.pgm`：占据栅格地图图像（二进制），由你本机实际建图产生，需手动复制到本目录。

## 复制真实 map.pgm

在你本机建图完成后，`map_saver` 生成的 `map.pgm` 位于保存目录（本示例为
`~/catkin_ws/src/mrobot_navigation/maps/`）。若你的功能包目录与本仓库目录不同，
将真实地图复制到本目录，例如：

```bash
cp ~/catkin_ws/src/mrobot_navigation/maps/map.pgm \
   <本仓库>/src/chap6/catkin_ws/src/mrobot_navigation/maps/map.pgm
```

## 本次建图真实数据

- 地图尺寸：576 × 544 像素
- 分辨率：0.050 m/pix
- `map.pgm` 大小：313396 字节
- `map.yaml` 字段：image / resolution / origin / negate / occupied_thresh / free_thresh
