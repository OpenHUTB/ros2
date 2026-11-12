# fire_detect 构建与 ament 资源索引

`src/chap5/fire_detect` 是火焰视觉检测实验的 `ament_python` 包，节点 `fire_detector_node` 订阅相机图像并发布检测结果。

## 资源索引

`ament_python` 包在 `setup.py` 的 `data_files` 中声明资源索引：

```python
('share/ament_index/resource_index/packages', ['resource/' + package_name]),
```

该路径必须是**普通文件**（通常为空文件）。如果写成目录（`resource/fire_detect/__init__.py`），`setuptools` 的 `install_data` 无法拷贝目录，构建阶段直接失败：

```text
error: can't copy 'resource/fire_detect': doesn't exist or not a regular file
```

## 复现

修复前：

```bash
python3 setup.py install --root=/tmp/fd_before --prefix=/usr
```

命令以 `exit 1` 结束，报 `can't copy 'resource/fire_detect'`。

把 `resource/fire_detect` 改为空文件后：

```bash
python3 setup.py install --root=/tmp/fd_after --prefix=/usr
ls /tmp/fd_after/usr/local/share/ament_index/resource_index/packages/fire_detect
```

安装成功（`exit 0`），索引文件出现在 `share/ament_index/resource_index/packages/fire_detect`。

## 运行

```bash
colcon build --packages-select fire_detect
source install/setup.bash
ros2 launch fire_detect fire_detect.launch.py
```

环境：Ubuntu 22.04 / Python 3.10.12 / setuptools 78.1.1。
