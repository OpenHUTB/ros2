# ROS/ROS2 示例

该仓库包含环境搭建、教程、人和载具的ROS/ROS2示例。

## 环境配置

### 本地文档预览 / Local documentation preview

在仓库根目录安装文档依赖并启动本地预览：

Install the documentation dependencies from the repository root and start the local preview:

```bash
python3 -m pip install -r requirements.txt
python3 -m mkdocs serve --dev-addr 127.0.0.1:8000
```

然后打开 <http://127.0.0.1:8000/>。MkDocs 会监视文件变化并自动重新生成页面。仓库根目录未提供 `serve.bat`；Windows 用户可在 PowerShell 中使用 `py -m pip install -r requirements.txt` 和 `py -m mkdocs serve --dev-addr 127.0.0.1:8000`。

Then open <http://127.0.0.1:8000/>. MkDocs watches for file changes and rebuilds the pages automatically. No `serve.bat` is provided at the repository root. On Windows PowerShell, use `py -m pip install -r requirements.txt` and `py -m mkdocs serve --dev-addr 127.0.0.1:8000`.

### ROS 2 示例开发 / ROS 2 example development

构建仓库中的 ROS 2 示例前，先载入对应发行版环境，再按目标功能包的 README 安装依赖并构建。Humble 功能包可使用以下工作区流程：

Before building a ROS 2 example in this repository, source the matching ROS distribution, then follow the target package README to install dependencies and build. For a Humble package, use this workspace workflow:

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select <package_name>
source install/setup.bash
ros2 launch <package_name> <launch_file>
```

发行版安装步骤见 [Ubuntu 20.04 与 ROS Noetic](docs/ubuntu20.04.md)，ROS 2 概念与教程见 [第 9 章](docs/chapter/chap9.md)。运行某个示例时，请使用该模块 README 中列出的包名、launch 文件和额外依赖。

See [Ubuntu 20.04 and ROS Noetic setup](docs/ubuntu20.04.md) for the existing distribution setup guide and [Chapter 9](docs/chapter/chap9.md) for ROS 2 concepts and tutorials. For a specific example, use the package name, launch file, and additional dependencies documented by that module's README.

文档撰写约定请参考 [OpenHUTB/doc](https://github.com/OpenHUTB/doc)。


## 注意事项

* 提交 Pull Request 前请检查：
    * **一定**要用 **`mkdocs serve --livereload`** 检查网页的是否为想要的修改效果后再提交
    * Pull Request 的标题和提交信息不能随意，需要说明为什么修改，而不仅仅是修改了什么
    * Pull Request 页面的 File changed 下的内容是否为想要的修改
    * 检查 Pull Request 页面是否有红色的提示（有冲突），如果有请先合并仓库最新修改再新建 Pull Request
    * 用截图，而不是拍照
    * 不需要说明个人信息，因为git log的提交记录中有
    * 每次 Pull Request 都需要保证能够通过main脚本直接运行整个模块，在提交信息中提供运行效果图（动图可以用[ScreenToGif](https://github.com/NickeManarin/ScreenToGif)），README.md文档中提供运行环境和运行步骤的说明
    * 使用大模型需要在最后声明，并对提交内容负全部责
    * 约定 Markdown 文档放到 [docs](https://github.com/OpenHUTB/ros2/tree/master/docs) 目录和相应的模块文件夹内，源代码（.py、.cpp等）放到 [src](https://github.com/OpenHUTB/ros2/tree/master/src) 目录和相应的模块文件夹内
    * 约定文件名命名方式为全英文，不能包含空格、中文和拼音，多个单词用下划线`_`分割
    * 每个模块的相关文件放在以模块命名的文件夹内，必须支持 launch 启动，入口为main.开头，比如：main.py、main.cpp、main.bat、main.sh等
    * 每个模块需要在导航栏和首页添加跳转的链接
    * 模块名不能宽泛，需要具体，不然就和其他人的容易重复
    * 仓库尽量保存文本文件，二进制文件需要慎重，图片约定使用.png且小于 3 MB，动图使用.gif且小于 10 MB，如运行需要示例数据，可以保存少量数据，大量数据可以通过提供永久网盘链接并说明下载链接和运行说明


* 每次合并到主分支之前需要至少一名其他人 测试并同意（另一台机器修改部分的运行结果）
* Github访问不稳定可以[使用Stem++、fastgithub进行加速（也可使用其他代理）](https://gitee.com/OpenHUTB/sw/releases/tag/up).

