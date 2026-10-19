# ROS/ROS2 示例

该仓库包含环境搭建、教程、人和载具的 ROS/ROS2 示例。

## 本仓库适合收录的内容

本仓库围绕 ROS/ROS2 与机器人仿真，适合补充可复现的环境配置、概念教程和能够独立启动的示例。示例主题可结合现有的人形机器人、地面/空中/水下载具等内容，优先选择目标平台明确、能展示实际输入输出的具体问题。

选题前建议先检查 `docs/`、`src/` 和导航目录，确认内容没有重复。新增模块应说明适用的仿真器和 ROS/ROS2 版本、依赖、启动命令与预期结果；文档与源码按下文约定放置，并同步更新 MkDocs 导航与首页入口。

## 环境配置

运行 [`serve.bat`](https://github.com/OpenHUTB/ros2/blob/master/serve.bat)（即运行 `mkdocs serve`，所做修改也需要通过该脚本在[本地网页](http://127.0.0.1:8000/)中验证。详情请参考 [serve.bat](https://github.com/OpenHUTB/.github/blob/master/serve.bat)，文档撰写约定请参考 [doc 仓库首页](https://github.com/OpenHUTB/doc)。

## 注意事项

* 提交 Pull Request 前请检查：
    * **一定**要用 **`mkdocs serve --livereload`** 检查网页是否呈现预期修改后再提交。
    * Pull Request 标题和提交信息应说明修改原因，而不仅是修改内容。
    * 检查 Pull Request 页面的 File changed 内容是否符合预期。
    * 检查页面是否有冲突提示；如有，请先合并仓库最新修改再新建 Pull Request。
    * 提交截图，不要提交拍摄屏幕的照片。
    * 无需说明个人信息，提交记录会显示贡献者。
    * 每次 Pull Request 都需要保证能够通过 main 脚本直接运行整个模块；在提交信息中提供运行效果图（动图可使用 [ScreenToGif](https://github.com/NickeManarin/ScreenToGif)），并在 README.md 中说明运行环境和步骤。
    * 使用大模型需要在最后声明，并对提交内容负全部责任。
    * Markdown 文档放到 [docs](https://github.com/OpenHUTB/ros2/tree/master/docs) 目录和相应模块文件夹内；源代码（.py、.cpp 等）放到 [src](https://github.com/OpenHUTB/ros2/tree/master/src) 目录和相应模块文件夹内。
    * 文件名使用英文，不含空格、中文或拼音；多个单词用下划线 `_` 分隔。
    * 每个模块的相关文件放在以模块命名的文件夹内，并支持 launch 启动；入口以 `main.` 开头，例如 `main.py`、`main.cpp`、`main.bat`、`main.sh`。
    * 每个模块都需要在导航栏和首页添加跳转链接。
    * 模块名应具体，避免与其他内容重复。
    * 仓库尽量保存文本文件；二进制文件需慎重。图片使用 .png 且小于 3 MB，动图使用 .gif 且小于 10 MB。运行所需示例数据可少量保存；大量数据请提供长期有效的下载链接和运行说明。

* 每次合并到主分支前，需要至少一名其他贡献者在另一台机器上测试并同意。
* GitHub 访问不稳定时，可以[使用 Stem++、fastgithub 加速，也可使用其他代理](https://gitee.com/OpenHUTB/sw/releases/tag/up)。
