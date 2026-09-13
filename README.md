# ROS/ROS2 示例

该仓库包含环境搭建、教程、人和载具的ROS/ROS2示例。

## 环境配置

运行 [`serve.bat`](https://github.com/OpenHUTB/ros2/blob/master/serve.bat)（即运行`mkdocs serve`，所做的修改同样需要运行该脚本验证[网页中](http://127.0.0.1:8000/)所修改内容的正确性），会打开浏览器显示主页，详情请参考[serve.bat](https://github.com/OpenHUTB/.github/blob/master/serve.bat)，文档撰写的约定请参考 [doc](https://github.com/OpenHUTB/doc) 仓库首页。


## 注意事项

* 提交 Pull Request 前请检查：
    * Pull Request 的标题和提交信息不能随意，需要说明为什么修改，而不仅仅是修改了什么
    * Pull Request 页面的 File changed 下的内容是否为想要的修改
    * 检查 Pull Request 页面是否有红色的提示（有冲突），如果有请先合并仓库最新修改再新建 Pull Requst
    * 用截图，而不是拍照
    * 不需要说明个人信息，因为git log的提交记录中有
    * 每次 Pull Request 都需要保证能够通过main脚本直接运行整个模块，在提交信息中提供运行效果图（动图可以用[ScreenToGif](https://github.com/NickeManarin/ScreenToGif)），README.md文档中提供运行环境和运行步骤的说明
    * 使用大模型需要在最后声明，并对提交内容负全部责
    * 约定 Markdown 文档放到 [docs](https://github.com/OpenHUTB/ros2/tree/master/docs) 目录和相应的模块文件夹内，源代码（.py、.cpp等）放到 [src](https://github.com/OpenHUTB/ros2/tree/master/src) 目录和相应的模块文件夹内
    * 每个模块的相关文件放在以模块命名的文件夹内，必须支持 launch 启动，入口为main.开头，比如：main.py、main.cpp、main.bat、main.sh等
    * 每个模块需要在导航栏和首页添加跳转的链接
    * 模块名不能宽泛，需要具体，不然就和其他人的容易重复
    * 仓库尽量保存文本文件，二进制文件需要慎重，图片约定使用.png且小于 3 MB，动图使用.gif且小于 10 MB，如运行需要示例数据，可以保存少量数据，大量数据可以通过提供永久网盘链接并说明下载链接和运行说明


* 每次合并到主分支之前需要至少一名其他人 测试并同意（另一台机器修改部分的运行结果）
* Github访问不稳定可以[使用Stem++、fastgithub进行加速（也可使用其他代理）](https://gitee.com/OpenHUTB/sw/releases/tag/up) 

