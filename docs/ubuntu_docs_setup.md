# Ubuntu 文档环境初始化

本页给出在 Ubuntu 20.04 或 22.04 上安装 MkDocs、预览本站并提供提交前截图的步骤。

## 为什么需要独立环境

仓库使用 Markdown 扩展、BibTeX、Mermaid 和视频插件。直接使用系统 Python 安装依赖
可能污染系统环境，也容易因 Ubuntu 自带的旧版包产生冲突。本流程在仓库内创建
`.venv`，不会修改其他项目。

## 一键安装

在仓库根目录执行：

```bash
chmod +x setup_ubuntu.sh serve.sh
./setup_ubuntu.sh
```

脚本会：

1. 检查 Python 3 和 `python3-venv`。
2. 在仓库根目录创建 `.venv`。
3. 安装 `requirements-docs.txt` 中的 MkDocs 与文档插件。

如需把虚拟环境放到其他位置，可指定 `DOCS_VENV`：

```bash
DOCS_VENV="$HOME/.virtualenvs/openhutb-docs" ./setup_ubuntu.sh
DOCS_VENV="$HOME/.virtualenvs/openhutb-docs" ./serve.sh
```

## 预览与截图

```bash
./serve.sh
```

默认地址为 <http://127.0.0.1:8000/>。修改 `docs/` 或 `mkdocs.yml` 后，浏览器会自动
刷新。提交 PR 前应逐页检查导航、图片、公式和链接，并使用截屏保存实际网页效果。

## 常见问题

- `python3-venv` 缺失：脚本会使用 `apt-get` 安装该软件包。
- 端口被占用：可执行 `./serve.sh -a 127.0.0.1:8001`。
- 依赖安装失败：先检查代理和网络，再重新执行 `./setup_ubuntu.sh`。脚本重复运行不会删除已有源码。