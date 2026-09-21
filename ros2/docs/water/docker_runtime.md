## HoloOcean ROS2 运行时 Docker 容器

受限于虚幻引擎的最终用户许可协议（EULA），该容器未打包发布至 DockerHub。


## 使用方法

1. **虚幻引擎协议**  
   请确保您已签署虚幻引擎相关协议。

2. **克隆仓库** 并进入 `docker/runtime` 目录：

   ```bash
   git clone git@github.com:byu-holoocean/holoocean-ros.git
   cd holoocean-ros/docker/runtime
   ```

3. **构建容器：**

   ```bash
   ./build_container.sh
   ```

   - 该脚本将执行以下操作：
     - 检查 Docker 是否已安装
     - 提示确认最终用户许可协议 (EULA)
     - 引导您完成 X11 显示访问权限的设置
     - 将 HoloOcean 仓库克隆到临时文件夹（需要 SSH 访问权限）
     - 构建 Docker 镜像
     - （可选）启动容器

4. **显示访问权限（用于 GUI）：**
   - 如果出现提示，请允许 Docker 访问您的 X11 显示：
     ```bash
     xhost +local:docker
     ```
   - **注意：** 每次重启后都必须重新运行此命令。


## **build_container.sh**

#### **命令行选项**

- `-b `: 指定 HoloOcean 的 git 分支（默认：`develop`）


## **使用提示**

- **重新构建：** 如果修改了 Dockerfile 或依赖项，请重新运行脚本以重建镜像。如果计划频繁进行重建，建议使用开发（dev）镜像，它更适合应对频繁的变更。

## **安全须知**

> **警告：**  
> 该镜像包含专有软件，且受限于与 Epic Games 签署的最终用户许可协议（EULA），仅供内部使用。
> **请勿在组织外部共享构建好的 Docker 镜像。**

