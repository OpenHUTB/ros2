# Docker 开发环境

关于在 Ubuntu 22.04 的开发容器环境中搭建 ROS 2 HoloOcean 工作空间的说明。

## 使用方法

1. **虚幻引擎协议**  
   请确保您已签署虚幻引擎相关协议。

2. **克隆源代码**  
   从 GitHub 克隆 HoloOcean 源代码：
   ```
   git clone git@github.com:byu-holoocean/HoloOcean.git
   ```

3. **克隆 ROS 2 软件包**  
   将 HoloOcean 的 ROS 2 软件包安装到您的 ROS 2 工作空间中：
   ```
   git clone git@github.com:byu-holoocean/holoocean-ros.git
   ```

4. **调整文件路径**  
    在启动容器之前，请更新 `docker-compose` 文件中的文件路径，使其指向克隆的 HoloOcean 仓库（具体为 `client` 文件夹）以及 holoocean-ros 软件包。 

5. **运行设置命令**  
   输入以下命令以配置容器并进入 HoloOcean 环境：

   ```bash
   xhost +                             # 授予容器访问屏幕的权限
   
   # 可选: 
   # docker compose build  # 在本地构建容器，而不是从 Docker Hub 拉取
   
   docker compose up -d                # 基于 HoloOcean ROS 镜像创建容器并进行配置。
   
   docker exec -it holoocean bash      # 进入容器
   ```


## 注意事项

**卷挂载 (Volume Mounts)**  
    HoloOcean 仓库和 ROS 软件包均以卷（volume）的形式挂载到容器中。这意味着您可以在宿主机上编辑代码，所做的更改会立即反映在容器内。

**预构建镜像**  
    此处使用的 Docker 镜像可在网上直接获取，**无需**在本地构建。只需按照使用说明拉取并运行该镜像即可。当您执行 `docker compose up -d` 且设备上尚无该镜像时，Docker 会自动尝试从 Docker Hub 拉取。

