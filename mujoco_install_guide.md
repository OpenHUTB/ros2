# MuJoCo安装教程

## 一、实验目的与环境说明
### 1.1 实验目的
1. 完成MuJoCo物理仿真引擎在Ubuntu20.04虚拟机的部署安装
2. 测试MuJoCo原生仿真窗口，验证引擎可以正常运行
3. 实现MuJoCo与ROS Noetic联动，编写Python节点完成仿真测试

### 1.2 实验环境
|项目|配置|
| ---- | ---- |
|操作系统|Ubuntu 20.04（VMware虚拟机）|
|ROS版本|ROS Noetic|
|MuJoCo版本|MuJoCo211|
|编程语言|Python3|
|功能包|mujoco_demo|

### 1.3 功能包目录结构
 
```text
mujoco_demo/
├── package.xml        # 包清单，声明依赖mujoco、rospy、geometry_msgs
├── CMakeLists.txt     # 编译配置，把python脚本安装到bin目录
└── scripts/
└── mujoco_test.py # 主程序：调用MuJoCo仿真的测试节点
``` 
plaintext
  

## 二、MuJoCo介绍
MuJoCo是多关节动力学物理仿真引擎，擅长机器人、机械臂、人形机器人动力学仿真，支持接触、碰撞、关节约束，常和ROS结合做机器人控制仿真。

## 三、安装步骤
### 3.1 安装依赖库
打开Ubuntu终端，依次执行：
```bash
sudo apt update
sudo apt install libgl1-mesa-glx libglfw3 libosmesa6-dev patchelf
```
 ![alt text](<Screenshot from 2026-10-11 01-19-41.png>)
### 3.2 下载MuJoCo压缩包
1. 进入MuJoCo官网下载：`https://github.com/deepmind/mujoco/releases`
2. 下载  `mujoco211-linux-x86_64.tar.gz` 
3. 在用户主目录创建文件夹`.mujoco`
 
```bash
  
mkdir ~/.mujoco
```
 ![alt text](<Screenshot from 2026-10-11 01-27-25.png>)
4. 将压缩包解压到  `.mujoco`  目录
 
```bash
  
tar -zxvf mujoco211-linux-x86_64.tar.gz -C ~/.mujoco/
```

### 3.3 配置环境变量
 
编辑`bashrc`文件：
 
```bash
  
gedit ~/.bashrc
```
 
在文件末尾粘贴下面两行：
 
```bash
  
export MUJOCO_KEY_PATH=~/.mujoco/mujoco211/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/.mujoco/mujoco211/bin
```
 
保存退出，执行命令让环境变量生效：
 
```bash
  
source ~/.bashrc
```
 
### 3.4 测试MuJoCo原生仿真
 
进入MuJoCo的bin目录，运行测试程序：
 
```bash
  
cd ~/.mujoco/mujoco211/bin
./simulate
```
 
✅ 成功：弹出仿真窗口，加载人形机器人模型，支持鼠标拖拽交互。
❌ 报错：检查 LD_LIBRARY_PATH 路径是否正确，确认解压目录无误。
 
### 3.5 Python绑定安装与ROS测试节点
 
安装mujoco Python库：
 
```bash
  
pip install mujoco
```
 
新建ROS功能包脚本  `mujoco_test.py` 
 
```python
  
import mujoco
import mujoco.viewer
import rospy

if __name__ == "__main__":
    rospy.init_node("mujoco_demo_node")
    # 加载人形模型
    model = mujoco.MjModel.from_xml_path('/home/$USER/.mujoco/mujoco211/model/humanoid.xml')
    data = mujoco.MjData(model)
    rospy.loginfo("MuJoCo仿真启动成功")
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running() and not rospy.is_shutdown():
            mujoco.mj_step(model, data)
            viewer.sync()
```
 
给脚本添加执行权限
 
```bash
  
chmod +x mujoco_test.py
 
```
回到catkin工作空间编译
 
```bash
  
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
 
运行ROS节点
 
```bash
  
rosrun mujoco_demo mujoco_test.py
```
 
运行成功会弹出MuJoCo人形仿真窗口。
 
## 四、MuJoCo + ROS Noetic联动（可选）
 
如需在ROS中使用mujoco_ros插件，在工作空间src下克隆源码，编译即可。
 
## 五、常见报错排查
 
1. error while loading shared libraries 
原因：环境变量未生效，重新执行  source ~/.bashrc ，重启终端。
2. 虚拟机无法弹出仿真窗口
原因：虚拟机显卡加速问题，VMware开启3D图形加速；或者使用offscreen离线渲染。
3. pip安装mujoco失败
 
```bash
  
pip install --upgrade pip
```
 
## 六、运行效果说明
 
1. 终端执行 `./simulate` ，弹出MuJoCo人形仿真界面，可拖拽模型。
2. ROS节点运行后，在ROS中调用MuJoCo仿真，可拓展编写控制代码。
3. 本地预览网页：仓库根目录执行  `mkdocs serve --livereload` ，浏览器打开 `http://127.0.0.1:8000`
 
## 七、测试说明
 
- 操作系统：Windows11宿主机 + Ubuntu20.04 VMware虚拟机
- 验证项：MuJoCo引擎安装、Python接口调用、ROS节点正常启动
- 文档导航：文档放置在docs/humanic文件夹，可在mkdocs网页左侧导航访问
 
## 八、大模型声明
 
模型：DeepSeek
用途：辅助编写MuJoCo安装教程、Python节点代码、排查安装报错。
