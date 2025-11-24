# 环境配置指南 - 给Li（Windows虚拟机用户）

## 📋 环境要求

- ✅ Windows系统 + Ubuntu 22.04 虚拟机（VMware/VirtualBox）
- ✅ 虚拟机至少分配 **4GB RAM**
- ✅ 磁盘空间至少 **20GB**
- ✅ 虚拟机可以联网

### 验证Ubuntu版本

在虚拟机终端执行：
```bash
lsb_release -a
# 必须显示: Ubuntu 22.04.x LTS
```

---

## 🔧 完整配置流程（3大步骤）

---

## 第一步：安装ROS2 Humble（一次性配置）

**耗时：约20分钟**

### 1.1 在Ubuntu虚拟机终端执行以下命令
```bash
# 更新系统
sudo apt update && sudo apt upgrade -y

# 安装必要工具
sudo apt install software-properties-common curl git -y

# 添加ROS2官方源
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装ROS2 Humble完整版
sudo apt update
sudo apt install ros-humble-desktop -y

# 安装编译工具
sudo apt install python3-colcon-common-extensions -y

# 自动配置环境变量
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 1.2 验证安装成功
```bash
ros2 --version
# 应该显示类似：ros2 cli version: ros2-0.18.5
```

**✅ 看到版本号，说明ROS2安装成功！**

---

## 第二步：克隆项目并编译

**耗时：约2-3分钟**

### 2.1 克隆项目
```bash
# 进入用户主目录
cd ~

# 克隆GitHub项目
git clone https://github.com/Mxx1233/Projeck-Autofarhen.git

# 进入项目目录
cd Projeck-Autofarhen

# 查看项目结构
ls -la
# 应该看到: src/ README.md .gitignore
```

### 2.2 编译项目
```bash
# 确保ROS2环境已加载
source /opt/ros/humble/setup.bash

# 编译整个工作空间
colcon build

# 应该看到类似输出：
# Starting >>> rusty_racer_interfaces
# Finished <<< rusty_racer_interfaces [Xs]
# Starting >>> control_pkg
# Finished <<< control_pkg [Xs]
# Summary: 2 packages finished

# 配置工作空间环境
source install/setup.bash
```

### 2.3 验证编译成功
```bash
# 查看安装目录
ls install/
# 应该看到: control_pkg/ rusty_racer_interfaces/

# 查看可执行文件
ros2 pkg executables control_pkg
# 应该显示: control_pkg controller_node_exe
```

**✅ 看到这些输出，说明编译成功！**

---

## 第三步：测试运行

**耗时：约1分钟**

### 3.1 运行控制节点
```bash
# 在项目目录下
cd ~/Projeck-Autofarhen
source install/setup.bash

# 运行控制节点
ros2 run control_pkg controller_node_exe
```

### 3.2 期望输出

你应该看到：
```
[INFO] [1763734325.199186775] [controller_node]: Controller node initialized
[INFO] [1763734325.200082946] [controller_node]: Control frequency: 50.0 Hz
[INFO] [1763734325.200080485] [controller_node]: Target velocity: 0.50 m/s
[INFO] [1763734325.200172148] [controller_node]: PID gains - Kp: 1.00, Ki: 0.00, Kd: 0.10
```

**✅ 看到这些日志，说明节点运行成功！**

按 `Ctrl+C` 停止节点。

---

## 💻 添加你自己的代码包

### 4.1 将你的包添加到项目
```bash
# 进入src目录
cd ~/Projeck-Autofarhen/src

# 方式A: 如果你的代码在GitHub
git clone 你的代码仓库地址

# 方式B: 如果是本地文件夹
# 把你的包文件夹复制到这里
# 例如: cp -r /path/to/你的包 .

# 查看当前包列表
ls
# 应该看到: control_pkg/ rusty_racer_interfaces/ 你的包/
```

### 4.2 重新编译
```bash
# 返回工作空间根目录
cd ~/Projeck-Autofarhen

# 重新编译（包含新添加的包）
colcon build

# 重新加载环境
source install/setup.bash
```

### 4.3 测试你的节点
```bash
# 运行你的节点
ros2 run 你的包名 你的节点名

# 例如：
# ros2 run low_level_pkg uc_bridge_adapter_node
```

---

## 🧪 集成测试

### 5.1 同时运行多个节点

**终端1：运行控制节点**
```bash
cd ~/Projeck-Autofarhen
source install/setup.bash
ros2 run control_pkg controller_node_exe
```

**终端2（Ctrl+Alt+T 打开新终端）：运行你的节点**
```bash
cd ~/Projeck-Autofarhen
source install/setup.bash
ros2 run 你的包名 你的节点名
```

**终端3：查看话题通信**
```bash
cd ~/Projeck-Autofarhen
source install/setup.bash

# 查看所有话题
ros2 topic list

# 查看节点信息
ros2 node list
ros2 node info /controller_node

# 监听控制命令输出
ros2 topic echo /motor_command
```

---

## 📌 每次启动虚拟机后的操作

每次重启虚拟机后，只需：
```bash
# 1. 进入项目目录
cd ~/Projeck-Autofarhen

# 2. 加载环境（ROS2环境已自动加载）
source install/setup.bash

# 3. 运行节点
ros2 run control_pkg controller_node_exe
```

**提示：** 可以把 `source ~/Projeck-Autofarhen/install/setup.bash` 加入 `~/.bashrc` 自动加载：
```bash
echo "source ~/Projeck-Autofarhen/install/setup.bash" >> ~/.bashrc
```

---

## 🔧 常见问题和解决方案

### 问题1：虚拟机运行慢

**解决方案：**
- 增加虚拟机RAM到至少4GB（建议8GB）
- 增加CPU核心数到2-4个
- 在虚拟机设置中启用3D加速
- 关闭Windows其他占用资源的程序

**VirtualBox设置：**
- 设置 → 系统 → 主板 → 内存：4096MB+
- 设置 → 系统 → 处理器 → CPU：2-4核
- 设置 → 显示 → 3D加速：启用

**VMware设置：**
- 虚拟机 → 设置 → 硬件 → 内存：4GB+
- 虚拟机 → 设置 → 硬件 → 处理器：2-4核

---

### 问题2：找不到某些ROS2包

**错误示例：**
```
CMake Error: find_package(xxx) not found
```

**解决方案：**
```bash
# 安装缺失的包
sudo apt install ros-humble-包名

# 常用包：
sudo apt install ros-humble-nav-msgs
sudo apt install ros-humble-geometry-msgs
sudo apt install ros-humble-std-msgs
sudo apt install ros-humble-sensor-msgs
```

---

### 问题3：编译rusty_racer_interfaces失败

**解决方案：**
```bash
cd ~/Projeck-Autofarhen

# 先单独编译接口包
colcon build --packages-select rusty_racer_interfaces
source install/setup.bash

# 再编译其他包
colcon build
source install/setup.bash
```

---

### 问题4：需要连接USB硬件（串口通信）

**VirtualBox设置：**
1. 虚拟机 → 设置 → USB
2. 启用USB 3.0控制器
3. 点击右侧 "+" 添加USB设备筛选器
4. 选择你的USB串口设备

**VMware设置：**
1. 虚拟机 → 可移动设备
2. 找到你的USB串口设备
3. 点击 "连接（断开与主机的连接）"

**在Ubuntu中添加串口权限：**
```bash
# 将用户添加到dialout组
sudo usermod -a -G dialout $USER

# 需要重新登录虚拟机才能生效
# 或者执行：
newgrp dialout

# 验证权限
groups
# 应该看到 dialout 在列表中
```

---

### 问题5：Git克隆速度慢

**解决方案A：使用国内镜像**
```bash
# 使用Gitee镜像（如果有）
git clone https://gitee.com/镜像地址

# 或使用代理
git config --global http.proxy http://代理地址:端口
```

**解决方案B：下载ZIP包**
1. 在GitHub页面点击绿色 "Code" 按钮
2. 选择 "Download ZIP"
3. 解压到 `~/Projeck-Autofarhen`

---

### 问题6：colcon build 非常慢

**解决方案：**
```bash
# 使用并行编译（根据CPU核心数调整）
colcon build --parallel-workers 2

# 或只编译修改过的包
colcon build --packages-select 包名
```

---

### 问题7：source命令找不到文件

**错误示例：**
```
bash: install/setup.bash: No such file or directory
```

**解决方案：**
```bash
# 确保在正确的目录
cd ~/Projeck-Autofarhen
pwd
# 应该显示: /home/用户名/Projeck-Autofarhen

# 确保已经编译过
ls install/
# 应该能看到编译产物

# 如果没有，重新编译
colcon build
```

---

## 📚 有用的ROS2命令

### 查看系统信息
```bash
# 查看所有节点
ros2 node list

# 查看节点详细信息
ros2 node info /节点名

# 查看所有话题
ros2 topic list

# 查看话题信息
ros2 topic info /话题名

# 实时查看话题数据
ros2 topic echo /话题名

# 查看话题发布频率
ros2 topic hz /话题名
```

### 调试命令
```bash
# 查看消息定义
ros2 interface show 消息类型

# 手动发布话题（测试用）
ros2 topic pub /话题名 消息类型 "数据"

# 例如：
ros2 topic pub /lane_deviation rusty_racer_interfaces/msg/LaneDeviation \
"lateral_error: 0.1
heading_error: 0.05
curvature: 0.0" -r 10
```

---

## 🎯 配置完成检查清单

完成以下检查确保环境配置正确：

- [ ] Ubuntu版本是22.04
- [ ] ROS2 Humble安装成功（`ros2 --version` 有输出）
- [ ] 项目克隆成功（`~/Projeck-Autofarhen` 目录存在）
- [ ] 编译成功（`install/` 目录存在且包含两个包）
- [ ] control_node运行成功（看到初始化日志）
- [ ] 可以查看话题列表（`ros2 topic list` 有输出）
- [ ] 环境变量自动加载（重启终端后 `ros2` 命令可用）

**全部打勾 = 环境配置完成！** ✅

---

## 📞 联系方式

有问题联系zx，或在GitHub仓库提Issue：
https://github.com/Mxx1233/Projeck-Autofarhen/issues

---

## 📝 附录：项目结构说明
```
Projeck-Autofarhen/
├── README.md                        # 项目说明文档
├── SETUP_FOR_LI.md                 # 本配置指南
├── .gitignore                      # Git忽略规则
├── src/                            # 源代码目录
│   ├── control_pkg/                # 控制算法包（zx负责）
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── include/
│   │   └── src/
│   │       └── controller_node.cpp # 控制节点源码
│   └── rusty_racer_interfaces/     # 自定义消息接口
│       ├── CMakeLists.txt
│       ├── package.xml
│       └── msg/
│           ├── LaneDeviation.msg   # 车道偏差消息
│           └── MotorCommand.msg    # 电机命令消息
├── build/                          # 编译中间文件（不上传Git）
├── install/                        # 编译产物（不上传Git）
└── log/                            # 编译日志（不上传Git）
```

---

**祝配置顺利！加油！** 🚀
