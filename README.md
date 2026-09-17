# 🤖 RAICOM 魔力元宝服务机器人

> 基于 ROS Noetic 的智能服务机器人系统，实现迎宾导览、智能巡检、视觉识别、语音交互、自主导航与充电对接。

<p align="left">
  <img src="https://img.shields.io/badge/ROS-Noetic-blue?logo=ros" />
  <img src="https://img.shields.io/badge/Python-3.8-blue?logo=python" />
  <img src="https://img.shields.io/badge/OpenCV-Vision-green?logo=opencv" />
  <img src="https://img.shields.io/badge/YOLO-Detection-purple" />
  <img src="https://img.shields.io/badge/Platform-Bobac-orange" />
</p>

## 🎬 项目演示

[![Bilibili](https://img.shields.io/badge/Bilibili-点击观看完整演示-00A1D6?logo=bilibili&logoColor=white)](https://www.bilibili.com/video/BV15JMB6pEbS/)

▶ **演示视频：**  
https://www.bilibili.com/video/BV15JMB6pEbS/

---

## 📖 项目简介

本项目为 **RAICOM 魔力元宝服务组省赛作品**。

项目基于 ROS Noetic、Gazebo 与 RViz 搭建服务机器人任务系统，整合人脸检测与识别、语音交互、自主定位导航、YOLO 目标检测、异常报警及自主充电等模块，实现从人员唤醒、任务理解、导航执行到视觉感知和任务收尾的完整机器人工作流程。

系统主要完成两类任务：

**迎宾导览**

`人员检测 → 语音交互 → 场馆识别 → 自主导航 → 精确停车 → 场馆介绍 → 返回出发区`

**智能巡检**

`管理员识别 → 接收巡检任务 → 五馆巡检 → 视觉检测 → 异常报警 → 自主前往充电区域 → 充电对接`

---

## 👨‍💻 主要工作

> 本部分建议只保留自己实际负责和完成的内容。

- 设计服务组任务编排逻辑，统一管理迎宾导览与智能巡检任务状态
- 联调人脸识别、语音交互、自主导航、视觉检测和任务控制模块
- 配置五个展馆、出发区和充电区域导航目标点
- 基于 `move_base` 实现多目标点导航、失败恢复与精确停车
- 构建火源与灭火器目标检测流程，并封装 ROS 视觉检测服务
- 实现巡检异常判断、报警及语音播报
- 完成自主充电定位、视觉辅助对准与停靠逻辑
- 编写任务一键启动、导航检查及赛前检查脚本

---

## ✨ 核心功能

| 模块 | 功能 |
| --- | --- |
| 人机交互 | 人脸检测、人脸识别、语音识别、TTS 播报 |
| 自主导航 | AMCL 定位、move_base 导航、Navfn 全局规划、DWA 局部规划 |
| 精确停车 | 基于 TF 实时位姿进行二次闭环修正 |
| 视觉检测 | YOLO 火源与灭火器识别 |
| 智能巡检 | 多场馆巡检、异常检测、语音报警 |
| 自主充电 | 充电区域导航、视觉对准、闭环停靠 |
| 安全控制 | 急停、导航取消、零速度控制、异常恢复 |

---

## 🧠 技术实现

### 自主导航

采用 ROS Navigation Stack：

```text
AMCL
  ↓
move_base
  ├── Navfn       全局路径规划
  └── DWA         局部路径规划 / 动态避障



机器人到达目标区域后，通过 TF 获取实时位姿并执行二次闭环修正，以提高停车精度。
导航失败时支持：
清理代价地图
    ↓
脱困运动
    ↓
重新规划
    ↓
短距离底盘直控兜底
视觉识别
视觉模块基于 Ultralytics YOLO + OpenCV 实现：
相机图像
   ↓
cv_bridge
   ↓
YOLO 推理
   ↓
fire / fire_extinguisher
   ↓
ROS Service
   ↓
巡检任务节点
机器人在每个巡检点进行多帧检测，并根据识别结果判断：
发现火源
→ 报警
→ 播报对应场馆

未检测到灭火器
→ 判定灭火器缺失
→ 语音报警
语音与人机交互
系统集成：
SenseVoice
sherpa-onnx
Silero VAD
Edge TTS
完成：
人员检测
→ 机器人唤醒
→ 语音识别
→ 指令解析
→ ROS 任务触发
→ 语音反馈
自主充电
充电任务流程：
导航至充电接近点
        ↓
视觉识别充电桩
        ↓
横向居中
        ↓
调整机器人姿态
        ↓
闭环位置修正
        ↓
完成充电停靠
🗺️ 任务流程
任务一 · 迎宾导览
检测访客
   ↓
机器人唤醒
   ↓
识别语音指令
   ↓
解析目标场馆
   ↓
自主导航
   ↓
精确停车
   ↓
播报场馆介绍
   ↓
返回出发区
支持：
北京馆
广州馆
吉林馆
上海馆
深圳馆
任务二 · 智能巡检
巡检路线：
北京馆
  ↓
广州馆
  ↓
吉林馆
  ↓
上海馆
  ↓
深圳馆
  ↓
自主充电
每个场馆完成：
导航
→ 精确停车
→ 视觉识别
→ 异常判断
→ 报警 / 播报
🛠 技术栈
机器人系统
ROS Noetic · Gazebo · RViz · TF · actionlib
导航
AMCL · move_base · Navfn · DWA
视觉
Python · OpenCV · cv_bridge · Ultralytics YOLO
语音
SenseVoice · sherpa-onnx · Silero VAD · Edge TTS
工程
Python · Bash · YAML · catkin · ROS launch
📁 项目结构
raicom-yuanbao-service-robot/
│
├── bobac3_ws/
│   └── src/
│       ├── service_group_mission/
│       │   └── 迎宾、巡检、精确停车与充电任务逻辑
│       │
│       ├── raicom_supervisor/
│       │   └── 一键启动、导航检查与急停管理
│       │
│       ├── raicom_vision/
│       │   └── YOLO 推理服务与视觉检测
│       │
│       └── bobac3_navigation/
│           └── 地图、AMCL、move_base、DWA/Navfn 配置
│
├── ros_workspace/
│   └── src/
│       └── local_voice_bridge/
│           └── ASR、关键词识别、意图解析与 TTS
│
├── .gitignore
└── README.md
🚀 环境
Ubuntu 20.04
ROS Noetic
Python 3.8
编译：
source /opt/ros/noetic/setup.bash

cd ~/bobac3_ws
catkin_make
source devel/setup.bash

cd ~/ros_workspace
catkin_make
source devel/setup.bash --extend
🏆 项目成果
实现服务组省赛核心任务完整闭环：
人员感知
   ↓
自然交互
   ↓
任务决策
   ↓
自主导航
   ↓
视觉感知
   ↓
异常处理
   ↓
任务收尾
完成迎宾导览、五馆自主巡检、火源与灭火器异常检测、语音报警及自主充电等功能。
