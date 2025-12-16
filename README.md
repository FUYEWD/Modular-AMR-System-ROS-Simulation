# 🤖 Modular AMR System – ROS Simulation

<div align="center">

**模組化自主移動機器人（AMR）系統模擬專案**

展示多節點協作、事件驅動控制與任務管理  
適用於自動倉儲、物流配送或外骨骼輔具系統的概念驗證

[![ROS](https://img.shields.io/badge/ROS-Noetic-blue.svg)](http://wiki.ros.org/noetic)
[![Python](https://img.shields.io/badge/Python-3.8+-green.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

</div>

---

## 📋 目錄

- [系統架構](#-系統架構)
- [專案特色](#-專案特色)
- [技術亮點](#-技術亮點)
- [功能展示](#-功能展示)
- [安裝與啟動](#-安裝與啟動)
- [系統架構詳解](#-系統架構詳解)
- [未來擴展](#-未來擴展方向)
- [授權](#-授權)

---

## 🏗 系統架構

本專案採用**模組化 ROS 節點架構**，實現多機器人協同作業與任務管理：

```
┌─────────────────────────────────────────────────┐
│              AMR Control System                 │
└─────────────────────────────────────────────────┘

         ┌──────────────────┐
         │  Sensor Node     │ 
         │  /scan_data      │
         └────────┬─────────┘
                  │
                  ▼
         ┌──────────────────┐
         │ Navigation Node  │
         │  /cmd_vel        │
         └────────┬─────────┘
                  │
                  ▼
         ┌──────────────────┐
         │ Task Manager     │
         │  /task_status    │
         └────────┬─────────┘
                  │
                  ▼
         ┌──────────────────┐
         │ State Monitor    │
         │  /alert          │
         └────────┬─────────┘
                  │
                  ▼
         ┌──────────────────┐
         │ Visualization    │
         │                  │
         └──────────────────┘
```

---

## 🌟 專案特色

### 🧩 模組化設計
- 每個 Node 功能單一明確
- 便於維護與功能擴展
- 支援獨立測試與除錯

### ⚡ 事件驅動架構
- 任務與控制指令即時回應
- 非阻塞式異步通訊
- 高效能資料傳輸

### 🔍 異常監控
- 即時設備狀態追蹤
- 智慧異常警示系統
- 電量/溫度/故障管理

### 🔌 硬體相容
- 支援 AMR、AGV 自走車
- 適用自動倉儲系統
- 可延伸至外骨骼輔具

---

## 💡 技術亮點

**ROS Framework**
- Publisher/Subscriber 架構
- Topic 通訊機制
- Node 模組化設計

**Python Development**
- 快速開發整合
- 資料處理與控制腳本
- 自動化測試腳本

**Robot System**
- 多模組訊息整合
- 事件驅動架構
- 即時控制系統

**Real Hardware Ready**
- 可連接實際 AMR 硬體
- AGV 自走車整合
- 智慧閘門系統對接

---

## ⚙ 功能展示

### 核心功能

✅ **多台 AGV/AMR 協同運作** - 支援多機器人任務分配與協調

✅ **任務分配與 Pick & Place** - 智慧搬運與定點放置控制

✅ **路徑規劃與避障** - 動態障礙物偵測與路徑重規劃

✅ **設備狀態管理** - 電量/溫度/載重/故障即時監控

✅ **RViz/GUI 視覺化** - 即時 3D 環境與機器人狀態顯示

---

## 🚀 安裝與啟動

### 前置需求

- Ubuntu 20.04 LTS
- ROS Noetic
- Python 3.8+

### 安裝步驟

**1. 克隆專案**

```bash
git clone https://github.com/yourusername/modular-amr-ros.git
cd modular-amr-ros
```

**2. 建立 ROS Workspace**

```bash
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workspace
cd ..
catkin_make
source devel/setup.bash
```

**3. 安裝相依套件**

```bash
sudo apt-get update
sudo apt-get install ros-noetic-navigation
sudo apt-get install ros-noetic-rviz
pip3 install -r requirements.txt
```

**4. 啟動模擬系統**

```bash
# 啟動 ROS Master 與所有節點
roslaunch amr_simulation simulation.launch

# 另開終端啟動 RViz 視覺化
rosrun rviz rviz
```

### 快速測試

```bash
# 發送測試任務
rostopic pub /task_command std_msgs/String "data: 'PICKUP_A1'"

# 查看機器人狀態
rostopic echo /task_status

# 監控感測器資料
rostopic echo /scan_data
```

---

## 📊 系統架構詳解

### 資料流向說明

**Sensor Node** → 持續發布環境感測資料至 `/scan_data`

**Navigation Node** → 訂閱感測資料，計算最佳路徑並發布速度指令至 `/cmd_vel`

**Task Manager** → 接收速度指令，執行任務邏輯，發布任務狀態至 `/task_status`

**State Monitor** → 監控任務狀態，偵測異常時發布警報至 `/alert`

**Visualization Node** → 整合所有資訊進行圖形化顯示

### Topic 列表

| Topic 名稱 | 訊息類型 | 說明 |
|-----------|---------|------|
| `/scan_data` | `sensor_msgs/LaserScan` | 雷射掃描資料 |
| `/cmd_vel` | `geometry_msgs/Twist` | 速度控制指令 |
| `/task_status` | `std_msgs/String` | 任務執行狀態 |
| `/alert` | `std_msgs/String` | 系統警報訊息 |

---

## 🔮 未來擴展方向

- [ ] 替換模擬 Sensor 與 Actuator 為真實硬體設備
- [ ] 整合 AI 演算法實現自動任務排程最佳化
- [ ] 與 IoT 工業自動化系統深度整合
- [ ] 開發 AMR 與外骨骼輔具協作任務模組
- [ ] 加入 SLAM 即時定位與地圖建構功能
- [ ] 支援多樓層電梯導航與跨區域調度

---

## 🤝 貢獻指南

歡迎提交 Issue 與 Pull Request！

1. Fork 本專案
2. 創建特性分支 (`git checkout -b feature/AmazingFeature`)
3. 提交變更 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 開啟 Pull Request

---

## 📄 授權

本專案採用 MIT License 授權 - 詳見 [LICENSE](LICENSE) 文件


Made with ❤️ for Robotics & Automation

</div>
