# CYIVS Robot Workspace (嘉義高工機器人專案)

歡迎來到 **CYIVS Robot** 工作空間！本專案專為機器人專題製作與教學設計，基於 ROS 2 架構，整合了全向移動底盤控制、micro-ROS 通訊、YDLIDAR 光達掃描以及 Nav2 自主導航系統，提供一套完整的機器人開發與學習環境。

## 📂 套件架構 (Packages)

本工作空間的 `src` 目錄包含以下核心套件，分別負責機器人的各項子系統：

* **`fishbot_bringup`**: 系統啟動套件。提供整合的 Launch 檔，用於一次性喚醒底層驅動、感測器及核心控制節點。
* **`fishbot_description`**: 機器人描述套件。存放 URDF 模型檔、3D 網格與硬體參數，支援 RViz2 視覺化與 Gazebo 物理模擬。
* **`fishbot_navigation2`**: Nav2 導航堆疊設定。包含 SLAM 建圖、路徑規劃、避障與 AMCL 定位的相關 YAML 參數配置。
* **`micro_ros_setup` & `uros`**: 微控制器通訊中介層。用於建立 ESP32 等微控制器與 ROS 2 主機之間的 micro-ROS 橋接 (Agent) 與通訊環境。
* **`relative_move_controller`**: 相對運動控制器。負責處理底盤特定的相對位移與運動學控制邏輯。
* **`ros_serial2wifi`**: 通訊轉換套件。將實體 Serial 訊號轉換為 Wi-Fi 無線網路封包，增強機器人通訊的靈活度。
* **`teleop_manager`**: 遠端遙控管理員。負責接收搖桿或鍵盤的操控指令，並轉換為標準速度控制訊號 (`/cmd_vel`)。
* **`ydlidar_ros2`**: 雷射測距儀驅動。YDLIDAR 的 ROS 2 原生驅動節點，負責發布環境的 2D 點雲資料 (`/scan`)。

## 🛠️ 環境需求

請確保您的開發環境符合以下配置：
* **作業系統**: Ubuntu 22.04
* **ROS 2 版本**: Humble Hawksbill
* **硬體設備**: 
  * 系統運算主機 (如 Raspberry Pi 或 x86 電腦)
  * ESP32 開發板 (執行底層控制與 micro-ROS)
  * YDLIDAR 光達模組

## 🚀 快速上手 (Quick Start)

請按照以下步驟進行工作空間的建置與編譯：

**1. 建立工作空間並獲取程式碼**
```bash
mkdir -p ~/cyivs_robot_ws
cd ~/cyivs_robot_ws
sudo apt install git
git clone https://github.com/a605042000/cyivs_robot.git src
```

**2. 安裝相依套件 (Dependencies)**
使用 `rosdep` 自動安裝此專案所需的所有依賴項：
```bash
cd ~/cyivs_robot_ws
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

**3. 編譯工作空間**
建議使用 `colcon` 進行編譯，並建立符號連結以利後續開發：
```bash
cd ~/cyivs_robot_ws
colcon build --symlink-install
```

**4. 載入環境變數**
編譯完成後，請將工作空間載入至當前終端機：
```bash
source install/setup.bash
```
*(建議將此指令加入 `~/.bashrc` 中以便每次開啟終端機時自動載入)*

## 📝 教學與開發備註
在進行整合測試時，請優先確認微控制器 (ESP32) 與主機間的 micro-ROS 連線狀態，並利用 RViz2 檢查 `/scan` 與 `TF` 座標系統的對應關係。祝專題實作順利！
