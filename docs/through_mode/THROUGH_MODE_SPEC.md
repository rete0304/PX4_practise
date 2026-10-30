# THROUGH Mode 實作規格書（PX4 v1.17 / gz sim / SITL）

> 本文件是給 Claude Code 的實作規格。所有設計決策已在規劃階段確認，實作時**不要自行更改設計方向**；遇到與本文件衝突的原始碼事實，先記錄在 `VERIFICATION.md`，再採用最接近本文件意圖的做法，並在報告中說明。

---

## 0. 給 Claude Code 的工作說明

- **範圍**：僅限模擬（SITL + gz sim），不會上真機。HITL 為後期選項，不在本次範圍。
- **基準版本**：PX4-Autopilot `v1.17.0`（若遇到無法解決的問題，可退回 `v1.16.x`，需在報告註明）。
- **不修改 commander**：新模式以「韌體內部模組透過 uORB 註冊外部模式」實作，參考 `src/modules/mc_nn_control` 與 `src/modules/commander/ModeManagement.cpp`。
- **工作順序**：先完成第 4 節「實作前驗證」並產出 `docs/through_mode/VERIFICATION.md`，再依第 14 節里程碑逐步實作，每個里程碑都要通過驗收條件才進入下一個。
- **程式風格**：遵守 PX4 既有 C++ 風格（`ModuleBase`、`ModuleParams`、`ScheduledWorkItem`、`uORB::Subscription` / `Publication`），Python 遵守 PEP 8。
- **語言**：程式碼與註解用英文；文件可用繁體中文。

---

## 1. 專案目標

建立一個多旋翼飛行模式 **THROUGH**，讓飛機利用機上相機追蹤一個有顏色的環，並以比例導引（PN）俯衝穿過環。

- 視覺節點（Python + OpenCV）從 gz 相機影像中找到目標，計算目標中心相對畫面中心的角度誤差（PanError / TiltError），並由人為觸發「鎖定」（Uncage）。
- **條件一**：`uncage == true` 且目標有效 → 才允許從 QGC 或 RC 切入 THROUGH。
- 進入後：對準 → 推力斜坡 → PN 導引俯衝 → 終端滑行穿環 → 自動退出到 Hold。
- 人為切換模式或解除鎖定時立即退出。

---

## 2. 環境與版本

| 項目 | 版本 / 選擇 |
|---|---|
| PX4 | v1.17.0 |
| 模擬器 | gz sim（依 PX4 v1.17 官方建議版本，Claude Code 查證） |
| ROS 2 | 依 PX4 v1.17 ROS 2 文件建議的發行版（Claude Code 查證並記錄） |
| 中介軟體 | uXRCE-DDS（`MicroXRCEAgent`） |
| 影像橋接 | `ros_gz_bridge`（gz Image → `sensor_msgs/Image`） |
| 視覺 | Python 3 + OpenCV + `cv_bridge` + `rclpy` + `px4_msgs` |
| 地面站 | QGroundControl 最新穩定版（需支援動態模式列表） |
| 機型基底 | `gz_x500_gimbal` |

---

## 3. 系統架構

```
 gz sim ──camera image──▶ ros_gz_bridge ──sensor_msgs/Image──▶ through_vision (Python)
   ▲                                                                │
   │ gimbal joint                                     px4_msgs/VisionTarget
   │                                                   /fmu/in/vision_target
   │                                                                ▼
 PX4 SITL ◀──────────────── uXRCE-DDS ◀──────────────────── MicroXRCEAgent
   │
   ├─ uORB vision_target ─────────────┐
   ├─ vehicle_attitude                │
   ├─ vehicle_local_position          ├──▶ through_control (新模組)
   ├─ gimbal_device_attitude_status   │      │
   ├─ hover_thrust_estimate           │      ├─▶ vehicle_attitude_setpoint ─▶ mc_att_control
   ├─ vehicle_status / land_detected ─┘      ├─▶ gimbal 指令（gimbal manager）
   │                                         ├─▶ vehicle_command（退出時切模式）
   │                                         ├─▶ through_status（log）
   │                                         └─▶ 模式註冊 / ArmingCheckReply
   └─ commander（ModeManagement）◀──────────────┘
```

---

## 4. 實作前驗證（必做，產出 `docs/through_mode/VERIFICATION.md`）

每一項都要寫出：查到的檔案路徑、實際欄位或行為、對本規格的影響。

| 編號 | 驗證項目 |
|---|---|
| V1 | v1.17 中 `RegisterExtComponentRequest` / `RegisterExtComponentReply` / `ArmingCheckRequest` / `ArmingCheckReply` / `config_control_setpoints`（`VehicleControlMode`）的實際欄位與流程。以 `mc_nn_control` 為範本。確認 `health_component_index`、`num_events` 必須設為 0。 |
| V2 | 模組內部如何切出到 Hold：使用 `VEHICLE_CMD_SET_NAV_STATE`（若存在）或 `VEHICLE_CMD_DO_SET_MODE`，以及正確參數。 |
| V3 | 外部註冊模式在 QGC（動態模式列表）中是否顯示、能否綁定到 RC 模式開關（`COM_FLTMODE*` 或其他機制），SITL 下是否可用。 |
| V4 | `mc_att_control` 在 attitude/rate/allocation 開啟、position/velocity/altitude/manual 關閉的控制旗標下，是否正確使用外部發布的 `vehicle_attitude_setpoint`；確認 `thrust_body` 的正負號與正規化定義、`q_d` 的座標約定；確認此旗標組合下沒有其他模組（`mc_pos_control`、`flight_mode_manager`、手動 stabilized）也在發布 `vehicle_attitude_setpoint`。 |
| V5 | `gz_x500_gimbal`：gimbal 姿態回授是否可用（`gimbal_device_attitude_status` 或其他 topic）、四元數是機體座標還是地球座標（看 flags）、模擬 gimbal 是否自我穩定、pitch 正負號約定、更新頻率、相機 gz topic 名稱與參數（解析度、FOV、update rate）。 |
| V6 | 模組如何取得 gimbal 控制權並發送 pitch 指令（`gimbal_manager_set_attitude`、`vehicle_command DO_GIMBAL_MANAGER_PITCHYAW`、或直接 actuator），以及需要的 `MNT_*` 參數。 |
| V7 | uXRCE-DDS 的時間同步：ROS 2 節點應如何填 `timestamp` / `timestamp_sample` 才會落在 PX4 時鐘（考慮 `use_sim_time` 與 SITL lockstep）；`/fmu/in/*` 需要的 QoS。 |
| V8 | v1.17 新增 uORB msg 的流程（`msg/CMakeLists.txt`、是否需放 `msg/versioned/`）、`dds_topics.yaml` 新增 subscription 的格式、`px4_msgs` 重新產生的步驟。 |
| V9 | 新模組加入 SITL build 與自動啟動的方式（Kconfig、`.px4board`、`module.yaml` 參數定義、ROMFS 啟動腳本）。 |
| V10 | `hover_thrust_estimate` topic 欄位與有效旗標；備援使用 `MPC_THR_HOVER`。 |

---

## 5. 介面定義

### 5.1 `msg/VisionTarget.msg`（topic: `vision_target`）

```
uint64 timestamp            # time since system start (us)
uint64 timestamp_sample     # image capture time (us, PX4 clock)

float32 pan_error           # [rad] +: target is RIGHT of image center
float32 tilt_error          # [rad] +: target is BELOW image center
float32 target_size         # [-] enclosing-circle diameter / image width (0..~1.5)
float32 confidence          # [-] 0..1, detection quality (optional, 1.0 if unused)

bool uncage                 # human lock state (true = locked by operator)
bool target_valid           # target detected in this frame

uint32 seq                  # frame sequence number
```

- 加入 `msg/CMakeLists.txt`，並在 `dds_topics.yaml` 新增為 subscription（ROS 2 → PX4）。
- 同步更新 ROS 2 工作區中的 `px4_msgs`。

### 5.2 `msg/ThroughStatus.msg`（topic: `through_status`，僅供 log 與除錯）

```
uint64 timestamp

uint8 PHASE_IDLE = 0
uint8 PHASE_ALIGN = 1
uint8 PHASE_RAMP = 2
uint8 PHASE_GUIDE = 3
uint8 PHASE_TERMINAL = 4
uint8 PHASE_EXIT = 5
uint8 phase

uint8 REASON_NONE = 0
uint8 REASON_COMPLETED = 1        # TERMINAL finished normally
uint8 REASON_USER_UNLOCK = 2
uint8 REASON_USER_MODE_SWITCH = 3
uint8 REASON_ALIGN_TIMEOUT = 4
uint8 REASON_TARGET_LOST = 5
uint8 REASON_MIN_ALT = 6
uint8 REASON_TILT_SAT = 7
uint8 REASON_MODE_TIMEOUT = 8
uint8 REASON_OPENING = 9          # Vc < 0 persistently (moving away)
uint8 exit_reason

float32 time_in_phase             # [s]
float32 time_in_mode              # [s]

float32[3] los_ned                # unit LOS vector (NED)
float32[3] los_rate               # omega_LOS [rad/s] (NED)
float32 los_elevation             # [rad] + = above horizon
float32 los_azimuth               # [rad]
float32 vc                        # closing velocity [m/s] (before Vc_min clamp)

float32[3] a_perp                 # PN lateral accel command [m/s^2] (NED)
float32 s_along                   # along-LOS accel [m/s^2]
float32[3] f_cmd                  # specific force command [m/s^2] (NED)
float32 thrust_cmd                # normalized [0..1]
float32 hover_thrust              # normalized
bool thrust_saturated
bool tilt_limited
bool a_perp_limited

float32 ramp_alpha                # 0..1

float32 pan_error
float32 tilt_error
float32 target_size
bool target_valid
bool uncage

float32 gimbal_pitch_cmd          # [rad]
float32 gimbal_pitch_meas         # [rad]
```

- `through_status` 與 `vision_target` 都要加入 `src/modules/logger/logged_topics.cpp`（預設 profile 即記錄，不要放在需另開的 debug 類別）。

---

## 6. 座標與符號約定

- 世界座標：**NED**（與 PX4 EKF 一致）。重力向量 `g_vec = [0, 0, g]`，`g = 9.80665`。
- 相機座標：x 向前（光軸）、y 向右、z 向下。gimbal 角度為 0 時與機體軸對齊。
- `pan_error > 0`：目標在畫面右側；`tilt_error > 0`：目標在畫面下方。單位為弧度。
- 相機座標下的目標射線：`r_c = normalize([1, tan(pan_error), tan(tilt_error)])`。
- 推力：正規化 [0, 1]，`vehicle_attitude_setpoint.thrust_body[2] = -thrust`（依 V4 結果確認）。
- gimbal pitch 正負號依 V5 結果轉換，模組內部一律使用「正值 = 鏡頭往下」並在介面邊界轉換。

---

## 7. 導引與控制律（`through_control` 模組內）

執行頻率：100 Hz（以 `vehicle_attitude` 更新觸發並限速，或 `ScheduledWorkItem` 100 Hz，擇一並說明）。

### Step 1：LOS 單位向量

```
R_cam_ned = camera attitude in NED
            優先：由 gimbal 姿態回授取得（V5）
            備援：R_nb(vehicle_attitude) * R_bc(gimbal_pitch_cmd)
u = normalize( R_cam_ned * r_c )
```

- 使用的姿態必須對應影像擷取時刻 `timestamp_sample`：保留最近 ~0.5 s 的姿態環形緩衝區，取時間最接近者（或線性插值）。
- `vision_target` 超過 `THRU_LOST_TO` 未更新或 `target_valid == false` 時，u 不更新（沿用上一個值），並標記為無效。

### Step 2：LOS 角速度

```
u_dot_raw = (u_k - u_{k-1}) / dt_vision     # 以影像時間戳差計算，只在新影像到達時更新
u_dot     = LPF(u_dot_raw, cutoff = THRU_LOS_LPF)
u_dot     = u_dot - dot(u_dot, u) * u        # 移除平行分量
omega_LOS = cross(u, u_dot)
```

- 濾波器在 ALIGN 階段就開始運作（暖機），進入 GUIDE 時已收斂。
- 目標無效期間濾波器暫停，恢復時以新的 u 重新初始化前一筆值（避免跳變）。

### Step 3：接近速度

```
vc_raw = dot(v_ned, u)                 # 目標靜止，Vc 為速度在 LOS 的投影
vc     = max(vc_raw, THRU_VC_MIN)
```

### Step 4：PN 橫向加速度

```
a_perp = THRU_PN_N * vc * cross(omega_LOS, u)     # 等同 N * vc * u_dot（垂直分量）
若 |a_perp| > THRU_A_MAX：等比例縮放，a_perp_limited = true
```

### Step 5：推力分配（固定基準 + 有限浮動）

```
T_h   = hover thrust（hover_thrust_estimate，無效時用 MPC_THR_HOVER）
f0    = g * THRU_THR_BASE / T_h       # 基準比推力
f_max = g * THRU_THR_MAX  / T_h
f_min = g * THRU_THR_MIN  / T_h

w      = a_perp - g_vec               # 比推力 f = a - g_vec，a = a_perp + s*u
wu     = dot(w, u)
w_perp = w - wu * u

# 5a. 確保可行：垂直 LOS 的需求不可超過最大推力（保留重力補償，優先縮 a_perp）
若 |w_perp| > f_max：
    縮放 a_perp 使 |w_perp| == f_max，重算 w, wu, w_perp；a_perp_limited = true

# 5b. 固定推力解
disc = f0^2 - |w_perp|^2
若 disc >= 0： s_thr = -wu + sqrt(disc)      # 取前進方向的根
否則：         s_thr = -wu                  # 推力必須高於基準，取最小推力解

# 5c. 速度上限
s_vlim = THRU_K_VLIM * (THRU_V_MAX - dot(v_ned, u))
s = min(s_thr, s_vlim)

# 5d. 輸出
f = w + s * u
thrust = |f| * T_h / g，clamp 到 [THRU_THR_MIN, THRU_THR_MAX]；被 clamp 時 thrust_saturated = true
```

### Step 6：推力向量 → 姿態設定點

```
body_z = -f / |f|
若 body_z 與 [0,0,-1] 夾角 > THRU_TILT_MAX：
    保持水平方位，將傾角壓到 THRU_TILT_MAX；tilt_limited = true

yaw_sp = atan2(u[1], u[0])            # LOS 方位角
    若 |u 的水平分量| < 0.1：維持上一個 yaw_sp（目標幾乎正下方時 atan2 不穩定）
    yaw 變化以 THRU_YAW_RMAX 限速

q_d = bodyzToAttitude(body_z, yaw_sp)  # 使用 PX4 ControlMath 既有函式
發布 vehicle_attitude_setpoint：q_d、thrust_body = [0, 0, -thrust]
```

### Step 7：Gimbal 指令

- 目標：讓相機光軸對準 u。
- 期望相機俯角（地球座標，正值往下）：`el_des = asin(u[2])`。
- 指令：`pitch_cmd = pitch_meas + THRU_GMB_K * (el_des - pitch_meas)`（`THRU_GMB_K` ∈ (0, 1]，避免與 gimbal 延遲形成振盪）。
- 若 V5/V6 顯示 gimbal 只接受機體座標角度，則轉換成機體座標再送出。
- gimbal yaw 跟隨機體（不獨立控制），roll 依模擬 gimbal 預設行為。
- Gimbal 行程需涵蓋約 −30°（相對機身往上）到 +90°（往下）；若 `gz_x500_gimbal` 不足，在自訂模型中調整 joint limit。

### 共用的懸停命令（ALIGN 與 RAMP 使用）

```
a_hover = -THRU_K_VDAMP * v_ned        # 只做速度阻尼，允許高度少量漂移
f = a_hover - g_vec → 同 Step 6 轉姿態，thrust = |f| * T_h / g（clamp）
```

---

## 8. 階段狀態機

```
IDLE ──(nav_state == THROUGH)──▶ ALIGN ──(對準持續 T_ok)──▶ RAMP ──(alpha==1)──▶ GUIDE
                                   │                                              │
                                   └─(逾時)─▶ EXIT                (尺寸門檻/遺失近距離) ▼
                                                                              TERMINAL
                                   GUIDE ─(安全觸發 / 跟丟 / 解鎖)─▶ EXIT ◀──(T_COAST)─┘

任何階段：nav_state 不再是 THROUGH → 立即回 IDLE（REASON_USER_MODE_SWITCH），停止發布 setpoint
```

### 8.1 各階段行為

| 階段 | 姿態/推力 | Yaw | Gimbal | 進入動作 |
|---|---|---|---|---|
| IDLE | 不發布 | — | 不控制 | 釋放 gimbal 控制權 |
| ALIGN | `a_hover` | 限速轉向 LOS 方位角 | 追蹤 | 取得 gimbal 控制權、初始化 LOS 濾波器 |
| RAMP | `a = (1-α)·a_hover + α·a_guide`，推力基準由 `T_h` 線性升到 `THRU_THR_BASE` | LOS 方位角 | 追蹤 | α 從 0 開始，於 `THRU_RAMP_T` 內升到 1 |
| GUIDE | 第 7 節 Step 4–6 | LOS 方位角 | 追蹤 | — |
| TERMINAL | 凍結進入當下的 `q_d` 與 thrust | 凍結 | 凍結 | 記錄凍結值 |
| EXIT | 發布最後一次凍結/懸停 setpoint 直到模式切換生效 | 凍結 | 釋放 | 送出切換到 `THRU_EXIT_MODE` 的指令（只送一次，必要時以 1 Hz 重試），記錄 `exit_reason` |

`a_guide` 指 Step 4 的 `a_perp` 加上 Step 5 算出的 `s·u`。RAMP 期間 Step 5 的 `f0` 使用斜坡中的推力基準。

### 8.2 轉換條件

| 從 | 條件 | 到 | exit_reason |
|---|---|---|---|
| ALIGN | `|pan|, |tilt| < THRU_ALN_ERR` 且 yaw 誤差 < THRU_ALN_ERR，持續 `THRU_ALN_T_OK` | RAMP | — |
| ALIGN | 階段時間 > `THRU_ALN_T_MAX` | EXIT | ALIGN_TIMEOUT |
| RAMP/GUIDE | `uncage == false` | EXIT | USER_UNLOCK |
| GUIDE | `target_size > THRU_TERM_SZ`（有效影像） | TERMINAL | — |
| GUIDE | 目標無效持續 > `THRU_LOST_TO`，且最後有效的 `target_size > THRU_TERM_SZ * THRU_TERM_NEAR` | TERMINAL | — |
| GUIDE | 目標無效持續 > `THRU_LOST_TO`，其他情況 | EXIT | TARGET_LOST |
| ALIGN/RAMP | 目標無效持續 > `THRU_LOST_TO` | EXIT | TARGET_LOST |
| RAMP/GUIDE | 高度（相對 home / local z）< `THRU_MIN_ALT` | EXIT | MIN_ALT |
| GUIDE | `tilt_limited` 或 `thrust_saturated` 連續 > `THRU_SAT_T` | EXIT | TILT_SAT |
| GUIDE | `vc_raw < 0` 連續 > 0.5 s | EXIT | OPENING |
| 任何 | 模式總時間 > `THRU_T_MAX` | EXIT | MODE_TIMEOUT |
| TERMINAL | 階段時間 ≥ `THRU_TERM_T` | EXIT | COMPLETED |

- TERMINAL 期間**忽略** `uncage` 與 `target_valid`。
- 每次轉換以 `mavlink_log_info` / events 送出可讀訊息到 QGC（例如 `THROUGH: GUIDE -> TERMINAL`、`THROUGH aborted: TARGET_LOST`）。

---

## 9. 模式註冊與條件一

### 9.1 註冊

- 模組啟動後發布 `RegisterExtComponentRequest`：註冊一個 mode（名稱 `THROUGH`）與一個 arming check；不取代任何內部模式。
- 等待 `RegisterExtComponentReply`，取得 `mode_id` 與 `arming_check_id`；失敗或逾時則每秒重試，最多 10 次，失敗則 `PX4_ERR` 並結束。
- 以 `mode_id` 在 `config_control_setpoints` 發布 `VehicleControlMode`：
  - 開啟：attitude、rates、allocation（以及 V1 確認需要的其他基本旗標，例如 armed 相關）
  - 關閉：manual、position、velocity、altitude、climb_rate、acceleration、offboard 相關
- 模組結束時依 V1 確認的方式註銷。

### 9.2 ArmingCheckReply（條件一）

收到 `ArmingCheckRequest` 時一定要回覆（`health_component_index = 0`、`num_events = 0`）。

| 狀態 | `can_arm_and_run` 的判斷 |
|---|---|
| 模式**未啟動** | `in_air`（`vehicle_land_detected.landed == false`）∧ `uncage` ∧ `target_valid` ∧ `vision_target` 新鮮（< `THRU_LOST_TO`）∧ 高度 ≥ `THRU_MIN_ALT` |
| 模式**啟動中** | 一律 `true`（退出由模組自行處理，避免觸發 commander failsafe） |

- 模式需求旗標：angular_velocity、attitude、local_alt、local_position（或 relaxed，依 V1）。
- 設定「此模式不可解鎖」（等同 `mode_req_prevent_arming`），因為只能在空中進入。
- 條件不成立時 QGC 應顯示無法切換；若 V1 確認可以附帶原因文字，填入「Target not locked」等訊息。

---

## 10. 參數（`module.yaml` 或 `*_params.c`，依 V9）

| 名稱 | 預設 | 單位 | 說明 |
|---|---|---|---|
| `THRU_EN` | 1 | — | 啟動模組 |
| `THRU_THR_BASE` | 0.70 | norm | 推力基準 |
| `THRU_THR_MIN` | 0.30 | norm | 推力下限 |
| `THRU_THR_MAX` | 0.90 | norm | 推力上限 |
| `THRU_PN_N` | 3.5 | — | PN 導航常數 |
| `THRU_A_MAX` | 9.8 | m/s² | `a_perp` 上限 |
| `THRU_V_MAX` | 10.0 | m/s | 沿 LOS 速度上限 |
| `THRU_K_VLIM` | 2.0 | 1/s | 速度上限迴路增益 |
| `THRU_VC_MIN` | 1.0 | m/s | PN 使用的最小 Vc |
| `THRU_TILT_MAX` | 60 | deg | 最大傾角 |
| `THRU_LOS_LPF` | 8.0 | Hz | `u_dot` 低通截止頻率 |
| `THRU_K_VDAMP` | 1.5 | 1/s | ALIGN/RAMP 速度阻尼增益 |
| `THRU_YAW_RMAX` | 60 | deg/s | yaw 速率限制 |
| `THRU_ALN_ERR` | 3.0 | deg | 對準完成門檻 |
| `THRU_ALN_T_OK` | 0.5 | s | 對準需持續時間 |
| `THRU_ALN_T_MAX` | 8.0 | s | ALIGN 逾時 |
| `THRU_RAMP_T` | 1.0 | s | 推力斜坡時間 |
| `THRU_TERM_SZ` | 0.6 | — | 進入 TERMINAL 的 target_size 門檻 |
| `THRU_TERM_NEAR` | 0.6 | — | 遺失時判定「接近」的比例（乘上 TERM_SZ） |
| `THRU_TERM_T` | 0.5 | s | TERMINAL 滑行時間 |
| `THRU_LOST_TO` | 0.2 | s | 目標遺失容忍時間 |
| `THRU_MIN_ALT` | 2.0 | m | 最低高度 |
| `THRU_SAT_T` | 1.0 | s | 飽和容忍時間 |
| `THRU_T_MAX` | 30 | s | 模式最長時間 |
| `THRU_GMB_K` | 0.8 | — | gimbal 追蹤增益 |
| `THRU_EXIT_MODE` | 0 | enum | 0 = Hold，1 = Position |

所有參數名稱 ≤ 16 字元；角度參數在程式內轉成弧度。

---

## 11. `through_control` 模組結構

```
src/modules/through_control/
├── CMakeLists.txt
├── Kconfig
├── module.yaml                 # 或 through_control_params.c（依 V9）
├── ThroughControl.hpp / .cpp   # ModuleBase + ScheduledWorkItem，狀態機、uORB I/O
├── ModeRegistration.hpp / .cpp # 註冊、ArmingCheckReply、VehicleControlMode 設定
├── LosEstimator.hpp / .cpp     # Step 1–3：u、u_dot、omega_LOS、Vc，姿態緩衝區
├── PnGuidance.hpp / .cpp       # Step 4–6：a_perp、推力分配、姿態設定點
├── GimbalTracker.hpp / .cpp    # Step 7：gimbal 指令與控制權
└── test/                       # 純數學部分的單元測試（gtest，見 14 節）
```

- 加入 `boards/px4/sitl/default.px4board`：`CONFIG_MODULES_THROUGH_CONTROL=y`。
- 自動啟動：依 V9，在 SITL 的多旋翼啟動腳本中以 `THRU_EN` 判斷是否啟動。
- `LosEstimator`、`PnGuidance` 必須是**不依賴 uORB 的純計算類別**，方便單元測試。

---

## 12. 視覺節點 `through_vision`（ROS 2 Python 套件）

### 12.1 輸入 / 輸出

| 方向 | Topic | 型別 |
|---|---|---|
| 輸入 | 相機影像（經 `ros_gz_bridge`） | `sensor_msgs/Image` |
| 輸入 | 相機內參（若橋接可用） | `sensor_msgs/CameraInfo` |
| 輸出 | `/fmu/in/vision_target` | `px4_msgs/VisionTarget`（QoS 依 V7） |
| 輸出（選用） | `~/debug_image` | `sensor_msgs/Image` |
| 服務 | `~/set_lock` | `std_srvs/SetBool` |

### 12.2 處理流程（每張影像）

1. 轉成 OpenCV BGR → HSV。
2. `inRange` 色彩閾值（參數化；紅色需處理 H 環繞，使用兩段範圍合併）。
3. 形態學 open + close 去雜訊。
4. 找外輪廓，取面積最大者；面積 < `min_area_px` 則 `target_valid = false`。
5. 目標中心：預設用輪廓的 `minEnclosingCircle` 圓心（對環形目標比 moments 穩定）；參數可切換為 moments 質心。
6. 角度換算（優先用 `CameraInfo` 的 fx, fy, cx, cy；沒有則由 `hfov` 參數推算 `fx = (W/2) / tan(hfov/2)`，fy = fx）：
   ```
   pan_error  = atan((u_t - cx) / fx)
   tilt_error = atan((v_t - cy) / fy)
   ```
7. `target_size = 2 * radius / W`。
8. `timestamp_sample` = 影像 header 時間換算成 PX4 時鐘（依 V7）；`timestamp` = 發布時刻。
9. 每張影像都發布一次（無論 valid 與否），`seq` 遞增。

### 12.3 鎖定（Uncage）

- `uncage` 表示**操作者的鎖定狀態**，與該幀是否偵測到目標無關。
- OpenCV 除錯視窗按鍵：`l` = 鎖定（只有當前 `target_valid` 時才生效）、`u` = 解鎖、`q` = 結束節點。
- 服務 `~/set_lock`（SetBool）提供腳本化測試用的鎖定 / 解鎖。
- 參數 `auto_unlock_s`（預設 0 = 停用）：目標連續遺失超過此秒數時自動解鎖。

### 12.4 除錯視窗

畫面中心十字、偵測輪廓與外接圓、目標中心點、pan/tilt（度）、target_size、鎖定狀態（鎖定時邊框變紅）、FPS。參數 `show_window` 可關閉（無頭執行時）。

### 12.5 參數（ROS 2 parameters）

`image_topic`、`camera_info_topic`、`hsv_lower_1`、`hsv_upper_1`、`hsv_lower_2`、`hsv_upper_2`、`min_area_px`、`center_method`（`circle` | `moments`）、`hfov`、`show_window`、`auto_unlock_s`、`use_sim_time`。

### 12.6 測試工具

`fake_vision_target.py`：不經影像，直接以固定或正弦變化的 pan/tilt 發布 `VisionTarget`，並可切換 uncage / valid。用於第 14 節 M1、M2 的模式註冊與條件一測試。

---

## 13. gz 模型與世界

### 13.1 機型 `x500_through`

- 以 `x500_gimbal` 為基底（複製模型與 airframe，不直接修改原檔）。
- 相機：解析度 640×480、HFOV 90°、update rate 60 Hz（若可提高則提高）、無雜訊、無畸變。
- gimbal pitch joint：行程涵蓋 −30°（上）到 +90°（下），速度與力矩上限調高，使響應盡量快。
- airframe：新的 SITL airframe 檔（ID 由 Claude Code 挑選未使用的編號），設定 gimbal 相關 `MNT_*` 參數（依 V6）與 `THRU_EN=1`。

### 13.2 世界 `through_ring.sdf`

- 地面、太陽光、單色天空；場景中不得有其他與目標同色的物體。
- 目標環 `through_ring`：
  - 顏色：紅色，設定 emissive 讓顏色不受光照角度影響。
  - 內徑 2.0 m，管徑 0.15 m（以 24 段 box/cylinder 組成，或產生 mesh，擇一）。
  - 中心位置（gz 世界 ENU）：`(25, 0, 5)` m。
  - 飛機起點：`(0, 0, 0)`，起飛後懸停於 20 m → 初始 LOS 俯角約 atan(15/25) ≈ 31°。
  - **環面傾斜（方案 A）**：環的法向量指向起點上方 20 m 處，也就是沿初始 LOS 方向，讓俯衝路徑大致垂直穿過環面。
  - 碰撞：開啟（可觀察撞環）。
- 環的位置、尺寸、傾角用 SDF 或 xacro/Python 產生腳本參數化，方便之後改難度。

### 13.3 穿環判定腳本 `ring_judge.py`

- 從 gz 取得飛機與環的真實位姿（gz transport 或 `ros_gz_bridge` 的 pose topic）。
- 偵測飛機穿過環面的瞬間，計算穿越點到環心的距離：
  - `< 內徑/2 − 機體半徑（0.35 m）` → **PASS**
  - `< 內徑/2 + 管徑` → **HIT**
  - 其他 → **MISS**
- 同時記錄穿越時的速度、速度與環法向量夾角，輸出到 CSV。
- 另外輸出真實 LOS 向量（飛機 → 環心，NED），供 M4 驗證 `los_ned` 精度。

### 13.4 啟動腳本 `run_through_sim.sh`

依序啟動：PX4 SITL（`x500_through` + `through_ring` 世界，使用 `PX4_SIM_MODEL` / `PX4_GZ_WORLD` 等 PX4 支援的方式，依查證結果）、`MicroXRCEAgent udp4 -p 8888`、`ros_gz_bridge`（相機影像、相機內參、pose）、`through_vision`、`ring_judge.py`。

---

## 14. 里程碑與驗收

| 里程碑 | 內容 | 驗收條件 |
|---|---|---|
| **M0** | 第 4 節驗證 | `VERIFICATION.md` 完成，V1–V10 皆有結論 |
| **M1** | `VisionTarget` / `ThroughStatus` msg、DDS 設定、`px4_msgs` 更新、`fake_vision_target.py` | PX4 shell `listener vision_target` 可看到 fake 資料，時間戳合理 |
| **M2** | `through_control` 骨架：註冊、ArmingCheckReply、ALIGN 懸停（暫不控制 gimbal） | QGC 模式清單出現 THROUGH；uncage=false 時切換被拒並顯示原因；uncage=true 時可從 QGC 與 RC 切入；切入後懸停 30 s 水平漂移 < 1 m；從 QGC 切走立即停止發布 |
| **M3** | `x500_through` 模型、`through_ring` 世界、`through_vision` 節點、`ring_judge.py`、`run_through_sim.sh` | 一鍵啟動；除錯視窗正確框出環；手動移動飛機時 pan/tilt 正負號符合第 6 節；`l`/`u` 鍵與 `set_lock` 服務正常 |
| **M4** | gimbal 追蹤 + LOS 重建（ALIGN 完整） | ALIGN 中 `|tilt_error|` 穩態 < 1°；`los_ned` 與 `ring_judge.py` 真實 LOS 夾角 < 1.5°；yaw 對準後進入 RAMP |
| **M5** | RAMP + GUIDE + 所有 EXIT 路徑 | 能完成俯衝接近；人為解鎖、切模式、低高度、逾時各觸發一次並記錄正確 `exit_reason`；退出後確實進入 Hold |
| **M6** | TERMINAL + 完整穿環 | 標準場景連續 10 次至少 8 次 PASS |
| **M7** | 分析工具與調參 | `analyze_through_log.py`（pyulog）畫出 phase、LOS 角/角速度、a_perp、推力、傾角、pan/tilt 時間序列；提出 N、LPF、TERM_SZ 的調整建議 |

### 單元測試（M2 起持續加入）

- `LosEstimator`：給定相機姿態與 pan/tilt，u 的方向正確；靜止情境 omega_LOS ≈ 0；等速直線運動情境 omega_LOS 與解析解一致。
- `PnGuidance`：
  - `a_perp = 0` 且水平 LOS 時，固定推力解使垂直加速度平衡的條件正確。
  - `disc < 0` 時走最小推力解，thrust 上升。
  - 速度超過 `THRU_V_MAX` 時 `s` 變小或為負。
  - 傾角限制生效時 body_z 傾角等於上限。
  - 目標幾乎正下方時 yaw 維持前值。

---

## 15. 測試情境（M5–M6 之後）

1. 標準：正對環、無風。
2. 橫向偏移起點（y = ±5 m），驗證 roll 修正橫向速度。
3. 起點高度 30 m（更陡的俯衝），觀察 `THRU_V_MAX` 與推力下限的作用。
4. 加入 gz 風場（例如 3 m/s 側風）。
5. GUIDE 中途以服務解鎖 → 應 USER_UNLOCK 退出。
6. GUIDE 中途暫時遮蔽目標（< 與 > `THRU_LOST_TO` 兩種）。
7. 未鎖定時嘗試切換 → 被拒。
8. 從 RC 開關切入 / 切出。

---

## 16. 未來工作（本次不實作）

- 終端角約束導引（讓飛行路徑以指定角度穿過直立的環）。
- 以 gyro + gimbal 角速度 + 影像角速度合成 omega_LOS，降低雜訊。
- 多環連續穿越。
- HITL 驗證。

---

## 17. 設計決策紀錄

| 主題 | 決策 |
|---|---|
| 平台 | 只做模擬，PX4 v1.17 + gz sim + 新版 QGC |
| 模式實作 | 韌體內部模組，透過 uORB 外部模式註冊，不修改 commander |
| 導引律 | 3D PN（`a_perp = N·Vc·ω_LOS×u`），目標靜止，Vc 取速度在 LOS 的投影 |
| 推力 | 固定基準（0.70）+ 有限浮動（0.30–0.90），沿 LOS 加速度為自由度，加速度上限保護 |
| 橫向修正 | 由推力向量方向決定（roll/pitch 自然分配），yaw 只負責讓相機對準 LOS 方位 |
| LOS 重建 | 以相機實際姿態（gimbal 回授）× 影像射線，取影像擷取時刻的姿態 |
| Gimbal | 只用 pitch，經 PX4 gimbal manager / actuator output 控制 |
| ALIGN | 只做速度阻尼，允許高度少量漂移 |
| 環 | 先用傾斜環（法向量沿初始 LOS），之後再挑戰直立環 |
| TERMINAL | 固定時間 0.5 s，忽略視覺遺失 |
| 退出 | 模組自行切到 Hold（預設），模式執行中 ArmingCheck 一律回 OK，不走 failsafe |
| 條件一 | 在 ArmingCheckReply 實作：在空中 ∧ uncage ∧ target_valid ∧ 資料新鮮 ∧ 高度足夠 |
| 鎖定 | 由操作者在視覺節點視窗按鍵或呼叫服務觸發 |
| 視覺 | Python + OpenCV HSV 顏色追蹤，經 ROS 2 / uXRCE-DDS 送入 PX4 |
