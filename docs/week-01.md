# 第 1 周学习日志

日期为 2026-11-20。实验目标是在 Windows 上运行 Panda 的单关节位置阶跃响应，并保存可复查的角度数据。

## 1. 核对官方 API

先查阅 MuJoCo 官方文档，再编写脚本。`MjModel.from_xml_path` 加载 MJCF，`MjData` 保存运行状态，`mj_resetDataKeyframe` 恢复 `home`，`mj_forward` 刷新初始派生状态，`mj_step` 推进仿真。`data.ctrl` 写入执行器目标，`data.joint("joint1").qpos` 读取实测角度。`mujoco.viewer.launch_passive` 提供可选的 3D 窗口。[官方 Python 绑定](https://mujoco.readthedocs.io/en/stable/python.html) · [官方函数参考](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html)

## 2. 检查环境并安装依赖

用 `Get-CimInstance`、`nvidia-smi`、`py -0p` 和 Python 包元数据检查了本机环境。默认 `python` 指向 3.10.11，项目显式选用本机的 64 位 Python 3.12.10，并用 `py -3.12 -m venv .venv` 创建隔离环境。检查确认 `sys.prefix != sys.base_prefix`。

| 项目 | 实测值 |
| --- | --- |
| Windows | Windows 11 家庭中文版，64 位，版本 `10.0.22631` |
| NVIDIA GPU | GeForce RTX 4060 Laptop GPU，驱动 `616.92`，显存 `8188 MiB` |
| 核显 | Intel Iris Xe Graphics，驱动 `31.0.101.4255` |
| 虚拟显示适配器 | ToDesk Virtual Display Adapter，驱动 `16.44.2.509` |
| Python | 默认 `3.10.11`，项目 `3.12.10`，另发现 uv 管理的 `3.14.5` |
| `.venv` 中的依赖 | pip `25.0.1`，MuJoCo `3.16.0`，NumPy `2.5.5`，Matplotlib `3.10.9`，pytest `9.2.0`，GLFW `2.11.0`，PyOpenGL `3.1.10` |

安装前，Python 3.10 中已有 MuJoCo `3.16.0`、NumPy `2.2.6`、Matplotlib `3.10.9` 和 GLFW `2.11.0`。Python 3.12 中已有 NumPy `2.5.1` 和 Matplotlib `3.10.9`。两个解释器均未发现 `mujoco-py` 和 pytest。

项目使用官方 `mujoco==3.16.0` 的 Windows 64 位预编译包。导入后的 Python 包版本与 `mj_versionString()` 的返回值均为 `3.16.0`。`pip freeze` 生成了 [requirements.txt](../requirements.txt)，随后用 `pip install --no-index -r requirements.txt` 核对当前环境，并通过 `pip check`。

```text
No broken requirements found.
```

## 3. 获取并加载 Panda 场景

固定到 MuJoCo Menagerie 提交 `370c43965323c0944b433b202aac5058b5f907a9`。用官方 GitHub API 的 Git 树核对 Panda 目录的所有文件，再实际加载 `scene.xml`。`joint1` 的关节限位和 `actuator1` 的控制范围均为 `[-2.8973, 2.8973] rad`。[模型来源与许可](model-source.md)

```text
Verified 80 Panda Git blobs, 36560830 bytes
nq=9, nv=9, nu=8, dt=0.002
joint1 initial angle: 0.0
joint1 range (rad): [-2.8973, 2.8973]
actuator1 range (rad): [-2.8973, 2.8973]
```

### 实际故障与恢复

- 首次通过 Git 和 GitHub API 获取文件时遇到握手失败。固定提交的归档请求也遇到 TLS EOF。Git 使用 OpenSSL 和 HTTP/1.1 重试后成功完成稀疏检出。逐文件下载遇到超时的文件从已检出的同一提交补齐，最终重新核对全部 80 个文件。

  ```text
  fatal: unable to access 'https://github.com/google-deepmind/mujoco_menagerie.git/': schannel: failed to receive handshake, SSL/TLS connection failed
  Received an unexpected EOF or 0 bytes from the transport stream.
  urllib.error.URLError: <urlopen error [SSL: UNEXPECTED_EOF_WHILE_READING] EOF occurred in violation of protocol (_ssl.c:1010)>
  curl: (28) SSL/TLS connection timeout
  Download failed: franka_emika_panda/assets/hand_1.obj
  Download failed: franka_emika_panda/assets/link6_8.obj
  ```

- `pip` 首次获取 MuJoCo wheel 的元数据时超时，自动重试后完成安装。

  ```text
  ReadTimeoutError("HTTPSConnectionPool(host='files.pythonhosted.org', port=443): Read timed out. (read timeout=15)")
  ```

- 模型加载探针第一次因 PowerShell 中嵌套引号被错误解析而退出。改用 here-string 传入相同探针后，得到上面的模型尺寸、步长和限位。

  ```text
  SyntaxError: '(' was never closed
  ```

## 4. 运行位置控制并核查曲线

保留 `home` 的控制量，在 `0.5 s` 时仅更新 `actuator1`。该执行器由上游 MJCF 的固定增益与仿射偏置实现位置伺服。指令和实测角度分别写入 CSV 的 `target_rad` 和 `angle_rad` 列。曲线上，目标线在 `0.5 s` 跳变，实测线随后收敛。[MuJoCo 执行器 XML 参考](https://mujoco.readthedocs.io/en/stable/XMLreference.html#actuator-position)

```text
MuJoCo 3.16.0 | joint1 | 1501 samples | 3.000 s
Angle: 0.000000 -> 0.500000 rad; target: 0.500000 rad
Final error: 0.000000 rad
```

终端将最终角度误差显示到小数点后六位。CSV 中最终实测角为 `0.49999999999546807 rad`，绝对误差约 `4.53e-12 rad`。PNG 已解码验证，尺寸为 `1200 × 600`。`--viewer` 运行完成，导出的 CSV 与默认运行的 SHA-256 相同。`--target -0.3` 也运行完成，最终角度约为 `-0.300000 rad`。

## 5. 运行测试

使用 `.venv` 执行 `python -m pytest -q`，覆盖 Panda 模型加载、正负阶跃跟踪、采样间隔、非法目标值，以及从其他工作目录运行并导出 CSV 和 PNG。手动传入 `--target 4.0` 时，命令按预期返回退出码 `2`。

```text
........                                                                 [100%]
8 passed in 3.82s
panda_position.py: error: joint1 target must be finite and within [-2.8973, 2.8973] rad
```

本次练习中，`model.opt.timestep` 决定相邻采样点的仿真时间间隔，3D 窗口的墙钟节奏只用来观察运动。CSV 可用于比较目标值和实测值，计算跟踪误差。

## 6. `joint1` 自动往复跟踪

### 加载后的执行器参数

用 MuJoCo 3.16.0 实际加载 `scene.xml`，得到 `model.nu = 8`、`model.nq = 9`、`model.nv = 9`。下表来自编译后的 `model.actuator(...)` 参数，八项的 `ctrllimited` 均为真。

| 执行器 | 传动对象 | 控制类型 | `ctrlrange` | `home` 指令 |
| --- | --- | --- | --- | ---: |
| `actuator1` | `joint1` | 位置伺服 | `[-2.8973, 2.8973] rad` | `0` |
| `actuator2` | `joint2` | 位置伺服 | `[-1.7628, 1.7628] rad` | `0` |
| `actuator3` | `joint3` | 位置伺服 | `[-2.8973, 2.8973] rad` | `0` |
| `actuator4` | `joint4` | 位置伺服 | `[-3.0718, -0.0698] rad` | `-1.57079` |
| `actuator5` | `joint5` | 位置伺服 | `[-2.8973, 2.8973] rad` | `0` |
| `actuator6` | `joint6` | 位置伺服 | `[-0.0175, 3.7525] rad` | `1.57079` |
| `actuator7` | `joint7` | 位置伺服 | `[-2.8973, 2.8973] rad` | `-0.7853` |
| `actuator8` | 夹爪腱 `split` | 缩放后的腱位置伺服 | `[0, 255]` 指令值 | `255` |

`actuator1` 到 `actuator7` 是 1:1 的转动关节传动，均使用固定增益和仿射偏置。编译后的参数满足 `gain = Kp`、`bias = [0, -Kp, -Kd]`。以 `actuator1` 为例，力矩限幅前为 `4500 × (目标角 − 实际角) − 450 × 角速度`，输出限幅为 `±87 N·m`。目标经位置反馈变成力矩。夹爪的 `gain = 0.01568627451`、`bias = [0, -100, -10]`，所以 `0–255` 对应 `0–0.04 m` 的目标腱长度。[Panda MJCF](../models/franka_emika_panda/panda.xml#L263) · [MuJoCo 执行器力学](https://mujoco.readthedocs.io/en/stable/computation/index.html#force-generation)

### 状态、命令和仿真推进

| API | 实验中的作用 |
| --- | --- |
| `data.qpos` | 当前广义位置。机械臂七个转动关节以弧度表示，两个滑动手指以米表示。每步只读取 `joint1` 的位置来记录实测角。 |
| `data.qvel` | 当前广义速度。转动关节用 `rad/s`，滑动手指用 `m/s`。 |
| `data.ctrl` | 执行器输入数组，含义由模型的传动、增益和偏置决定。本实验每步更新 `actuator1` 的角度指令，其余七个输入沿用 `home`。 |
| `mujoco.mj_step(model, data)` | 根据控制量、动力学和约束积分一个 `0.002 s` 的仿真步，更新 `qpos`、`qvel` 和 `data.time`。 |

`--tracking` 使用振幅 `0.8 rad`、周期 `4 s` 的正弦轨迹，围绕 `home` 的目标角 `0 rad` 运行两轮。代码仅在初始化时重置到 `home`，往复过程中用 `data.ctrl` 驱动。Viewer 将仿真状态复制到独立的 `MjData`，在查看器锁内拷贝数据，再调用 `sync(state_only=True)`，以半速播放。[MuJoCo Python 绑定](https://mujoco.readthedocs.io/en/stable/python.html) · [步进 API](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-step)

### 实测结果

```powershell
.\.venv\Scripts\python.exe .\src\panda_position.py --tracking
.\.venv\Scripts\python.exe .\src\panda_position.py --tracking --viewer
.\.venv\Scripts\python.exe -m pytest -q
```

两种模式均完成了 `4,000` 个物理步，生成 `4,001` 个采样点，CSV 的 SHA-256 相同。Viewer 中观察到了不同转动方向的姿态。自动测试逐步检查了 `ctrl`、`qpos` 和 `qvel` 的有限性、MuJoCo 警告、其它执行器的固定指令和姿态偏差。`joint2` 到 `joint7` 的最大姿态偏差约为 `0.00695 rad`。

```text
MuJoCo 3.16.0 | joint1 | 4001 samples | 8.000 s
Angle: 0.000000 -> -0.123158 rad; target: 0.000000 rad
Final error: 0.123158 rad
Measured angle range: [-0.791104, 0.791105] rad
Tracking error: max abs 0.124545 rad | RMS 0.086507 rad
11 passed in 4.67s
```

两轮中均可见跟踪滞后。结束时目标已经过零，实测还在 `-0.123158 rad`，因为仿真在轨迹仍在变化时结束。两种模式均正常完成，测试中的 4,000 个物理步未产生 MuJoCo 警告。[跟踪曲线](../outputs/figures/joint1_tracking.png) · [采样数据](../outputs/joint1_tracking.csv)
