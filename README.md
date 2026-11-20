# arm-lab

本项目使用 Python 和 MuJoCo 学习 Franka Emika Panda 机械臂的控制。当前包含七关节协同的末端轨迹实验，以及从上游 `home` 姿态开始的 `joint1` 阶跃和往复跟踪。

## 七关节协同轨迹

在 PowerShell 运行以下命令，夹爪会水平移动、下降、上升，再到达第二个三维目标。Viewer 中青色为目标线，橙色为实时累积的实测轨迹。

```powershell
Set-Location -LiteralPath 'D:\studay\AAAA\armlab'
.\.venv\Scripts\python.exe .\src\panda_cartesian.py --viewer
.\.venv\Scripts\python.exe -m pytest -q
```

省略 `--viewer` 可直接生成结果。默认输出为 `outputs/panda_cartesian_tracking.csv`、`outputs/figures/panda_cartesian_xyz_error.png` 和 `outputs/figures/panda_cartesian_joint_angles.png`。[坐标、控制器与验证结果](docs/panda-cartesian.md)

## 单关节实验

从新打开的 PowerShell 启动时，先进入项目目录。用 `--tracking` 运行往复运动，加上 `--viewer` 可观察 3D 动画。

```powershell
Set-Location -LiteralPath 'D:\studay\AAAA\armlab'
.\.venv\Scripts\python.exe .\src\panda_position.py --tracking --viewer
.\.venv\Scripts\python.exe -m pytest -q
```

每行单独执行。`.\` 表示 PowerShell 的当前目录，提示符应显示 `PS D:\studay\AAAA\armlab>`。新副本需要先在项目目录中创建环境并安装依赖。

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

### 往复跟踪实验

`joint1` 的目标角按正弦曲线在 `−0.8 rad` 和 `+0.8 rad` 之间变化，周期为 `4 s`，共运行两轮。Viewer 以半速演示，含开场等待约 `18 s`。只加 `--tracking` 时直接运行仿真并保存结果。

| 文件 | 内容 |
| --- | --- |
| `outputs/joint1_tracking.csv` | 仿真时间 `time_s`、目标角 `target_rad`、实测角 `angle_rad` 和有符号误差 `error_rad`（目标减实测） |
| `outputs/figures/joint1_tracking.png` | 目标角和实测角随仿真时间变化的曲线 |

### 阶跃实验

不带 `--tracking` 时，仿真在 `0.5 s` 时将 `joint1` 的目标从 `0` 改为 `0.5 rad`，运行至 `3.0 s`。结果写入 `outputs/`。

| 文件 | 内容 |
| --- | --- |
| `joint_angle.csv` | 每个仿真采样点的时间 `time_s`、目标角 `target_rad` 和实测角 `angle_rad` |
| `joint_angle.png` | 目标角与实测角随时间变化的曲线 |

阶跃模式的 Viewer 以五分之一速度播放，整段演示约 `17 s`。窗口默认收起面板，并显示由脚本控制的运动。

```powershell
.\.venv\Scripts\python.exe .\src\panda_position.py --viewer
.\.venv\Scripts\python.exe .\src\panda_position.py --target -0.3 --output-dir .\outputs\negative
```

`--target` 的单位为弧度，允许范围由模型的关节限位和执行器控制范围共同决定。输出目录内同名的 CSV 和 PNG 会被覆盖。

- [Panda 模型来源和许可](docs/model-source.md)
- [第 1 周学习与验证日志](docs/week-01.md)
