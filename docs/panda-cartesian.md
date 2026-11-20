# Panda 七关节末端轨迹控制

`src/panda_cartesian.py` 使用项目现有的 `scene.xml` 和 `home` 关键帧。由 `hand` 刚体局部坐标 `(0, 0, 0.1034) m` 定义工具中心点（TCP），接近两指接触垫中心的中点。世界坐标用 `hand` 的世界旋转和位置变换得到，目标运动中保持 `home` 的夹爪方向。[Panda 手部 MJCF](../models/franka_emika_panda/panda.xml)

## 模型与路径

加载后共有 8 个执行器。七个机械臂执行器逐一核对关节传动、单位传动比、固定增益、仿射位置偏置及限幅。第八个执行器驱动夹爪腱，沿用 `home` 指令。以下数值来自编译后的模型。

| 关节 | 执行器 | 关节限位与位置指令范围（rad） | 执行器力矩限幅（N·m） |
| --- | --- | --- | --- |
| `joint1` | `actuator1` | `[-2.8973, 2.8973]` | `±87` |
| `joint2` | `actuator2` | `[-1.7628, 1.7628]` | `±87` |
| `joint3` | `actuator3` | `[-2.8973, 2.8973]` | `±87` |
| `joint4` | `actuator4` | `[-3.0718, -0.0698]` | `±87` |
| `joint5` | `actuator5` | `[-2.8973, 2.8973]` | `±12` |
| `joint6` | `actuator6` | `[-0.0175, 3.7525]` | `±12` |
| `joint7` | `actuator7` | `[-2.8973, 2.8973]` | `±12` |

轨迹点以实测 `home` TCP 为原点加上相对偏移，并以世界坐标记录。每段用五次插值限速，段端的参考速度和加速度均为零。终点保持 `1.3 s`。下表给出世界坐标和实测三维位置误差。

| 仿真时间（s） | 动作 | 目标 XYZ（m，四位小数） | 当时误差（mm） |
| ---: | --- | --- | ---: |
| 0.0 | 初始姿态 | `(0.5545, 0.0000, 0.5211)` | `0.000` |
| 2.2 | 水平移动 | `(0.5995, 0.1200, 0.5211)` | `0.044` |
| 4.2 | 沿 Z 下降 `0.13 m` | `(0.5995, 0.1200, 0.3911)` | `0.061` |
| 6.2 | 沿 Z 上升 `0.13 m` | `(0.5995, 0.1200, 0.5211)` | `0.051` |
| 8.7 | 抵达第二目标 | `(0.4645, 0.2000, 0.5761)` | `0.043` |
| 10.0 | 保持结束 | `(0.4645, 0.2000, 0.5761)` | `< 0.001` |

## 逆运动学与动力学跟踪

1. 用 `mj_jac` 计算夹爪 TCP 的世界坐标平移与角速度 Jacobian，保留七个机械臂自由度的列。拼接 XYZ 误差和固定方向误差后，用 `Δq = Jᵀ (J Jᵀ + λ² I)⁻¹ e` 求更新量，阻尼 `λ = 0.015`。逐点以上一解为初值，用限位裁剪与回溯减少姿态跳变，并预留 `0.05 rad` 的关节限位裕量。[MuJoCo Jacobian](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-jac)
2. IK 在独立的 `MjData` 上写候选角度、计算运动学和接触。以 `0.002 s` 间隔检查整条路径的 IK 收敛性与模型接触，并在动力学运行中再次检查接触。检测到模型碰撞几何穿透超过 `1 µm` 时终止。可达性结论适用于给定的方向约束与采样路径。碰撞检查覆盖当前场景已启用的碰撞几何和采样时刻。
3. 关节控制器使用 IK 参考角与速度做 PD 跟踪，并加入模型的 `qfrc_bias` 前馈。由已核实的执行器关系 `τ = Kp × (ctrl − q) − Kd × q̇` 反算 `data.ctrl`，先限制力矩指令，再限制输入。规划峰值上限为 `0.8 rad/s`、`1.8 rad/s²`，动力学运行监测上限为 `1.0 rad/s`、`2.5 rad/s²`。运动状态由 `mj_step` 更新，随后用 `mj_forward` 刷新新状态对应的 TCP 和接触。[MuJoCo 执行器力学](https://mujoco.readthedocs.io/en/stable/computation/index.html#force-generation)
4. Viewer 使用独立的显示数据。复制状态和写入 `user_scn` 时持有 Viewer 锁，随后调用 `sync(state_only=True)`。青色线和圆点是目标路径及路径点，橙色线是逐帧累积的实测轨迹。[MuJoCo Python Viewer](https://mujoco.readthedocs.io/en/stable/python.html)

## 验证与输出

```powershell
.\.venv\Scripts\python.exe .\src\panda_cartesian.py
.\.venv\Scripts\python.exe .\src\panda_cartesian.py --viewer
.\.venv\Scripts\python.exe -m pytest -q
```

无窗口和 Viewer 各完成 `5,000` 个物理步、记录 `5,001` 个点，两份 CSV 的 SHA-256 一致。实测最大三维误差为 `0.162 mm`，均方根误差为 `0.093 mm`。七个关节的角度变化范围为 `0.039–0.416 rad`，最大实测关节速度和加速度分别为 `0.378 rad/s` 和 `0.703 rad/s²`。运行中未触发接触、限位或非有限状态报警。

最终复跑结果为 `14 passed in 6.29s`。自动测试覆盖不可达点、碰撞拒绝、关节速度和加速度、力矩限幅、多轴跟踪、CSV 和图像导出。端到端测试在每次 `mj_step` 前核对 `qpos` 与上一物理步结果相同。碰撞用例初次运行报 `Failed: DID NOT RAISE RuntimeError`。诊断发现修改已编译地面的局部 `pos` 后，实际世界坐标仍为 `[0, 0, 0]`。测试改用重新编译的场景，在夹爪附近放置球形障碍，实际检测到 `world / link6` 穿透 `0.009802 m` 并拒绝规划。

CSV 包含目标和实测 XYZ、三轴误差、三维误差范数、七关节目标与实测角、实测关节速度及上一个物理步的执行器力矩。两个 PNG 分别对照 XYZ 跟踪误差和七关节目标与实测角度。[CSV](../outputs/panda_cartesian_tracking.csv) · [XYZ 误差图](../outputs/figures/panda_cartesian_xyz_error.png) · [七关节角度图](../outputs/figures/panda_cartesian_joint_angles.png)
