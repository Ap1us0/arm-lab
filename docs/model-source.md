# Panda 模型来源

项目使用 Google DeepMind 的 MuJoCo Menagerie 中的 Franka Emika Panda MJCF。上游 README 记录了从公开的 Franka ROS URDF 到 MJCF 的转换过程，并声明该模型采用 Apache-2.0 许可。[上游模型目录](https://github.com/google-deepmind/mujoco_menagerie/tree/370c43965323c0944b433b202aac5058b5f907a9/franka_emika_panda)

| 项目 | 值 |
| --- | --- |
| 上游提交 | `370c43965323c0944b433b202aac5058b5f907a9` |
| Panda 目录的 Git 树 | `386f680ac533760a49cb91933345949ad186b4c7` |
| 本地位置 | `models/franka_emika_panda/` |
| 加载入口 | `scene.xml`，引用 `panda.xml` 及其网格素材 |
| 本地许可 | [Apache-2.0 LICENSE](../models/franka_emika_panda/LICENSE) |

模型文件保留上游原样。上游 README 和许可随模型保留。下载文件按固定提交的 Git 树逐个核对 Git blob 哈希。
