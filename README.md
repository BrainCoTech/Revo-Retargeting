# Revo-Retargeting

这是一个用于 MANUS 手套遥操作 BrainCo Revo 灵巧手的 ROS 2 Humble 仓库。

仓库按硬件型号分支管理。`main` 分支只作为开源入口和项目说明；真正可编译、可运行的 ROS 2 包在对应的硬件分支里。

## 选择分支

| 硬件 | 分支 | 用途 |
| --- | --- | --- |
| Revo2 | `revo2_retargeting` | MANUS 到 Revo2 的遥操作 workspace |
| Revo3 | `revo3_retargeting` | MANUS 到 Revo3 的遥操作 workspace |

克隆仓库后，切到你要使用的手对应的分支：

```bash
git clone https://github.com/BrainCoTech/Revo-Retargeting.git
cd Revo-Retargeting

# Revo2
git checkout revo2_retargeting

# Revo3
git checkout revo3_retargeting
git submodule update --init --recursive
```

然后按照该分支里的 README 继续配置、编译和启动。
