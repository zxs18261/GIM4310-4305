# 手搓 / SguanFOC 对照文档索引

本目录对比：

1. **手搓工程**内嵌的 FOC 控制模块（`Application/control`）  
2. **SguanFOC_Library-main** 第三方库（`SguanFOC库文件编写/SguanFOC_Library-main`）

| 文档 | 内容 |
|---|---|
| [01_手搓工程_FOC模块说明.md](01_手搓工程_FOC模块说明.md) | 手搓侧 FOC 文件、模式、接口 |
| [02_SguanFOC_Library说明.md](02_SguanFOC_Library说明.md) | Sguan 库版本树与移植骨架 |
| [03_手搓与SguanFOC_相同点.md](03_手搓与SguanFOC_相同点.md) | 理论与工程共性 |
| [04_手搓与SguanFOC_不同点.md](04_手搓与SguanFOC_不同点.md) | 形态、算法、协议、硬件差异与选型 |

**核心结论**：手搓**未集成** Sguan 源码；两套都是有感 FOC 思路，但调度、SVPWM、调试协议与产品绑定不同，不能当同一库使用。
