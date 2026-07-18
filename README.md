# XPBD Bone Baker

面向 Minecraft Bedrock / Blockbench 骨骼模型的桌面物理动画烘焙工具。使用 JavaFX 构建界面，提供 XPBD 与 Bullet 刚体两种求解方式，可预览物理效果并导出 Bedrock animation JSON。

## 下载与运行

Windows x64 用户可以从 [Releases](https://github.com/ranjitmutum/xpbd_baker/releases/latest) 下载 `XPBD_Baker.zip`。

1. 完整解压 ZIP，不要单独取出主 JAR，也不要删除 `lib` 目录。
2. 安装 64 位 Java 17 或更高版本。
3. 双击 `run.bat` 启动程序。

发布包包含 JavaFX、Gson 和 LibGDX/Bullet 运行依赖，但不包含 JVM。启动脚本会自动查找包内可选的 `runtime`、`JAVA_HOME`、`PATH`、Windows 注册表以及常见 Java 安装目录。

## 主要功能

- 导入 Bedrock/Blockbench 骨骼模型与动画 JSON
- XPBD 粒子约束与 Bullet 刚体物理解算
- 骨骼动画实时预览和后台烘焙
- 碰撞、摩擦、弹性、CCD 与关节参数配置
- 循环动画接缝修正与稳定性分析
- 导出可用于 Bedrock 的 animation JSON
- 支持可取消的长时间导入、烘焙和导出任务

## 环境要求

| 用途 | 要求 |
| --- | --- |
| 运行发布包 | Windows x64、64 位 Java 17+ |
| 源码构建 | Windows x64、64 位 JDK 17+ |

## 从源码构建

在 Windows 命令提示符或 PowerShell 中运行：

```bat
mvnw.cmd clean package
```

构建会运行自动化测试，并在 `target\release` 生成完整运行目录。也可以双击根目录的 `build.bat`。构建完成后，双击 `target\release\run.bat` 启动。

开发环境下可运行根目录的 `run.bat`，或执行：

```bat
mvnw.cmd javafx:run
```

## 项目结构

- `src/main/java`：应用源码
- `src/test`：自动化测试与测试资源
- `distribution`：Windows 启动器、用户指南、第三方声明和许可证
- `docs`：构建、分发、模型兼容和速度缓存文档
- `.mvn`、`mvnw.cmd`：Maven Wrapper

详细使用方法见 [`distribution/XPBD中文使用指南.md`](distribution/XPBD中文使用指南.md)，发布说明见 [`docs/DISTRIBUTION_GUIDE.md`](docs/DISTRIBUTION_GUIDE.md)。

## 鸣谢

- 感谢 MicroCraft 作者提供的 XPBD 物理库。
- 碰撞与刚体计算使用 LibGDX Bullet，桌面界面使用 OpenJFX。

## 许可证

项目源码采用 [Apache License 2.0](LICENSE)。第三方组件及其许可证请参阅 [`distribution/THIRD_PARTY_NOTICES.txt`](distribution/THIRD_PARTY_NOTICES.txt) 和 [`distribution/licenses`](distribution/licenses)。
