# XPBD Bone Baker 分发说明

## 给普通使用者

发送整个 `xpbd-baker-javafx-windows.zip`。使用者解压后双击 `run.bat`，不要只拿出 JAR，也不要删除 `lib` 文件夹。

运行条件：Windows x64、64 位 Java 17 或更新版本。普通用户只需要 JRE/runtime，源码构建才要求带 `javac.exe` 的 JDK。`run.bat` 会自动检查发布包内 runtime、`JAVA_HOME`、`PATH`、Windows 注册表和常见 Java 安装目录，并拒绝旧版本或非 x64 Java；用户通常不必手动配置 `JAVA_HOME`。发布包不包含 GLFW、Maven 或模型资源，但会在 `lib` 中携带 gdx-bullet desktop JNI/native JAR；不能删除或只复制主 JAR。

模型与动画由使用者在界面内选择，不必固定放在程序目录。Bedrock 模型 JSON 与对应动画 JSON 需要一起提供给实际调试模型的使用者。

当前版本包含两种正式求解模式，并共用显式身体碰撞根：默认 XPBD 以物理骨 pivot 和所选动画 cube 六平面碰撞；Bullet 刚体模式把每根物理骨自己的 cubes 编译为 compound body，并支持宽面接触、摩擦、弹性、CCD、运动学 sweep 与父子 6DoF spring joint。碰撞根为空时不创建身体障碍物；两种模式当前均不提供三角网格或地面。

刚体后端现已接入 UI、实时预览、正式烘焙与 Bedrock animation JSON 导出。完整烘焙、最终碰撞审计和大文件导入/导出均在可取消的后台任务运行。外层仍按 60Hz 录制并吸附到原动画推断出的公共帧率栅格，默认内部以 2 个子步固定 120Hz 求解；XPBD 保持默认以兼容旧模型。隔离的 `xpbd.rigidbody.RigidBodySpike` 继续作为底层诊断入口。第三方依赖清单位于 `THIRD_PARTY_NOTICES.txt`，完整许可文本位于 `licenses/`。

## 给开发者

如果需要共享可继续修改的源码，请发送以下内容：

- `src/`
- `pom.xml`
- `mvnw.cmd`
- `.mvn/`
- `build.bat`、`run.bat`
- `docs/VELOCITY_CACHE.md`、`docs/GENERIC_MODEL_GUIDE.md`、`docs/DISTRIBUTION_GUIDE.md`
- `distribution/`（发布脚本、第三方声明与许可证）

不需要发送 `target/`、`analysis/`、`.planning/`、IDE 配置、日志、Agent 工作记录或本机 Maven 缓存。开发者需要 64 位 JDK 17+；只有 JRE 时构建脚本会明确提示缺少 `javac.exe`。`build.bat` 会先运行物理、碰撞和 Windows 启动矩阵回归，再在 `target/release` 生成包含 gdx-bullet desktop natives 的完整 JavaFX 发布目录；也可直接用 `mvnw.cmd test` 做快速验证。
