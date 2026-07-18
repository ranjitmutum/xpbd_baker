# XPBD 速度缓存

YSM/Bedrock 动画 JSON 没有标准的骨骼速度通道。本项目因此把速度写入独立的
`*.velocity.json` sidecar 文件，普通 `*.animation.json` 仍只包含兼容的
position/rotation/scale 数据。

在 GUI 中完成整段模拟后，选择 `File -> Export Velocity Cache`。

## 字段

- `frame_rate`：最终输出采样率。求解器保持固定物理步长；若识别到源动画时间格，输出和速度缓存会一起重采样到该帧率，无法可靠识别时默认 60 Hz。
- `space`：`model`，表示速度处于模型空间。
- `units`：`model_units_per_second`。
- `frames[].time`：相对烘焙动画周期的秒数。
- `frames[].bones.<name>.linear_velocity`：物理骨粒子的 `[x, y, z]` 线速度。

速度缓存是给后续工具、运动模糊、状态续算或二次处理使用的自定义格式，YSM
不会自动读取它。
