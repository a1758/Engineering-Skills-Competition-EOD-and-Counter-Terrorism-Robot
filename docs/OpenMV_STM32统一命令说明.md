# OpenMV 与 STM32 统一命令说明

本文档只描述 OpenMV 与 STM32 之间的路线控制命令。所有命令统一使用纯命令词：

- 不包含空格
- 不要求 CR、LF 或 CRLF
- 大小写不敏感
- 如果通信库自动附带换行，STM32 会忽略换行，不重复执行命令

## 接线与串口

使用 USART3，115200、8N1、共地：

- STM32 PD8 TX → OpenMV RX
- STM32 PD9 RX ← OpenMV TX

## 命令流程

### 1. OpenMV 启动路线

OpenMV 发送：

```text
start
```

STM32 接受并锁存路线后发送：

```text
finishstart
```

STM32 同时可能向调试端输出：

```text
0 BUSY ROUTE
```

OpenMV 收到 `finishstart` 后，不要再次发送 `start`。

### 2. 二维码阶段

STM32 完成首段前进 6800 mm 后发送：

```text
qr
```

OpenMV 完成二维码处理后发送：

```text
finishqr
```

STM32 收到 `finishqr` 后继续执行路线。二维码识别失败或状态未知时不要发送 `finishqr`，STM32 会保持停车等待。

### 3. 小球阶段

STM32 完成右转、等待和前进 6100 mm 后发送：

```text
globule
```

OpenMV 完成小球处理后发送：

```text
finishglobule
```

STM32 收到 `finishglobule` 后结束当前测试路线并发送路线完成状态。

## 当前路线顺序

```text
start
↓
finishstart
↓
前进 6800 mm
↓
STM32 发送 qr
↓
OpenMV 发送 finishqr
↓
左移 7200 mm
↓
前进 10600 mm
↓
左移 8300 mm
↓
前进 10350 mm
↓
底盘右转 90°
↓
前进 6100 mm
↓
STM32 发送 globule
↓
OpenMV 发送 finishglobule
```

## 超时和重复命令

STM32 等待 `finishqr` 或 `finishglobule` 最长 30 秒。超时后发送：

```text
ERROR VISION_TIMEOUT
```

收到错误阶段的完成命令、重复完成命令或未知命令时，不会推进路线。路线停止或超时后，需要由调试端发送 `ROUTE_RESET`，再重新发送 `start`。

OpenMV 不需要解析 STM32 的计数输出，也不要等待周期计数来决定是否发送完成命令。视觉处理只根据 `qr` 和 `globule` 两个请求执行。

## OpenMV 伪代码

```python
uart.write(b"start")

while True:
    command = read_command_word()

    if command == "finishstart":
        continue
    elif command == "qr":
        if process_qr():
            uart.write(b"finishqr")
    elif command == "globule":
        if process_globule():
            uart.write(b"finishglobule")
```

`read_command_word()` 应按协议识别完整命令词；如果底层串口驱动使用换行分帧，也可以去除末尾 CR/LF 后再比较。


## 完整任务流程协议

完整比赛流程采用 `ACTIONS_V2` 的 `VCMD/VEVT` 事件协议，具体事件码、模式切换、动作确认和 VIS verdict 以 [任务流程对接.md](任务流程对接.md) 为准。`main.py` 已切换到 ACTIONS_V2 基础实现：OpenMV负责视觉、P7/P8机构和P9激光；STM32负责底盘平移、直行、转向、停车及水平机构动作。

启动阶段仍由 STM32 的路线固件接收 `START` 后开始底盘直行；若使用 ACTIONS_V2 固件，则应按该固件的启动握手实现，不要同时启用旧 `FIXED_ROUTE` 文本握手。
