# OpenMV 与 STM32 实际联调流程

> 本文以当前 STM32 程序协议为准。修改 STM32 或 OpenMV 后，需要重新核对命令名和动作时序。

## 串口配置

- STM32 端：**USART3**。
- OpenMV 端：可以使用自身的 **UART1** 与 STM32 USART3 相连。
- STM32 USART1 用于底盘驱动板，不能作为本流程串口。
- 参数：`115200 8N1`。

## 总体握手

```text
qr
→ OpenMV 识别二维码
→ finishqr
→ STM32 移动到纠偏段
→ rectify
→ OpenMV 持续发送 VERR
→ finishrectify
→ STM32 执行小球夹取
→ ball
→ finishball
→ down
→ wrong / right
→ （找到后）grab
→ none / one / two
→ STM32 执行搬运动作
→ home
→ finishhome
→ 下一段视觉纠偏和移动
```

## 二维码

二维码为三位数字，例如 `123`。当前 OpenMV 使用第一位选择小球颜色；后两位由比赛任务规则解释。

二维码确认成功后，OpenMV 发送：

```text
finishqr\r\n
```

## 纠偏协议

STM32 发送：

| 命令 | 含义 |
|---|---|
| `rectify` | 开始视觉纠偏 |
| `finishrectify` | 结束视觉纠偏 |

OpenMV 周期发送：

```text
VERR 1 x theta quality age\r\n
```

例如：

```text
VERR 1 -12.5 3.2 80 35
```

STM32 端判定有效的条件：

- `quality >= 50`；
- `age <= 150 ms`；
- 连续超过 1 秒没有有效数据时，停车并报错。

因此 `quality=45` 在 STM32 端属于无效数据。OpenMV 未检测到有效轨道时可发送：

```text
VERR 0 0.0 0.0 0 0
```

## 小球夹取握手

```text
STM32 → ball
OpenMV → finishball
STM32 → down
OpenMV → wrong 或 right
```

- `right`：当前位置找到目标小球。
- `wrong`：当前位置未找到目标小球，STM32 换位后再次发送 `down`。
- 最多发送两次 `wrong`，第三处仍失败时，当前 STM32 尚未处理 `NO_TARGET`。
- 在 `NO_TARGET` 协议尚未加入 STM32 前，OpenMV 不应依赖该消息完成闭环；STM32 会在等待约 30 秒后超时停车。

找到小球后：

```text
STM32 → grab
```

OpenMV 收到 `grab` 后执行实际夹取动作：

1. 夹爪张开到 `140°`；
2. 保持约 `3 秒`；
3. 夹爪闭合到 `100°`；
4. 根据换位次数回复：

```text
none\r\n
one\r\n
two\r\n
```

对应关系：

| 回复 | 小球位置 |
|---|---|
| `none` | 第一个位置找到 |
| `one` | 第一次换位后找到 |
| `two` | 第二次换位后找到 |

## 夹取完成后的流程

收到 `none`、`one` 或 `two` 后，STM32 可根据自身任务逻辑继续处理。本文不规定后续 RACK、LIFT、位移或转向动作。

STM32 完成自身动作后发送：

```text
home
```

OpenMV 完成释放并回复：

```text
finishhome\r\n
```

发送 `finishhome` 后，本次 OpenMV 流程结束，不再自动执行后续纠偏、移动或夹取动作。

## 当前未统一或未实现的消息

- STM32 当前发送的是 `rectify` / `finishrectify`，不是 `correct` / `finishcorrect`。
- STM32 当前不会发送 `stopcorrect` 或 `route_reset`。
- `ROUTE_RESET` 是发给 STM32 的调试命令，不是 STM32 发给 OpenMV 的命令。
- STM32 当前尚未处理 OpenMV 的 `NO_TARGET`。

## 联调顺序

1. 确认 STM32 USART3 与 OpenMV UART1 的 TX/RX 交叉连接，并共地。
2. STM32 发送 `qr`，确认 OpenMV 回复 `finishqr`。
3. STM32 发送 `rectify`，确认收到持续有效的 `VERR`。
4. STM32 发送 `finishrectify`。
5. STM32 发送 `ball`，确认收到 `finishball`。
6. STM32 完成下放和移动后发送 `down`。
7. OpenMV 回复 `right` 或 `wrong`。
8. 找到小球后 STM32 发送 `grab`，确认收到 `none/one/two`。
9. STM32 完成 RACK、LIFT、位移和右转后发送 `home`。
10. 确认收到 `finishhome`，本次流程结束。

