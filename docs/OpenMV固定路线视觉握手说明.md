# OpenMV 固定路线视觉握手说明

当前 STM32 固定路线先接入两个视觉等待点，供 OpenMV 联调：

1. STM32 完成前进 680 mm 后，通过 USART3 发送 `qr\r\n`。
2. OpenMV 完成二维码处理后，向 STM32 发送 `finishqr\r\n`。
3. STM32 收到正确回复后继续左移 720 mm、前进 1060 mm、左移 830 mm、前进 1035 mm。
4. STM32 执行底盘右转 90°，等待 300 ms，再前进 610 mm。
5. STM32 发送 `globule\r\n`。
6. OpenMV 完成小球相关处理后发送 `finishglobule\r\n`。

当前收到 `finishglobule` 后路线即结束并由 STM32 回复 `0 DONE ROUTE`，后续路线暂未接入。

## 串口

使用 STM32 USART3，115200、8N1、共地。STM32 TX 接 OpenMV RX，STM32 RX 接 OpenMV TX。每条消息以 CR、LF 或 CRLF 结束。

OpenMV 只在收到对应请求后发送对应完成消息。重复发送完成消息不会重复执行路线；过早发送或不匹配的完成消息会被忽略。STM32 等待视觉回复时底盘保持停车。

STM32 等待每个视觉回复最长 30 秒。超时后停止路线并回复 `ERROR VISION_TIMEOUT`；之后需要发送 `ROUTE_RESET`，再重新完成必要的准备后才能再次 `START`。

## OpenMV 侧伪代码

```python
if line == "qr":
    ok = process_qr()
    if ok:
        uart.write(b"finishqr\r\n")

if line == "globule":
    ok = process_globule()
    if ok:
        uart.write(b"finishglobule\r\n")
```

识别失败、状态不确定或仍需重试时不要发送完成消息，让 STM32 保持停车。当前协议没有单独的失败回复和重试命令，后续根据实车效果再扩展。
