 from pyb import Pin
import time

laser = Pin("P5", Pin.OUT_PP)
laser.high()

try:
    print("P5 HIGH: laser ON")
    while True:
        time.sleep_ms(100)
finally:
    laser.low()  # 程序退出时关闭
