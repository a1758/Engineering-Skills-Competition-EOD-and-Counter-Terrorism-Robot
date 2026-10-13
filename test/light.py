from pyb import Pin

light = Pin("P9", Pin.OUT_PP)
light.high()  # 常亮
# light.low()  # 关闭