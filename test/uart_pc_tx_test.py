# OpenMV H7 Plus -> PC serial assistant via a USB-to-TTL adapter.
# OpenMV P0 (UART1 TX) -> adapter RXD; OpenMV GND -> adapter GND.
# Use 3.3V TTL logic; independently powered OpenMV needs no adapter VCC.
# PC serial assistant: adapter COM port, 115200 8N1, text/ASCII display.
from pyb import UART
import time

UART_ID = 1
UART_BAUD = 115200
SEND_INTERVAL_MS = 1000


def write_all(uart, data):
    offset = 0
    while offset < len(data):
        sent = uart.write(data[offset:])
        if not sent:
            return False
        offset += sent
    return True


uart = UART(UART_ID, UART_BAUD, bits=8, parity=None, stop=1,
            timeout=20, timeout_char=2)
sequence = 0
print('UART1 -> PC: P0 TX, 115200 8N1')
while True:
    sequence += 1
    message = 'OPENMV_TEST,seq=%d,time_ms=%d\r\n' % (sequence, time.ticks_ms())
    sent = write_all(uart, message.encode('ascii'))
    print('TX OK:' if sent else 'TX FAILED:', message.strip())
    time.sleep_ms(SEND_INTERVAL_MS)
