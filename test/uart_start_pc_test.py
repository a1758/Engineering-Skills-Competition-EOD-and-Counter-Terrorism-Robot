# OpenMV H7 Plus: send exactly START to a PC USB-to-TTL adapter.
# UART1: OpenMV P1 TX -> adapter RXD; OpenMV GND -> adapter GND.
# Use 3.3 V TTL logic. OpenMV is independently powered; leave VCC disconnected.
# PC: adapter COM port, 115200 baud, 8 data bits, no parity, 1 stop bit.
# Enable HEX display: each transmission must be 53 54 41 52 54.
# For STM32: P1 TX -> PD9 RX; P0 RX <- PD8 TX; common GND.
# For PC bidirectional test: adapter TXD -> OpenMV P0 RX.
# No camera, servo, or light dependency.
from pyb import UART
import time

UART_ID = 1
UART_BAUD = 115200
START_DELAY_MS = 5000
SEND_INTERVAL_MS = 1000
MESSAGE = b"START"  # exactly five bytes; no spaces or newline


def write_all(uart, data):
    offset = 0
    while offset < len(data):
        sent = uart.write(data[offset:])
        if not sent:
            return False
        offset += sent
    return True


def show_received(uart):
    if not uart.any():
        return
    data = uart.read(64)
    if data:
        # Each read is a chunk; a command can arrive split across chunks.
        print("RX RAW:", repr(data))
        print("RX HEX:", " ".join("%02X" % byte for byte in data))
        try:
            print("RX TEXT:", data.decode("ascii"))
        except UnicodeError:
            print("RX TEXT: non-ASCII; see RAW/HEX")


def run():
    uart = UART(UART_ID, UART_BAUD, bits=8, parity=None, stop=1,
                timeout=20, timeout_char=20, read_buf_len=256)
    print("UART1: P1 TX, P0 RX; 115200 8N1")
    print("Expected TX HEX: 53 54 41 52 54")
    print("Receiving now; first START in", START_DELAY_MS, "ms")
    next_send = time.ticks_add(time.ticks_ms(), START_DELAY_MS)
    sequence = 0
    while True:
        show_received(uart)
        now = time.ticks_ms()
        if time.ticks_diff(now, next_send) >= 0:
            sequence += 1
            sent = write_all(uart, MESSAGE)
            print("TX", sequence, "OK" if sent else "FAILED",
                  "HEX: 53 54 41 52 54")
            next_send = time.ticks_add(time.ticks_ms(), SEND_INTERVAL_MS)
        time.sleep_ms(5)


if __name__ == "__main__":
    run()
