# STM32 PD8 TX -> OpenMV P0 RX
# STM32 PD9 RX <- OpenMV P1 TX
# P9 -> isolated light EN, HIGH=ON (not lamp power).
from pyb import Pin, UART
import time

UART_ID = 1
UART_BAUD = 115200
LIGHT_PIN = 'P9'
REQUEST = b'openmvtest'
REPLY = b'open'
REPLY_TIMEOUT_MS = 1000
REPEAT_DELAY_MS = 1000
LIGHT_OFF_DELAY_MS = 2000
RX_LIMIT = 64


def write_all(uart, data):
    offset = 0
    while offset < len(data):
        sent = uart.write(data[offset:])
        if not sent:
            print('UART TX failed')
            return False
        offset += sent
    return True


def receive_reply(uart):
    received = bytearray()
    started = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), started) < REPLY_TIMEOUT_MS:
        if uart.any():
            chunk = uart.read(RX_LIMIT - len(received))
            if chunk:
                received.extend(chunk)
                # Accept split reception: o + pe + n. Case-sensitive.
                if bytes(received).strip(b'\r\n') == REPLY:
                    return True, bytes(received)
                if len(received) >= RX_LIMIT:
                    break
        time.sleep_ms(5)
    return False, bytes(received)


def run():
    light = Pin(LIGHT_PIN, Pin.OUT_PP)
    light.high()  # stay ON until STM32 replies with open
    uart = UART(UART_ID, UART_BAUD, bits=8, parity=None, stop=1,
                timeout=20, timeout_char=20, read_buf_len=256)
    print('UART1 ready: PD8->P0 RX, PD9<-P1 TX; P9 light starts ON')
    attempt = 0
    try:
        while True:
            # Bounded drain of previous-round data.
            for _ in range(4):
                if not uart.any():
                    break
                uart.read(128)
            attempt += 1
            print('TEST', attempt, 'TX:', REQUEST)
            if write_all(uart, REQUEST):
                passed, received = receive_reply(uart)
            else:
                passed, received = False, b''
            if passed:
                print('PASS RX:', received, 'light OFF in 2 seconds')
                time.sleep_ms(LIGHT_OFF_DELAY_MS)
                light.low()
                print('PASS RX:', received, 'P9 light OFF; handshake complete')
                # One successful handshake, then hold light OFF until stopped.
                while True:
                    time.sleep_ms(100)
            print('FAIL RX:', received, 'P9 light remains ON; retrying')
            time.sleep_ms(REPEAT_DELAY_MS)
    finally:
        light.low()


if __name__ == '__main__':
    run()
