# k210/main.py
# OV2640 -> JPEG -> SPI1 chunks (10B hdr + payload)
# Header format (little-endian, 10 bytes):
#   <I H B B H
#   frame_id(u32), chunk_id(u16), flags(u8), rsv(u8), payload_len(u16)
#
# Handshake:
#   - Wait RDY=1 before sending header
#   - Send header (one SPI transaction)
#   - Wait RDY=1 before sending payload
#   - Send payload (one SPI transaction)
#
# Notes:
#   - This matches your ESP32-C3 slave code which does 2 spi_slave_transmit() calls:
#       (1) HDR_LEN bytes, (2) payload_len bytes

import time
import ustruct
import sensor, image
from fpioa_manager import fm
from Maix import GPIO
from machine import SPI

# ---- pins (MaixBit IO, FPIOA) ----
PIN_SCLK = 22
PIN_MISO = 23
PIN_MOSI = 24
PIN_CS   = 25
PIN_RDY  = 21  # input from ESP32-C3 (RDY output)

# ---- stream params ----
JPEG_QUALITY  = 50
CHUNK_PAYLOAD = 1400   # must be <= ESP PAYLOAD_MAX (2048). 1400 is safe
SPI_BAUD      = 20_000_000

FLAG_START = 1
FLAG_END   = 2

HDR_FMT = "<I H B B H"
HDR_LEN = 10


def wait_rdy(timeout_ms=2000):
    """Wait RDY high from ESP32-C3."""
    t0 = time.ticks_ms()
    while True:
        if rdy.value() == 1:
            return True
        if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms:
            return False
        time.sleep_ms(1)


# ---- map SPI1 via FPIOA ----
fm.register(PIN_CS,   fm.fpioa.SPI1_SS0)
fm.register(PIN_SCLK, fm.fpioa.SPI1_SCLK)
fm.register(PIN_MOSI, fm.fpioa.SPI1_D0)  # master out
fm.register(PIN_MISO, fm.fpioa.SPI1_D1)  # master in (not used by this design, but wired)

# RDY pin
fm.register(PIN_RDY, fm.fpioa.GPIOHS0)
rdy = GPIO(GPIO.GPIOHS0, GPIO.IN)

# SPI master
spi = SPI(
    SPI.SPI1,
    mode=SPI.MODE_MASTER,
    baudrate=SPI_BAUD,
    polarity=0,
    phase=0,
    bits=8,
    firstbit=SPI.MSB,
)

# ---- camera ----
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=1200)

print("[k210/main.py] ready: SPI1 baud=%d, CHUNK_PAYLOAD=%d, HDR_LEN=%d"
      % (SPI_BAUD, CHUNK_PAYLOAD, HDR_LEN))

frame_id = 0

while True:
    img = sensor.snapshot()
    jpeg = img.compress(quality=JPEG_QUALITY)
    total = len(jpeg)

    chunk_id = 0
    off = 0

    while off < total:
        payload = jpeg[off: off + CHUNK_PAYLOAD]
        payload_len = len(payload)
        off += payload_len

        flags = 0
        if chunk_id == 0:
            flags |= FLAG_START
        if off >= total:
            flags |= FLAG_END

        # 10-byte header: frame_id(u32), chunk_id(u16), flags(u8), rsv(u8), payload_len(u16)
        hdr = ustruct.pack(HDR_FMT, frame_id, chunk_id, flags, 0, payload_len)

        # ---- send header (transaction #1) ----
        if not wait_rdy(2000):
            print("[k210] RDY timeout before HDR frame=%d chunk=%d (rdy=%d)"
                  % (frame_id, chunk_id, rdy.value()))
            break

        # Optional: print occasionally to prove SPI loop is running
        # if (chunk_id == 0) and ((frame_id % 10) == 0):
        #     print("[k210] send HDR frame=%d total=%d" % (frame_id, total))

        spi.write(hdr)

        # ---- send payload (transaction #2) ----
        if not wait_rdy(2000):
            print("[k210] RDY timeout before PAYLOAD frame=%d chunk=%d (rdy=%d)"
                  % (frame_id, chunk_id, rdy.value()))
            break

        spi.write(payload)

        chunk_id += 1

    print("[k210] sent frame_id=%d bytes=%d chunks=%d"
          % (frame_id, total, chunk_id))

    frame_id += 1
    time.sleep_ms(50)
