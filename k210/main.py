# /flash/main.py
# OV2640 -> JPEG bytes -> SPI1 chunks (10B hdr + payload) -> ESP32-C3
#
# Header format (little-endian, 10 bytes): <I H B B H
#   frame_id(u32), chunk_id(u16), flags(u8), rsv(u8), payload_len(u16)
#
# Handshake:
#   - Wait RDY=1 before sending header
#   - CS low, spi.write(header), CS high
#   - Wait RDY=1 before sending payload
#   - CS low, spi.write(payload), CS high

import time
import ustruct
import sensor, image
from fpioa_manager import fm
from Maix import GPIO
from machine import SPI

# ---- pins (MaixBit IO) ----
PIN_SCLK = 22
PIN_MISO = 23
PIN_MOSI = 24
PIN_CS = 25  # manual CS
PIN_RDY = 21  # input from ESP32-C3 (RDY output)

# ---- stream params ----
JPEG_QUALITY = 50
CHUNK_PAYLOAD = 1400  # <= ESP PAYLOAD_MAX (2048)
SPI_BAUD = 10_000_000  # 先 10MHz，稳定后可提到 20MHz

FLAG_START = 1
FLAG_END = 2

HDR_FMT = "<IHBBH"
HDR_LEN = 10


def wait_rdy(timeout_ms=2000):
    t0 = time.ticks_ms()
    while True:
        if rdy.value() == 1:
            return True
        if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms:
            return False
        time.sleep_ms(1)


# ---- copied/adapted from your MaixDuino WiFi code ----
def _to_bytes_maybe(obj):
    if obj is None:
        return None
    if isinstance(obj, (bytes, bytearray)):
        return obj
    if hasattr(obj, "to_bytes"):
        try:
            b = obj.to_bytes()
            if isinstance(b, (bytes, bytearray)):
                return b
        except Exception:
            pass
    if hasattr(obj, "bytearray"):
        try:
            b = obj.bytearray()
            if isinstance(b, (bytes, bytearray)):
                return b
        except Exception:
            pass
    return None


def jpeg_bytes_from_image(img, quality):
    j = img.compress(quality=quality)
    b = _to_bytes_maybe(j)
    if b and len(b) > 200:
        return b
    try:
        b = bytes(j)
        if isinstance(b, (bytes, bytearray)) and len(b) > 200:
            return b
    except Exception:
        pass
    # 如果还不行，打印类型，便于你进一步对固件 API 做精确适配
    raise Exception("JPEG encode returned non-bytes object: type=%s" % str(type(j)))


# ---- map SPI1 via FPIOA (NO hardware CS) ----
fm.register(PIN_SCLK, fm.fpioa.SPI1_SCLK)
fm.register(PIN_MOSI, fm.fpioa.SPI1_D0)  # master out
fm.register(PIN_MISO, fm.fpioa.SPI1_D1)  # master in

# manual CS as GPIOHS2
fm.register(PIN_CS, fm.fpioa.GPIOHS2)
cs = GPIO(GPIO.GPIOHS2, GPIO.OUT)
cs.value(1)

# RDY input
fm.register(PIN_RDY, fm.fpioa.GPIOHS0)
rdy = GPIO(GPIO.GPIOHS0, GPIO.IN)

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

print(
    "[k210] ready: SPI1 baud=%d CHUNK=%d HDR_LEN=%d (manual CS)"
    % (SPI_BAUD, CHUNK_PAYLOAD, HDR_LEN)
)

frame_id = 0

while True:
    img = sensor.snapshot()

    # ✅ 关键：拿到真正 JPEG bytes（解决你现在 len(Image) 报错）
    jpeg = jpeg_bytes_from_image(img, JPEG_QUALITY)
    total = len(jpeg)

    chunk_id = 0
    off = 0

    while off < total:
        payload = jpeg[off : off + CHUNK_PAYLOAD]
        payload_len = len(payload)
        off += payload_len

        flags = 0
        if chunk_id == 0:
            flags |= FLAG_START
        if off >= total:
            flags |= FLAG_END

        hdr = ustruct.pack(HDR_FMT, frame_id, chunk_id, flags, 0, payload_len)

        # ---- send header ----
        if not wait_rdy(2000):
            print(
                "[k210] RDY timeout before HDR frame=%d chunk=%d (rdy=%d)"
                % (frame_id, chunk_id, rdy.value())
            )
            break

        cs.value(0)
        spi.write(hdr)
        cs.value(1)

        # ---- send payload ----
        if not wait_rdy(2000):
            print(
                "[k210] RDY timeout before PAYLOAD frame=%d chunk=%d (rdy=%d)"
                % (frame_id, chunk_id, rdy.value())
            )
            break

        cs.value(0)
        spi.write(payload)
        cs.value(1)

        chunk_id += 1

    print("[k210] sent frame=%d bytes=%d chunks=%d" % (frame_id, total, chunk_id))
    frame_id += 1
    time.sleep_ms(30)
