# k210/main.py
# OV2640 -> JPEG -> SPI1 chunks (12B hdr + payload)
import time, ustruct
import sensor, image
from fpioa_manager import fm
from Maix import GPIO
from machine import SPI

# ---- pins (MaixBit IO) ----
PIN_SCLK = 22
PIN_MISO = 23
PIN_MOSI = 24
PIN_CS = 25
PIN_RDY = 21  # input from ESP32-C3

# ---- stream params ----
JPEG_QUALITY = 50
CHUNK_PAYLOAD = 1400  # <= 2048 is safe
SPI_BAUD = 20_000_000

FLAG_START = 1
FLAG_END = 2


def wait_rdy(timeout_ms=2000):
    t0 = time.ticks_ms()
    while True:
        if rdy.value() == 1:
            return True
        if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms:
            return False
        time.sleep_ms(1)


# ---- map SPI1 via FPIOA ----
fm.register(PIN_CS, fm.fpioa.SPI1_SS0)
fm.register(PIN_SCLK, fm.fpioa.SPI1_SCLK)
fm.register(PIN_MOSI, fm.fpioa.SPI1_D0)
fm.register(PIN_MISO, fm.fpioa.SPI1_D1)

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

print("[k210/main.py] ready: SPI1 baud=%d, CHUNK=%d" % (SPI_BAUD, CHUNK_PAYLOAD))

frame_id = 0
while True:
    img = sensor.snapshot()
    jpeg = img.compress(quality=JPEG_QUALITY)
    total = len(jpeg)

    chunk_id = 0
    off = 0
    while off < total:
        payload = jpeg[off : off + CHUNK_PAYLOAD]
        off += len(payload)

        flags = 0
        if chunk_id == 0:
            flags |= FLAG_START
        if off >= total:
            flags |= FLAG_END

        # hdr: frame_id(u32), chunk_id(u16), flags(u8), rsv(u8), payload_len(u16) = 12 bytes
        hdr = ustruct.pack("<IHB BH", frame_id, chunk_id, flags, 0, len(payload))
        # (note: "<IHB BH" is 4 + 2 + 1 + 1 + 2 = 10? No. Safer: pack explicitly)
        hdr = ustruct.pack("<I H B B H", frame_id, chunk_id, flags, 0, len(payload))

        if not wait_rdy(2000):
            print(
                "[k210/main.py] RDY timeout frame_id=%d chunk_id=%d"
                % (frame_id, chunk_id)
            )
            break

        spi.write(hdr)
        spi.write(payload)
        chunk_id += 1

    print(
        "[k210/main.py] sent frame_id=%d bytes=%d chunks=%d"
        % (frame_id, total, chunk_id)
    )
    frame_id += 1
    time.sleep_ms(50)
