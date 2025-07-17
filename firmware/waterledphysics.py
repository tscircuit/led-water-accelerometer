from machine import Pin, SPI
import math, time, struct, neopixel

# ───────── LED-string setup ─────────
LED_PIN   = 6
NUM_LEDS  = 42           # 7 × 6 matrix, row-major
COLS, ROWS = 7, 6

np = neopixel.NeoPixel(Pin(LED_PIN), NUM_LEDS)

# ───── Visual parameters ─────
MAX_LEVEL  = 30          # cap (0-15) → tweak for brightness
SOFT_WIDTH = 1.5         # fade distance (matrix units) under the line
COLOR_CH   = (0, 0, 1)   # per-channel multipliers (R,G,B). (1,0,0) = red only

# ───── Pre-compute LED coordinates (origin at matrix centre) ─────
x_off = (COLS - 1) / 2      # 3
y_off = (ROWS - 1) / 2      # 2.5
coords = []
for idx in range(NUM_LEDS):
    r, c = divmod(idx, COLS)
    coords.append((c - x_off, y_off - r))   # (x, y)

# ───── Accelerometer (LIS3DHTR) setup ─────
SCK_PIN, MOSI_PIN, MISO_PIN, CS_PIN = 10, 11, 12, 17
spi = SPI(1, baudrate=1_000_000, polarity=1, phase=1,
          sck=Pin(SCK_PIN), mosi=Pin(MOSI_PIN), miso=Pin(MISO_PIN))
cs  = Pin(CS_PIN, Pin.OUT, value=1)

def _w(addr, val):                       # write 1 byte
    cs(0); spi.write(bytearray([addr & 0x3F, val & 0xFF])); cs(1)

def _r(addr, n=1):                       # read n bytes
    cmd = addr | 0x80 | (0x40 if n > 1 else 0)
    cs(0); spi.write(bytearray([cmd])); data = spi.read(n); cs(1); return data

if _r(0x0F)[0] != 0x33:
    raise RuntimeError("No LIS3DHTR")
_w(0x20, 0x57)          # ODR 100 Hz, XYZ on
_w(0x23, 0x08)          # hi-res, ±2 g

# ───── Mapping constants ─────
LSB_G    = 0.00098      # 1 mg / LSB  (hi-res ±2 g)
G_CLAMP  = 15           # your observed ±15 range  → ±90 °
DEG_PER  = 90 / G_CLAMP # degrees per accel unit

# ───── LED update helper ─────
def update(angle_rad: float) -> None:
    cos_a, sin_a = math.cos(angle_rad), math.sin(angle_rad)
    sat_step     = MAX_LEVEL / SOFT_WIDTH

    for i, (x, y) in enumerate(coords):
        y_rot = x * sin_a + y * cos_a      # rotate point

        if y_rot >= 0:
            lvl = 0
        else:
            lvl = int(min(-y_rot * sat_step, MAX_LEVEL))

        np[i] = (lvl * COLOR_CH[0],
                 lvl * COLOR_CH[1],
                 lvl * COLOR_CH[2])

    np.write()

# ───── Main loop ─────
while True:
    raw = _r(0x28, 6)
    x_raw, _, _ = struct.unpack("<hhh", raw)
    x_val = x_raw * LSB_G                 # ≈ -15 … +15
    x_val = max(min(x_val,  G_CLAMP), -G_CLAMP)

    angle_rad = -math.radians(x_val * DEG_PER)
    update(angle_rad)

    time.sleep(0.05)                      # ~20 fps
