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


# ─── LED update helper ───
def update(gx: float, gy: float, gz: float) -> None:
    """
    gx, gy, gz are gravity components in g‑units (‑1 … +1).
    Project gravity onto the LED plane (x,y) and fill pixels
    whose dot‑product with that vector is positive (below water line).
    """
    # 1. Project gravity onto board plane
    vx, vy = -gx, -gy           # minus sign because we want "down" direction
    mag = (vx * vx + vy * vy) ** 0.5 

    # 2. If board is nearly flat, fake gravity toward the edge selected by gz
    if mag < 0.05:             # ≈ 0.05 g threshold
        # choose +Y (top) when board is upside‑down, else ‑Y (bottom)
        vy = 1.0 if gz > 0 else -1.0
        vx = 0.0
        mag = 1.0

    # 3. Scaling for fade below the water‑line
    sat_step = (MAX_LEVEL / SOFT_WIDTH) / mag

    for i, (x, y) in enumerate(coords):
        depth = -(x * vx + y * vy)      # >0 means “below water‑line”
        if depth <= 0:
            lvl = 0
        else:
            lvl = int(min(depth * sat_step, MAX_LEVEL))

        np[i] = (lvl * COLOR_CH[0],
                 lvl * COLOR_CH[1],
                 lvl * COLOR_CH[2])

    np.write()


# ─── Main loop ───
while True:
    raw = _r(0x28, 6)
    x_raw, y_raw, z_raw = struct.unpack("<hhh", raw)

    # Convert to g‑units and clamp
    ax = max(min(x_raw * LSB_G,  G_CLAMP), -G_CLAMP)
    ay = max(min(y_raw * LSB_G,  G_CLAMP), -G_CLAMP)
    az =            z_raw * LSB_G               # no clamp needed here

    update(ax, ay, az)
    time.sleep(0.05)          # ~20 fps
