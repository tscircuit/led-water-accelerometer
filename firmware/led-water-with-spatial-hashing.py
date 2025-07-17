from machine import Pin, SPI
import math, time, struct, neopixel, random

# ───── LED Setup ─────
LED_PIN = 6
COLS, ROWS = 7, 6
NUM_LEDS = COLS * ROWS
np = neopixel.NeoPixel(Pin(LED_PIN), NUM_LEDS)

MAX_LEVEL = 50
COLOR_CH = (0, 0, 1)
PARTICLE_COUNT = 25
PARTICLE_RADIUS = 0.5

# ───── Accelerometer (LIS3DHTR) ─────
SCK_PIN, MOSI_PIN, MISO_PIN, CS_PIN = 10, 11, 12, 17
spi = SPI(1, baudrate=1_000_000, polarity=1, phase=1,
          sck=Pin(SCK_PIN), mosi=Pin(MOSI_PIN), miso=Pin(MISO_PIN))
cs = Pin(CS_PIN, Pin.OUT, value=1)

def _w(addr, val):
    cs(0); spi.write(bytearray([addr & 0x3F, val & 0xFF])); cs(1)

def _r(addr, n=1):
    cmd = addr | 0x80 | (0x40 if n > 1 else 0)
    cs(0); spi.write(bytearray([cmd])); data = spi.read(n); cs(1); return data

if _r(0x0F)[0] != 0x33:
    raise RuntimeError("No LIS3DHTR")
_w(0x20, 0x57)
_w(0x23, 0x08)

LSB_G = 0.00098
G_CLAMP = 15

# ───── Particle ─────
class Particle:
    __slots__ = ("x", "y", "vx", "vy")
    def __init__(self):
        self.x = random.uniform(1, COLS - 1)
        self.y = random.uniform(1, ROWS - 1)
        self.vx = random.uniform(-0.2, 0.2)
        self.vy = random.uniform(-0.2, 0.2)

    def update(self, gravity_x, gravity_y, buckets):
        self.vx += gravity_x * 0.2
        self.vy += gravity_y * 0.2

        self.x += self.vx
        self.y += self.vy

        if self.x < PARTICLE_RADIUS or self.x > COLS - PARTICLE_RADIUS:
            self.vx *= -0.7
            self.x = max(PARTICLE_RADIUS, min(COLS - PARTICLE_RADIUS, self.x))
        if self.y < PARTICLE_RADIUS or self.y > ROWS - PARTICLE_RADIUS:
            self.vy *= -0.7
            self.y = max(PARTICLE_RADIUS, min(ROWS - PARTICLE_RADIUS, self.y))

        self.vx *= 0.92
        self.vy *= 0.92

        gx, gy = int(self.x), int(self.y)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                nx, ny = int(gx + dx), int(gy + dy)
                if 0 <= nx < COLS and 0 <= ny < ROWS:
                    for other in buckets[ny][nx]:
                        if other is self:
                            continue
                        dx = other.x - self.x
                        dy = other.y - self.y
                        dist_sq = dx*dx + dy*dy
                        min_dist = PARTICLE_RADIUS * 2
                        if 0.01 < dist_sq < min_dist * min_dist:
                            dist = math.sqrt(dist_sq)
                            if dist > 0:
                                overlap = min_dist - dist
                                nxn = dx / dist
                                nyn = dy / dist
                                self.x -= nxn * overlap * 0.5
                                self.y -= nyn * overlap * 0.5
                                self.vx -= nxn * 0.1
                                self.vy -= nyn * 0.1

# ───── Global State ─────
particles = [Particle() for _ in range(PARTICLE_COUNT)]
brightness = [[0.0 for _ in range(COLS)] for _ in range(ROWS)]
buckets = [[[] for _ in range(COLS)] for _ in range(ROWS)]

# ───── LED Update ─────
def update_leds(gx, gy):
    # Clear brightness and buckets
    for y in range(ROWS):
        row_b = brightness[y]
        row_buck = buckets[y]
        for x in range(COLS):
            row_b[x] = 0.0
            row_buck[x].clear()

    # Assign particles to buckets
    for p in particles:
        px, py = int(p.x), int(p.y)
        if 0 <= px < COLS and 0 <= py < ROWS:
            buckets[py][px].append(p)

    # Update all particles
    for p in particles:
        p.update(gx, gy, buckets)

    # Influence LEDs
    for p in particles:
        gx, gy = int(p.x), int(p.y)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                nx, ny = gx + dx, gy + dy
                if 0 <= nx < COLS and 0 <= ny < ROWS:
                    dist_x = abs(p.x - (nx + 0.5))
                    dist_y = abs(p.y - (ny + 0.5))
                    if dist_x < 1.2 and dist_y < 1.2:
                        influence = max(0, 1.0 - (dist_x + dist_y) * 0.7)
                        brightness[ny][nx] += influence * 1.5

    # LED write
    for y in range(ROWS):
        for x in range(COLS):
            idx = y * COLS + x
            lvl = int(min(brightness[y][x] * 6, MAX_LEVEL))
            np[idx] = (
                lvl * COLOR_CH[0],
                lvl * COLOR_CH[1],
                lvl * COLOR_CH[2],
            )
    np.write()

# ───── Main Loop ─────
while True:
    raw = _r(0x28, 6)
    x_raw, y_raw, z_raw = struct.unpack("<hhh", raw)
    ax = max(min(x_raw * LSB_G, G_CLAMP), -G_CLAMP)
    ay = max(min(y_raw * LSB_G, G_CLAMP), -G_CLAMP)
    az = z_raw * LSB_G

    mag = math.sqrt(ax * ax + ay * ay)
    if mag > 0.05:
        gx = ax / mag
        gy = -ay / mag
    else:
        gx = 0.0
        gy = 1.0 if az > 0 else -1.0

    update_leds(gx, gy)
    # time.sleep(0.01)  # ~100 fps if needed

