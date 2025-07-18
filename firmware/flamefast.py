#####################################################################
#  optimised_flame.py – same API, faster with @micropython.native   #
#  Modified: LED brightness now scales with local heat density      #
#####################################################################
from machine import Pin, SPI
import micropython, neopixel, time, struct, math, random

micropython.alloc_emergency_exception_buf(100)

# ───────── Constants ─────────
micropython.const(1)
LED_PIN   = const(6)
NUM_LEDS  = const(42)
COLS      = const(7)
ROWS      = const(6)

PARTICLE_COUNT   = const(12)
TURBULENCE       = 50.0
LIFETIME         = const(18)
HEAT_INTENSITY   = 0.3
FLAME_WIDTH      = 2.0
SPAWN_RATE       = const(1)           # integer → simpler native loop
GRAVITY_STRENGTH = 0.8

LSB_G   = 0.00098
G_CLAMP = 15

# ───────── LED & geometry setup (unchanged) ─────────
np = neopixel.NeoPixel(Pin(LED_PIN), NUM_LEDS)
x_off = (COLS - 1) / 2
y_off = (ROWS - 1) / 2
coords = tuple((c - x_off, y_off - r) for idx in range(NUM_LEDS)
               for r, c in (divmod(idx, COLS),))

# ───────── LIS3DHTR setup (unchanged) ─────────
SCK_PIN, MOSI_PIN, MISO_PIN, CS_PIN = 10, 11, 12, 17
spi = SPI(1, 1_000_000, polarity=1, phase=1,
          sck=Pin(SCK_PIN), mosi=Pin(MOSI_PIN), miso=Pin(MISO_PIN))
cs  = Pin(CS_PIN, Pin.OUT, value=1)

def _w(a, v):
    cs(0); spi.write(bytearray([a & 0x3F, v & 0xFF])); cs(1)

def _r(a, n=1):
    cmd = a | 0x80 | (0x40 if n > 1 else 0)
    cs(0); spi.write(bytearray([cmd])); d = spi.read(n); cs(1); return d

if _r(0x0F)[0] != 0x33:
    raise RuntimeError("No LIS3DHTR")
_w(0x20, 0x57)               # 100 Hz, all axes
_w(0x23, 0x08)               # hi-res, ±2 g

# ───────── Particle class ─────────
class Particle:
    __slots__ = ("x","y","z","vx","vy","vz","life","max_life","temperature")
    def __init__(self, x, y, gx, gy):
        self.x = x
        self.y = y
        self.z = (random.random()-0.5)*0.5

        s = 0.05 + random.random()*0.08
        self.vx = gx*s + (random.random()-0.5)*0.02
        self.vy = gy*s + (random.random()-0.5)*0.02
        self.vz = (random.random()-0.5)*0.01

        self.life = random.randint(LIFETIME//2, LIFETIME)
        self.max_life = self.life
        self.temperature = random.random()*0.5 + 0.5

    __temperature_map = (
        (0.25, (255,150,100)),
            (0.15, (255,100,0)),
        (0.15, (255,150,0)),
        (0.0, (255,  15, 0)),
    )

    @micropython.native
    def get_color(self):
        # Look‑up instead of nested if/elif increases speed
        tr = self.temperature
        for t,(r,g,b) in self.__temperature_map:
            if tr > t:
                return r, g, b
        return 0, 0, 0  # fallback – should not occur

_particles = []          # global list – keep name short for native code

# ───────── native helpers ─────────
@micropython.native
def _spawn_particle(gx: float, gy: float):
    """Return *new* Particle spawned opposite gravity vector."""
    d = 4.0
    return Particle(gx*d + (random.random()-0.5)*FLAME_WIDTH,
                    gy*d + (random.random()-0.5)*FLAME_WIDTH*0.5,
                    gx, gy)

@micropython.native
def _update_particles(gx: float, gy: float):
    """Maintain list _particles in‑place, spawn & update."""
    plist = _particles
    i = 0
    # remove dead – faster than list‑comp in native
    while i < len(plist):
        if plist[i].life <= 0:
            plist.pop(i)
        else:
            i += 1
    # spawn
    if len(plist) < PARTICLE_COUNT:
        for _ in range(SPAWN_RATE):
            plist.append(_spawn_particle(gx, gy))
            if len(plist) >= PARTICLE_COUNT:
                break
    # update
    for p in plist:
        # gravity
        p.vx -= gx*GRAVITY_STRENGTH*0.1
        p.vy -= gy*GRAVITY_STRENGTH*0.1
        # turbulence
        p.vx += (random.random()-0.5)*TURBULENCE*0.001
        p.vy += (random.random()-0.5)*TURBULENCE*0.001
        p.vz += (random.random()-0.5)*TURBULENCE*0.0005
        # integrate
        p.x += p.vx; p.y += p.vy; p.z += p.vz
        p.vx *= 0.95; p.vy *= 0.95; p.vz *= 0.92
        p.life -= 1
        p.temperature = (p.life / p.max_life)*HEAT_INTENSITY

# ───────── LED colour helper ─────────
#   ‼️  Updated: brightness now scales with local heat density
@micropython.native
def _led_color_for(x: float, y: float):
    """Blend contribution of nearby particles and scale brightness.

    The cumulative heat density (tot_i) is used as the brightness
    factor (0‥1).  Hue is taken from the average particle colour so
    hue stays stable while brightness varies with the flame’s vigour.
    """
    tot_i = r_acc = g_acc = b_acc = 0.0
    for p in _particles:
        dx = p.x - x; dy = p.y - y; dz = p.z
        dist2 = dx*dx + dy*dy + dz*dz
        if dist2 < 4.0:                  # 2.0**2
            inv = 1.0 - dist2*0.25       # 1 - dist²/4 (0‥1)
            life_ratio = p.life / p.max_life
            weight = inv * life_ratio    # heat contribution (0‥1)
            r, g, b = p.get_color()      # ints 0‑255
            tot_i += weight
            r_acc += r * weight
            g_acc += g * weight
            b_acc += b * weight

    if tot_i <= 0.0:
        return 0, 0, 0

    # Base colour (average of contributing particles)
    r_base = r_acc / tot_i
    g_base = g_acc / tot_i
    b_base = b_acc / tot_i

    # Brightness scales with heat density (clamp ≤1)
    brightness = (tot_i if tot_i < 1.0 else 1.0) * 0.3

    r_val = int(r_base * brightness)
    g_val = int(g_base * brightness)
    b_val = int(b_base * brightness)

    # Ensure values stay within 0‑255
    if r_val > 255: r_val = 255
    if g_val > 255: g_val = 255
    if b_val > 255: b_val = 255

    return r_val, g_val, b_val

# ───────── High‑level update (native) ─────────
@micropython.native
def update(gx: float, gy: float, gz: float):
    """Public: advance simulation one frame & push to LEDs."""
    # normalise g vector in‑plane
    mag = (gx*gx + gy*gy) ** 0.5
    if mag < 0.05:
        gy =  1.0 if gz > 0 else -1.0
        gx = 0.0
    else:
        inv = 1.0 / mag
        gx *= inv; gy *= inv

    _update_particles(gx, gy)

    # paint LEDs
    for i, (x, y) in enumerate(coords):
        np[i] = _led_color_for(x, y)
    np.write()

# ───────── Main loop ─────────
print("Starting flame simulation (native)…")
while True:
    try:
        raw = _r(0x28, 6)
        ax, ay, az = struct.unpack("<hhh", raw)
        ax = max(min(ax * LSB_G, G_CLAMP), -G_CLAMP)
        ay = max(min(ay * LSB_G, G_CLAMP), -G_CLAMP)
        update(ax, ay, az * LSB_G)

    except Exception as e:
        print("err:", e)
        time.sleep(0.1)

