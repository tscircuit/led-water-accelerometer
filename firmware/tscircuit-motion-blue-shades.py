from machine import Pin, Timer, SPI
import neopixel
import time
import math
import struct
import random
import rp2

# ───── LED Setup ─────
LED_PIN = 6
COLS, ROWS = 7, 6          # 7×6 LED matrix
NUM_LEDS = COLS * ROWS
np = neopixel.NeoPixel(Pin(LED_PIN), NUM_LEDS)

# On-board LED for feedback
onboard_led = Pin('LED', Pin.OUT)

# ───── BOOTSEL button globals ─────
last_button_state = 0
current_mode = 0
switch_requested = False
total_presses = 0

def check_bootsel_button(timer):
    """Timer callback to read BOOTSEL and request a mode switch"""
    global last_button_state, current_mode, switch_requested, total_presses

    current_state = rp2.bootsel_button()

    # rising edge = press
    if current_state == 1 and last_button_state == 0:
        total_presses += 1
        current_mode = (current_mode + 1) % 2
        switch_requested = True
        print(f"🎬 BOOTSEL pressed → mode {current_mode}  (press #{total_presses})")
        onboard_led.on()
        show_switch_feedback()

    # falling edge = release
    elif current_state == 0 and last_button_state == 1:
        print("🔄 BOOTSEL released")
        onboard_led.off()

    last_button_state = current_state

def show_switch_feedback():
    """Flash the whole matrix twice when the mode changes"""
    for _ in range(2):
        for i in range(NUM_LEDS):
            np[i] = (10, 10, 10)
        np.write()
        time.sleep(0.1)

        for i in range(NUM_LEDS):
            np[i] = (0, 0, 0)
        np.write()
        time.sleep(0.1)

# ════════════════════════════════════════════════════════════════════════
# ───── Scrolling-text animation (Mode 0) ─────
# ════════════════════════════════════════════════════════════════════════

class NeoPixelMatrix:
    def __init__(self, strip, cols, rows):
        self.np = strip
        self.cols = cols
        self.rows = rows
        self.clear()

    def xy_to_index(self, x, y):
        return y * self.cols + x

    def clear(self):
        for i in range(NUM_LEDS):
            self.np[i] = (0, 0, 0)
        self.np.write()

    def set_pixel(self, x, y, color):
        if 0 <= x < self.cols and 0 <= y < self.rows:
            self.np[self.xy_to_index(x, y)] = color

# Variable-width 3- to 4-pixel glyphs (all 6 rows tall)
LETTER_PATTERNS = {
    'T': [
        [1,1,1],
        [0,1,0],
        [0,1,0],
        [0,1,0],
        [0,1,0],
        [0,1,0],
    ],
    'S': [
        [0,1,1,1],
        [1,0,0,0],
        [0,1,1,0],
        [0,0,0,1],
        [1,0,0,1],
        [0,1,1,0],
    ],
    'C': [
        [0,1,1,1],
        [1,0,0,0],
        [1,0,0,0],
        [1,0,0,0],
        [1,0,0,0],
        [0,1,1,1],
    ],
    'I': [
        [1,1,1],
        [0,1,0],
        [0,1,0],
        [0,1,0],
        [0,1,0],
        [1,1,1],
    ],
    'R': [
        [1,1,1,0],
        [1,0,0,1],
        [1,1,1,0],
        [1,0,1,0],
        [1,0,0,1],
        [1,0,0,1],
    ],
    'U': [
        [1,0,0,1],
        [1,0,0,1],
        [1,0,0,1],
        [1,0,0,1],
        [1,0,0,1],
        [0,1,1,0],
    ],
}

# Color palette for each letter - bright and vibrant!
LETTER_COLORS = {
    'T': (50, 0, 0),    # Red
    'S': (50, 25, 0),   # Orange
    'C': (50, 50, 0),   # Yellow
    'I': (0, 50, 0),    # Green
    'R': (0, 0, 50),    # Blue
    'C': (25, 0, 50),   # Purple (second C gets different color)
    'U': (50, 0, 25),   # Magenta
    'I': (0, 50, 25),   # Cyan (second I gets different color)
    'T': (50, 15, 15),  # Pink (second T gets different color)
}

# Different shades of blue for each letter
def get_letter_color(letter, position):
    """Get color for a letter based on its position in the text - various shades of blue"""
    blue_shades = [
        (10, 20, 50),   # Deep blue - T
        (5, 15, 40),    # Dark blue - S
        (15, 25, 45),   # Medium-dark blue - C
        (20, 30, 50),   # Medium blue - I
        (25, 35, 55),   # Medium-light blue - R
        (15, 30, 40),   # Steel blue - C
        (10, 25, 45),   # Navy blue - U
        (30, 40, 60),   # Light blue - I
        (20, 35, 50),   # Bright blue - T
    ]
    return blue_shades[position % len(blue_shades)]

def draw_letter_to_buffer(buf, letter, start_x, start_y, color):
    pat = LETTER_PATTERNS.get(letter)
    if not pat:
        return
    for y, row in enumerate(pat):
        for x, px in enumerate(row):
            if px:
                sx, sy = start_x + x, start_y + y
                if 0 <= sx < COLS and 0 <= sy < ROWS:
                    buf[sy][sx] = color

def update_matrix_from_buffer(matrix, buf):
    for y in range(ROWS):
        for x in range(COLS):
            matrix.set_pixel(x, y, buf[y][x] if buf[y][x] else (0, 0, 0))
    matrix.np.write()

def scroll_text_continuous_animation():
    """Mode 0: Scroll 'TSCIRCUIT' with variable-width glyphs and different colors"""
    global switch_requested

    matrix = NeoPixelMatrix(np, COLS, ROWS)
    text = "TSCIRCUIT"
    speed = 0.1
    letter_spacing = 1

    print("🎬 Mode 0 → Blue shades scrolling text")

    while not switch_requested:
        # compute full marquee width
        total_width = (
            sum(len(LETTER_PATTERNS[c][0]) for c in text)
            + (len(text) - 1) * letter_spacing
        )
        start_x = COLS
        end_x = -total_width
        pos = float(start_x)

        frame = [[0 for _ in range(COLS)] for _ in range(ROWS)]

        while pos > end_x and not switch_requested:
            # clear buffer
            for y in range(ROWS):
                for x in range(COLS):
                    frame[y][x] = 0

            # draw each glyph at its current x-offset with unique color
            cx = pos
            for i, ch in enumerate(text):
                color = get_letter_color(ch, i)
                draw_letter_to_buffer(frame, ch, int(cx), 0, color)
                cx += len(LETTER_PATTERNS[ch][0]) + letter_spacing

            update_matrix_from_buffer(matrix, frame)
            pos -= 1.0
            time.sleep(speed)

        if not switch_requested:
            time.sleep(0.2)

# ════════════════════════════════════════════════════════════════════════
# ───── Particle-physics animation (Mode 1) … unchanged ─────
# ════════════════════════════════════════════════════════════════════════

MAX_LEVEL = 50
COLOR_CH = (0, 0, 1)        # blue particles
PARTICLE_COUNT = 25
PARTICLE_RADIUS = 0.5

SCK_PIN, MOSI_PIN, MISO_PIN, CS_PIN = 10, 11, 12, 17

def init_accelerometer():
    spi = SPI(1, baudrate=1_000_000, polarity=1, phase=1,
              sck=Pin(SCK_PIN), mosi=Pin(MOSI_PIN), miso=Pin(MISO_PIN))
    cs = Pin(CS_PIN, Pin.OUT, value=1)

    def _w(addr, val):
        cs(0); spi.write(bytearray([addr & 0x3F, val & 0xFF])); cs(1)

    def _r(addr, n=1):
        cmd = addr | 0x80 | (0x40 if n > 1 else 0)
        cs(0); spi.write(bytearray([cmd])); data = spi.read(n); cs(1); return data

    try:
        if _r(0x0F)[0] != 0x33:
            raise RuntimeError("No LIS3DHTR")
        _w(0x20, 0x57)
        _w(0x23, 0x08)
        print("✅ Accelerometer OK")
        return spi, cs, _w, _r
    except:
        print("⚠️  Accelerometer not found, simulating gravity")
        return None, None, None, None

class Particle:
    __slots__ = ("x", "y", "vx", "vy")
    def __init__(self):
        self.x = random.uniform(1, COLS - 1)
        self.y = random.uniform(1, ROWS - 1)
        self.vx = random.uniform(-0.2, 0.2)
        self.vy = random.uniform(-0.2, 0.2)

    def update(self, gx, gy, buckets):
        self.vx += gx * 0.2
        self.vy += gy * 0.2
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

        gx_i, gy_i = int(self.x), int(self.y)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                nx, ny = gx_i + dx, gy_i + dy
                if 0 <= nx < COLS and 0 <= ny < ROWS:
                    for other in buckets[ny][nx]:
                        if other is self:
                            continue
                        dx_d = other.x - self.x
                        dy_d = other.y - self.y
                        dist_sq = dx_d*dx_d + dy_d*dy_d
                        min_d = PARTICLE_RADIUS * 2
                        if 0.01 < dist_sq < min_d * min_d:
                            dist = math.sqrt(dist_sq)
                            if dist > 0:
                                overlap = min_d - dist
                                nxn, nyn = dx_d / dist, dy_d / dist
                                self.x -= nxn * overlap * 0.5
                                self.y -= nyn * overlap * 0.5
                                self.vx -= nxn * 0.1
                                self.vy -= nyn * 0.1

def update_leds(gx, gy, particles, brt, buckets):
    # clear brightness + buckets
    for y in range(ROWS):
        for x in range(COLS):
            brt[y][x] = 0.0
            buckets[y][x].clear()

    # assign to buckets
    for p in particles:
        bx, by = int(p.x), int(p.y)
        if 0 <= bx < COLS and 0 <= by < ROWS:
            buckets[by][bx].append(p)

    # physics update
    for p in particles:
        p.update(gx, gy, buckets)

    # LED influence
    for p in particles:
        gx_i, gy_i = int(p.x), int(p.y)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                nx, ny = gx_i + dx, gy_i + dy
                if 0 <= nx < COLS and 0 <= ny < ROWS:
                    dx_d = abs(p.x - (nx + 0.5))
                    dy_d = abs(p.y - (ny + 0.5))
                    if dx_d < 1.2 and dy_d < 1.2:
                        influence = max(0, 1.0 - (dx_d + dy_d) * 0.7)
                        brt[ny][nx] += influence * 1.5

    # push to strip
    for y in range(ROWS):
        for x in range(COLS):
            idx = y * COLS + x
            lvl = int(min(brt[y][x] * 6, MAX_LEVEL))
            np[idx] = (lvl * COLOR_CH[0],
                       lvl * COLOR_CH[1],
                       lvl * COLOR_CH[2])
    np.write()

def particle_physics_animation():
    """Mode 1: bouncing particles affected by gravity / accelerometer"""
    global switch_requested

    print("🎬 Mode 1 → Particle physics")
    spi, cs, _w, _r = init_accelerometer()

    particles = [Particle() for _ in range(PARTICLE_COUNT)]
    brightness = [[0.0 for _ in range(COLS)] for _ in range(ROWS)]
    buckets = [[[] for _ in range(COLS)] for _ in range(ROWS)]

    LSB_G = 0.00098
    G_CLAMP = 15
    sim_time = 0.0

    while not switch_requested:
        if spi:
            try:
                raw = _r(0x28, 6)
                xr, yr, zr = struct.unpack("<hhh", raw)
                ax = max(min(xr * LSB_G, G_CLAMP), -G_CLAMP)
                ay = max(min(yr * LSB_G, G_CLAMP), -G_CLAMP)
                az = zr * LSB_G
                mag = math.sqrt(ax*ax + ay*ay)
                if mag > 0.05:
                    gx, gy = ax / mag, -ay / mag
                else:
                    gx, gy = 0.0, 1.0 if az > 0 else -1.0
            except:
                gx = 0.0
                gy = 1.0
        else:
            # simulated gravity sweeps around slowly
            sim_time += 0.1
            gx = math.sin(sim_time * 0.3) * 0.5
            gy = math.cos(sim_time * 0.2) * 0.5

        update_leds(gx, gy, particles, brightness, buckets)
        # unthrottled for max FPS

# ════════════════════════════════════════════════════════════════════════
# ───── Main program & mode switching ─────
# ════════════════════════════════════════════════════════════════════════

def main():
    global switch_requested

    # poll BOOTSEL @ 50 Hz
    btn_timer = Timer()
    btn_timer.init(freq=50, mode=Timer.PERIODIC, callback=check_bootsel_button)

    animations = [scroll_text_continuous_animation,
                  particle_physics_animation]
    names = ["Blue Shades Scrolling Text", "Particle Physics"]

    print("\n🎮 BOOTSEL animation switcher")
    print("Press BOOTSEL to cycle modes")
    print("-" * 40)

    try:
        while True:
            print(f"\n>>> starting mode {current_mode}: {names[current_mode]}")
            switch_requested = False
            animations[current_mode]()
            time.sleep(0.1)   # small gap between modes
    except KeyboardInterrupt:
        print("\n🛑 stopped by user  |  total BOOTSEL presses:", total_presses)
    finally:
        # clear LEDs & tidy up
        for i in range(NUM_LEDS):
            np[i] = (0, 0, 0)
        np.write()
        onboard_led.off()
        btn_timer.deinit()
        print("✅ cleanup done")

if __name__ == "__main__":
    main()
