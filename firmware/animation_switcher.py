from machine import Pin, Timer, SPI
import neopixel
import time
import math
import struct
import random
import rp2
import micropython

micropython.alloc_emergency_exception_buf(100)

# ───── LED Setup ─────
LED_PIN = 6
COLS, ROWS = 7, 6
NUM_LEDS = COLS * ROWS
np = neopixel.NeoPixel(Pin(LED_PIN), NUM_LEDS)

# Initialize onboard LED for feedback
onboard_led = Pin('LED', Pin.OUT)

# Global variables for BOOTSEL button handling
last_button_state = 0
current_mode = 0
switch_requested = False
total_presses = 0

# Accelerometer globals
spi, cs, _w, _r = None, None, None, None
accelerometer_ok = False
LSB_G = 0.00098
G_CLAMP = 15

def check_bootsel_button(timer):
    """Timer callback to check BOOTSEL button state"""
    global last_button_state, current_mode, switch_requested, total_presses
    
    current_state = rp2.bootsel_button()
    
    if current_state == 1 and last_button_state == 0:
        total_presses += 1
        current_mode = (current_mode + 1) % 3  # Switch between 0, 1, and 2
        switch_requested = True
        
        print(f"🎬 BOOTSEL pressed! Switching to mode {current_mode} (Press #{total_presses})")
        
        onboard_led.on()
        show_switch_feedback()
        
    elif current_state == 0 and last_button_state == 1:
        print("🔄 BOOTSEL button released")
        onboard_led.off()
    
    last_button_state = current_state

def show_switch_feedback():
    """Visual feedback when switching modes"""
    for _ in range(2):
        for i in range(NUM_LEDS):
            np[i] = (30, 30, 30)
        np.write()
        time.sleep(0.1)
        
        for i in range(NUM_LEDS):
            np[i] = (0, 0, 0)
        np.write()
        time.sleep(0.1)

def init_accelerometer():
    """Initialize accelerometer and set global variables."""
    global spi, cs, _w, _r, accelerometer_ok
    SCK_PIN, MOSI_PIN, MISO_PIN, CS_PIN = 10, 11, 12, 17
    try:
        spi = SPI(1, 1_000_000, polarity=1, phase=1,
                  sck=Pin(SCK_PIN), mosi=Pin(MOSI_PIN), miso=Pin(MISO_PIN))
        cs  = Pin(CS_PIN, Pin.OUT, value=1)
    
        def write_reg(a, v):
            cs(0); spi.write(bytearray([a & 0x3F, v & 0xFF])); cs(1)
    
        def read_reg(a, n=1):
            cmd = a | 0x80 | (0x40 if n > 1 else 0)
            cs(0); spi.write(bytearray([cmd])); d = spi.read(n); cs(1); return d
        
        _w = write_reg
        _r = read_reg
    
        if _r(0x0F)[0] != 0x33:
            raise RuntimeError("No LIS3DHTR")
        _w(0x20, 0x57)
        _w(0x23, 0x08)
        accelerometer_ok = True
        print("✅ Accelerometer initialized successfully")
    except Exception as e:
        print(f"⚠️ Accelerometer not found: {e}")
        accelerometer_ok = False

# ═══════════════════════════════════════════════════════════════════════════════
# ANIMATION 0: SCROLLING TEXT
# ═══════════════════════════════════════════════════════════════════════════════

class NeoPixelMatrix:
    def __init__(self, neopixel_strip, cols, rows):
        self.np = neopixel_strip
        self.cols = cols
        self.rows = rows
        self.clear_matrix()
        
    def xy_to_index(self, x, y):
        return y * self.cols + x
    
    def clear_matrix(self):
        for i in range(NUM_LEDS):
            self.np[i] = (0, 0, 0)
        self.np.write()
    
    def set_pixel(self, x, y, color):
        if 0 <= x < self.cols and 0 <= y < self.rows:
            idx = self.xy_to_index(x, y)
            self.np[idx] = color

LETTER_PATTERNS = {
    'T': [[1,1,1,1,1],[0,0,1,0,0],[0,0,1,0,0],[0,0,1,0,0],[0,0,1,0,0],[0,0,1,0,0]],
    'S': [[0,1,1,1,1],[1,0,0,0,0],[0,1,1,1,0],[0,0,0,0,1],[0,0,0,0,1],[1,1,1,1,0]],
    'C': [[0,1,1,1,1],[1,0,0,0,0],[1,0,0,0,0],[1,0,0,0,0],[1,0,0,0,0],[0,1,1,1,1]],
    'I': [[1,1,1,1,1],[0,0,1,0,0],[0,0,1,0,0],[0,0,1,0,0],[0,0,1,0,0],[1,1,1,1,1]],
    'R': [[1,1,1,1,0],[1,0,0,0,1],[1,1,1,1,0],[1,0,1,0,0],[1,0,0,1,0],[1,0,0,0,1]],
    'U': [[1,0,0,0,1],[1,0,0,0,1],[1,0,0,0,1],[1,0,0,0,1],[1,0,0,0,1],[0,1,1,1,0]]
}

def draw_letter_to_buffer(buffer, letter, start_x, start_y, color):
    if letter not in LETTER_PATTERNS: return
    pattern = LETTER_PATTERNS[letter]
    for row_idx, row_pattern in enumerate(pattern):
        for col_idx, pixel in enumerate(row_pattern):
            if pixel == 1:
                screen_x, screen_y = start_x + col_idx, start_y + row_idx
                if 0 <= screen_x < COLS and 0 <= screen_y < ROWS:
                    buffer[screen_y][screen_x] = color

def update_matrix_from_buffer(matrix, buffer):
    for y in range(ROWS):
        for x in range(COLS):
            matrix.set_pixel(x, y, buffer[y][x] if buffer[y][x] != 0 else (0,0,0))
    matrix.np.write()

def scroll_text_continuous_animation():
    global switch_requested
    matrix = NeoPixelMatrix(np, COLS, ROWS)
    text = "TSCIRCUIT"
    color, speed, letter_spacing = (0, 0, 50), 0.05, 1
    print("🎬 Running Mode 0: Scrolling Text")
    
    while not switch_requested:
        letter_width = 5
        total_width = len(text) * (letter_width + letter_spacing) - letter_spacing
        start_x, end_x = COLS, -total_width
        frame_buffer = [[0 for _ in range(COLS)] for _ in range(ROWS)]
        position = float(start_x)
        
        while position > end_x and not switch_requested:
            for y in range(ROWS):
                for x in range(COLS):
                    frame_buffer[y][x] = 0
            current_x = position
            for char in text:
                if char in LETTER_PATTERNS:
                    draw_letter_to_buffer(frame_buffer, char, int(current_x), 0, color)
                current_x += letter_width + letter_spacing
            update_matrix_from_buffer(matrix, frame_buffer)
            position -= 1.0
            time.sleep(speed)
        
        if not switch_requested:
            time.sleep(0.2)

# ═══════════════════════════════════════════════════════════════════════════════
# ANIMATION 1: PARTICLE PHYSICS (Water)
# ═══════════════════════════════════════════════════════════════════════════════

def particle_physics_animation():
    global switch_requested, accelerometer_ok, _r, LSB_G, G_CLAMP
    
    print("🎬 Running Mode 1: Particle Physics (Water)")

    MAX_LEVEL = 50
    COLOR_CH = (0, 0, 1)
    PARTICLE_COUNT = 25
    PARTICLE_RADIUS = 0.5
    
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
                            if other is self: continue
                            dx_dist = other.x - self.x
                            dy_dist = other.y - self.y
                            dist_sq = dx_dist*dx_dist + dy_dist*dy_dist
                            min_dist = PARTICLE_RADIUS * 2
                            if 0.01 < dist_sq < min_dist * min_dist:
                                dist = math.sqrt(dist_sq)
                                if dist > 0:
                                    overlap = min_dist - dist
                                    nxn, nyn = dx_dist / dist, dy_dist / dist
                                    self.x -= nxn * overlap * 0.5
                                    self.y -= nyn * overlap * 0.5
                                    self.vx -= nxn * 0.1
                                    self.vy -= nyn * 0.1

    def update_leds(gx, gy, particles, brightness, buckets):
        for y in range(ROWS):
            for x in range(COLS):
                brightness[y][x] = 0.0
                buckets[y][x].clear()
        for p in particles:
            px, py = int(p.x), int(p.y)
            if 0 <= px < COLS and 0 <= py < ROWS:
                buckets[py][px].append(p)
        for p in particles:
            p.update(gx, gy, buckets)
        for p in particles:
            gx_led, gy_led = int(p.x), int(p.y)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    nx, ny = gx_led + dx, gy_led + dy
                    if 0 <= nx < COLS and 0 <= ny < ROWS:
                        dist_x, dist_y = abs(p.x - (nx+0.5)), abs(p.y - (ny+0.5))
                        if dist_x < 1.2 and dist_y < 1.2:
                            influence = max(0, 1.0 - (dist_x + dist_y) * 0.7)
                            brightness[ny][nx] += influence * 1.5
        for y in range(ROWS):
            for x in range(COLS):
                idx = y * COLS + x
                lvl = int(min(brightness[y][x] * 6, MAX_LEVEL))
                np[idx] = (lvl * COLOR_CH[0], lvl * COLOR_CH[1], lvl * COLOR_CH[2])
        np.write()

    particles = [Particle() for _ in range(PARTICLE_COUNT)]
    brightness = [[0.0 for _ in range(COLS)] for _ in range(ROWS)]
    buckets = [[[] for _ in range(COLS)] for _ in range(ROWS)]
    sim_time = 0
    
    while not switch_requested:
        if accelerometer_ok:
            try:
                raw = _r(0x28, 6)
                x_raw, y_raw, z_raw = struct.unpack("<hhh", raw)
                ax = max(min(x_raw*LSB_G, G_CLAMP), -G_CLAMP)
                ay = max(min(y_raw*LSB_G, G_CLAMP), -G_CLAMP)
                az = z_raw * LSB_G
                mag = math.sqrt(ax*ax + ay*ay)
                if mag > 0.05:
                    gx, gy = ax / mag, -ay / mag
                else:
                    gx, gy = 0.0, 1.0 if az > 0 else -1.0
            except:
                gx, gy = 0.0, 1.0 # Fallback
        else:
            sim_time += 0.1
            gx = math.sin(sim_time * 0.3) * 0.5
            gy = math.cos(sim_time * 0.2) * 0.5

        update_leds(gx, gy, particles, brightness, buckets)

# ═══════════════════════════════════════════════════════════════════════════════
# ANIMATION 2: FLAME
# ═══════════════════════════════════════════════════════════════════════════════

def flame_animation():
    global switch_requested, accelerometer_ok, _r, LSB_G, G_CLAMP
    print("🎬 Running Mode 2: Flame")
    if not accelerometer_ok:
        print("Flame animation requires an accelerometer.")
        # Could show a message on the matrix here.
        time.sleep(2) # Show message before switching
        return

    micropython.const(1)
    PARTICLE_COUNT   = const(12)
    TURBULENCE       = 50.0
    LIFETIME         = const(18)
    HEAT_INTENSITY   = 0.3
    FLAME_WIDTH      = 2.0
    SPAWN_RATE       = const(1)
    GRAVITY_STRENGTH = 0.8

    x_off = (COLS - 1) / 2
    y_off = (ROWS - 1) / 2
    coords = tuple((c-x_off, y_off-r) for idx in range(NUM_LEDS) for r,c in (divmod(idx,COLS),))

    class Particle:
        __slots__ = ("x","y","z","vx","vy","vz","life","max_life","temperature")
        def __init__(self, x, y, gx, gy):
            self.x, self.y, self.z = x, y, (random.random()-0.5)*0.5
            s = 0.05 + random.random()*0.08
            self.vx = gx*s + (random.random()-0.5)*0.02
            self.vy = gy*s + (random.random()-0.5)*0.02
            self.vz = (random.random()-0.5)*0.01
            self.life = random.randint(LIFETIME//2, LIFETIME)
            self.max_life = self.life
            self.temperature = random.random()*0.5 + 0.5

        __temperature_map = ((0.25,(255,150,100)),(0.15,(255,100,0)),(0.15,(255,150,0)),(0.0,(255,15,0)))
        @micropython.native
        def get_color(self):
            tr = self.temperature
            for t,(r,g,b) in self.__temperature_map:
                if tr > t: return r, g, b
            return 0, 0, 0

    _particles = []

    @micropython.native
    def _spawn_particle(gx: float, gy: float):
        d = 4.0
        return Particle(gx*d+(random.random()-0.5)*FLAME_WIDTH, gy*d+(random.random()-0.5)*FLAME_WIDTH*0.5, gx, gy)

    @micropython.native
    def _update_particles(gx: float, gy: float):
        i = 0
        while i < len(_particles):
            if _particles[i].life <= 0: _particles.pop(i)
            else: i += 1
        if len(_particles) < PARTICLE_COUNT:
            for _ in range(SPAWN_RATE):
                _particles.append(_spawn_particle(gx, gy))
                if len(_particles) >= PARTICLE_COUNT: break
        for p in _particles:
            p.vx -= gx*GRAVITY_STRENGTH*0.1; p.vy -= gy*GRAVITY_STRENGTH*0.1
            p.vx += (random.random()-0.5)*TURBULENCE*0.001
            p.vy += (random.random()-0.5)*TURBULENCE*0.001
            p.vz += (random.random()-0.5)*TURBULENCE*0.0005
            p.x += p.vx; p.y += p.vy; p.z += p.vz
            p.vx *= 0.95; p.vy *= 0.95; p.vz *= 0.92
            p.life -= 1; p.temperature = (p.life / p.max_life)*HEAT_INTENSITY
    
    @micropython.native
    def _led_color_for(x: float, y: float):
        tot_i = r_acc = g_acc = b_acc = 0.0
        for p in _particles:
            dx, dy, dz = p.x-x, p.y-y, p.z
            dist2 = dx*dx + dy*dy + dz*dz
            if dist2 < 4.0:
                inv = 1.0 - dist2*0.25
                weight = inv * (p.life/p.max_life)
                r,g,b = p.get_color()
                tot_i += weight; r_acc += r*weight; g_acc += g*weight; b_acc += b*weight
        if tot_i <= 0.0: return 0,0,0
        r_base, g_base, b_base = r_acc/tot_i, g_acc/tot_i, b_acc/tot_i
        brightness = (tot_i if tot_i < 1.0 else 1.0) * 0.3
        r,g,b = int(r_base*brightness), int(g_base*brightness), int(b_base*brightness)
        if r>255:r=255
        if g>255:g=255
        if b>255:b=255
        return r,g,b

    @micropython.native
    def update(gx: float, gy: float, gz: float):
        mag = (gx*gx + gy*gy)**0.5
        if mag < 0.05:
            gy = 1.0 if gz > 0 else -1.0; gx = 0.0
        else:
            inv = 1.0 / mag; gx *= inv; gy *= inv
        _update_particles(gx, gy)
        for i, (x, y) in enumerate(coords):
            np[i] = _led_color_for(x, y)
        np.write()

    while not switch_requested:
        try:
            raw = _r(0x28, 6)
            ax, ay, az = struct.unpack("<hhh", raw)
            ax = max(min(ax * LSB_G, G_CLAMP), -G_CLAMP)
            ay = max(min(ay * LSB_G, G_CLAMP), -G_CLAMP)
            update(ax, ay, az * LSB_G)
        except Exception as e:
            print("err:", e)
            time.sleep(0.1)

# ═══════════════════════════════════════════════════════════════════════════════
# MAIN PROGRAM & MODE SWITCHING
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    global switch_requested, current_mode
    
    init_accelerometer()

    button_timer = Timer()
    button_timer.init(freq=50, mode=Timer.PERIODIC, callback=check_bootsel_button)
    
    animations = [scroll_text_continuous_animation, particle_physics_animation, flame_animation]
    animation_names = ["Scrolling Text", "Particle Physics (Water)", "Flame"]
    
    print("\n🎮 BOOTSEL Button Animation Switcher")
    print("Press BOOTSEL button to switch between animations:")
    print("  Mode 0: Scrolling Text (TSCIRCUIT)")
    print("  Mode 1: Particle Physics with Accelerometer (Water)")
    print("  Mode 2: Flame with Accelerometer")
    print("-" * 50)
    
    try:
        while True:
            print(f"\n🎬 Starting Mode {current_mode}: {animation_names[current_mode]}")
            switch_requested = False
            animations[current_mode]()
            time.sleep(0.1)
    except KeyboardInterrupt:
        print(f"\n🛑 Program stopped by user. Total BOOTSEL presses: {total_presses}")
    finally:
        for i in range(NUM_LEDS):
            np[i] = (0, 0, 0)
        np.write()
        onboard_led.off()
        button_timer.deinit()
        print("✅ Cleanup done.")

if __name__ == "__main__":
    main()
