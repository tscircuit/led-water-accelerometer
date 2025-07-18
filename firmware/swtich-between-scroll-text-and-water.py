from machine import Pin, Timer, SPI
import neopixel
import time
import math
import struct
import random
import rp2

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

def check_bootsel_button(timer):
    """Timer callback to check BOOTSEL button state"""
    global last_button_state, current_mode, switch_requested, total_presses
    
    # Use rp2.bootsel_button() to read the BOOTSEL button
    current_state = rp2.bootsel_button()
    
    # Button pressed when it goes from 0 to 1
    if current_state == 1 and last_button_state == 0:
        total_presses += 1
        current_mode = (current_mode + 1) % 2  # Switch between 0 and 1
        switch_requested = True
        
        print(f"🎬 BOOTSEL pressed! Switching to mode {current_mode} (Press #{total_presses})")
        
        # Visual feedback - turn on onboard LED
        onboard_led.on()
        
        # Flash the LED matrix
        show_switch_feedback()
        
    # Button released when it goes from 1 to 0
    elif current_state == 0 and last_button_state == 1:
        print("🔄 BOOTSEL button released")
        onboard_led.off()
    
    last_button_state = current_state

def show_switch_feedback():
    """Visual feedback when switching modes"""
    # Flash white 2 times
    for flash in range(2):
        for i in range(NUM_LEDS):
            np[i] = (30, 30, 30)
        np.write()
        time.sleep(0.1)
        
        for i in range(NUM_LEDS):
            np[i] = (0, 0, 0)
        np.write()
        time.sleep(0.1)

# ═══════════════════════════════════════════════════════════════════════════════
# ANIMATION 1: SCROLLING TEXT (Your first animation - exactly as provided)
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

# Letter patterns - exactly as provided
LETTER_PATTERNS = {
    'T': [
        [1,1,1,1,1],
        [0,0,1,0,0],
        [0,0,1,0,0],
        [0,0,1,0,0],
        [0,0,1,0,0],
        [0,0,1,0,0]
    ],
    'S': [
        [0,1,1,1,1],
        [1,0,0,0,0],
        [0,1,1,1,0],
        [0,0,0,0,1],
        [0,0,0,0,1],
        [1,1,1,1,0]
    ],
    'C': [
        [0,1,1,1,1],
        [1,0,0,0,0],
        [1,0,0,0,0],
        [1,0,0,0,0],
        [1,0,0,0,0],
        [0,1,1,1,1]
    ],
    'I': [
        [1,1,1,1,1],
        [0,0,1,0,0],
        [0,0,1,0,0],
        [0,0,1,0,0],
        [0,0,1,0,0],
        [1,1,1,1,1]
    ],
    'R': [
        [1,1,1,1,0],
        [1,0,0,0,1],
        [1,1,1,1,0],
        [1,0,1,0,0],
        [1,0,0,1,0],
        [1,0,0,0,1]
    ],
    'U': [
        [1,0,0,0,1],
        [1,0,0,0,1],
        [1,0,0,0,1],
        [1,0,0,0,1],
        [1,0,0,0,1],
        [0,1,1,1,0]
    ]
}

def draw_letter_to_buffer(buffer, letter, start_x, start_y, color):
    if letter not in LETTER_PATTERNS:
        return
    
    pattern = LETTER_PATTERNS[letter]
    
    for row_idx, row_pattern in enumerate(pattern):
        for col_idx, pixel in enumerate(row_pattern):
            if pixel == 1:
                screen_x = start_x + col_idx
                screen_y = start_y + row_idx
                
                if 0 <= screen_x < COLS and 0 <= screen_y < ROWS:
                    buffer[screen_y][screen_x] = color

def update_matrix_from_buffer(matrix, buffer):
    for y in range(ROWS):
        for x in range(COLS):
            if buffer[y][x] != 0:
                matrix.set_pixel(x, y, buffer[y][x])
            else:
                matrix.set_pixel(x, y, (0, 0, 0))
    matrix.np.write()

def scroll_text_continuous_animation():
    """Scrolling text animation - exactly as provided"""
    global switch_requested
    
    matrix = NeoPixelMatrix(np, COLS, ROWS)
    text = "TSCIRCUIT"
    color = (0, 0, 50)  # Blue color as in your original
    speed = 0.05
    letter_spacing = 1
    
    print("🎬 Running Mode 0: Scrolling Text Animation")
    print("📜 Continuously scrolling: TSCIRCUIT")
    
    while not switch_requested:
        # Calculate positions - exactly as your original
        letter_width = 5
        total_width = len(text) * (letter_width + letter_spacing) - letter_spacing
        start_x = COLS
        end_x = -total_width
        
        frame_buffer = [[0 for _ in range(COLS)] for _ in range(ROWS)]
        position = float(start_x)
        
        while position > end_x and not switch_requested:
            # Clear the frame buffer
            for y in range(ROWS):
                for x in range(COLS):
                    frame_buffer[y][x] = 0
            
            # Draw each letter at its current position
            current_x = position
            for char in text:
                if char in LETTER_PATTERNS:
                    draw_letter_to_buffer(frame_buffer, char, int(current_x), 0, color)
                current_x += letter_width + letter_spacing
            
            # Update matrix from buffer
            update_matrix_from_buffer(matrix, frame_buffer)
            
            # Move position smoothly - exactly as your original
            position -= 1.0
            time.sleep(speed)
        
        # Brief pause between loops - exactly as your original
        if not switch_requested:
            time.sleep(0.2)

# ═══════════════════════════════════════════════════════════════════════════════
# ANIMATION 2: PARTICLE PHYSICS (Your second animation - exactly as provided)
# ═══════════════════════════════════════════════════════════════════════════════

# Particle physics constants - exactly as provided
MAX_LEVEL = 50
COLOR_CH = (0, 0, 1)  # Blue particles
PARTICLE_COUNT = 25
PARTICLE_RADIUS = 0.5

# Accelerometer setup - exactly as provided
SCK_PIN, MOSI_PIN, MISO_PIN, CS_PIN = 10, 11, 12, 17

def init_accelerometer():
    """Initialize accelerometer - exactly as provided"""
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
        print("✅ Accelerometer initialized successfully")
        return spi, cs, _w, _r
    except:
        print("⚠️ Accelerometer not found, using simulated gravity")
        return None, None, None, None

# Particle class - exactly as provided
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
                        dx_dist = other.x - self.x
                        dy_dist = other.y - self.y
                        dist_sq = dx_dist*dx_dist + dy_dist*dy_dist
                        min_dist = PARTICLE_RADIUS * 2
                        if 0.01 < dist_sq < min_dist * min_dist:
                            dist = math.sqrt(dist_sq)
                            if dist > 0:
                                overlap = min_dist - dist
                                nxn = dx_dist / dist
                                nyn = dy_dist / dist
                                self.x -= nxn * overlap * 0.5
                                self.y -= nyn * overlap * 0.5
                                self.vx -= nxn * 0.1
                                self.vy -= nyn * 0.1

def update_leds(gx, gy, particles, brightness, buckets):
    """Update LEDs - exactly as provided"""
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
        gx_led, gy_led = int(p.x), int(p.y)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                nx, ny = gx_led + dx, gy_led + dy
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

def particle_physics_animation():
    """Particle physics animation - exactly as provided"""
    global switch_requested
    
    print("🎬 Running Mode 1: Particle Physics Animation")
    
    # Initialize accelerometer
    spi, cs, _w, _r = init_accelerometer()
    
    # Initialize particles and state - exactly as provided
    particles = [Particle() for _ in range(PARTICLE_COUNT)]
    brightness = [[0.0 for _ in range(COLS)] for _ in range(ROWS)]
    buckets = [[[] for _ in range(COLS)] for _ in range(ROWS)]
    
    LSB_G = 0.00098
    G_CLAMP = 15
    
    # Simulation time for gravity effect when no accelerometer
    sim_time = 0
    
    while not switch_requested:
        if spi and cs and _w and _r:
            # Real accelerometer data - exactly as provided
            try:
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
            except:
                # Fallback to simulated gravity
                gx = 0.0
                gy = 1.0
        else:
            # Simulated gravity that changes direction
            sim_time += 0.1
            gx = math.sin(sim_time * 0.3) * 0.5
            gy = math.cos(sim_time * 0.2) * 0.5

        # Update LEDs - exactly as provided
        update_leds(gx, gy, particles, brightness, buckets)
        
        # No sleep here - exactly as your original (commented out)
        # time.sleep(0.01)  # ~100 fps if needed

def main():
    """Main function"""
    global switch_requested, current_mode
    
    # Set up timer to check BOOTSEL button at 50Hz
    button_timer = Timer()
    button_timer.init(freq=50, mode=Timer.PERIODIC, callback=check_bootsel_button)
    
    animations = [scroll_text_continuous_animation, particle_physics_animation]
    animation_names = ["Scrolling Text", "Particle Physics"]
    
    print("🎮 BOOTSEL Button Animation Switcher")
    print("====================================")
    print("Press BOOTSEL button to switch between animations:")
    print("  Mode 0: Scrolling Text (TSCIRCUIT)")
    print("  Mode 1: Particle Physics with Accelerometer")
    print("The onboard LED will light up when button is pressed")
    print("-" * 50)
    
    try:
        while True:
            print(f"🎬 Starting Mode {current_mode}: {animation_names[current_mode]}")
            
            # Reset switch flag
            switch_requested = False
            
            # Run current animation
            animations[current_mode]()
            
            # Brief pause between mode switches
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print(f"\n🛑 Program stopped by user")
        print(f"Total BOOTSEL button presses: {total_presses}")
        
        # Clear the LEDs
        for i in range(NUM_LEDS):
            np[i] = (0, 0, 0)
        np.write()
        print("✅ LEDs cleared")
        
        # Clean up
        onboard_led.off()
        button_timer.deinit()

if __name__ == "__main__":
    main()
