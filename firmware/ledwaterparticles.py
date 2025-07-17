from machine import Pin, SPI
import math, time, struct, neopixel, random

# ───────── LED-string setup ─────────
LED_PIN   = 6
NUM_LEDS  = 42           # 7 × 6 matrix, row-major
COLS, ROWS = 7, 6

np = neopixel.NeoPixel(Pin(LED_PIN), NUM_LEDS)

# ───── Visual parameters ─────
MAX_LEVEL  = 35          # cap (0-15) → tweak for brightness
COLOR_CH   = (0, 0, 1)   # per-channel multipliers (R,G,B). (1,0,0) = red only

# ───── Matrix parameters ─────
MATRIX_WIDTH = COLS       # 7 units wide
MATRIX_HEIGHT = ROWS      # 6 units tall
PARTICLE_COUNT = 25       # Reduced for better performance with collisions

# ───── Accelerometer (LIS3DHTR) setup - for gravity direction ─────
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

# ───── Simplified Particle System ─────
class Particle:
    def __init__(self):
        self.x = random.uniform(1.0, MATRIX_WIDTH - 1.0)
        self.y = random.uniform(1.0, MATRIX_HEIGHT - 1.0)
        self.vx = random.uniform(-0.2, 0.2)
        self.vy = random.uniform(-0.2, 0.2)
        self.radius = 0.4  # Increased particle size
    
    def update(self, gravity_x, gravity_y, particles):
        # Apply gravity
        self.vx += gravity_x * 0.05
        self.vy += gravity_y * 0.05
        
        # Simple collision detection with other particles
        for other in particles:
            if other is self:
                continue
            
            dx = other.x - self.x
            dy = other.y - self.y
            distance_sq = dx * dx + dy * dy
            min_distance = self.radius + other.radius
            
            if distance_sq < min_distance * min_distance and distance_sq > 0.01:
                # Push particles apart
                distance = math.sqrt(distance_sq)
                overlap = min_distance - distance
                
                # Normalize and apply separation
                if distance > 0:
                    dx_norm = dx / distance
                    dy_norm = dy / distance
                    
                    # Move particles apart
                    self.x -= dx_norm * overlap * 0.5
                    self.y -= dy_norm * overlap * 0.5
                    
                    # Add some velocity exchange
                    self.vx -= dx_norm * 0.1
                    self.vy -= dy_norm * 0.1
        
        # Update position
        self.x += self.vx
        self.y += self.vy
        
        # Bounce off walls with damping
        if self.x < self.radius or self.x > MATRIX_WIDTH - self.radius:
            self.vx *= -0.7
            self.x = max(self.radius, min(MATRIX_WIDTH - self.radius, self.x))
        
        if self.y < self.radius or self.y > MATRIX_HEIGHT - self.radius:
            self.vy *= -0.7
            self.y = max(self.radius, min(MATRIX_HEIGHT - self.radius, self.y))
        
        # Simple damping
        self.vx *= 0.9
        self.vy *= 0.9

# Initialize particles
particles = [Particle() for _ in range(PARTICLE_COUNT)]

# Pre-allocate LED brightness array
led_brightness = [0] * NUM_LEDS

# ─── LED update helper ───
def update_leds(gravity_x, gravity_y):
    """
    Update particles and calculate LED brightness based on particle positions.
    """
    # Update all particles
    for particle in particles:
        particle.update(gravity_x, gravity_y, particles)
    
    # Reset LED brightness
    for i in range(NUM_LEDS):
        led_brightness[i] = 0
    
    # Calculate LED brightness based on particle positions
    for particle in particles:
        # Find which LED grid cell this particle is in
        grid_x = int(particle.x)
        grid_y = int(particle.y)
        
        # Make sure we're within bounds
        if 0 <= grid_x < COLS and 0 <= grid_y < ROWS:
            led_index = grid_y * COLS + grid_x
            led_brightness[led_index] += 1
        
        # Also add influence to neighboring LEDs for smoother effect
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                nx, ny = grid_x + dx, grid_y + dy
                if 0 <= nx < COLS and 0 <= ny < ROWS:
                    led_index = ny * COLS + nx
                    # Calculate distance influence (bigger radius for larger particles)
                    dist_x = abs(particle.x - nx - 0.5)
                    dist_y = abs(particle.y - ny - 0.5)
                    if dist_x < 1.2 and dist_y < 1.2:  # Increased influence radius
                        influence = max(0, 1.0 - (dist_x + dist_y) * 0.7)
                        led_brightness[led_index] += influence * 0.8  # Stronger influence
    
    # Update LEDs
    for i in range(NUM_LEDS):
        # Scale brightness and clamp
        brightness = led_brightness[i] * 6  # Reduced multiplier since particles have more influence
        lvl = int(min(brightness, MAX_LEVEL))
        
        np[i] = (lvl * COLOR_CH[0],
                 lvl * COLOR_CH[1],
                 lvl * COLOR_CH[2])
    
    np.write()

# ─── Main loop ───
while True:
    # Read accelerometer for gravity direction
    raw = _r(0x28, 6)
    x_raw, y_raw, z_raw = struct.unpack("<hhh", raw)
    
    # Convert to g‑units and clamp
    ax = max(min(x_raw * LSB_G,  G_CLAMP), -G_CLAMP)
    ay = max(min(y_raw * LSB_G,  G_CLAMP), -G_CLAMP)
    az = z_raw * LSB_G
    
    # Use accelerometer data as gravity for particles
    # Normalize gravity vector
    gravity_mag = math.sqrt(ax*ax + ay*ay)
    if gravity_mag > 0.05:
        gravity_x = ax / gravity_mag  # Negative because we want "down" direction
        gravity_y = -ay / gravity_mag
    else:
        # If board is nearly flat, use gz to determine direction
        gravity_x = 0.0
        gravity_y = 1.0 if az > 0 else -1.0
    
    update_leds(gravity_x, gravity_y)
    #time.sleep(0.01)  # ~33 fps
