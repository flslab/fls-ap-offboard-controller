import time
import board
import neopixel_spi as neopixel

NUM_PIXELS = 46
PIXEL_ORDER = neopixel.GRB
spi = board.SPI()

pixels = neopixel.NeoPixel_SPI(
    spi, NUM_PIXELS, pixel_order=PIXEL_ORDER, auto_write=False, brightness=1.0
)

# Parameters
dot_color = (227, 253, 255)
tail_decay = 0.5  # How quickly the tail fades (0.0–1.0)
delay = 0.02  # Time between frames

# Initialize an array of RGB values (like a framebuffer)
leds = [(0, 0, 0)] * NUM_PIXELS


def fade_tail():
    for i in range(NUM_PIXELS):
        r, g, b = leds[i]
        leds[i] = (
            int(r * tail_decay),
            int(g * tail_decay),
            int(b * tail_decay)
        )


def draw_frame():
    for i, color in enumerate(leds):
        pixels[i] = color
    pixels.show()


while True:
    for pos in range(NUM_PIXELS):
        fade_tail()
        leds[pos] = dot_color
        draw_frame()
        time.sleep(delay)
