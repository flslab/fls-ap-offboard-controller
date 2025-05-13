import time
import board
import neopixel_spi as neopixel

NUM_PIXELS = 46
PIXEL_ORDER = neopixel.GRB
COLORS = (0xFF0000, 0x00FF00, 0x0000FF)
DELAY = 0.5

spi = board.SPI()

pixels = neopixel.NeoPixel_SPI(
    spi, NUM_PIXELS, pixel_order=PIXEL_ORDER, auto_write=False
)

# pixels.fill((0,0,0))

while True:
    for color in COLORS:
        for i in range(NUM_PIXELS):
            pixels[i] = color
            pixels.show()
            time.sleep(DELAY)
            pixels.fill(0)