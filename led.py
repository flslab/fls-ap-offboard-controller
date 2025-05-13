import time
import board
import neopixel_spi as neopixel

NUM_PIXELS = 46
PIXEL_ORDER = neopixel.RGB
COLORS = (0xFF0000, 0x00FF00, 0x0000FF)
DELAY = 0.1
LENGTH = 5

spi = board.SPI()

pixels = neopixel.NeoPixel_SPI(
    spi, NUM_PIXELS, pixel_order=PIXEL_ORDER, auto_write=False
)


# pixels.fill((0,0,0))
def rgb_to_hex_int(r, g, b):
    return (r << 16) | (g << 8) | b


if __name__ == "__main__":
    base_color = (227, 253, 255)
    colors = []

    for i in range(LENGTH):
        brightness = (LENGTH - i) / LENGTH
        colors.append(rgb_to_hex_int(
            int(base_color[0] * brightness),
            int(base_color[1] * brightness),
            int(base_color[2] * brightness))
        )

    print(colors)

    try:
        while True:
            for i in range(NUM_PIXELS):
                for j in range(LENGTH):
                    pixels[(i+j) % NUM_PIXELS] = colors[j]
                    print(colors[j])
                pixels.show()
                time.sleep(DELAY)
                pixels.fill(0)
    finally:
        pixels.fill((0, 0, 0))
