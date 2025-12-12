from pi5RC import pi5RC


pwm_pins = [18, 19, 12, 13]

class Servo:
    def __init__(self, num=2):
        self.servos = [pi5RC(pwm_pins[i]) for i in range(num)]

    def set(self, index, a):
        self.servos[index].set(a)

    def set_all(self, a):
        for i in range(len(self.servos)):
            self.servos[i].set(a)

    def __del__(self):
        for i in range(len(self.servos)):
            del self.servos[i]


if __name__ == '__main__':
    servos = Servo(4)
    try:
        i = 0
        while True:
            angle = int(input(f"Enter angle between 0 and 180 for servo {i % 2 + 1}:\n"))
            if angle > 180:
                angle = 180
            if angle < 0:
                angle = 0

            servos.set(i % 4, angle)
            i += 1

    except KeyboardInterrupt:
        print("Program stopped by user")

    finally:
        del servos
