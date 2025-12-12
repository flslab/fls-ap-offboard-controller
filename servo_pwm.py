import argparse

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
        for servo in self.servos:
            del servo
        self.servos.clear()


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument("-n", "--num", default=2, type=int, help="number of servos [1, 4]")
    args = ap.parse_args()

    servos = Servo(args.n)

    try:
        while True:
            user_input = input(f"Enter servo index and angle between 0 and 180:\n")
            if user_input:
                j, angle = user_input.strip().split(" ")
                j, angle = int(j), int(angle)
                if angle > 180:
                    angle = 180
                if angle < 0:
                    angle = 0

                if 0 <= j < args.n:
                    servos.set(j, angle)

    except KeyboardInterrupt:
        print("Program stopped by user")

    finally:
        del servos
