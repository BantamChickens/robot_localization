import math


class FakeMotionModel:
    def __init__(self):
        self.max_angle = 90
        self.angle_step = 15

    def degrees2radians(self, num_list):
        return [math.radians(angle) for angle in num_list]

    def turn(self):
        """Generate and print candidate turn angles."""
        angles_list = list(
            range(
                -self.max_angle,
                self.max_angle + 1,
                self.angle_step,
            )
        )

        angles = self.degrees2radians(angles_list)

        print("Degrees:", angles_list)
        print("Radians:", angles)

        return angles


if __name__ == "__main__":
    model = FakeMotionModel()
    angles = model.turn()