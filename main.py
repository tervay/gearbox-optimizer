from math import pi

from collections import defaultdict
from pprint import pprint

PINION_MIN = 10
PINION_MAX = 14
GEAR_MIN = 20
GEAR_MAX = 54

rpm = 5880
speed_loss = 0.90
wheels = [4, 6, 8]
min_speed = 12.5
max_speed = 15.5


def fps(wheel_size, a, b, c, d):
    return rpm * (a / b) * (c / d) * wheel_size * pi / 12 / 60 * speed_loss


def spacing(teeth):
    return {8: 10, 9: 10, 10: 12, 11: 12, 12: 12, 13: 14, 14: 14}.get(teeth, teeth)


configs = defaultdict(list)

for g1 in range(PINION_MIN, PINION_MAX + 1):
    for g2 in range(GEAR_MIN, GEAR_MAX + 2, 2):
        for g3 in range(GEAR_MIN, GEAR_MAX + 2, 2):
            for g4 in range(GEAR_MIN, GEAR_MAX + 2, 2):
                if (g1 + g2 + 4) / 20 < 2.5:
                    continue
                configs[(spacing(g1) + g2), (g3 + g4)].append([g1, g2, g3, g4])


config_counts = defaultdict(lambda: defaultdict(list))
for sums, options in configs.items():
    for option in options:
        for wheel in wheels:
            fps_ = round(fps(wheel, option[0], option[1], option[2], option[3]), 1)
            if min_speed <= fps_ <= max_speed:
                config_counts[sums][wheel].append(option + [fps_])

pprint(
    sorted(
        config_counts.items(),
        key=lambda t: sum(
            [len(t[1][w]) for w in wheels]
        ),  # len(t[1][4]) + len(t[1][6])
        reverse=True,
    )[0]
)
