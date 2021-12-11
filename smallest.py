from math import pi

from collections import defaultdict
from pprint import pprint

PINION_MIN = 10
PINION_MAX = 14
GEAR_MIN = 20
GEAR_MAX = 72

rpm = 5880
speed_loss = 0.90
wheels = 6
min_speed = 12.5
max_speed = 15.5


def fps(wheel_size, a, b, c, d):
    return rpm * (a / b) * (c / d) * wheel_size * pi / 12 / 60 * speed_loss


def spacing(teeth):
    return {8: 10, 9: 10, 10: 12, 11: 12, 12: 12, 13: 14, 14: 14}.get(teeth, teeth)


def minimum(teeth):
    return {
        10: 46,
        12: 44,
        14: 42,
    }.get(teeth)


def size(gears):
    if len(gears) == 2:
        return (gears[0] + gears[1]) / 20

    if len(gears) == 4:
        pinion_diameter = gears[0] / 20
        gear1_radius = gears[1] / 40
        second_stage = (gears[2] + gears[3]) / 20
        return pinion_diameter + gear1_radius + second_stage


sr_results = []
dr_results = []

for g1 in range(PINION_MIN, PINION_MAX + 1):
    for g2 in range(GEAR_MIN, GEAR_MAX + 2, 2):
        if minimum(spacing(g1)) > g2:
            continue
        for g3 in range(GEAR_MIN, GEAR_MAX + 2, 2):
            for g4 in range(GEAR_MIN, GEAR_MAX + 2, 2):
                if (g1 + g2 + 4) / 20 < 2.5:
                    continue
            
                fps_ = fps(wheels, g1, g2, g3, g4)
                if min_speed <= fps_ <= max_speed:
                    dr_results.append([g1, g2, g3, g4, fps_])

pprint(
    sorted(
        dr_results,
        key=lambda r: size(r[:-1]),
    )[:5]
)
