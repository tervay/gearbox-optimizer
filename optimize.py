from math import pi, sqrt
from pprint import pprint
import numpy as np
import json
from typing import List


def fps(rpm, wheel_size, ratio, speed_loss=13.84 / 16.68):
    raw_fps = rpm / ratio * wheel_size * pi / 12 / 60 * speed_loss
    return round(raw_fps * 4) / 4


def calc(
    ratio_: float,
    wheel_diameter_: int,
    num_motors_: int,
    current_limit_: int,
):
    max_sim_time = _aa46 = 4
    min_gear_ratio = _aa47 = 3
    max_gear_ratio = _aa48 = 15
    filtering = _aa49 = 1
    graph_current_limit = _aa50 = 60
    max_speed_accel_threshold = _aa51 = 0.15
    throttle_response_min = _aa52 = 0.5
    throttle_response_max = _aa53 = 0.99

    neo_free_speed = 5676
    neo_stall_torque = 2.590
    neo_stall_current = 105
    neo_free_current = 1.8
    neo_spec_voltage = 12
    neo_controller_current_limit = neo_stall_current
    neo_controller_limited_torque = (
        neo_stall_torque / neo_stall_current * neo_controller_current_limit
    )
    neo_average_duty_cycle = 0.75
    neo_peak_power = neo_free_speed / 2 * neo_stall_torque / 2 * 2 * pi / 60
    neo_kv = neo_free_speed / neo_spec_voltage
    neo_kt = neo_stall_torque / neo_stall_current

    # num_motors = _f5 = 6
    _f5 = num_motors_
    efficiency1 = _f7 = 0.975 ** (_f5 / 2)
    efficiency2 = _f8 = 0.95
    efficiency3 = _f9 = 0.92
    # ratio = e45 = 7.56
    e45 = ratio_
    gearing = _f10 = e46 = 1 / ratio_
    # wheel_diameter = _f15 = 4
    _f15 = wheel_diameter_
    coeff_friction_static = _f16 = 1.1
    coeff_friction_dynamic = _f17 = 0.9
    coeff_friction_lateral = _f18 = 1.1
    wheel_base_length = _f19 = 27
    wheel_base_width = _f20 = 20
    weight_distribution_front = _f21 = 0.5
    weight_distribution_sides = _f22 = 0.5
    weight = _f25 = 120
    weight_auxilliary = _f26 = 0
    sprint_distance = _f29 = 15
    target_time_to_goal = _f30 = 2.0
    cycles_per_match = _f31 = 24
    deceleration_method = _f32 = "Brake"
    battery_voltage_rest = _f35 = 12.6
    applied_voltage_ramp = _f36 = 1200
    # motor_current_limit = _f37 = 55
    _f37 = current_limit_
    battery_resistance = _f39 = 0.018
    battery_amp_hour_rating = _f40 = 18
    peak_battery_discharge = _f41 = 20

    num_sim_rows = c3 = 100
    expected_voltage_loss = c7 = 0.18
    spec_voltage = c8 = neo_spec_voltage
    applied_voltage_ratio = c9 = (_f35 - c7) / c8
    specced_free_speed = c10 = neo_free_speed
    actual_free_speed = c11 = c10 / c8 * _f35
    stall_torque = c12 = neo_stall_torque * c9
    duty_cycle = c13 = neo_average_duty_cycle
    stall_current = c14 = neo_stall_current * c9
    free_current = c15 = neo_free_current * c9
    current_limit = c16 = min([_f37, _f35 / 12 * _f40 * _f41 / _f5])
    sim_time_res = c17 = _aa46 / c3
    max_delta_volts = c18 = c17 * _f36
    num_motors = c19 = _f5
    ratio_spread = c20 = 1
    gearbox_wheel_efficiency = c21 = _f7 * _f8 * _f9
    distance_to_goal = c22 = _f29 * 12 * 0.0254
    mass = c23 = (_f25 + _f26) * 0.4536
    radius = c24 = _f15 / 2 * 0.0254
    current_limited_max_motor_torque = c25 = (c16 - c15) / (c14 - c15) * c12
    weight_times_cof = c34 = c23 / 0.4536 * _f16 * 4.448
    max_torque_at_wheel = c35 = c34 * c24
    max_motor_torque = c26 = c35 * e46 / c19 / c21
    current_while_pushing = c27 = (c26 / c12 * (c14 - c15) + c15) * c19 * c13
    dist_coa_com = c28 = sqrt(
        ((1 - _f21) * _f19 - _f19 / 2) ** 2 + (_f20 * _f22 - (_f20 / 2)) ** 2
    )
    dimension_turn_cond = _f16 * _f20 > _f18 * (_f19 - 4 * c28 ** 2 / _f19)

    geared_stall_torque = e62 = c12 * c19 / e46 * c21

    force_to_turn = c31 = (
        _f18
        * (sum([_f25, _f26]) * 4.448222)
        / (_f20 * 0.0254)
        * ((_f19 * 0.0254) / 4 - (c28 * 0.0254) ** 2 / (_f19 * 0.0254))
        / c21
    )
    force_turn_cond = e62 / c24 / 4 > c31

    torque_to_turn = c32 = c31 * c24
    estimated_torque_loss = c33 = (
        (1 - c21) * 4.448222 * sum([_f25, _f26]) / (4.448222 * sum([_f25, _f26]))
    ) * c12

    max_motor_torque_before_wheel_slip = e63 = c26
    max_tractive_force_at_wheels = e65 = min(e63, c25) * c19 * e45 / c24 / 4.448 * c21
    output_current_max_tractive_force = e67 = (
        (min(e63, c25) / c12 * (c14 - c15) + c15) * c19 * c13 / c21
    )

    turning_motor_load = e71 = c32 * e46 / c19 + c33 * e46
    turning_current = e70 = min(
        ((e71 / c12) * (c14 - c15) + c15) * c19, _f37 * num_motors
    )

    for time in np.arange(0, _aa46, _aa46 / c3):
        if time == 0:
            distance = pu = 0
            hit_target = oy = pu > _f29
            applied_voltage_ratio_ = time

    return {
        "tractive force": max_tractive_force_at_wheels,
        "time to goal": 0,
        "can turn": force_turn_cond,
        "pushing current": output_current_max_tractive_force,
        "turning current": turning_current,
    }


# =======================================


def reduction_to_num(reduction):
    return reduction[0] / reduction[1]


def overall_ratio(ratio):
    if len(ratio) == 2:
        return ratio[0] / ratio[1]
    elif len(ratio) == 4:
        return (ratio[0] / ratio[1]) * (ratio[2] / ratio[3])
    elif len(ratio) == 6:
        return (ratio[0] / ratio[1]) * (ratio[2] / ratio[3]) * (ratio[4] / ratio[5])


class Gearbox:
    def __init__(self, name, vendors, motors, ratios) -> None:
        self.name = name
        self.vendors = vendors
        self.motors = motors
        self.ratios = []
        for stage in ratios:
            if len(self.ratios) == 0:
                for reduction in stage:
                    self.ratios.append(reduction)
            else:
                prev_ratios = self.ratios
                self.ratios = []
                for reduction in stage:
                    for prev_reduction in prev_ratios:
                        self.ratios.append(prev_reduction + reduction)

        for ratio in self.ratios:
            ratio.append(overall_ratio(ratio))


def cots_gearboxes() -> List[Gearbox]:
    with open("cots_gearboxes.json", "r") as f:
        content = json.load(f)

    return [
        Gearbox(gb["name"], gb["vendors"], gb["motors"], gb["ratios"]) for gb in content
    ]


gearboxes = cots_gearboxes()

ratio_choices = np.arange(3.0, 40.0, 0.01)
wheel_sizes = [
    4,
]
num_motors = [4, 6]
current_limits = list(range(20, 61, 5))
enable_acceptable_ratio_ranges = False
enabled_custom_gearboxes = True
acceptable_ratio_range = {
    # (motors, wheel_size): (min_ratio, max_ratio),
    (2, 4): (5, 11),
    (3, 4): (3.75, 10.5),
    (2, 6): (8, 15),
    (3, 6): (5.75, 15),
}

PINION_MIN = 10
PINION_MAX = 14
GEAR_MIN = 20
GEAR_MAX = 70

ratio_choices = []

if enabled_custom_gearboxes:
    for g1 in range(PINION_MIN, PINION_MAX + 1):
        for g2 in range(GEAR_MIN, GEAR_MAX + 2, 2):
            for g3 in range(GEAR_MIN, GEAR_MAX + 2, 2):
                for g4 in range(GEAR_MIN, GEAR_MAX + 2, 2):
                    if (g1 + g2 + 4) / 20 < 2.5:
                        continue
                    gearboxes.append(
                        Gearbox(
                            name="Custom",
                            vendors=[],
                            motors=[2, 3],
                            ratios=[[[g1, g2]], [[g3, g4]]],
                        )
                    )


def ratio_to_num(ratio):
    return 1 / ((ratio[0] / ratio[1]) * (ratio[2] / ratio[3]))


options = []
# for ratio in ratio_choices:
#     for wheel in wheel_sizes:
#         for motors in num_motors:
#             for cl in current_limits:
#                 if motors * cl > 200:
#                     continue
#                 options.append(
#                     {
#                         "ratio": ratio,
#                         "wheel": wheel,
#                         "motors": motors,
#                         "current limit": cl,
#                         "tractive force": round(
#                             calc(ratio_to_num(ratio), wheel, motors, cl)[
#                                 "tractive force"
#                             ],
#                             2,
#                         ),
#                         "fps": round(fps(5880, wheel, ratio_to_num(ratio)), 2),
#                     }
#                 )
for gearbox in gearboxes:
    for wheel in wheel_sizes:
        for motors in gearbox.motors:
            for cl in current_limits:
                if 2 * motors * cl > 300:
                    continue

                for ratio in gearbox.ratios:
                    stages = ratio[:-1]
                    total_reduction = ratio[-1]
                    min_allowed, max_allowed = acceptable_ratio_range[(motors, wheel)]
                    if (
                        not (min_allowed < (1 / total_reduction) < max_allowed)
                        and enable_acceptable_ratio_ranges
                    ):
                        continue

                    ilite_data = calc(
                        1 / overall_ratio(stages),
                        wheel_diameter_=wheel,
                        num_motors_=motors * 2,
                        current_limit_=cl,
                    )
                    if not ilite_data["can turn"]:
                        continue
                    if ilite_data["pushing current"] > 240:
                        continue

                    options.append(
                        {
                            "gearbox": gearbox.name,
                            "total reduction": round(1 / total_reduction, 2),
                            "stages": tuple(stages),
                            "current limit": cl,
                            "motors": motors,
                            "wheel size": wheel,
                            "tractive force": round(
                                ilite_data["tractive force"],
                                2,
                            ),
                            "fps": round(
                                fps(
                                    5880,
                                    wheel_size=wheel,
                                    ratio=1 / overall_ratio(stages),
                                ),
                                2,
                            ),
                            "pushing current": round(ilite_data["pushing current"]),
                            "turning current": round(ilite_data["turning current"]),
                        }
                    )

options = [dict(t) for t in {tuple(d.items()) for d in options}]


pprint(
    sorted(
        options,
        key=lambda t: (
            t["tractive force"],
            t["fps"],
            -t["pushing current"],
            -t["turning current"],
            -(t["motors"] * t["current limit"]),
            -len(t["stages"]),
        ),
        reverse=True,
    )
)
