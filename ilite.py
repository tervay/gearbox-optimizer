import json
from math import pi, sqrt
from pprint import pprint
from typing import List

import numpy as np
from rich.console import Console
from rich.table import Table
from tqdm import tqdm

from motors import motors


def signum(x):
    return x and (1, -1)[x < 0]


def nearest_multiple(n, to):
    return round(n * to) / to


def fps(rpm, wheel_size, ratio, speed_loss=13.84 / 16.68):
    raw_fps = rpm / ratio * wheel_size * pi / 12 / 60 * speed_loss
    return nearest_multiple(raw_fps, 4)


def calc(
    ratio_: float,
    wheel_diameter_: int,
    num_motors_: int,
    current_limit_: int,
    motor_: str,
):
    max_sim_time = _aa46 = 5
    min_gear_ratio = _aa47 = 3
    max_gear_ratio = _aa48 = 15
    filtering = _aa49 = 1
    graph_current_limit = _aa50 = 60
    max_speed_accel_threshold = _aa51 = 0.15
    throttle_response_min = _aa52 = 0.5
    throttle_response_max = _aa53 = 0.99

    selected_motor_free_speed = motors[motor_]["freeSpeed"]["magnitude"]
    selected_motor_stall_torque = motors[motor_]["stallTorque"]["magnitude"]
    selected_motor_stall_current = motors[motor_]["stallCurrent"]["magnitude"]
    selected_motor_free_current = motors[motor_]["freeCurrent"]["magnitude"]
    selected_motor_spec_voltage = 12
    selected_motor__controller_current_limit = selected_motor_stall_current
    selected_motor__controller_limited_torque = (
        selected_motor_stall_torque
        / selected_motor_stall_current
        * selected_motor__controller_current_limit
    )
    selected_motor__average_duty_cycle = 0.75
    selected_motor__peak_power = (
        selected_motor_free_speed / 2 * selected_motor_stall_torque / 2 * 2 * pi / 60
    )
    selected_motor_kv = selected_motor_free_speed / selected_motor_spec_voltage
    selected_motor_kt = selected_motor_stall_torque / selected_motor_stall_current
    motors_c30 = "Reverse"
    motors_c31 = "Coast"
    motors_c32 = "Brake"
    motors_d30_32_from_c30_32 = {
        "Reverse": 1.0,
        "Coast": 0.05,
        "Brake": 0.20,
    }
    max_speed_acceleration_threshold = c40 = max_speed_accel_threshold

    # num_motors = _f5 = 6
    _f5 = num_motors_
    efficiency1 = _f7 = 0.975 ** (num_motors_ / 2)
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
    sprint_distance = _f29 = 10
    target_time_to_goal = _f30 = 2.0
    cycles_per_match = _f31 = 24
    deceleration_method = _f32 = "Brake"
    deceleration_complete_threshold = c41 = motors_d30_32_from_c30_32[_f32]
    battery_voltage_rest = _f35 = 12.6
    applied_voltage_ramp = _f36 = 1200
    # motor_current_limit = _f37 = 55
    _f37 = current_limit_
    battery_resistance = _f39 = 0.018
    battery_amp_hour_rating = _f40 = 18
    peak_battery_discharge = _f41 = 20

    num_sim_rows = c3 = 100
    expected_voltage_loss = c7 = 0.18
    spec_voltage = c8 = selected_motor_spec_voltage
    applied_voltage_ratio = c9 = (_f35 - c7) / c8
    specced_free_speed = c10 = selected_motor_free_speed
    actual_free_speed = c11 = c10 / c8 * _f35
    stall_torque = c12 = selected_motor_stall_torque * c9
    duty_cycle = c13 = selected_motor__average_duty_cycle
    stall_current = c14 = selected_motor_stall_current * c9
    free_current = c15 = selected_motor_free_current * c9
    current_limit = c16 = min([_f37, _f35 / 12 * _f40 * _f41 / num_motors_])

    sim_time_res = c17 = max_sim_time / c3
    max_delta_volts = c18 = c17 * _f36
    num_motors = c19 = num_motors_
    ratio_spread = c20 = 1
    gearbox_wheel_efficiency = c21 = efficiency1 * efficiency2 * efficiency3
    distance_to_goal = c22 = _f29 * 12 * 0.0254
    mass = c23 = (_f25 + _f26) * 0.4536
    radius = c24 = wheel_diameter_ / 2 * 0.0254
    current_limited_max_motor_torque = c25 = (c16 - c15) / (c14 - c15) * c12
    weight_times_cof = c34 = c23 / 0.4536 * _f16 * 4.448
    max_torque_at_wheel = c35 = c34 * c24
    max_motor_torque = c26 = c35 * gearing / c19 / c21
    current_while_pushing = c27 = (c26 / c12 * (c14 - c15) + c15) * c19 * c13
    dist_coa_com = c28 = sqrt(
        ((1 - _f21) * _f19 - _f19 / 2) ** 2 + (_f20 * _f22 - (_f20 / 2)) ** 2
    )
    dimension_turn_cond = _f16 * _f20 > _f18 * (_f19 - 4 * c28 ** 2 / _f19)

    geared_stall_torque = e62 = c12 * c19 / gearing * c21

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
    deceleration_mode = c37 = _f32
    voltage_ramp = c38 = 1200
    delta_volts = c39 = c38 * c17

    max_motor_torque_before_wheel_slip = e63 = c26
    max_tractive_force_at_wheels = e65 = (
        min(e63, c25) * c19 * ratio_ / c24 / 4.448 * c21
    )
    output_current_max_tractive_force = e67 = (
        (min(e63, c25) / c12 * (c14 - c15) + c15) * c19 * c13 / c21
    )
    voltage_max_tractive_force = (
        battery_voltage_rest - output_current_max_tractive_force * battery_resistance
    )

    turning_motor_load = e71 = c32 * gearing / c19 + c33 * gearing
    turning_current = e70 = min(
        ((e71 / c12) * (c14 - c15) + c15) * c19, _f37 * num_motors
    )

    s5 = c16
    s6 = gearing
    ao6 = c11 * s6 * wheel_diameter_ * pi / 12 / 60

    _times = r = list(np.arange(max_sim_time / c3, max_sim_time, max_sim_time / c3))

    offset = lambda n: n - 8

    _applied_voltage_ratios = s = [0]
    _motor_speed = ao = [0]
    _attempted_torque_at_motor = bk = [0]
    _attempted_current_draw = cg = [0]
    _actual_applied_torque = eu = [0]
    _distance = pu = [0]
    _is_max_speed = jw = [False]
    _floor_speed = lo = [0]
    _applied_voltage = ng = [0]
    _throttle_response_up = rm = []
    _throttle_response_down = si = []

    _is_hit_target = oy = [_distance[offset(8)] >= _f29]
    _is_finished_decelerating = ks = [_is_hit_target[offset(8)]]

    _applied_acceleration = gm = [
        0
        if _is_finished_decelerating[offset(8)] and _is_hit_target[offset(8)]
        else _actual_applied_torque[offset(8)] * c14 / s6 / c24 / c23 * 3.39 * c21
    ]
    _abs_acceleration = hi = [abs(_applied_acceleration[offset(8)])]

    _clamped_per_motor_current_draw = dc = [
        min(abs(_attempted_current_draw[offset(8)]), s5)
        * signum(_attempted_current_draw[offset(8)])
    ]

    _actual_current_draw = fq = [
        0
        if _is_finished_decelerating[offset(8)] and _is_hit_target[offset(8)]
        else min(
            (abs((c14 - c15) * _actual_applied_torque[offset(8)] / c12) + c15) / c21,
            _clamped_per_motor_current_draw[offset(8)],
        )
    ]

    _abs_applied_voltage = ie = [abs(_applied_voltage[offset(8)])]
    _is_current_limiting = ja = [
        6
        if (
            abs(_actual_current_draw[offset(8)])
            >= _clamped_per_motor_current_draw[offset(8)] - 1
        )
        and (abs(_attempted_current_draw[offset(8)]) >= s5)
        else None
    ]
    _system_voltage = oc = [_f35 - (_actual_current_draw[offset(8)] * c13) * c19 * _f39]
    _coulombs = qq = [
        min(_actual_current_draw[offset(8)], c16) * c17 * 1000 / 60 / 60 / c21
    ]
    _applied_cof = dy = [(c23 / 0.4536 * _f16 * 4.448 * c24 * s6)]
    _is_wheel_slipping = mk = [None]

    ttg = None

    for row, time in enumerate(_times, start=9):
        _floor_speed.append(
            max(
                _applied_acceleration[offset(row - 1)] * c17
                + _floor_speed[offset(row - 1)],
                0,
            )
        )

        # ng
        if _is_hit_target[offset(row - 1)] == 0:
            val = (
                min(
                    _f35 - _actual_current_draw[offset(row - 1)] * c19 * _f39,
                    _applied_voltage[offset(row - 1)] + c39,
                )
                - c7
            )
        else:
            if _floor_speed[offset(row - 1)] - 1 <= 0:
                val = 0
            else:
                if c37 == motors_c30:
                    val = max(
                        [
                            max(_applied_voltage) * -1,
                            _applied_voltage[offset(row - 1)] - c39,
                        ]
                    )
                else:
                    val = max(
                        [
                            0,
                            max(_applied_voltage) * -1,
                            _applied_voltage[offset(row - 1)] - c39,
                        ]
                    )
        _applied_voltage.append(val)
        _applied_voltage_ratios.append(_applied_voltage[offset(row)] / _f35)
        _abs_applied_voltage.append(abs(_applied_voltage[offset(row)]))
        _motor_speed.append(
            _floor_speed[offset(row)]
            * 12
            * 60
            / (pi * wheel_diameter_)
            / s6
            * abs(_applied_voltage_ratios[offset(row)])
        )
        if (_is_finished_decelerating[offset(row - 1)]) or (
            (c37 == motors_c32) and (_is_hit_target[offset(row - 1)])
        ):
            val = 0
        else:
            if _applied_voltage[offset(row)] == 0:
                val = (
                    -(_attempted_current_draw[offset(row - 1)] + c15)
                    / (c14 - c15)
                    * c12
                )
            else:
                val = (
                    (
                        c11 * _applied_voltage_ratios[offset(row)]
                        - _motor_speed[offset(row)]
                    )
                    / (c11 * _applied_voltage_ratios[offset(row)])
                    * c12
                    * _applied_voltage_ratios[offset(row)]
                )
        _attempted_torque_at_motor.append(val)
        _attempted_current_draw.append(
            0
            if _is_finished_decelerating[offset(row - 1)]
            and _is_hit_target[offset(row - 1)]
            else abs(_attempted_torque_at_motor[offset(row)] / c12 * (c14 - c15) + c15)
            * filtering
            + _attempted_current_draw[offset(row - 1)] * (1 - filtering)
        )

        _is_current_limiting.append(
            6 if abs(_attempted_current_draw[offset(row)]) >= s5 else None
        )

        _clamped_per_motor_current_draw.append(
            min(abs(_attempted_current_draw[offset(row)]), s5)
            * signum(_attempted_current_draw[offset(row)])
        )

        b = max(_actual_applied_torque) * -1

        if _is_hit_target[offset(row - 1)] and _applied_voltage[offset(row)] < 0:
            a1_ = -1
        else:
            a1_ = 1

        a1 = (
            a1_
            * (_clamped_per_motor_current_draw[offset(row)] - c15)
            / (c14 - c15)
            * c12
            * c21
            - c33 * _floor_speed[offset(row)] / ao6
            - _is_hit_target[offset(row - 1)] * c33 / c21
        )
        a2 = _applied_cof[offset(row - 1)] / c19
        a = min(a1, a2)
        _actual_applied_torque.append(max(a, b))
        _is_wheel_slipping.append(
            3
            if all(
                [
                    abs(_attempted_current_draw[offset(row)]) > 0,
                    _actual_applied_torque[offset(row)] * c19
                    >= _applied_cof[offset(row - 1)],
                    _applied_voltage[offset(row)] != 0,
                ]
            )
            else None
        )
        _applied_acceleration.append(
            0
            if (
                (_is_finished_decelerating[offset(row - 1)])
                and (_is_hit_target[offset(row - 1)])
            )
            else _actual_applied_torque[offset(row)] * c19 / s6 / c24 / c23 * 3.39 * c21
        )
        _distance.append(
            _distance[offset(row - 1)]
            + 0.5 * _applied_acceleration[offset(row)] * c17 ** 2
            + _floor_speed[offset(row)] * c17
        )

        _abs_acceleration.append(abs(_applied_acceleration[offset(row)]))

        _is_hit_target.append(_distance[offset(row)] >= _f29)
        if _is_hit_target[-1] and ttg is None:
            ttg = time

        if _is_hit_target[offset(row)] == 0:
            if _is_wheel_slipping[offset(row - 1)] is None:
                a = _f16
            else:
                a = _f17

            b = (
                c23
                / 0.4536
                * a
                * 4.448
                * c24
                * s6
                * signum(_attempted_torque_at_motor[offset(row)])
            )
            val = b
        else:
            val = _attempted_torque_at_motor[offset(row)]

        _applied_cof.append(val)

        _is_finished_decelerating.append(
            _is_finished_decelerating[offset(row - 1)]
            or (_is_hit_target[offset(row)] and _floor_speed[offset(row)] <= c41)
        )

        _actual_current_draw.append(
            0
            if (_is_finished_decelerating[offset(row)] and _is_hit_target[offset(row)])
            else min(
                (
                    abs((c14 - c15) * _actual_applied_torque[offset(row)] / c12 / c21)
                    + c15
                )
                / c21,
                _clamped_per_motor_current_draw[offset(row)],
            )
        )

        _system_voltage.append(
            _f35 - (_actual_current_draw[offset(row)] * c13) * c19 * _f39
        )

        _coulombs.append(
            min(_actual_current_draw[offset(row)], c16) * c17 * 1000 / 60 / 60 / c21
        )

        _is_max_speed.append(
            (not _is_finished_decelerating[offset(row)])
            and (_abs_acceleration[offset(row)] <= max_speed_acceleration_threshold)
        )

    return {
        "tractive force": max_tractive_force_at_wheels,
        "time to goal": 0,
        "can turn": force_turn_cond,
        "pushing current": output_current_max_tractive_force,
        "turning current": turning_current,
        "time to goal": None if ttg is None else nearest_multiple(ttg, 100),
        "max speed": max(_floor_speed),
        "avg speed": 0 if ttg is None else _f29 / ttg,
        "min voltage": min(_system_voltage),
        "push voltage": voltage_max_tractive_force,
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
wheel_sizes = [4, 6]
current_limits = list(range(20, 55, 1))
motor = "Falcon 500"

PINION_MIN = 10
PINION_MAX = 14
GEAR_MIN = 20
GEAR_MAX = 50

enabled_custom_gearboxes = False
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


progress = []
options = []
for gearbox in gearboxes:
    for wheel in wheel_sizes:
        for motor_count in gearbox.motors:
            for cl in current_limits:
                for ratio in gearbox.ratios:
                    progress.append([gearbox, wheel, motor_count, cl, ratio])


for (gearbox, wheel, motor_count, cl, ratio) in tqdm(progress):
    stages = ratio[:-1]
    total_reduction = ratio[-1]
    if len(stages) > 4:
        continue

    ilite_data = calc(
        1 / overall_ratio(stages),
        wheel_diameter_=wheel,
        num_motors_=motor_count * 2,
        current_limit_=cl,
        motor_=motor,
    )
    if not ilite_data["can turn"]:
        continue
    # if ilite_data["pushing current"] > 240:
    #     continue
    if ilite_data["time to goal"] is None:
        continue
    if ilite_data["min voltage"] <= 7 or ilite_data["push voltage"] <= 7:
        continue

    options.append(
        {
            "gearbox": gearbox.name,
            "total reduction": round(1 / total_reduction, 2),
            "stages": tuple(stages),
            "current limit": cl,
            "motors": motor_count,
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
            "time to goal": ilite_data["time to goal"],
            "max speed": nearest_multiple(ilite_data["max speed"], 20),
            "avg speed": ilite_data["avg speed"],
            "min voltage": nearest_multiple(ilite_data["min voltage"], 20),
            "test": nearest_multiple(
                ilite_data["time to goal"] / ilite_data["tractive force"], 10000
            ),
            "push voltage": round(ilite_data["push voltage"]),
        }
    )

options = [dict(t) for t in {tuple(d.items()) for d in options}]
sorted_options = sorted(
    options,
    key=lambda t: (
        -t["test"],
        -t["time to goal"],
        t["tractive force"],
        t["min voltage"],
        -t["pushing current"],
        -t["turning current"],
        t["max speed"],
        -len(t["stages"]),
    ),
    reverse=True,
)

table = Table(show_header=True, width=200)
for col in [
    "Name",
    "Motors",
    "Whl",
    "CurrLim",
    "Test",
    "TTG",
    "TractForce",
    "MinVolt",
    "PushCurr",
    "TurnCurr",
    "MaxSpd",
    "Stages",
]:
    table.add_column(col, width={"Name": 47, "Stages": 25}.get(col, None))

for opt in sorted_options:
    table.add_row(
        opt["gearbox"],
        str(opt["motors"] * 2),
        str(opt["wheel size"]),
        str(opt["current limit"]),
        str(opt["test"]),
        str(opt["time to goal"]),
        str(opt["tractive force"]),
        str(opt["min voltage"]),
        str(opt["pushing current"]),
        str(opt["turning current"]),
        str(opt["max speed"]),
        str(opt["stages"]),
    )

with open(f"rich_{motor}.txt", "w+") as f:
    console = Console(file=f, width=200)
    console.print(table)
