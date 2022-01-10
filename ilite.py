import json
from math import pi, sqrt
from pprint import pprint, pformat
from typing import List

import numpy as np
from rich.console import Console
from rich.table import Table
from tqdm import tqdm

from motors import motors

import plotly.graph_objects as go
import plotly.express as px


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
    sprint_distance_: int,
    rint_: float,
    weight_: int,
    weight_auxilliary_: int,
    efficiency_: float = None,
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
    # efficiency1 = _f7 = 0.95
    # efficiency2 = _f8 = 0.95
    # efficiency3 = _f9 = 1.0
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
    weight = _f25 = weight_
    weight_auxilliary = _f26 = weight_auxilliary_
    sprint_distance = _f29 = sprint_distance_
    target_time_to_goal = _f30 = 2.0
    cycles_per_match = _f31 = 24
    deceleration_method = _f32 = "Reverse"
    deceleration_complete_threshold = c41 = motors_d30_32_from_c30_32[
        deceleration_method
    ]
    battery_voltage_rest = _f35 = 12.6
    applied_voltage_ramp = _f36 = 1200
    # motor_current_limit = _f37 = 55
    _f37 = current_limit_
    battery_resistance = _f39 = rint_
    battery_amp_hour_rating = _f40 = 18
    peak_battery_discharge = _f41 = 20

    num_sim_rows = c3 = 500
    expected_voltage_loss = c7 = 0.18
    spec_voltage = c8 = selected_motor_spec_voltage
    applied_voltage_ratio = c9 = (
        battery_voltage_rest - expected_voltage_loss
    ) / spec_voltage
    specced_free_speed = c10 = selected_motor_free_speed
    actual_free_speed = c11 = specced_free_speed / spec_voltage * battery_voltage_rest
    stall_torque = c12 = selected_motor_stall_torque * applied_voltage_ratio
    duty_cycle = c13 = selected_motor__average_duty_cycle
    stall_current = c14 = selected_motor_stall_current * applied_voltage_ratio
    free_current = c15 = selected_motor_free_current * applied_voltage_ratio
    current_limit = c16 = min(
        [
            current_limit_,
            battery_voltage_rest
            / 12
            * battery_amp_hour_rating
            * peak_battery_discharge
            / num_motors_,
        ]
    )

    sim_time_res = c17 = max_sim_time / num_sim_rows
    max_delta_volts = c18 = sim_time_res * applied_voltage_ramp
    num_motors = c19 = num_motors_
    ratio_spread = c20 = 1
    gearbox_wheel_efficiency = c21 = efficiency_
    distance_to_goal = c22 = sprint_distance * 12 * 0.0254
    mass = c23 = (weight + weight_auxilliary) * 0.4536
    radius = c24 = wheel_diameter_ / 2 * 0.0254
    current_limited_max_motor_torque = c25 = (
        (current_limit - free_current) / (stall_current - free_current) * stall_torque
    )
    weight_times_cof = c34 = mass / 0.4536 * coeff_friction_static * 4.448
    max_torque_at_wheel = c35 = weight_times_cof * radius
    max_motor_torque = c26 = (
        max_torque_at_wheel * gearing / num_motors / gearbox_wheel_efficiency
    )
    current_while_pushing = c27 = (
        (
            max_motor_torque / stall_torque * (stall_current - free_current)
            + free_current
        )
        * num_motors
        * duty_cycle
    )
    dist_coa_com = c28 = sqrt(
        ((1 - weight_distribution_front) * wheel_base_length - wheel_base_length / 2)
        ** 2
        + (wheel_base_width * weight_distribution_sides - (wheel_base_width / 2)) ** 2
    )
    dimension_turn_cond = (
        coeff_friction_static * wheel_base_width
        > coeff_friction_lateral
        * (wheel_base_length - 4 * dist_coa_com ** 2 / wheel_base_length)
    )

    geared_stall_torque = e62 = (
        stall_torque * num_motors / gearing * gearbox_wheel_efficiency
    )

    force_to_turn = c31 = (
        coeff_friction_lateral
        * (sum([weight, weight_auxilliary]) * 4.448222)
        / (wheel_base_width * 0.0254)
        * (
            (wheel_base_length * 0.0254) / 4
            - (dist_coa_com * 0.0254) ** 2 / (wheel_base_length * 0.0254)
        )
        / gearbox_wheel_efficiency
    )
    force_turn_cond = geared_stall_torque / radius / 4 > force_to_turn

    torque_to_turn = c32 = force_to_turn * radius
    estimated_torque_loss = c33 = (
        (1 - gearbox_wheel_efficiency)
        * 4.448222
        * sum([weight, weight_auxilliary])
        / (4.448222 * sum([weight, weight_auxilliary]))
    ) * stall_torque
    deceleration_mode = c37 = deceleration_method
    voltage_ramp = c38 = 1200
    delta_volts = c39 = voltage_ramp * sim_time_res

    max_motor_torque_before_wheel_slip = e63 = max_motor_torque
    max_tractive_force_at_wheels = e65 = (
        min(max_motor_torque_before_wheel_slip, current_limited_max_motor_torque)
        * num_motors
        * ratio_
        / radius
        / 4.448
        * gearbox_wheel_efficiency
    )
    output_current_max_tractive_force = e67 = (
        (
            min(max_motor_torque_before_wheel_slip, current_limited_max_motor_torque)
            / stall_torque
            * (stall_current - free_current)
            + free_current
        )
        * num_motors
        * duty_cycle
        / gearbox_wheel_efficiency
    )
    voltage_max_tractive_force = (
        battery_voltage_rest - output_current_max_tractive_force * battery_resistance
    )

    turning_motor_load = e71 = (
        torque_to_turn * gearing / num_motors + estimated_torque_loss * gearing
    )
    turning_current = e70 = min(
        (
            (turning_motor_load / stall_torque) * (stall_current - free_current)
            + free_current
        )
        * num_motors,
        current_limit_ * num_motors,
    )

    s5 = current_limit
    s6 = gearing
    max_theoretical_speed = ao6 = (
        actual_free_speed * gearing * wheel_diameter_ * pi / 12 / 60
    )

    can_turn = all(
        [force_turn_cond, turning_current < 120, turning_current / num_motors < 40]
    )

    _times = r = list(
        np.arange(
            max_sim_time / num_sim_rows, max_sim_time, max_sim_time / num_sim_rows
        )
    )

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

    _is_hit_target = oy = [_distance[offset(8)] >= sprint_distance]
    _is_finished_decelerating = ks = [_is_hit_target[offset(8)]]

    _applied_acceleration = gm = [
        0
        if _is_finished_decelerating[offset(8)] and _is_hit_target[offset(8)]
        else _actual_applied_torque[offset(8)]
        * stall_current
        / gearing
        / radius
        / mass
        * 3.39
        * gearbox_wheel_efficiency
    ]
    _abs_acceleration = hi = [abs(_applied_acceleration[offset(8)])]

    _clamped_per_motor_current_draw = dc = [
        min(abs(_attempted_current_draw[offset(8)]), current_limit)
        * signum(_attempted_current_draw[offset(8)])
    ]

    _actual_current_draw = fq = [
        0
        if _is_finished_decelerating[offset(8)] and _is_hit_target[offset(8)]
        else min(
            (
                abs(
                    (stall_current - free_current)
                    * _actual_applied_torque[offset(8)]
                    / stall_torque
                )
                + free_current
            )
            / gearbox_wheel_efficiency,
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
        and (abs(_attempted_current_draw[offset(8)]) >= current_limit)
        else None
    ]
    _system_voltage = oc = [
        battery_voltage_rest
        - (_actual_current_draw[offset(8)] * duty_cycle)
        * num_motors
        * battery_resistance
    ]
    _coulombs = qq = [
        min(_actual_current_draw[offset(8)], current_limit)
        * sim_time_res
        * 1000
        / 60
        / 60
        / gearbox_wheel_efficiency
    ]
    _applied_cof = dy = [
        (mass / 0.4536 * coeff_friction_static * 4.448 * radius * gearing)
    ]
    _is_wheel_slipping = mk = [None]

    ttg = None

    for row, time in enumerate(_times, start=9):
        _floor_speed.append(
            max(
                _applied_acceleration[offset(row - 1)] * sim_time_res
                + _floor_speed[offset(row - 1)],
                0,
            )
        )

        # ng
        if not _is_hit_target[offset(row - 1)]:
            val = (
                min(
                    battery_voltage_rest
                    - _actual_current_draw[offset(row - 1)]
                    * num_motors
                    * battery_resistance,
                    _applied_voltage[offset(row - 1)] + delta_volts,
                )
                - expected_voltage_loss
            )
        else:
            if _floor_speed[offset(row - 1)] - 1 <= 0:
                val = 0
            else:
                if deceleration_mode == motors_c30:
                    val = max(
                        [
                            max(_applied_voltage) * -1,
                            _applied_voltage[offset(row - 1)] - delta_volts,
                        ]
                    )
                else:
                    val = max(
                        [
                            0,
                            max(_applied_voltage) * -1,
                            _applied_voltage[offset(row - 1)] - delta_volts,
                        ]
                    )
        _applied_voltage.append(val)
        _applied_voltage_ratios.append(
            _applied_voltage[offset(row)] / battery_voltage_rest
        )
        _abs_applied_voltage.append(abs(_applied_voltage[offset(row)]))
        _motor_speed.append(
            _floor_speed[offset(row)]
            * 12
            * 60
            / (pi * wheel_diameter_)
            / gearing
            * abs(_applied_voltage_ratios[offset(row)])
        )
        if (_is_finished_decelerating[offset(row - 1)]) or (
            (deceleration_mode == motors_c32) and (_is_hit_target[offset(row - 1)])
        ):
            val = 0
        else:
            if _applied_voltage[offset(row)] == 0:
                val = (
                    -(_attempted_current_draw[offset(row - 1)] + free_current)
                    / (stall_current - free_current)
                    * stall_torque
                )
            else:
                val = (
                    (
                        actual_free_speed * _applied_voltage_ratios[offset(row)]
                        - _motor_speed[offset(row)]
                    )
                    / (actual_free_speed * _applied_voltage_ratios[offset(row)])
                    * stall_torque
                    * _applied_voltage_ratios[offset(row)]
                )
        _attempted_torque_at_motor.append(val)
        _attempted_current_draw.append(
            0
            if _is_finished_decelerating[offset(row - 1)]
            and _is_hit_target[offset(row - 1)]
            else abs(
                _attempted_torque_at_motor[offset(row)]
                / stall_torque
                * (stall_current - free_current)
                + free_current
            )
            * filtering
            + _attempted_current_draw[offset(row - 1)] * (1 - filtering)
        )

        _is_current_limiting.append(
            6 if abs(_attempted_current_draw[offset(row)]) >= current_limit else None
        )

        _clamped_per_motor_current_draw.append(
            min(abs(_attempted_current_draw[offset(row)]), current_limit)
            * signum(_attempted_current_draw[offset(row)])
        )

        b = max(_actual_applied_torque) * -1

        if _is_hit_target[offset(row - 1)] and _applied_voltage[offset(row)] < 0:
            a1_ = -1
        else:
            a1_ = 1

        a1 = (
            a1_
            * (_clamped_per_motor_current_draw[offset(row)] - free_current)
            / (stall_current - free_current)
            * stall_torque
            * gearbox_wheel_efficiency
            - estimated_torque_loss * _floor_speed[offset(row)] / max_theoretical_speed
            - _is_hit_target[offset(row - 1)]
            * estimated_torque_loss
            / gearbox_wheel_efficiency
        )
        a2 = _applied_cof[offset(row - 1)] / num_motors
        a = min(a1, a2)
        _actual_applied_torque.append(max(a, b))
        _is_wheel_slipping.append(
            3
            if all(
                [
                    abs(_attempted_current_draw[offset(row)]) > 0,
                    _actual_applied_torque[offset(row)] * num_motors
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
            else _actual_applied_torque[offset(row)]
            * num_motors
            / gearing
            / radius
            / mass
            * 3.39
            * gearbox_wheel_efficiency
        )
        _distance.append(
            _distance[offset(row - 1)]
            + 0.5 * _applied_acceleration[offset(row)] * sim_time_res ** 2
            + _floor_speed[offset(row)] * sim_time_res
        )

        _abs_acceleration.append(abs(_applied_acceleration[offset(row)]))

        _is_hit_target.append(_distance[offset(row)] >= sprint_distance)
        if _is_hit_target[-1] and ttg is None:
            ttg = time

        if _is_hit_target[offset(row)] == 0:
            if _is_wheel_slipping[offset(row - 1)] is None:
                a = coeff_friction_static
            else:
                a = coeff_friction_dynamic

            b = (
                mass
                / 0.4536
                * a
                * 4.448
                * radius
                * gearing
                * signum(_attempted_torque_at_motor[offset(row)])
            )
            val = b
        else:
            val = _attempted_torque_at_motor[offset(row)]

        _applied_cof.append(val)

        _is_finished_decelerating.append(
            _is_finished_decelerating[offset(row - 1)]
            or (
                _is_hit_target[offset(row)]
                and _floor_speed[offset(row)] <= deceleration_complete_threshold
            )
        )

        _actual_current_draw.append(
            0
            if (_is_finished_decelerating[offset(row)] and _is_hit_target[offset(row)])
            else min(
                (
                    abs(
                        (stall_current - free_current)
                        * _actual_applied_torque[offset(row)]
                        / stall_torque
                        / gearbox_wheel_efficiency
                    )
                    + free_current
                )
                / gearbox_wheel_efficiency,
                _clamped_per_motor_current_draw[offset(row)],
            )
        )

        _system_voltage.append(
            battery_voltage_rest
            - (_actual_current_draw[offset(row)] * duty_cycle)
            * num_motors
            * battery_resistance
        )

        _coulombs.append(
            min(_actual_current_draw[offset(row)], current_limit)
            * sim_time_res
            * 1000
            / 60
            / 60
            / gearbox_wheel_efficiency
        )

        _is_max_speed.append(
            (not _is_finished_decelerating[offset(row)])
            and (_abs_acceleration[offset(row)] <= max_speed_acceleration_threshold)
        )

    return {
        "tractive force": max_tractive_force_at_wheels,
        "can turn": force_turn_cond,
        "pushing current": output_current_max_tractive_force,
        "turning current": turning_current,
        "time to goal": None if ttg is None else nearest_multiple(ttg, 100),
        "max speed": max(_floor_speed),
        "avg speed": 0 if ttg is None else sprint_distance / ttg,
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


if __name__ == "__main__":
    gearboxes = cots_gearboxes()
    wheel_sizes = [4, 5, 6]
    current_limits = list(range(30, 82, 2))
    motor = "NEO"
    sprint_distance = 12
    RInt = 0.015
    efficiency = 0.95 * 0.95
    include_swerve = False
    weight = 125
    extra_weight = 14 + 10

    PINION_MIN = 10
    PINION_MAX = 14
    GEAR_MIN = 20
    GEAR_MAX = 60

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
        if ("[Swerve]" in gearbox.name) and (not include_swerve):
            continue
        for wheel in wheel_sizes:
            for motor_count in gearbox.motors:
                for cl in current_limits:
                    for ratio in gearbox.ratios:
                        progress.append([gearbox, wheel, motor_count, cl, ratio])

    for (gearbox, wheel, motor_count, cl, ratio) in tqdm(progress):
        stages = ratio[:-1]
        total_reduction = ratio[-1]
        if any(
            [
                len(stages) > 4 and "[Swerve]" not in gearbox.name,
                wheel != 4 and "[Swerve]" in gearbox.name,
                stages[0] in [8, 9],
            ]
        ):
            continue

        ilite_data = calc(
            1 / overall_ratio(stages),
            wheel_diameter_=wheel,
            num_motors_=motor_count * 2,
            current_limit_=cl,
            motor_=motor,
            sprint_distance_=sprint_distance,
            rint_=RInt,
            weight_=weight,
            weight_auxilliary_=extra_weight,
            efficiency_=efficiency,
        )
        if any(
            [
                not ilite_data["can turn"] and "Swerve" not in gearbox.name,
                ilite_data["time to goal"] is None,
                ilite_data["min voltage"] <= 8 or ilite_data["push voltage"] <= 8,
            ]
        ):
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
                ),
                "pushing current": round(ilite_data["pushing current"]),
                "turning current": round(ilite_data["turning current"]),
                "time to goal": ilite_data["time to goal"],
                "max speed": nearest_multiple(ilite_data["max speed"], 100),
                "avg speed": ilite_data["avg speed"],
                "min voltage": nearest_multiple(ilite_data["min voltage"], 100),
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

    cleaned_options = []
    seen = set()
    for option in tqdm(sorted(options, key=lambda o: -o["current limit"])):
        found = False
        if (
            option["gearbox"],
            option["motors"],
            option["wheel size"],
            option["stages"],
        ) in seen:
            found = True

        if not found:
            cleaned_options.append(option)
            seen.add(
                (
                    option["gearbox"],
                    option["motors"],
                    option["wheel size"],
                    option["stages"],
                )
            )

    sorted_options = sorted(
        cleaned_options,
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

    csv = [[]]
    table = Table(show_header=True, width=200)
    for col in [
        "Name",
        "Motors",
        "WheelSize",
        "CurrLim",
        "TTG/TractForce",
        "TTG",
        "TractForce",
        "MinVolt",
        "PushCurr",
        "TurnCurr",
        "MaxSpd",
        "Stages",
        "Ratio",
    ]:
        table.add_column(col, width={"Name": 47, "Stages": 25}.get(col, None))
        csv[0].append(col)

    for opt in tqdm(sorted_options):
        to_add = ()

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
            str(opt["total reduction"]),
        )
        csv.append(
            [
                opt["gearbox"].replace(",", " /"),
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
                str(opt["stages"]).replace(",", ":"),
                str(opt["total reduction"]),
            ]
        )

    with open(f"data_{motor}.txt", "w+") as f:
        console = Console(file=f, width=200)
        console.print(table)

    with open(f"data_{motor}.csv", "w+") as f:
        for l in csv:
            print(",".join(l), file=f)

    def graph(cleaned_options):
        motors4 = [o for o in cleaned_options if o["motors"] == 2]
        motors6 = [o for o in cleaned_options if o["motors"] == 3]

        # ratios = [1 / overall_ratio(o["stages"]) for o in motors4] + [
        #     1 / overall_ratio(o["stages"]) for o in motors6
        # ]
        # min_ = min(ratios)
        # max_ = max(ratios)

        fig = go.Figure()
        fig.add_trace(
            go.Scatter(
                x=[o["time to goal"] for o in motors4],
                y=[o["tractive force"] for o in motors4],
                text=[pformat(o).replace("\n", "<br>") for o in motors4],
                name="4 Motors",
                marker_color="blue",
                # marker=dict(
                #     color=[1 / overall_ratio(o["stages"]) for o in motors4],
                #     colorbar=dict(
                #         title="",
                #     ),
                #     cmin=min_,
                #     cmax=max_,
                #     colorscale="Viridis",
                # ),
                mode="markers",
            )
        )
        fig.add_trace(
            go.Scatter(
                x=[o["time to goal"] for o in motors6],
                y=[o["tractive force"] for o in motors6],
                text=[pformat(o).replace("\n", "<br>") for o in motors6],
                name="6 Motors",
                marker_color="red",
                # marker=dict(
                #     color=[1 / overall_ratio(o["stages"]) for o in motors6],
                #     cmin=min_,
                #     cmax=max_,
                #     colorscale="Viridis",
                # ),
                mode="markers",
            )
        )

        fig.update_yaxes(rangemode="tozero")
        fig.update_xaxes(rangemode="tozero")
        fig.update_layout(
            xaxis_title="Time to Goal (s) (Rounded to nearest 0.05s)",
            yaxis_title="Tractive Force (lbs) (Rounded to nearest 1lbs)",
            title=f"Tractive Force vs Time to Goal in COTS Gearboxes ({sprint_distance} ft sprint, {motor} motor, {RInt}Ω RInt, {round(efficiency * 100, 1)}% efficiency, {weight + extra_weight}lbs robot)",
        )

        fig.show()

    graph(cleaned_options)
