from math import pi, sqrt
from pprint import pprint
import numpy as np
import json
from typing import List
from tqdm import tqdm


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
    motors_c30 = "Reverse"
    motors_c31 = "Coast"
    motors_c32 = "Brake"
    motors_d30_32_from_c30_32 = {
        "Reverse": 1.0,
        "Coast": 0.05,
        "Brake": 0.20,
    }
    max_speed_acceleration_threshold = c40 = _aa51

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
    deceleration_mode = c37 = _f32
    voltage_ramp = c38 = 1200
    delta_volts = c39 = c38 * c17

    max_motor_torque_before_wheel_slip = e63 = c26
    max_tractive_force_at_wheels = e65 = min(e63, c25) * c19 * e45 / c24 / 4.448 * c21
    output_current_max_tractive_force = e67 = (
        (min(e63, c25) / c12 * (c14 - c15) + c15) * c19 * c13 / c21
    )

    turning_motor_load = e71 = c32 * e46 / c19 + c33 * e46
    turning_current = e70 = min(
        ((e71 / c12) * (c14 - c15) + c15) * c19, _f37 * num_motors
    )

    s5 = c16
    s6 = e46
    ao6 = c11 * s6 * _f15 * pi / 12 / 60

    _times = r = list(np.arange(_aa46 / c3, _aa46, _aa46 / c3))

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

    _is_hit_target = oy = [pu[offset(8)] >= _f29]
    _is_finished_decelerating = ks = [oy[offset(8)]]

    assert oy == [False]
    assert ks == [False]

    _applied_acceleration = gm = [
        0
        if ks[offset(8)] and oy[offset(8)]
        else eu[offset(8)] * c14 / s6 / c24 / c23 * 3.39 * c21
    ]
    assert gm == [0.0]
    _abs_acceleration = hi = [abs(gm[offset(8)])]
    assert hi == [0.0]

    _clamped_per_motor_current_draw = dc = [
        min(abs(cg[offset(8)]), s5) * signum(cg[offset(8)])
    ]
    assert dc == [0.0]

    _actual_current_draw = fq = [
        0
        if ks[offset(8)] and oy[offset(8)]
        else min((abs((c14 - c15) * eu[offset(8)] / c12) + c15) / c21, dc[offset(8)])
    ]
    assert fq == [0.0]

    _abs_applied_voltage = ie = [abs(ng[offset(8)])]
    assert ie == [0.0]

    _is_current_limiting = ja = [
        6
        if (abs(fq[offset(8)]) >= dc[offset(8)] - 1) and (abs(cg[offset(8)]) >= s5)
        else None
    ]
    assert ja == [None]

    _system_voltage = oc = [_f35 - (fq[offset(8)] * c13) * c19 * _f39]
    assert oc == [12.6]

    _coulombs = qq = [min(fq[offset(8)], c16) * c17 * 1000 / 60 / 60 / c21]
    assert qq == [0.0]

    _applied_cof = dy = [(c23 / 0.4536 * _f16 * 4.448 * c24 * s6)]
    _is_wheel_slipping = mk = [None]

    ttg = None

    for row, time in enumerate(_times, start=9):
        lo.append(max(gm[offset(row - 1)] * c17 + lo[offset(row - 1)], 0))
        if row == 10:
            assert lo == [0.0, 0.0, 1.1848664082357143], lo

        # ng
        if oy[offset(row - 1)] == 0:
            val = (
                min(
                    _f35 - fq[offset(row - 1)] * c19 * _f39,
                    ng[offset(row - 1)] + c39,
                )
                - c7
            )
        else:
            if lo[offset(row - 1)] - 1 <= 0:
                val = 0
            else:
                if c37 == motors_c30:
                    val = max([max(ng) * -1, ng[offset(row - 1)] - c39])
                else:
                    val = max([0, max(ng) * -1, ng[offset(row - 1)] - c39])
        ng.append(val)
        if row == 10:
            assert ng == [0.0, 12.42, 7.5600000000000005], ng

        s.append(ng[offset(row)] / _f35)
        if row == 10:
            assert s == [0.0, 0.9857142857142858, 0.6000000000000001], s

        ie.append(abs(ng[offset(row)]))
        if row == 10:
            assert ie == [0.0, 12.42, 7.5600000000000005], ie

        ao.append(lo[offset(row)] * 12 * 60 / (pi * _f15) / s6 * abs(s[offset(row)]))
        if row == 10:
            assert ao == [0.0, 0.0, 244.39624012343268], ao

        # print(lo[offset(row)])
        # bk.append(
        #     0
        #     if (ks[offset(row) - 1]) or (c37 == motors_c32 and oy[offset(row) - 1])
        #     else (
        #         -(cg[offset(row - 1)] + c15) / (c14 - c15) * c12
        #         if ng[offset(row)] == 0
        #         else (c11 * s[offset(row)] - ao[offset(row)])
        #         / (c11 * s[offset(row)])
        #         * c12
        #         * s[offset(row)]
        #     )
        # )
        # bk
        if (ks[offset(row - 1)]) or ((c37 == motors_c32) and (oy[offset(row - 1)])):
            val = 0
        else:
            if ng[offset(row)] == 0:
                val = -(cg[offset(row - 1)] + c15) / (c14 - c15) * c12
            else:
                val = (
                    (c11 * s[offset(row)] - ao[offset(row)])
                    / (c11 * s[offset(row)])
                    * c12
                    * s[offset(row)]
                )
        bk.append(val)
        if row == 10:
            assert bk == [0.0, 2.642355], bk
        # if row == 9:
        #     dy.append(
        #         (c23 / 0.4536 * _f16 * 4.448 * c24 * s6) * signum(bk[offset(row)])
        #     )

        cg.append(
            0
            if ks[offset(row - 1)] and oy[offset(row - 1)]
            else abs(bk[offset(row)] / c12 * (c14 - c15) + c15) * _aa49
            + cg[offset(row - 1)] * (1 - _aa49)
        )
        if row == 10:
            assert cg == [0.0, 107.14911428571429], cg

        ja.append(6 if abs(cg[offset(row)]) >= s5 else None)
        if row == 10:
            assert ja == [None, 6], ja

        dc.append(min(abs(cg[offset(row)]), s5) * signum(cg[offset(row)]))
        if row == 10:
            assert dc == [0.0, 45.0], dc
        # eu.append(
        #     max(
        #         min(
        #             (
        #                 (-1 if (oy[offset(row - 1)] and ng[offset(row)] < 0) else 1)
        #                 * (dc[offset(row)] - c15)
        #                 / (c14 - c15)
        #                 * c12
        #                 * c21
        #                 - c33 * lo[offset(row)] / ao6
        #                 - oy[offset(row - 1)] * c33 / c21
        #             ),
        #             dy[offset(row - 1)] / c19,
        #         ),
        #         max(eu) * -1,
        #     )
        # )
        # eu
        b = max(eu) * -1

        if oy[offset(row - 1)] and ng[offset(row)] < 0:
            a1_ = -1
        else:
            a1_ = 1

        a1 = (
            a1_ * (dc[offset(row)] - c15) / (c14 - c15) * c12 * c21
            - c33 * lo[offset(row)] / ao6
            - oy[offset(row - 1)] * c33 / c21
        )
        a2 = dy[offset(row - 1)] / c19
        a = min(a1, a2)
        eu.append(max(a, b))
        if row == 10:
            assert eu == [0.0, 0.8285141333333333], eu

        mk.append(
            3
            if all(
                [
                    abs(cg[offset(row)]) > 0,
                    eu[offset(row)] * c19 >= dy[offset(row - 1)],
                    ng[offset(row)] != 0,
                ]
            )
            else None
        )
        if row == 10:
            assert mk == [None, 3], mk

        gm.append(
            0
            if ((ks[offset(row - 1)]) and (oy[offset(row - 1)]))
            else eu[offset(row)] * c19 / s6 / c24 / c23 * 3.39 * c21
        )
        if row == 10:
            assert gm == [0.0, 29.621660205892855], gm

        pu.append(
            pu[offset(row - 1)]
            + 0.5 * gm[offset(row)] * c17 ** 2
            + lo[offset(row)] * c17
        )
        if row == 10:
            assert pu == [0.0, 0.023697328164714284], pu

        hi.append(abs(gm[offset(row)]))
        if row == 10:
            assert hi == [0.0, 29.621660205892855], hi

        oy.append(pu[offset(row)] >= _f29)
        if oy[-1] and ttg is None:
            ttg = time

        if row == 10:
            assert oy == [False, False], oy

        if oy[offset(row)] == 0:
            if mk[offset(row - 1)] is None:
                a = _f16
            else:
                a = _f17

            b = c23 / 0.4536 * a * 4.448 * c24 * s6 * signum(bk[offset(row)])
            val = b
        else:
            val = bk[offset(row)]

        dy.append(val)
        if row == 10:
            assert dy == [4.9710848, 4.9710848], dy

        ks.append(ks[offset(row - 1)] or (oy[offset(row)] and lo[offset(row)] <= c41))
        if row == 10:
            assert ks == [False, False], ks

        fq.append(
            0
            if (ks[offset(row)] and oy[offset(row)])
            else min(
                (abs((c14 - c15) * eu[offset(row)] / c12 / c21) + c15) / c21,
                dc[offset(row)],
            )
        )
        if row == 10:
            assert fq == [0, 45], fq

        oc.append(_f35 - (fq[offset(row)] * c13) * c19 * _f39)
        if row == 10:
            assert oc == [12.6, 8.955], oc

        qq.append(min(fq[offset(row)], c16) * c17 * 1000 / 60 / 60 / c21)
        if row == 10:
            assert qq == [0.0, 0.6172267285559908], qq

        jw.append((not ks[offset(row)]) and (hi[offset(row)] <= c40))
        if row == 10:
            assert jw == [False, False], jw

    # ng.append(
    #     min(_f35 - fq[offset(row - 1)] * c19 * _f39, ng[offset(row - 1)] + c39) - c7
    #     if oy[offset(row - 1)]
    #     else (
    #         0
    #         if lo[offset(row - 1)] - 1 <= 0
    #         else (
    #             max(max(ng) * -1, ng[offset(8)] - c39)
    #             if c37 == motors_c30
    #             else max(0, max(max(ng) * -1, ng[offset(row - 1)] - c39))
    #         )
    #     )
    # )

    # eu.append(
    #     max(
    #         min(
    #             (-1 if oy[offset(row - 1)] and ng[offset(row)] < 0 else 1)
    #             * (dc[offset(row)] - c15)
    #             / (c14 - c15)
    #             * c12
    #             * c21
    #             - c33 * lo[offset(row)] / ao6
    #             - oy[offset(row - 1)] * c33 / c21,
    #             dy[offset(row)] / c19,
    #         ),
    #         max(eu) * -1,
    #     )
    # )
    # gm.append(
    #     0
    #     if ks[offset(row - 1)] and oy[offset(row - 1)]
    #     else eu[offset(row)] * c19 / s6 / c24 / c23 * 3.39 * c21
    # )
    # pu.append(
    #     pu[offset(row - 1)]
    #     + 0.5 * gm[offset(row)] * c17 ** 2
    #     + lo[offset(row)] * c17
    # )
    # oy.append(pu[offset(row)] >= _f29)

    return {
        "tractive force": max_tractive_force_at_wheels,
        "time to goal": 0,
        "can turn": force_turn_cond,
        "pushing current": output_current_max_tractive_force,
        "turning current": turning_current,
        "time to goal": None if ttg is None else nearest_multiple(ttg, 100),
        "max speed": max(lo),
        "avg speed": 0 if ttg is None else _f29 / ttg,
        "min voltage": min(oc),
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
current_limits = list(range(20, 80, 2))
enable_acceptable_ratio_ranges = False
enabled_custom_gearboxes = False
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


# gearboxes = [Gearbox(name="asdf", vendors=[], motors=[3], ratios=[[[10, 60]]])]
# current_limits = [45]
# wheel_sizes = [4]

progress = []
options = []
for gearbox in gearboxes:
    for wheel in wheel_sizes:
        for motors in gearbox.motors:
            for cl in current_limits:
                if 2 * motors * cl > 300:
                    continue

                for ratio in gearbox.ratios:
                    progress.append([gearbox, wheel, motors, cl, ratio])


for (gearbox, wheel, motors, cl, ratio) in tqdm(progress):
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
    if ilite_data["time to goal"] is None:
        continue
    if ilite_data["min voltage"] <= 7:
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
            "time to goal": ilite_data["time to goal"],
            "max speed": ilite_data["max speed"],
            "avg speed": ilite_data["avg speed"],
            "min voltage": nearest_multiple(ilite_data["min voltage"], 20),
        }
    )

options = [dict(t) for t in {tuple(d.items()) for d in options}]


pprint(
    sorted(
        options,
        key=lambda t: (
            t["tractive force"],
            -t["time to goal"],
            t["min voltage"],
            -t["pushing current"],
            -t["turning current"],
            t["fps"],
            -(t["motors"] * t["current limit"]),
            -len(t["stages"]),
        ),
        reverse=True,
    )
)
