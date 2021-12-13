function nearestMultiple(n: number, to: number): number {
  return Math.round(n * to) / to;
}

function linspace(start: number, stop: number, step: number): number[] {
  const ret = [];
  for (let x = start; x <= stop; x += step) {
    ret.push(x);
  }
  return ret;
}

type Measurement = {
  magnitude: number;
  unit: string;
};
type Motor = {
  name: string;
  freeSpeed: Measurement;
  stallTorque: Measurement;
  stallCurrent: Measurement;
  freeCurrent: Measurement;
  weight: Measurement;
  url: string;
};

const _motors: Motor[] = [
  {
    name: "Falcon 500",
    freeSpeed: { magnitude: 6380, unit: "rpm" },
    stallTorque: { magnitude: 4.69, unit: "N*m" },
    stallCurrent: { magnitude: 257, unit: "A" },
    freeCurrent: { magnitude: 1.5, unit: "A" },
    weight: { magnitude: 1.1, unit: "lbs" },
    url: "https://www.vexrobotics.com/217-6515.html",
  },
  {
    name: "NEO",
    freeSpeed: { magnitude: 5880, unit: "rpm" },
    stallTorque: { magnitude: 3.36, unit: "N*m" },
    stallCurrent: { magnitude: 166, unit: "A" },
    freeCurrent: { magnitude: 1.3, unit: "A" },
    weight: { magnitude: 0.94, unit: "lbs" },
    url: "https://www.revrobotics.com/rev-21-1650/",
  },
  {
    name: "NEO 550",
    freeSpeed: { magnitude: 11710, unit: "rpm" },
    stallTorque: { magnitude: 1.08, unit: "N*m" },
    stallCurrent: { magnitude: 111, unit: "A" },
    freeCurrent: { magnitude: 1.1, unit: "A" },
    weight: { magnitude: 0.31, unit: "lbs" },
    url: "https://www.revrobotics.com/rev-21-1651/",
  },
  {
    name: "775pro",
    freeSpeed: { magnitude: 18730, unit: "rpm" },
    stallTorque: { magnitude: 0.71, unit: "N*m" },
    stallCurrent: { magnitude: 134, unit: "A" },
    freeCurrent: { magnitude: 0.7, unit: "A" },
    weight: { magnitude: 0.8, unit: "lbs" },
    url: "https://www.vexrobotics.com/775pro.html",
  },
  {
    name: "775 RedLine",
    freeSpeed: { magnitude: 19500, unit: "rpm" },
    stallTorque: { magnitude: 0.64, unit: "N*m" },
    stallCurrent: { magnitude: 122, unit: "A" },
    freeCurrent: { magnitude: 2.6, unit: "A" },
    weight: { magnitude: 0.806, unit: "lbs" },
    url: "https://www.andymark.com/products/andymark-775-redline-motor-v2",
  },
  {
    name: "CIM",
    freeSpeed: { magnitude: 5330, unit: "rpm" },
    stallTorque: { magnitude: 2.41, unit: "N*m" },
    stallCurrent: { magnitude: 131, unit: "A" },
    freeCurrent: { magnitude: 2.7, unit: "A" },
    weight: { magnitude: 2.82, unit: "lbs" },
    url: "https://www.vexrobotics.com/217-2000.html",
  },
  {
    name: "MiniCIM",
    freeSpeed: { magnitude: 5840, unit: "rpm" },
    stallTorque: { magnitude: 1.41, unit: "N*m" },
    stallCurrent: { magnitude: 89, unit: "A" },
    freeCurrent: { magnitude: 3, unit: "A" },
    weight: { magnitude: 2.16, unit: "lbs" },
    url: "https://www.vexrobotics.com/217-3371.html",
  },
  {
    name: "BAG",
    freeSpeed: { magnitude: 13180, unit: "rpm" },
    stallTorque: { magnitude: 0.43, unit: "N*m" },
    stallCurrent: { magnitude: 53, unit: "A" },
    freeCurrent: { magnitude: 1.8, unit: "A" },
    weight: { magnitude: 0.71, unit: "lbs" },
    url: "https://www.vexrobotics.com/217-3351.html",
  },
  {
    name: "AM-9015",
    freeSpeed: { magnitude: 14270, unit: "rpm" },
    stallTorque: { magnitude: 0.36, unit: "N*m" },
    stallCurrent: { magnitude: 71, unit: "A" },
    freeCurrent: { magnitude: 3.7, unit: "A" },
    weight: { magnitude: 0.5, unit: "lbs" },
    url: "https://www.andymark.com/products/9015-motor",
  },
  {
    name: "NeveRest",
    freeSpeed: { magnitude: 5480, unit: "rpm" },
    stallTorque: { magnitude: 0.173, unit: "N*m" },
    stallCurrent: { magnitude: 9.8, unit: "A" },
    freeCurrent: { magnitude: 0.355, unit: "A" },
    weight: { magnitude: 0.587, unit: "lbs" },
    url: "https://www.andymark.com/products/neverest-series-motor-only",
  },
  {
    name: "Snowblower",
    freeSpeed: { magnitude: 100, unit: "rpm" },
    stallTorque: { magnitude: 7.90893775, unit: "N*m" },
    stallCurrent: { magnitude: 24, unit: "A" },
    freeCurrent: { magnitude: 5, unit: "A" },
    weight: { magnitude: 1.1, unit: "lbs" },
    url: "https://www.andymark.com/products/snow-blower-motor-with-hex-shaft",
  },
];
const motors = Object.fromEntries(_motors.map((m) => [m.name, m]));

type DecelerationMethod = "Brake" | "Coast" | "Reverse";
const decelerationMethodToAccelerationThreshold: Record<
  DecelerationMethod,
  number
> = {
  Brake: 0.2,
  Coast: 0.05,
  Reverse: 1.0,
};
function ilite(args: {
  motorsAndGearbox: {
    motor: Motor;
    motorCount: number;
    ratio: number;
  };
  wheelsAndWheelBase: {
    wheelDiameter: number;
    coefficientOfFrictionStatic: number;
    coefficientOfFrictionDynamic: number;
    coefficientOfFrictionLateral: number;
    wheelBaseLength: number;
    wheelTrackWidth: number;
    weightDistributionFrontBack: number;
    weightDistributionLeftRight: number;
  };
  weight: {
    robotInspectionWeight: number;
    auxilliaryWeight: number;
  };
  fieldAndMatchSettings: {
    sprintDistance: number;
    targetTimeToGoal: number;
    numCyclesPerMatch: number;
    decelerationMethod: DecelerationMethod;
  };
  electricalSystemSettings: {
    batteryVoltageAtRest: number;
    appliedVoltageRamp: number;
    motorCurrentLimit: number;
    batteryResistance: number;
    batteryAmpHourRating: number;
    peakBatteryDischarge: number;
  };
  advancedSettings: {
    maxSimulationTime: number;
    minGearRatio: number;
    maxGearRatio: number;
    filtering: number;
    maxSpeedAccelerationThreshold: number;
    throttleResponseMin: number;
    throttleResponseMax: number;
  };
}) {
  const totalWeight =
    args.weight.robotInspectionWeight + args.weight.auxilliaryWeight;
  const efficiencies = [
    0.975 ** (args.motorsAndGearbox.motorCount / 2),
    0.95,
    0.92,
  ];
  const gearing = 1 / args.motorsAndGearbox.ratio;
  const deceleration_complete_threshold =
    decelerationMethodToAccelerationThreshold[
      args.fieldAndMatchSettings.decelerationMethod
    ];
  const spec_voltage = 12;
  const expected_voltage_loss = 0.18;
  const num_sim_rows = 100;
  const applied_voltage_ratio =
    (args.electricalSystemSettings.batteryVoltageAtRest -
      expected_voltage_loss) /
    spec_voltage;
  const actual_free_speed =
    (args.motorsAndGearbox.motor.freeSpeed.magnitude *
      args.electricalSystemSettings.batteryVoltageAtRest) /
    spec_voltage;
  const stall_torque =
    args.motorsAndGearbox.motor.stallTorque.magnitude * applied_voltage_ratio;
  const duty_cycle = 0.75;
  const stall_current =
    args.motorsAndGearbox.motor.stallCurrent.magnitude * applied_voltage_ratio;
  const free_current =
    args.motorsAndGearbox.motor.freeCurrent.magnitude * applied_voltage_ratio;
  const current_limit = Math.min(
    args.electricalSystemSettings.motorCurrentLimit,
    ((args.electricalSystemSettings.batteryVoltageAtRest / 12) *
      args.electricalSystemSettings.batteryAmpHourRating *
      args.electricalSystemSettings.peakBatteryDischarge) /
      args.motorsAndGearbox.motorCount
  );
  const sim_time_res = args.advancedSettings.maxSimulationTime / num_sim_rows;
  const gearbox_wheel_efficiency = efficiencies.reduce(
    (prev, curr) => prev * curr
  );
  const mass = totalWeight * 0.4536;
  const radius = (args.wheelsAndWheelBase.wheelDiameter / 2) * 0.0254;
  const current_limited_max_motor_torque =
    ((current_limit - free_current) / (stall_current - free_current)) *
    stall_torque;
  const weight_times_cof =
    (mass / 0.4536) *
    args.wheelsAndWheelBase.coefficientOfFrictionStatic *
    4.448;
  const max_torque_at_wheel = weight_times_cof * radius;
  const max_motor_torque =
    (max_torque_at_wheel * gearing) /
    args.motorsAndGearbox.motorCount /
    gearbox_wheel_efficiency;
  const current_while_pushing =
    ((max_motor_torque / stall_torque) * (stall_current - free_current) +
      free_current) *
    args.motorsAndGearbox.motorCount *
    duty_cycle;
  const dist_coa_com = Math.sqrt(
    ((1 - args.wheelsAndWheelBase.weightDistributionFrontBack) *
      args.wheelsAndWheelBase.wheelBaseLength -
      args.wheelsAndWheelBase.wheelBaseLength / 2) **
      2 +
      (args.wheelsAndWheelBase.wheelTrackWidth *
        args.wheelsAndWheelBase.weightDistributionLeftRight -
        args.wheelsAndWheelBase.wheelTrackWidth / 2) **
        2
  );
  const dimension_turn_cond =
    args.wheelsAndWheelBase.coefficientOfFrictionStatic *
      args.wheelsAndWheelBase.wheelTrackWidth >
    args.wheelsAndWheelBase.coefficientOfFrictionLateral *
      (args.wheelsAndWheelBase.wheelBaseLength -
        (4 * dist_coa_com ** 2) / args.wheelsAndWheelBase.wheelBaseLength);
  const geared_stall_torque =
    ((stall_torque * args.motorsAndGearbox.motorCount) / gearing) *
    gearbox_wheel_efficiency;
  const force_to_turn =
    ((args.wheelsAndWheelBase.coefficientOfFrictionLateral *
      totalWeight *
      4.448222) /
      (args.wheelsAndWheelBase.wheelTrackWidth * 0.0254)) *
    ((args.wheelsAndWheelBase.wheelBaseLength * 0.0254) / 4 -
      (dist_coa_com * 0.0254) ** 2 /
        (args.wheelsAndWheelBase.wheelBaseLength * 0.0254));
  const force_turn_cond = geared_stall_torque / radius / 4 > force_to_turn;
  const torque_to_turn = force_to_turn * radius;
  const estimated_torque_loss =
    (((1 - gearbox_wheel_efficiency) * 4.448222 * totalWeight) /
      (4.448222 * totalWeight)) *
    stall_torque;
  const delta_volts =
    args.electricalSystemSettings.appliedVoltageRamp * sim_time_res;
  const max_motor_torque_before_wheel_slip = max_motor_torque;
  const max_tractive_force_at_wheels =
    ((Math.min(
      max_motor_torque_before_wheel_slip,
      current_limited_max_motor_torque
    ) *
      args.motorsAndGearbox.motorCount *
      args.motorsAndGearbox.ratio) /
      radius /
      4.448) *
    gearbox_wheel_efficiency;
  const output_current_max_tractive_force =
    (((Math.min(
      max_motor_torque_before_wheel_slip,
      current_limited_max_motor_torque
    ) /
      stall_torque) *
      (stall_current - free_current) +
      free_current) *
      args.motorsAndGearbox.motorCount *
      duty_cycle) /
    gearbox_wheel_efficiency;
  const voltage_max_tractive_force =
    args.electricalSystemSettings.batteryVoltageAtRest -
    output_current_max_tractive_force *
      args.electricalSystemSettings.batteryResistance;
  const turning_motor_load =
    (torque_to_turn * gearing) / args.motorsAndGearbox.motorCount +
    estimated_torque_loss * gearing;
  const turning_Current = Math.min(
    ((turning_motor_load / stall_torque) * (stall_current - free_current) +
      free_current) *
      args.motorsAndGearbox.motorCount,
    current_limit * args.motorsAndGearbox.motorCount
  );
  const max_theoretical_speed =
    (actual_free_speed *
      gearing *
      args.wheelsAndWheelBase.wheelDiameter *
      Math.PI) /
    12 /
    60;

  const offset = (n: number) => n - 8;
  const _applied_voltage_ratios = [0];
  const _motor_speed = [0];
  const _attempted_torque_at_motor = [0];
  const _attempted_current_draw = [0];
  const _actual_applied_torque = [0];
  const _distance = [0];
  const _is_max_speed = [false];
  const _floor_speed = [0];
  const _applied_voltage = [0];
  const _is_hit_target = [
    _distance[offset(8)] >= args.fieldAndMatchSettings.sprintDistance,
  ];
  const _is_finished_decelerating = [_is_hit_target[offset(8)]];
  const _applied_acceleration = [
    _is_finished_decelerating[offset(8)] && _is_hit_target[offset(8)]
      ? 0
      : ((_actual_applied_torque[offset(8)] * stall_current) /
          gearing /
          radius /
          mass) *
        3.39 *
        gearbox_wheel_efficiency,
  ];

  const _abs_acceleration = [Math.abs(_applied_acceleration[offset(8)])];
  const _clamped_per_motor_current_draw = [
    Math.min(Math.abs(_attempted_current_draw[offset(8)]), current_limit) *
      Math.sign(_attempted_current_draw[offset(8)]),
  ];

  const _actual_current_draw = [
    _is_finished_decelerating[offset(8)] && _is_hit_target[offset(8)]
      ? 0
      : Math.min(
          (Math.abs(
            ((stall_current - free_current) *
              _actual_applied_torque[offset(8)]) /
              stall_torque
          ) +
            free_current) /
            gearbox_wheel_efficiency,
          _clamped_per_motor_current_draw[offset(8)]
        ),
  ];
  const _abs_applied_voltage = [Math.abs(_applied_voltage[offset(8)])];
  const _is_current_limiting = [
    Math.abs(_actual_current_draw[offset(8)]) >=
      _clamped_per_motor_current_draw[offset(8)] - 1 &&
      Math.abs(_attempted_current_draw[offset(8)]) >= current_limit,
  ];
  const _system_voltage = [
    args.electricalSystemSettings.batteryVoltageAtRest -
      _actual_current_draw[offset(8)] *
        duty_cycle *
        args.motorsAndGearbox.motorCount *
        args.electricalSystemSettings.batteryResistance,
  ];
  const _coulombs = [
    (Math.min(_actual_current_draw[offset(8)], current_limit) *
      sim_time_res *
      1000) /
      60 /
      60 /
      gearbox_wheel_efficiency,
  ];
  const _applied_cof = [
    (mass / 0.4536) *
      args.wheelsAndWheelBase.coefficientOfFrictionStatic *
      4.448 *
      radius *
      gearing,
  ];
  const _is_wheel_slipping = [false];
  const _times = linspace(
    args.advancedSettings.maxSimulationTime / num_sim_rows,
    args.advancedSettings.maxSimulationTime,
    args.advancedSettings.maxSimulationTime / num_sim_rows
  );

  let ttg = 0;
  for (const [row_, time] of _times.entries()) {
    const row = row_ + 9;

    _floor_speed.push(
      Math.max(
        _applied_acceleration[offset(row - 1)] * sim_time_res +
          _floor_speed[offset(row - 1)],
        0
      )
    );

    if (!_is_hit_target[offset(row - 1)]) {
      _applied_voltage.push(
        Math.min(
          args.electricalSystemSettings.batteryVoltageAtRest -
            _actual_current_draw[offset(row - 1)] *
              args.motorsAndGearbox.motorCount *
              args.electricalSystemSettings.batteryResistance,
          _applied_voltage[offset(row - 1)] + delta_volts
        ) - expected_voltage_loss
      );
    } else {
      if (_floor_speed[offset(row - 1)] - 1 <= 0) {
        _applied_voltage.push(0);
      } else {
        if (args.fieldAndMatchSettings.decelerationMethod == "Reverse") {
          _applied_voltage.push(
            Math.max(
              Math.max(..._applied_voltage) * -1,
              _applied_voltage[offset(row - 1)] - delta_volts
            )
          );
        } else {
          _applied_voltage.push(
            Math.max(
              0,
              Math.max(..._applied_voltage) * -1,
              _applied_voltage[offset(row - 1)] - delta_volts
            )
          );
        }
      }
    }

    _applied_voltage_ratios.push(
      _applied_voltage[offset(row)] /
        args.electricalSystemSettings.batteryVoltageAtRest
    );

    _abs_applied_voltage.push(Math.abs(_applied_voltage[offset(row)]));

    _motor_speed.push(
      ((_floor_speed[offset(row)] * 12 * 60) /
        (Math.PI * args.wheelsAndWheelBase.wheelDiameter) /
        gearing) *
        Math.abs(_applied_voltage_ratios[offset(row)])
    );

    if (
      _is_finished_decelerating[offset(row - 1)] ||
      (args.fieldAndMatchSettings.decelerationMethod == "Brake" &&
        _is_hit_target[offset(row - 1)])
    ) {
      _attempted_torque_at_motor.push(0);
    } else {
      if (_applied_voltage[offset(row)] == 0) {
        _attempted_torque_at_motor.push(
          (-(_attempted_current_draw[offset(row - 1)] + free_current) /
            (stall_current - free_current)) *
            stall_torque
        );
      } else {
        _attempted_torque_at_motor.push(
          ((actual_free_speed * _applied_voltage_ratios[offset(row)] -
            _motor_speed[offset(row)]) /
            (actual_free_speed * _applied_voltage_ratios[offset(row)])) *
            stall_torque *
            _applied_voltage_ratios[offset(row)]
        );
      }
    }

    _attempted_current_draw.push(
      _is_finished_decelerating[offset(row - 1)] &&
        _is_hit_target[offset(row - 1)]
        ? 0
        : Math.abs(
            (_attempted_torque_at_motor[offset(row)] / stall_torque) *
              (stall_current - free_current) +
              free_current
          ) *
            args.advancedSettings.filtering +
            _attempted_current_draw[offset(row - 1)] *
              (1 - args.advancedSettings.filtering)
    );

    _is_current_limiting.push(
      Math.abs(_attempted_current_draw[offset(row)]) >= current_limit
    );

    _clamped_per_motor_current_draw.push(
      Math.min(
        Math.abs(_attempted_current_draw[offset(row)]),
        current_limit * Math.sign(_attempted_current_draw[offset(row)])
      )
    );

    _actual_applied_torque.push(
      Math.max(
        Math.min(
          (((_is_hit_target[offset(row - 1)] &&
          _applied_voltage[offset(row)] < 0
            ? -1
            : 1) *
            (_clamped_per_motor_current_draw[offset(row)] - free_current)) /
            (stall_current - free_current)) *
            stall_torque *
            gearbox_wheel_efficiency -
            (estimated_torque_loss * _floor_speed[offset(row)]) /
              max_theoretical_speed -
            ((_is_hit_target[offset(row - 1)] ? 1 : 0) *
              estimated_torque_loss) /
              gearbox_wheel_efficiency,
          _applied_cof[offset(row - 1)] / args.motorsAndGearbox.motorCount
        ),
        Math.max(..._actual_applied_torque) * -1
      )
    );

    _is_wheel_slipping.push(
      Math.abs(_attempted_current_draw[offset(row)]) > 0 &&
        _actual_applied_torque[offset(row)] *
          args.motorsAndGearbox.motorCount >=
          _applied_cof[offset(row - 1)] &&
        _applied_voltage[offset(row)] != 0
    );

    _applied_acceleration.push(
      _is_finished_decelerating[offset(row - 1)] &&
        _is_hit_target[offset(row - 1)]
        ? 0
        : ((_actual_applied_torque[offset(row)] *
            args.motorsAndGearbox.motorCount) /
            gearing /
            radius /
            mass) *
            3.39 *
            gearbox_wheel_efficiency
    );

    _abs_acceleration.push(Math.abs(_applied_acceleration[offset(row)]));

    _distance.push(
      _distance[offset(row - 1)] +
        0.5 * _applied_acceleration[offset(row)] * sim_time_res ** 2 +
        _floor_speed[offset(row)] * sim_time_res
    );

    _is_hit_target.push(
      _distance[offset(row)] >= args.fieldAndMatchSettings.sprintDistance
    );
    if (_is_hit_target[_is_hit_target.length - 1] && ttg == 0) {
      ttg = time;
    }

    if (_is_hit_target[offset(row)]) {
      _applied_cof.push(_attempted_torque_at_motor[offset(row)]);
    } else {
      _applied_cof.push(
        (mass / 0.4536) *
          (_is_wheel_slipping[offset(row - 1)]
            ? args.wheelsAndWheelBase.coefficientOfFrictionDynamic
            : args.wheelsAndWheelBase.coefficientOfFrictionStatic) *
          4.448 *
          radius *
          gearing *
          Math.sign(_attempted_torque_at_motor[offset(row)])
      );
    }

    _is_finished_decelerating.push(
      _is_finished_decelerating[offset(row - 1)] ||
        (_is_hit_target[offset(row)] &&
          _floor_speed[offset(row)] <= deceleration_complete_threshold)
    );

    _actual_current_draw.push(
      _is_finished_decelerating[offset(row)] && _is_hit_target[offset(row)]
        ? 0
        : Math.min(
            (Math.abs(
              ((stall_current - free_current) *
                _actual_applied_torque[offset(row)]) /
                stall_torque /
                gearbox_wheel_efficiency
            ) +
              free_current) /
              gearbox_wheel_efficiency,
            _clamped_per_motor_current_draw[offset(row)]
          )
    );

    _system_voltage.push(
      args.electricalSystemSettings.batteryVoltageAtRest -
        _actual_current_draw[offset(row)] *
          duty_cycle *
          args.motorsAndGearbox.motorCount *
          args.electricalSystemSettings.batteryResistance
    );

    _coulombs.push(
      (Math.min(_actual_current_draw[offset(row)], current_limit) *
        sim_time_res *
        1000) /
        60 /
        60 /
        gearbox_wheel_efficiency
    );

    _is_max_speed.push(
      !_is_finished_decelerating[offset(row)] &&
        _abs_acceleration[offset(row)] <=
          args.advancedSettings.maxSpeedAccelerationThreshold
    );
  }

  console.log({
    max_tractive_force_at_wheels,
    ttg,
  });
}

ilite({
  motorsAndGearbox: {
    motor: motors["NEO"],
    motorCount: 4,
    ratio: 6,
  },
  weight: {
    robotInspectionWeight: 120,
    auxilliaryWeight: 0,
  },
  wheelsAndWheelBase: {
    wheelDiameter: 4,
    coefficientOfFrictionStatic: 1.1,
    coefficientOfFrictionDynamic: 0.9,
    coefficientOfFrictionLateral: 1.1,
    wheelBaseLength: 27,
    wheelTrackWidth: 20,
    weightDistributionFrontBack: 0.5,
    weightDistributionLeftRight: 0.5,
  },
  fieldAndMatchSettings: {
    sprintDistance: 40,
    targetTimeToGoal: 2.0,
    numCyclesPerMatch: 24,
    decelerationMethod: "Brake",
  },
  electricalSystemSettings: {
    batteryVoltageAtRest: 12.6,
    appliedVoltageRamp: 1200,
    motorCurrentLimit: 50,
    batteryResistance: 0.018,
    batteryAmpHourRating: 18,
    peakBatteryDischarge: 20,
  },
  advancedSettings: {
    maxSimulationTime: 4,
    minGearRatio: 3,
    maxGearRatio: 15,
    filtering: 1.0,
    maxSpeedAccelerationThreshold: 0.15,
    throttleResponseMin: 0.5,
    throttleResponseMax: 0.99,
  },
});
