function nearestMultiple(n: number, to: number): number {
  return Math.round(n * to) / to;
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
  const mass =
    (args.weight.robotInspectionWeight + args.weight.auxilliaryWeight) * 0.4536;
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
