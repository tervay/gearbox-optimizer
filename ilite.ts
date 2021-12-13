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

function* enumerate<T>(it: Iterable<T>, start = 0): Iterable<[number, T]> {
  let i = start;
  for (const x of it) {
    yield [i++, x];
  }
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
  const specVoltage = 12;
  const expectedVoltageLoss = 0.18;
  const numSimRows = 100;
  const appliedVoltageRatio =
    (args.electricalSystemSettings.batteryVoltageAtRest - expectedVoltageLoss) /
    specVoltage;
  const actualFreeSpeed =
    (args.motorsAndGearbox.motor.freeSpeed.magnitude *
      args.electricalSystemSettings.batteryVoltageAtRest) /
    specVoltage;
  const stallTorque =
    args.motorsAndGearbox.motor.stallTorque.magnitude * appliedVoltageRatio;
  const dutyCycle = 0.75;
  const stallCurrent =
    args.motorsAndGearbox.motor.stallCurrent.magnitude * appliedVoltageRatio;
  const freeCurrent =
    args.motorsAndGearbox.motor.freeCurrent.magnitude * appliedVoltageRatio;
  const currentLimit = Math.min(
    args.electricalSystemSettings.motorCurrentLimit,
    ((args.electricalSystemSettings.batteryVoltageAtRest / 12) *
      args.electricalSystemSettings.batteryAmpHourRating *
      args.electricalSystemSettings.peakBatteryDischarge) /
      args.motorsAndGearbox.motorCount
  );
  const simTimeRes = args.advancedSettings.maxSimulationTime / numSimRows;
  const gearboxWheelEfficiency = efficiencies.reduce(
    (prev, curr) => prev * curr
  );
  const mass = totalWeight * 0.4536;
  const radius = (args.wheelsAndWheelBase.wheelDiameter / 2) * 0.0254;
  const currentLimitedMaxMotorTorque =
    ((currentLimit - freeCurrent) / (stallCurrent - freeCurrent)) * stallTorque;
  const weightTimesCOF =
    (mass / 0.4536) *
    args.wheelsAndWheelBase.coefficientOfFrictionStatic *
    4.448;
  const maxTorqueAtWheel = weightTimesCOF * radius;
  const maxMotorTorque =
    (maxTorqueAtWheel * gearing) /
    args.motorsAndGearbox.motorCount /
    gearboxWheelEfficiency;
  const currentWhilePushing =
    ((maxMotorTorque / stallTorque) * (stallCurrent - freeCurrent) +
      freeCurrent) *
    args.motorsAndGearbox.motorCount *
    dutyCycle;
  const distanceCOM = Math.sqrt(
    ((1 - args.wheelsAndWheelBase.weightDistributionFrontBack) *
      args.wheelsAndWheelBase.wheelBaseLength -
      args.wheelsAndWheelBase.wheelBaseLength / 2) **
      2 +
      (args.wheelsAndWheelBase.wheelTrackWidth *
        args.wheelsAndWheelBase.weightDistributionLeftRight -
        args.wheelsAndWheelBase.wheelTrackWidth / 2) **
        2
  );
  const dimensionTurnCondition =
    args.wheelsAndWheelBase.coefficientOfFrictionStatic *
      args.wheelsAndWheelBase.wheelTrackWidth >
    args.wheelsAndWheelBase.coefficientOfFrictionLateral *
      (args.wheelsAndWheelBase.wheelBaseLength -
        (4 * distanceCOM ** 2) / args.wheelsAndWheelBase.wheelBaseLength);
  const gearedStallTorque =
    ((stallTorque * args.motorsAndGearbox.motorCount) / gearing) *
    gearboxWheelEfficiency;
  const forceToTurn =
    ((args.wheelsAndWheelBase.coefficientOfFrictionLateral *
      totalWeight *
      4.448222) /
      (args.wheelsAndWheelBase.wheelTrackWidth * 0.0254)) *
    ((args.wheelsAndWheelBase.wheelBaseLength * 0.0254) / 4 -
      (distanceCOM * 0.0254) ** 2 /
        (args.wheelsAndWheelBase.wheelBaseLength * 0.0254));
  const torqueToTurn = forceToTurn * radius;
  const forceTurnConditon = gearedStallTorque / radius / 4 > forceToTurn;
  const estimatedTorqueLoss =
    (((1 - gearboxWheelEfficiency) * 4.448222 * totalWeight) /
      (4.448222 * totalWeight)) *
    stallTorque;
  const deltaVolts =
    args.electricalSystemSettings.appliedVoltageRamp * simTimeRes;
  const maxMotorTorqueBeforeWheelSlip = maxMotorTorque;
  const maxTractiveForceAtWheels =
    ((Math.min(maxMotorTorqueBeforeWheelSlip, currentLimitedMaxMotorTorque) *
      args.motorsAndGearbox.motorCount *
      args.motorsAndGearbox.ratio) /
      radius /
      4.448) *
    gearboxWheelEfficiency;
  const outputCurrentAtMaxTractiveForce =
    (((Math.min(maxMotorTorqueBeforeWheelSlip, currentLimitedMaxMotorTorque) /
      stallTorque) *
      (stallCurrent - freeCurrent) +
      freeCurrent) *
      args.motorsAndGearbox.motorCount *
      dutyCycle) /
    gearboxWheelEfficiency;
  const voltageAtMaxTractiveForce =
    args.electricalSystemSettings.batteryVoltageAtRest -
    outputCurrentAtMaxTractiveForce *
      args.electricalSystemSettings.batteryResistance;
  const turningMotorLoad =
    (torqueToTurn * gearing) / args.motorsAndGearbox.motorCount +
    estimatedTorqueLoss * gearing;
  const turningCurrent = Math.min(
    ((turningMotorLoad / stallTorque) * (stallCurrent - freeCurrent) +
      freeCurrent) *
      args.motorsAndGearbox.motorCount,
    currentLimit * args.motorsAndGearbox.motorCount
  );
  const maxTheoreticalSpeed =
    (actualFreeSpeed *
      gearing *
      args.wheelsAndWheelBase.wheelDiameter *
      Math.PI) /
    12 /
    60;

  const appliedVoltageRatios = [0];
  const motorSpeed = [0];
  const attemptedTorqueAtMotor = [0];
  const attemptedCurrentDraw = [0];
  const actualAppliedTorque = [0];
  const distanceTraveled = [0];
  const isMaxSpeed = [false];
  const isWheelSlipping = [false];
  const floorSpeed = [0];
  const appliedVoltage = [0];
  const isHitTarget = [
    distanceTraveled[0] >= args.fieldAndMatchSettings.sprintDistance,
  ];
  const isFinishedDecelerating = [isHitTarget[0]];
  const appliedAcceleration = [
    isFinishedDecelerating[0] && isHitTarget[0]
      ? 0
      : ((actualAppliedTorque[0] * stallCurrent) / gearing / radius / mass) *
        3.39 *
        gearboxWheelEfficiency,
  ];

  const absAcceleration = [Math.abs(appliedAcceleration[0])];
  const clampedPerMotorCurrentDraw = [
    Math.min(Math.abs(attemptedCurrentDraw[0]), currentLimit) *
      Math.sign(attemptedCurrentDraw[0]),
  ];

  const actualCurrentDraw = [
    isFinishedDecelerating[0] && isHitTarget[0]
      ? 0
      : Math.min(
          (Math.abs(
            ((stallCurrent - freeCurrent) * actualAppliedTorque[0]) /
              stallTorque
          ) +
            freeCurrent) /
            gearboxWheelEfficiency,
          clampedPerMotorCurrentDraw[0]
        ),
  ];
  const absAppliedVoltage = [Math.abs(appliedVoltage[0])];
  const isCurrentLimiting = [
    Math.abs(actualCurrentDraw[0]) >= clampedPerMotorCurrentDraw[0] - 1 &&
      Math.abs(attemptedCurrentDraw[0]) >= currentLimit,
  ];
  const systemVoltage = [
    args.electricalSystemSettings.batteryVoltageAtRest -
      actualCurrentDraw[0] *
        dutyCycle *
        args.motorsAndGearbox.motorCount *
        args.electricalSystemSettings.batteryResistance,
  ];
  const coulombs = [
    (Math.min(actualCurrentDraw[0], currentLimit) * simTimeRes * 1000) /
      60 /
      60 /
      gearboxWheelEfficiency,
  ];
  const appliedCOF = [
    (mass / 0.4536) *
      args.wheelsAndWheelBase.coefficientOfFrictionStatic *
      4.448 *
      radius *
      gearing,
  ];
  const timesteps = linspace(
    args.advancedSettings.maxSimulationTime / numSimRows,
    args.advancedSettings.maxSimulationTime,
    args.advancedSettings.maxSimulationTime / numSimRows
  );

  let ttg = 0;
  for (const [row, time] of enumerate(timesteps, 1)) {
    floorSpeed.push(
      Math.max(
        appliedAcceleration[row - 1] * simTimeRes + floorSpeed[row - 1],
        0
      )
    );

    if (!isHitTarget[row - 1]) {
      appliedVoltage.push(
        Math.min(
          args.electricalSystemSettings.batteryVoltageAtRest -
            actualCurrentDraw[row - 1] *
              args.motorsAndGearbox.motorCount *
              args.electricalSystemSettings.batteryResistance,
          appliedVoltage[row - 1] + deltaVolts
        ) - expectedVoltageLoss
      );
    } else {
      if (floorSpeed[row - 1] - 1 <= 0) {
        appliedVoltage.push(0);
      } else {
        if (args.fieldAndMatchSettings.decelerationMethod == "Reverse") {
          appliedVoltage.push(
            Math.max(
              Math.max(...appliedVoltage) * -1,
              appliedVoltage[row - 1] - deltaVolts
            )
          );
        } else {
          appliedVoltage.push(
            Math.max(
              0,
              Math.max(...appliedVoltage) * -1,
              appliedVoltage[row - 1] - deltaVolts
            )
          );
        }
      }
    }

    appliedVoltageRatios.push(
      appliedVoltage[row] / args.electricalSystemSettings.batteryVoltageAtRest
    );

    absAppliedVoltage.push(Math.abs(appliedVoltage[row]));

    motorSpeed.push(
      ((floorSpeed[row] * 12 * 60) /
        (Math.PI * args.wheelsAndWheelBase.wheelDiameter) /
        gearing) *
        Math.abs(appliedVoltageRatios[row])
    );

    if (
      isFinishedDecelerating[row - 1] ||
      (args.fieldAndMatchSettings.decelerationMethod == "Brake" &&
        isHitTarget[row - 1])
    ) {
      attemptedTorqueAtMotor.push(0);
    } else {
      if (appliedVoltage[row] == 0) {
        attemptedTorqueAtMotor.push(
          (-(attemptedCurrentDraw[row - 1] + freeCurrent) /
            (stallCurrent - freeCurrent)) *
            stallTorque
        );
      } else {
        attemptedTorqueAtMotor.push(
          ((actualFreeSpeed * appliedVoltageRatios[row] - motorSpeed[row]) /
            (actualFreeSpeed * appliedVoltageRatios[row])) *
            stallTorque *
            appliedVoltageRatios[row]
        );
      }
    }

    attemptedCurrentDraw.push(
      isFinishedDecelerating[row - 1] && isHitTarget[row - 1]
        ? 0
        : Math.abs(
            (attemptedTorqueAtMotor[row] / stallTorque) *
              (stallCurrent - freeCurrent) +
              freeCurrent
          ) *
            args.advancedSettings.filtering +
            attemptedCurrentDraw[row - 1] *
              (1 - args.advancedSettings.filtering)
    );

    isCurrentLimiting.push(Math.abs(attemptedCurrentDraw[row]) >= currentLimit);

    clampedPerMotorCurrentDraw.push(
      Math.min(
        Math.abs(attemptedCurrentDraw[row]),
        currentLimit * Math.sign(attemptedCurrentDraw[row])
      )
    );

    actualAppliedTorque.push(
      Math.max(
        Math.min(
          (((isHitTarget[row - 1] && appliedVoltage[row] < 0 ? -1 : 1) *
            (clampedPerMotorCurrentDraw[row] - freeCurrent)) /
            (stallCurrent - freeCurrent)) *
            stallTorque *
            gearboxWheelEfficiency -
            (estimatedTorqueLoss * floorSpeed[row]) / maxTheoreticalSpeed -
            ((isHitTarget[row - 1] ? 1 : 0) * estimatedTorqueLoss) /
              gearboxWheelEfficiency,
          appliedCOF[row - 1] / args.motorsAndGearbox.motorCount
        ),
        Math.max(...actualAppliedTorque) * -1
      )
    );

    isWheelSlipping.push(
      Math.abs(attemptedCurrentDraw[row]) > 0 &&
        actualAppliedTorque[row] * args.motorsAndGearbox.motorCount >=
          appliedCOF[row - 1] &&
        appliedVoltage[row] != 0
    );

    appliedAcceleration.push(
      isFinishedDecelerating[row - 1] && isHitTarget[row - 1]
        ? 0
        : ((actualAppliedTorque[row] * args.motorsAndGearbox.motorCount) /
            gearing /
            radius /
            mass) *
            3.39 *
            gearboxWheelEfficiency
    );

    absAcceleration.push(Math.abs(appliedAcceleration[row]));

    distanceTraveled.push(
      distanceTraveled[row - 1] +
        0.5 * appliedAcceleration[row] * simTimeRes ** 2 +
        floorSpeed[row] * simTimeRes
    );

    isHitTarget.push(
      distanceTraveled[row] >= args.fieldAndMatchSettings.sprintDistance
    );
    if (isHitTarget[isHitTarget.length - 1] && ttg == 0) {
      ttg = time;
    }

    if (isHitTarget[row]) {
      appliedCOF.push(attemptedTorqueAtMotor[row]);
    } else {
      appliedCOF.push(
        (mass / 0.4536) *
          (isWheelSlipping[row - 1]
            ? args.wheelsAndWheelBase.coefficientOfFrictionDynamic
            : args.wheelsAndWheelBase.coefficientOfFrictionStatic) *
          4.448 *
          radius *
          gearing *
          Math.sign(attemptedTorqueAtMotor[row])
      );
    }

    isFinishedDecelerating.push(
      isFinishedDecelerating[row - 1] ||
        (isHitTarget[row] && floorSpeed[row] <= deceleration_complete_threshold)
    );

    actualCurrentDraw.push(
      isFinishedDecelerating[row] && isHitTarget[row]
        ? 0
        : Math.min(
            (Math.abs(
              ((stallCurrent - freeCurrent) * actualAppliedTorque[row]) /
                stallTorque /
                gearboxWheelEfficiency
            ) +
              freeCurrent) /
              gearboxWheelEfficiency,
            clampedPerMotorCurrentDraw[row]
          )
    );

    systemVoltage.push(
      args.electricalSystemSettings.batteryVoltageAtRest -
        actualCurrentDraw[row] *
          dutyCycle *
          args.motorsAndGearbox.motorCount *
          args.electricalSystemSettings.batteryResistance
    );

    coulombs.push(
      (Math.min(actualCurrentDraw[row], currentLimit) * simTimeRes * 1000) /
        60 /
        60 /
        gearboxWheelEfficiency
    );

    isMaxSpeed.push(
      !isFinishedDecelerating[row] &&
        absAcceleration[row] <=
          args.advancedSettings.maxSpeedAccelerationThreshold
    );
  }

  console.log({
    max_tractive_force_at_wheels: maxTractiveForceAtWheels,
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
