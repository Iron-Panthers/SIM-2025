package frc.robot.subsystems.superstructure.elevator;

import static frc.robot.subsystems.superstructure.elevator.ElevatorConstants.*;

import frc.robot.subsystems.superstructure.GenericSuperstructureConfiguration;
import frc.robot.subsystems.superstructure.GenericSuperstructureIOTalonFX;

public class ElevatorIOTalonFX extends GenericSuperstructureIOTalonFX implements ElevatorIO {

  public ElevatorIOTalonFX() {

    super(
        new GenericSuperstructureConfiguration()
            .withID(ELEVATOR_CONFIG.motorID())
            .withMotorDirection(MOTOR_DIRECTION)
            .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
            .withReduction(ELEVATOR_CONFIG.reduction())
            .withUpperVoltageLimit(UPPER_VOLT_LIMIT)
            .withLowerVoltageLimit(LOWER_VOLT_LIMIT)
            .withZeroingVolts(ZEROING_VOLTS)
            .withZeroingOffset(ZEROING_OFFSET)
            .withZeroingVoltageThreshold(ZEROING_VOLTAGE_THRESHOLD)
            .withSecondaryMotorID(ELEVATOR_CONFIG.motorID2())
            .withSecondaryMotorDirection(OPOSE_MOTOR)
            .withUpperExtensionLimit(UPPER_EXTENSION_LIMIT));

    setSlot0(
        GAINS.kP(),
        GAINS.kI(),
        GAINS.kD(),
        GAINS.kS(),
        GAINS.kV(),
        GAINS.kA(),
        GAINS.kG(),
        MOTION_MAGIC_CONFIG.acceleration(),
        MOTION_MAGIC_CONFIG.cruiseVelocity(),
        MOTION_MAGIC_CONFIG.jerk(),
        GRAVITY_TYPE);
  }
}
