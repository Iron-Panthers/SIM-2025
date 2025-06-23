package frc.robot.subsystems.superstructure.elevator;

import static frc.robot.subsystems.superstructure.elevator.ElevatorConstants.*;

import frc.robot.subsystems.superstructure.GenericSuperstructureIOTalonFX;

public class ElevatorIOTalonFX extends GenericSuperstructureIOTalonFX implements ElevatorIO {

  public ElevatorIOTalonFX() {
    super(
        ELEVATOR_CONFIG.motorID(),
        INVERT_MOTOR,
        SUPPLY_CURRENT_LIMIT,
        ELEVATOR_CONFIG.reduction(),
        UPPER_VOLT_LIMIT,
        LOWER_VOLT_LIMIT,
        ZEROING_VOLTS,
        ZEROING_OFFSET,
        ZEROING_VOLTAGE_THRESHOLD);

    setSecondMotorConfigs(ELEVATOR_CONFIG.motorID2(), OPOSE_MOTOR);

    setUpperExtensionLimit(UPPER_EXTENSION_LIMIT);

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
