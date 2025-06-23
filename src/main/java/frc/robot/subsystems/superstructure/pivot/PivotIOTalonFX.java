package frc.robot.subsystems.superstructure.pivot;

import static frc.robot.subsystems.superstructure.pivot.PivotConstants.*;

import frc.robot.subsystems.superstructure.GenericSuperstructureIOTalonFX;

public class PivotIOTalonFX extends GenericSuperstructureIOTalonFX implements PivotIO {

  public PivotIOTalonFX() {
    super(
        PIVOT_CONFIG.motorID(),
        INVERT_MOTOR,
        SUPPLY_CURRENT_LIMIT,
        PIVOT_CONFIG.reduction(),
        UPPER_VOLT_LIMIT,
        LOWER_VOLT_LIMIT,
        ZEROING_VOLTS,
        ZEROING_OFFSET,
        ZEROING_VOLTAGE_THRESHOLD);
    
    setCanCoderConfigs(PIVOT_CONFIG.canCoderID(), PIVOT_CONFIG.canCoderOffset(), SENSOR_DIRECTION);

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
        0,
        GRAVITY_TYPE);
  }

  @Override
  public void runPosition(double position) {
    super.runPosition(position / 360d); // convert degrees to rotations
  }
}
