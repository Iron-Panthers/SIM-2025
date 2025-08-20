package frc.robot.subsystems.superstructure.pivot;

import static frc.robot.subsystems.superstructure.pivot.PivotConstants.*;

import frc.robot.lib.generic_subsystems.superstructure.*;

public class PivotIOTalonFX extends GenericSuperstructureIOTalonFX implements PivotIO {

  public PivotIOTalonFX() {
    super(
        new GenericSuperstructureConfiguration()
            .withID(PIVOT_CONFIG.motorID())
            .withMotorDirection(MOTOR_DIRECTION)
            .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
            .withReduction(PIVOT_CONFIG.reduction())
            .withUpperVoltageLimit(UPPER_VOLT_LIMIT)
            .withLowerVoltageLimit(LOWER_VOLT_LIMIT)
            .withZeroingVolts(ZEROING_VOLTS)
            .withZeroingOffset(ZEROING_OFFSET)
            .withZeroingVoltageThreshold(ZEROING_VOLTAGE_THRESHOLD)
            .withCANCoderID(PIVOT_CONFIG.canCoderID())
            .withCANCoderOffset(PIVOT_CONFIG.canCoderOffset())
            .withCANCoderDirection(CANCODER_DIRECTION)
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
        0,
        GRAVITY_TYPE);
  }

  @Override
  public void runPosition(double position) {
    super.runPosition(position / 360d); // convert degrees to rotations
  }
}
