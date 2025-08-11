package frc.robot.subsystems.superstructure.climb;

import static frc.robot.subsystems.superstructure.climb.ClimbConstants.*;

import com.ctre.phoenix6.configs.VoltageConfigs;
import frc.robot.subsystems.superstructure.GenericSuperstructureIOTalonFX;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;

public class ClimbIOTalonFX extends GenericSuperstructureIOTalonFX implements ClimbIO {

  public ClimbIOTalonFX() {
    super(
        CLIMB_CONFIG.motorID(),
        Optional.empty(),
        INVERT_MOTOR,
        Optional.empty(),
        SUPPLY_CURRENT_LIMIT,
        CLIMB_CONFIG.canCoderID(),
        CLIMB_CONFIG.canCoderOffset(),
        CANCODER_DIRECTION,
        SENSOR_DISCONTINUITY_POINT,
        CLIMB_CONFIG.reduction(),
        UPPER_EXTENSION_LIMIT,
        LOWER_EXTENSION_LIMIT,
        UPPER_VOLT_LIMIT,
        LOWER_VOLT_LIMIT,
        ZEROING_VOLTS,
        ZEROING_OFFSET,
        ZEROING_VOLTAGE_THRESHOLD);
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

  @AutoLogOutput(key = "Superstructure/Climb/ModdedRotations")
  public double moddedRotations;

  @Override
  public void runPosition(double rotations) {
    moddedRotations =
        rotations
            - (talon.getPosition().getValueAsDouble()
                - (talon.getPosition().getValueAsDouble()
                    % 0.4)); // calculates how much the fricking encoder is off by (so sad🥲)
    VoltageConfigs voltageConfigs = new VoltageConfigs();
    voltageConfigs.withPeakForwardVoltage(
        talon.getPosition().getValueAsDouble() % 0.4 > 0.2
            ? UPPER_VOLT_LIMIT_CLIMBING
            : UPPER_VOLT_LIMIT);
    voltageConfigs.withPeakReverseVoltage(LOWER_VOLT_LIMIT);
    talon.getConfigurator().apply(voltageConfigs);
    super.runPosition(moddedRotations);
  }
}
