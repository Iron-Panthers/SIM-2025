package frc.robot.lib.generic_subsystems.superstructure;

import com.ctre.phoenix6.signals.GravityTypeValue;
import org.littletonrobotics.junction.AutoLog;

public interface GenericSuperstructureIO {
  @AutoLog
  class GenericSuperstructureIOInputs {
    public boolean connected = true;
    public double positionRotations = 0;
    public double velocityRotPerSec = 0;
    public double appliedVolts = 0;
    public double supplyCurrentAmps = 0;
    public double tempCelsius = 0;
    public boolean zeroing = false;
  }

  @AutoLog
  class GenericSuperstructureIOInputsMotor2 {
    public boolean connected2 = true;
    public double positionRotations2 = 0;
    public double velocityRotPerSec2 = 0;
    public double appliedVolts2 = 0;
    public double supplyCurrentAmps2 = 0;
    public double tempCelsius2 = 0;
  }

  default void updateInputs(GenericSuperstructureIOInputs inputs) {}

  default void updateSecondaryInputs(GenericSuperstructureIOInputsMotor2 inputs) {}

  default void runPosition(double position) {}

  default void runCharacterization() {}

  default void setSlot0(
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kA,
      double kG,
      double motionMagicAcceleration,
      double motionMagicCruiseVelocity,
      double jerk,
      GravityTypeValue gravityTypeValue) {}

  default void stop() {}

  default void setOffset() {}
}
