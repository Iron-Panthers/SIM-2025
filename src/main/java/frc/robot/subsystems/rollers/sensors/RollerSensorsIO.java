package frc.robot.subsystems.rollers.sensors;

import org.littletonrobotics.junction.AutoLog;

public interface RollerSensorsIO {
  @AutoLog
  class RollerSensorsIOInputs {
    public boolean intakeDetected = false;
  }

  public default void updateInputs(RollerSensorsIOInputs inputs) {}
}
