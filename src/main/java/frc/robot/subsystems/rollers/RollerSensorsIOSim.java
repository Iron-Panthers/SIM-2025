package frc.robot.subsystems.rollers;

public class RollerSensorsIOSim implements RollerSensorsIO {

  @Override
  public void updateInputs(RollerSensorsIOInputs inputs) {
    inputs.intakeDetected = false;
  }
}
