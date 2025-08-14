package frc.robot.subsystems.superstructure.tongue;

public class TongueIOSim implements TongueIO {

  public TongueIOSim() {}

  @Override
  public void runPosition(double position) {}

  @Override
  public void stop() {}

  @Override
  public void updateInputs(TongueIOInputsAutoLogged inputs) {
    inputs.angle = 0; // Simulated angle, replace with actual simulation logic if needed
    inputs.pole1Detected =
        false; // Simulated sensor state, replace with actual simulation logic if needed
    inputs.pole2Detected =
        false; // Simulated sensor state, replace with actual simulation logic if needed
  }
}
