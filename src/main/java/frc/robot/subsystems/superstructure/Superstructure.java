package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.elevator.Elevator;

public class Superstructure extends SubsystemBase {
  // This class serves as a placeholder for the superstructure subsystem.
  // It can be expanded with methods and properties related to the superstructure.
  private Elevator elevator;

  public Superstructure(Elevator elevator) {
    // Constructor logic can be added here if needed.
    this.elevator = elevator;
  }

  public void periodic() {
    elevator.periodic();
  }
}
