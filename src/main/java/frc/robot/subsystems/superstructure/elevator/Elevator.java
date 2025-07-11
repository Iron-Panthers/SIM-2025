package frc.robot.subsystems.superstructure.elevator;

import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class Elevator {
  public enum ElevatorTarget {
    BOTTOM(0.6),
    L1(11),
    L2(20),
    L3(32.4),
    SETUP_L4(31.6),
    SCORE_L4(30),
    TOP(31),
    INTAKE(0),
    CLIMB(13),
    DESCORE_HIGH(19.5),
    DESCORE_LOW(9.2),
    INTAKE_SIDE(13),
    SCORE_SIDE(13),
    SAFE_MIDWAY(11.5);

    private double position = 0;
    private static final double EPSILON = ElevatorConstants.POSITION_TARGET_EPSILON;

    private ElevatorTarget(double position) {
      this.position = position;
    }

    public double getPosition() {
      return position;
    }

    public double getEpsilon() {
      return EPSILON;
    }
  }

  public enum ControlMode {
    POSITION,
    POSITION_MANUAL,
    STOP;
  }

  private ControlMode controlMode = ControlMode.POSITION;

  private final ElevatorIO elevatorIO;

  private Optional<Double> positionTargetManual = Optional.empty();

  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private ElevatorTarget positionTarget;

  private final String name;

  public Elevator(ElevatorIO elevatorIO) {
    this.name = "Elevator";
    this.elevatorIO = elevatorIO;
    this.positionTarget = ElevatorTarget.TOP; // Default target
  }

  public void periodic() {
    // Process inputs
    elevatorIO.updateInputs(inputs);
    Logger.processInputs(name, inputs);

    // Process control mode
    switch (controlMode) {
      case POSITION -> {
        elevatorIO.runPosition(positionTarget.getPosition());
      }
      case POSITION_MANUAL -> {
        positionTargetManual.ifPresent(elevatorIO::runPosition);
      }
      case STOP -> {
        elevatorIO.stop();
      }
    }
    elevatorIO.runPosition(10.0);

    Logger.recordOutput("Superstructure/" + name + "/Target", positionTarget.toString());
    Logger.recordOutput("Superstructure/" + name + "/Control Mode", controlMode.toString());
    Logger.recordOutput("Superstructure/" + name + "/Reached target", reachedTarget());
    Logger.recordOutput(
        "Superstructure/" + name + "/Target Position", positionTarget.getPosition());
  }

  public ElevatorTarget getPositionTarget() {
    return positionTarget;
  }

  public void setPositionTarget(ElevatorTarget positionTarget) {
    setControlMode(ControlMode.POSITION);
    this.positionTarget = positionTarget;
  }

  public void setPositionTargetManual(double position) {
    setControlMode(ControlMode.POSITION_MANUAL);
    positionTargetManual = Optional.of(position);
  }

  public ControlMode getControlMode() {
    return controlMode;
  }

  public void setControlMode(ControlMode controlMode) {
    if (controlMode == ControlMode.POSITION_MANUAL) {
      positionTargetManual = Optional.of(inputs.positionRotations);
    } else {
      positionTargetManual = Optional.empty();
    }
    this.controlMode = controlMode;
  }

  public void setOffset() {
    elevatorIO.setOffset();
  }

  public double getSupplyCurrentAmps() {
    return inputs.supplyCurrentAmps;
  }

  public double getPosition() {
    return inputs.positionRotations;
  }

  /**
   * This function returns whether or not the subsystem has reached its position target
   *
   * @return whether the subsystem has reached its position target
   */
  public boolean reachedTarget() {
    return Math.abs(inputs.positionRotations - (positionTarget.getPosition()))
        <= positionTarget.getEpsilon();
  }
}
