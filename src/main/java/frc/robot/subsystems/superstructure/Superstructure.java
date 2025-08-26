package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructure.ControlMode;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.Elevator.ElevatorTarget;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants;
import frc.robot.subsystems.superstructure.pivot.Pivot;
import frc.robot.subsystems.superstructure.pivot.Pivot.PivotTarget;
import frc.robot.subsystems.superstructure.pivot.PivotConstants;
import frc.robot.subsystems.superstructure.tongue.Tongue;
import frc.robot.subsystems.superstructure.tongue.Tongue.TongueTarget;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {

  private enum SubstructureType { // for defining some of the stuff
    ELEVATOR,
    PIVOT,
    TONGUE
  }

  /**
   * Options for transitioning between superstructure states.
   *
   * @param defaultState The default state to transition to.
   * @param stateTransitions Map of possible state transitions for each state. (key: next state,
   *     target states that can be reached by going to that state)
   * @param elevatorTarget Optional target for the elevator mechanism.
   * @param pivotTarget Optional target for the pivot mechanism.
   * @param tongueTarget Optional target for the tongue mechanism.
   * @param targetChangeConditions Map of mechanism types to conditions that must be met for target
   *     changes.
   */
  private record StateTransitionOptions(
      ElevatorTarget elevatorTarget, PivotTarget pivotTarget, TongueTarget tongueTarget) {}

  public enum SuperstructureState {
    L1(
        new StateTransitionOptions(
            ElevatorTarget.L1, PivotTarget.L1, TongueTarget.L1)), // Scoring in the trough
    L2(
        new StateTransitionOptions(
            ElevatorTarget.L2, PivotTarget.L2, TongueTarget.L2)), // Scoring in L2
    SETUP_L3(
        new StateTransitionOptions(
            ElevatorTarget.L3, null /* PivotTarget.SETUP_L3 */, TongueTarget.L3)),
    SCORE_L3(
        new StateTransitionOptions(
            ElevatorTarget.L3, PivotTarget.SCORE_L3, TongueTarget.L3)), // Scoring in L3
    PREVENT_TIPPING(), // null because it has its own logic
    SETUP_L4(), // Setting up in L4 - has its own logic
    SCORE_L4(
        new StateTransitionOptions(
            ElevatorTarget.SCORE_L4, PivotTarget.SCORE_L4, TongueTarget.STOW)), // Scoring
    // in L4
    TOP(
        new StateTransitionOptions(
            ElevatorTarget.TOP, null /* PivotTarget.TOP */, TongueTarget.TOP)), // Apex
    STOW(
        new StateTransitionOptions(
            null /* ElevatorTarget.BOTTOM */, PivotTarget.STOW, TongueTarget.STOW)),
    INTAKE(
        new StateTransitionOptions(ElevatorTarget.INTAKE, PivotTarget.INTAKE, TongueTarget.INTAKE)),
    CLIMB(
        new StateTransitionOptions(
            ElevatorTarget.CLIMB, null /* PivotTarget.CLIMB */, TongueTarget.CLIMB)),
    DESCORE_HIGH(
        new StateTransitionOptions(
            null /* ElevatorTarget.DESCORE_HIGH */,
            PivotTarget.DESCORE_HIGH,
            TongueTarget.DESCORE)), // Algae hitting on L3
    DESCORE_LOW(
        new StateTransitionOptions(
            null /* ElevatorTarget.DESCORE_LOW */,
            PivotTarget.DESCORE_LOW,
            TongueTarget.DESCORE)), // Algae hitting on L2
    ZERO(); // Zero the motor

    static {
      // Define all of the transitions for the states
      L1.transitions = Set.of(L2, STOW, CLIMB, TOP);
      L2.transitions = Set.of(L1, STOW, CLIMB, TOP);
      SETUP_L3.transitions = Set.of(SCORE_L3, DESCORE_HIGH, DESCORE_LOW, PREVENT_TIPPING, TOP);
      SCORE_L3.transitions = Set.of(SETUP_L3);
      PREVENT_TIPPING.transitions =
          Set.of(SETUP_L4, DESCORE_HIGH, DESCORE_LOW, TOP); // Use diff logic
      SETUP_L4.transitions = Set.of(SCORE_L4, PREVENT_TIPPING); // Use diff logic
      SCORE_L4.transitions = Set.of(SETUP_L4, PREVENT_TIPPING);
      TOP.transitions =
          Set.of(PREVENT_TIPPING, CLIMB, L2, L1, DESCORE_HIGH, DESCORE_LOW, SETUP_L3, STOW);
      STOW.transitions = Set.of(INTAKE, L1, L2, CLIMB, TOP);
      INTAKE.transitions = Set.of(STOW, L1, L2, CLIMB, TOP);
      CLIMB.transitions = Set.of(L1, STOW, L2, TOP);
      DESCORE_HIGH.transitions = Set.of(PREVENT_TIPPING, DESCORE_LOW, SETUP_L3, TOP);
      DESCORE_LOW.transitions = Set.of(PREVENT_TIPPING, DESCORE_HIGH, SETUP_L3, TOP);
      // none for zero
    }

    // here we define some properties of the enum
    private Set<SuperstructureState> transitions;
    private StateTransitionOptions transitionOptions;

    // Constructor for states with transitions
    private SuperstructureState(StateTransitionOptions transitionOptions) {
      this.transitionOptions = transitionOptions;
    }

    private SuperstructureState() {
      this.transitionOptions = null;
    }

    // getters for the properties
    public StateTransitionOptions getTransitionOptions() {
      return transitionOptions;
    }

    public Set<SuperstructureState> getTransitions() {
      return transitions != null ? transitions : Set.of(); // return empty set if no transitions
    }
  }

  private SuperstructureState currentState = SuperstructureState.STOW; // current state
  private SuperstructureState targetState = SuperstructureState.STOW; // current target state
  private SuperstructureState bufferCurrentState = SuperstructureState.STOW;
  private boolean stop = false;

  private final Elevator elevator;
  private final Pivot pivot;
  private final Tongue tongue;

  private Pose3d elevatorPose3d;
  private Pose3d pivotPose3d;

  private boolean overrideIsAtTarget = false;

  public Superstructure(Elevator elevator, Pivot pivot, Tongue tongue) {

    this.elevator = elevator;
    this.pivot = pivot;
    this.tongue = tongue;

    pivot.setPositionTarget(PivotTarget.STOW);
    elevator.setPositionTarget(ElevatorTarget.BOTTOM);
    tongue.setPositionTarget(TongueTarget.STOW);

    pivot.setParent(elevator);

    elevatorPose3d = Pose3d.kZero;
    pivotPose3d = Pose3d.kZero;
  }

  /**
   * Finds a path from the start state to the goal state using breadth-first search.
   *
   * @param start The starting SuperstructureState.
   * @param goal The goal SuperstructureState.
   * @return A list of SuperstructureStates representing the path from start to goal, or null if no
   *     path is found.
   */
  public static List<SuperstructureState> findPath(
      SuperstructureState start, SuperstructureState goal) {
    Queue<List<SuperstructureState>> queue = new LinkedList<>();
    Set<SuperstructureState> visited = new HashSet<>();

    queue.add(List.of(start));

    while (!queue.isEmpty()) {
      List<SuperstructureState> path = queue.poll();
      SuperstructureState current = path.get(path.size() - 1);

      if (current == goal) {
        return path;
      }

      if (visited.contains(current)) {
        continue;
      }

      visited.add(current);

      for (SuperstructureState neighbor : current.getTransitions()) {
        List<SuperstructureState> newPath = new ArrayList<>(path);
        newPath.add(neighbor);
        queue.add(newPath);
      }
    }

    return null; // no path found
  }

  /**
   * Iterates the current state of the superstructure, updating subsystem targets and handling state
   * transitions. If a direct transition to the target state is not available, attempts to find a
   * path using findPath.
   *
   * <p>Use this method after other logic in the periodic method to modify it
   */
  private void iterateState() {
    if (currentState.transitionOptions != null) {
      // default action that will encompass most states
      if (currentState.transitionOptions.elevatorTarget != null) {
        elevator.setPositionTarget(currentState.transitionOptions.elevatorTarget);
      }
      if (currentState.transitionOptions.pivotTarget != null) {
        pivot.setPositionTarget(currentState.transitionOptions.pivotTarget);
      }
      if (currentState.transitionOptions.tongueTarget != null) {
        tongue.setPositionTarget(currentState.transitionOptions.tongueTarget);
      }
      // check if we have reached our target and then transition if we go there
      if (this.superstructureReachedTarget() && this.targetState != this.currentState) {
        // figure out which state to go to next
        if (currentState
            .getTransitions()
            .contains(targetState)) { // if we have a transition, just take it
          setCurrentState(targetState);
        } else {
          // if we don't have a transition, use the findPath method to find a path
          List<SuperstructureState> path = findPath(currentState, targetState);
          if (path != null && !path.isEmpty()) {
            // set the next state to the first state in the path
            System.out.println(
                "Transitioning from " + currentState + " to " + path.get(1) + " via path: " + path);
            setCurrentState(path.get(1));
          } else {
            // if no path is found, just stay in the current state
            System.out.println("No path found from " + currentState + " to " + targetState);
          }
        }
      }
    }
  }

  @Override
  public void periodic() {
    currentState = bufferCurrentState;
    if (!stop) {
      switch (currentState) { // switch on the target state
          // Completely overridden states (don't call iterateState)
        case ZERO -> {
          // zeroing system for not killing the robot on zero
          // set our pivot pos
          if (pivot.getPosition() < PivotConstants.ZEROING_HIGH_THRESHOLD) {
            pivot.setPositionTarget(PivotTarget.ZERO_LOW);
            tongue.setPositionTarget(TongueTarget.STOW);
          } else {
            pivot.setPositionTarget(PivotTarget.ZERO_HIGH);
            tongue.setPositionTarget(TongueTarget.L4);
          }

          // wait for pivot to go to safe pos before zeroing
          if (pivot.reachedTarget()) {
            elevator.setZeroing(true);
          } else {
            elevator.setZeroing(false);
          }

          // check if we have have hit our hardstop, if so we can zero the elevator
          if (elevator.getFilteredSupplyCurrentAmps()
              > ElevatorConstants.ZEROING_VOLTAGE_THRESHOLD) {
            // check if the elevator is done zeroing and set offsets accordingly
            elevator.setOffset();
            elevator.setControlMode(ControlMode.POSITION);
            elevator.setZeroing(false);

            if (pivot.getPositionTarget() == PivotTarget.ZERO_HIGH) {
              setTargetState(SuperstructureState.STOW);
              setCurrentState(SuperstructureState.TOP);
            } else {
              setTargetState(SuperstructureState.STOW);
              setCurrentState(SuperstructureState.STOW);
            }
          }
        }
        case PREVENT_TIPPING -> {
          // this one we have to make the elevator manually go to the correct position to
          // avoid the
          // pivot touching the intake

          // switch our pivot based on our next state
          switch (targetState) {
            case SETUP_L4, SCORE_L4, PREVENT_TIPPING -> pivot.setPositionTarget(
                PivotTarget.SCORE_SIDE);
            default -> pivot.setPositionTarget(PivotTarget.INTAKE_SIDE);
          }

          if (pivot.getPosition() > -60.0
              && (pivot.getPositionTarget() == PivotTarget.INTAKE_SIDE
                  || (pivot.getPositionTarget() == PivotTarget.SCORE_SIDE
                      && pivot.getPosition() < 90.0))) { // not in constants
            elevator.setPositionTarget(ElevatorTarget.INTAKE_SIDE);
          } else {
            switch (targetState) { // set elevator pos based on target state
              case PREVENT_TIPPING -> elevator.setPositionTarget(
                  pivot.getPosition() > -60.0 ? ElevatorTarget.INTAKE_SIDE : ElevatorTarget.TOP);
              case SETUP_L4, SCORE_L4 -> elevator.setPositionTarget(ElevatorTarget.SETUP_L4);
              case SETUP_L3, SCORE_L3 -> elevator.setPositionTarget(ElevatorTarget.L3);
              default -> elevator.setPositionTarget(ElevatorTarget.TOP);
            }
          }

          if (pivot.getPosition() > 90.0) {
            tongue.setPositionTarget(TongueTarget.L4);
          } else {
            tongue.setPositionTarget(TongueTarget.STOW);
          }

          if (currentState != targetState
              && elevator.reachedTarget()
              && (Math.abs(
                      pivot.getPositionTarget().getPosition() / 360d - pivot.getPosition() / 360d)
                  < 0.05)) {
            switch (targetState) {
              case SETUP_L4, SCORE_L4 -> setCurrentState(SuperstructureState.SETUP_L4);
              case DESCORE_HIGH -> setCurrentState(SuperstructureState.DESCORE_HIGH);
              case DESCORE_LOW -> setCurrentState(SuperstructureState.DESCORE_LOW);
              default -> setCurrentState(SuperstructureState.TOP);
            }
          }
        }
        case SETUP_L4 -> {
          elevator.setPositionTarget(ElevatorTarget.SETUP_L4);
          pivot.setPositionTarget(PivotTarget.SETUP_L4);
          tongue.setPositionTarget(TongueTarget.L4);
          // check for state transitions
          if (currentState != targetState && this.superstructureReachedTarget()) {
            switch (targetState) {
              case SCORE_L4 -> {
                if (tonguePoleDetected()) {
                  setCurrentState(SuperstructureState.SCORE_L4);
                }
              }
              default -> setCurrentState(SuperstructureState.PREVENT_TIPPING);
            }
          }
        }
          // slightly altered states (still call iterateState)
        case SETUP_L3 -> {
          if (elevator.getPosition() > 32) {
            pivot.setPositionTarget(PivotTarget.SETUP_L3);
          }
          iterateState();
        }
        case TOP -> {
          if (elevator.getPosition() > 5) {
            pivot.setPositionTarget(PivotTarget.TOP);
          }
          iterateState();
        }
        case STOW -> {
          if (pivot.getPosition() < -0.27) { // idek just kept this here because I am paranoid
            elevator.setPositionTarget(ElevatorTarget.BOTTOM);
          }
          if (pivot.getPosition() < -100
              && elevator.getPosition() < 12) { // if were too low just wait on the elevator
            elevator.setPositionTarget(ElevatorTarget.SAFE_MIDWAY);
          }
          iterateState();
        }
        case CLIMB -> {
          if (elevator.getPosition() < 27 && elevator.getPosition() > 4) {
            pivot.setPositionTarget(PivotTarget.CLIMB);
          }
          iterateState();
        }
        case DESCORE_HIGH -> {
          if (pivot.getPosition() > -60) {
            elevator.setPositionTarget(ElevatorTarget.DESCORE_HIGH);
          }
          iterateState();
        }
        case DESCORE_LOW -> {
          if (pivot.getPosition() > -60) {
            elevator.setPositionTarget(ElevatorTarget.DESCORE_LOW);
          }
          iterateState();
        }
        default -> {
          iterateState(); // if we don't have any special logic, just iterate the state
        }
      }
    } else {
      elevator.setControlMode(ControlMode.STOP);
      pivot.setControlMode(ControlMode.STOP);
    }

    elevator.periodic();
    pivot.periodic();
    tongue.periodic();

    Logger.recordOutput("Superstructure/TargetState", targetState);
    Logger.recordOutput("Superstructure/CurrentState", currentState);
    Logger.recordOutput("Superstructure/Elevator reached target", elevator.reachedTarget());
    Logger.recordOutput("Superstructure/Pivot reached target", pivot.reachedTarget());
    Logger.recordOutput("Superstructure/Reached Target", superstructureReachedTarget());
    Logger.recordOutput(
        "Superstructure/Mechanism Poses/Elevator Pose", elevator.getDisplayPose3d());
    Logger.recordOutput("Superstructure/Mechanism Poses/Pivot Pose", pivot.getDisplayPose3d());
    Logger.recordOutput("Superstructure/Coral shooting position", getCoralEjectPosition());
  }

  // Target state getter and setter
  public void setTargetState(SuperstructureState superstructureState) {
    this.stop = false;
    this.targetState = superstructureState;
  }

  public SuperstructureState getTargetState() {
    return targetState;
  }

  // Current state getter and setter
  public void setCurrentState(SuperstructureState superstructureState) {
    this.stop = false;
    this.bufferCurrentState = superstructureState;
  }

  public SuperstructureState getCurrentState() {
    return currentState;
  }

  public void setStopped(boolean stopped) {
    this.stop = stopped;
  }

  public boolean getStopped() {
    return stop;
  }

  // go to target state command factory
  public Command goToStateCommand(SuperstructureState superstructureState) {
    return new FunctionalCommand(
        () -> {
          System.out.println("Setting superstructure state to: " + superstructureState);
          setTargetState(superstructureState);
        },
        () -> {},
        (e) -> {},
        () -> {
          return currentState == targetState && superstructureReachedTarget();
        },
        this);
  }

  /**
   * Get the position of the elevator
   *
   * @return the position of the elevator
   */
  public double getElevatorPosition() {
    return elevator.getPosition();
  }

  /**
   * Get the position of the pivot
   *
   * @return the position of the pivot
   */
  public double getPivotPosition() {
    return pivot.getPosition();
  }

  /**
   * Get the supply current of the elevator
   *
   * @return the supply current of the elevator
   */
  public double getElevatorSupplyCurrentAmps() {
    return elevator.getSupplyCurrentAmps();
  }

  /**
   * Get the supply current of the pivot
   *
   * @return the supply current of the pivot
   */
  public double getPivotSupplyCurrentAmps() {
    return pivot.getSupplyCurrentAmps();
  }

  /**
   * @return a boolean that says whether or not both of our mechanisms have finished zeroing
   */
  public boolean notZeroing() {
    return !elevator.isZeroing();
  }

  /**
   * @return if both subsystems in the superstructure have reached their target
   */
  public boolean superstructureReachedTarget() {
    boolean output =
        (elevator.reachedTarget()
                && pivot.reachedTarget()
                && currentState != SuperstructureState.ZERO)
            || overrideIsAtTarget;

    overrideIsAtTarget = false;
    return output;
  }

  public void oneTimeOverride() {
    overrideIsAtTarget = true;
  }

  public Command oneTimeOverrideCommand() {
    return new InstantCommand(() -> oneTimeOverride());
  }

  public boolean tonguePoleDetected() {
    return tongue.poleDetected();
  }

  public Pose3d getCoralEjectPosition() {
    return pivot
        .getDisplayPose3d()
        .plus(
            new Transform3d(
                Translation3d.kZero,
                new Rotation3d(
                    0, Math.toRadians(90), 0))) // Account for the 90 degree offset of the pivot
        .plus(PivotConstants.PIVOT_TO_OUTTAKE_TRANSFORM)
        .plus(
            new Transform3d(
                Translation3d.kZero,
                new Rotation3d(0, Math.toRadians(-105), 0))); // kindof arbitrary magic number
  }
}
