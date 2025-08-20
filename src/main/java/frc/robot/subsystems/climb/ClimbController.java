package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructure.ControlMode;
import frc.robot.subsystems.climb.Climb.ClimbTarget;

public class ClimbController extends SubsystemBase {

  private final Climb climb;
  /** Creates a new ClimbController. */
  public ClimbController(Climb climb) {
    this.climb = climb;
    // climb.setOffset();
    // climb.setPositionTarget(ClimbTarget.STOW);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    climb.periodic();
  }

  public Command setPositionTargetCommand(ClimbTarget target) {
    return new InstantCommand(
        () -> {
          climb.setPositionTarget(target);
        });
  }

  // Flick the climb to let coral fall out
  public Command clearCoral() {
    return new SequentialCommandGroup(
        // Wait until we get to the clear position
        new FunctionalCommand(
            () -> {
              climb.setPositionTarget(ClimbTarget.CLEAR);
            },
            () -> {},
            (e) -> {},
            climb::reachedTarget),

        // Then just go back up to stow
        new InstantCommand(
            () -> {
              climb.setPositionTarget(ClimbTarget.STOW);
            }));
  }

  public boolean climbHitCage() {
    return climb.hitCage();
  }

  public ClimbTarget getClimbTarget() {
    return climb.getPositionTarget();
  }

  public void setClimbTarget(ClimbTarget target) {
    climb.setControlMode(ControlMode.POSITION);
    climb.setPositionTarget(target);
  }

  public void setStopped(boolean stopped) {
    climb.setControlMode(ControlMode.STOP);
  }
}
