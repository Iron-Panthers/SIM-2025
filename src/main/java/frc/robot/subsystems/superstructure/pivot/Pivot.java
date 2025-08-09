package frc.robot.subsystems.superstructure.pivot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.superstructure.GenericSuperstructure;
import frc.robot.utility.LoggableMechanism3d;
import org.littletonrobotics.junction.Logger;

public class Pivot extends GenericSuperstructure<Pivot.PivotTarget> implements LoggableMechanism3d {
  public enum PivotTarget implements GenericSuperstructure.PositionTarget {
    TOP(-79),
    INTAKE(-96),
    STOW(-96),
    L1(-110),
    L2(-107),
    SCORE_L3(-117),
    SETUP_L3(-79),
    CLIMB(-115),
    ZERO_LOW(-95.2),
    ZERO_HIGH(90),
    SETUP_L4(144),
    SCORE_L4(149),
    DESCORE_HIGH(0),
    INTAKE_SIDE(40),
    SCORE_SIDE(100),

    // for the algae on L2
    DESCORE_LOW(-15);

    private double position;
    private static final double EPSILON = PivotConstants.POSITION_TARGET_EPSILON;

    private PivotTarget(double position) {
      this.position = position;
    }

    public double getPosition() {
      return position;
    }

    @Override
    public double getEpsilon() {
      return EPSILON;
    }
  }

  public Pivot(PivotIO io) {
    super("Pivot", io);
    setPositionTarget(PivotTarget.STOW);
    setControlMode(ControlMode.STOP);
  }

  @Override
  public void periodic() {
    super.periodic();
    Logger.recordOutput(
        "Superstructure/Pivot/PositionTargetRotations", getPositionTarget().getPosition() / 360d);
  }

  /**
   * This function returns whether or not the subsystem has reached its position target
   *
   * @return whether the subsystem has reached its position target
   */
  public boolean reachedTarget() {
    return Math.abs(super.getPosition() - (super.getPositionTarget().getPosition() / 360d))
        <= super.getPositionTarget().getEpsilon();
  }

  public double getPosition() {
    return super.getPosition() * 360.0;
  }

  @Override
  public Pose3d getDisplayPose3d(Pose3d parentPose3d) {
    return parentPose3d
        .plus(PivotConstants.ELEVATOR_TO_PIVOT_TRANSFORM)
        .plus(
            new Transform3d(
                Translation3d.kZero, new Rotation3d(0, -Math.toRadians(getPosition() + 90), 0)));
  }
}
