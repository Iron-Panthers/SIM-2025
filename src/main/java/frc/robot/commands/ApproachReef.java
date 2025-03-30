package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import java.util.function.DoubleSupplier;

// wrapper for autoalign FollowPathCommand
public class ApproachReef extends Command {
  private final boolean bSide;
  private Command approachReef;
  private DoubleSupplier levelOffsetSupplier;

  public enum LevelOffsets {
    // metres
    L4_OFFSET(0.14),
    L3_OFFSET(0.05),
    L2_OFFSET(0),
    L1_OFFSET(0);
    public double levelOffset;

    private LevelOffsets(double levelOffset) {
      this.levelOffset = levelOffset;
    }

    public double getLevelOffset() {
      return levelOffset;
    }
  }

  public ApproachReef(DoubleSupplier levelOffsetSupplier, boolean bSide) {
    this.levelOffsetSupplier = levelOffsetSupplier;
    this.bSide = bSide;
  }

  @Override
  public void initialize() {

    try {
      approachReef =
          RobotState.getInstance().approachReefCommand(levelOffsetSupplier.getAsDouble(), bSide);
      approachReef.initialize();
    } catch (Exception e) {
      e.printStackTrace();
      this.end(true);
      System.out.println("Already at target");
    }

    System.out.println(
        "Initializing ApproachReefCommand, offset of "
            + levelOffsetSupplier.getAsDouble()
            + ", bSide is "
            + bSide);
  }

  @Override
  public void execute() {
    if (approachReef != null) {
      approachReef.execute();
    }
  }

  @Override
  public boolean isFinished() {
    return approachReef == null ? true : approachReef.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    if (approachReef != null) {
      approachReef.end(interrupted);
    }
    approachReef = null;
  }
}
