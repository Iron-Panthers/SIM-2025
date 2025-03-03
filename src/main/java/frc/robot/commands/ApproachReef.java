package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import java.util.function.DoubleSupplier;

public class ApproachReef extends Command {
  private final boolean bSide;
  private Command approachReef;
  private DoubleSupplier levelOffsetSupplier;

  public enum LevelOffsets {
    // these offsets are in metres
    // FIXME: PLEASE tweak these offset values because these are just estimates/guesses
    L4_OFFSET(0.2),
    L3_OFFSET(0.2),
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
    approachReef =
        RobotState.getInstance().approachReefCommand(levelOffsetSupplier.getAsDouble(), bSide);
    approachReef.initialize();
  }

  @Override
  public void execute() {
    approachReef.execute();
  }

  @Override
  public boolean isFinished() {
    return approachReef.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    approachReef.end(interrupted);
  }
}
