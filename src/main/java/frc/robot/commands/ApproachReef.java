package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;

public class ApproachReef extends Command {
  private final boolean bSide;
  private Command approachReef;
  private DoubleSupplier levelOffsetSupplier;

  public enum LevelOffsets {
    // these offsets are in metres
    // FIXME: PLEASE tweak these offset values because these are just estimates/guesses
    L4_OFFSET(0.1016),
    L3_OFFSET(0.0889),
    L2_OFFSET(0.0762),
    L1_OFFSET(0.0635);
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
    approachReef = RobotState.getInstance().approachReefCommand(levelOffsetSupplier.getAsDouble(), bSide);
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
