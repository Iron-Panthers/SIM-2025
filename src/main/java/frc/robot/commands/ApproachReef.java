package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;

public class ApproachReef extends Command {
  private final double offset;
  private final boolean bSide;
  private Command approachReef;

  public ApproachReef(double offset, boolean bSide) {
    this.offset = offset;
    this.bSide = bSide;
  }

  @Override
  public void initialize() {
    approachReef = RobotState.getInstance().approachReefCommand(offset, bSide);
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
