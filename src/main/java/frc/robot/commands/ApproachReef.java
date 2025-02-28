package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;

public class ApproachReef extends Command {
  private Command approachReef;

  @Override
  public void initialize() {
    approachReef = RobotState.getInstance().approachReefCommand();
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
