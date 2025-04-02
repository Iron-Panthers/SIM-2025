package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState;
import frc.robot.subsystems.swerve.Drive;
import java.util.function.DoubleSupplier;

// wrapper for autoalign FollowPathCommand
public class ApproachReef extends SequentialCommandGroup {
  private final boolean bSide;
  private final DoubleSupplier levelOffsetSupplier;
  private Command approachReef;

  private double iteration = 0;

  public enum LevelOffsets {
    // metres
    L4_OFFSET(0.14),
    L3_OFFSET(0.1),
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

  public class ReefAlign extends Command {
    private final DoubleSupplier levelOffsetSupplier;
    private final boolean bSide;
    private Command reefAlign;

    public ReefAlign(DoubleSupplier levelOffsetSupplier, boolean bSide) {
      this.levelOffsetSupplier = levelOffsetSupplier;
      this.bSide = bSide;
    }

    @Override
    public void initialize() {
      try {
        reefAlign =
            RobotState.getInstance().approachReefCommand(levelOffsetSupplier.getAsDouble(), bSide);
        reefAlign.initialize();
      } catch (Exception e) {
        e.printStackTrace();
        if (Math.abs(RobotState.getInstance().getVelocity().getNorm()) < 0.1) {
          end(true);
        }
        System.out.println("Already at target.");
      }
    }

    @Override
    public void execute() {
      if (reefAlign != null) {
        reefAlign.execute();
      }
      if (iteration > 20 && !(RobotState.getInstance().alignError() < 0.5)) {
        try {
          reefAlign =
              RobotState.getInstance()
                  .approachReefCommand(levelOffsetSupplier.getAsDouble(), bSide);
          reefAlign.initialize();
        } catch (Exception e) {
          e.printStackTrace();
          if (Math.abs(RobotState.getInstance().getVelocity().getNorm()) < 0.1) {
            end(true);
          }
          System.out.println("Already at target.");
        }
        iteration = 0;
      }
      iteration++;
    }

    @Override
    public boolean isFinished() {
      return reefAlign == null
          ? false
          : reefAlign.isFinished()
              && RobotState.getInstance().alignError() < 0.5
              && Math.abs(RobotState.getInstance().getVelocity().getNorm()) < 0.1;
    }

    @Override
    public void end(boolean interrupted) {
      if (reefAlign != null) {
        reefAlign.end(interrupted);
      }
      reefAlign = null;
    }
  }

  public ApproachReef(DoubleSupplier levelOffsetSupplier, boolean bSide, Drive drive) {
    this.levelOffsetSupplier = levelOffsetSupplier;
    this.bSide = bSide;

    addCommands(new VelocityClamp(drive), new ReefAlign(levelOffsetSupplier, bSide));
  }
}
