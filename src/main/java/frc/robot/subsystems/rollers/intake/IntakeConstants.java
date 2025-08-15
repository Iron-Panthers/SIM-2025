package frc.robot.subsystems.rollers.intake;

import frc.robot.Constants;

public class IntakeConstants {
  public static final int ID =
      switch (Constants.getRobotType()) {
        case COMP -> 26;
        case ALPHA -> 14;
        case SIM -> 26;
        default -> 0;
      };
  public static final int CURRENT_LIMIT_AMPS =
      switch (Constants.getRobotType()) {
        case COMP -> 40;
        case ALPHA -> 40;
        case SIM -> 40;
        default -> 40;
      };
  public static final boolean INVERTED =
      switch (Constants.getRobotType()) {
        case COMP -> false;
        case ALPHA -> true;
        case SIM -> false;
        default -> false;
      };
  public static final boolean BRAKE =
      switch (Constants.getRobotType()) {
        default -> false;
      };
  public static final double REDUCTION =
      switch (Constants.getRobotType()) {
        case COMP -> 1;
        case ALPHA -> 2;
        case SIM -> 1;
        default -> 1;
      };

  public static final double MOI = 0.000105;
}
