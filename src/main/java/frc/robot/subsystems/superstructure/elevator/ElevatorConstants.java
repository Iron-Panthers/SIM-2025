package frc.robot.subsystems.superstructure.elevator;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;
import frc.robot.subsystems.canWatchdog.CANWatchdogConstants.CAN;

public class ElevatorConstants {

  public static final ElevatorConfig ELEVATOR_CONFIG =
      switch (Constants.getRobotType()) {
        case COMP -> new ElevatorConfig(
            CAN.at(43, "Elevator 1"), CAN.at(44, "Elevator 2"), (58.0 / 14.0) / 6);
        case SIM -> new ElevatorConfig(
            CAN.at(43, "Elevator 1"), CAN.at(44, "Elevator 2"), (58.0 / 14.0));
        default -> new ElevatorConfig(0, 0, 1); // FIXME
      };

  public static final PIDGains GAINS =
      switch (Constants.getRobotType()) {
        case COMP -> new PIDGains(2, 0, 0, 0, 0.08, 0.002, 0.35);
        case SIM -> new PIDGains(2, 0, 0, 0, 0.08, 0.002, 0.35);
        default -> new PIDGains(0, 0, 0, 0, 0, 0, 0);
      };

  public static final MotionMagicConfig MOTION_MAGIC_CONFIG =
      switch (Constants.getRobotType()) {
        case COMP -> new MotionMagicConfig(500, 100, 0);
        case SIM -> new MotionMagicConfig(500, 100, 0);
        default -> new MotionMagicConfig(0, 0, 0);
      };

  public record ElevatorConfig(int motorID, int motorID2, double reduction) {}

  public record PIDGains(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  public record MotionMagicConfig(double acceleration, double cruiseVelocity, double jerk) {}

  public static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Elevator_Static;

  public static final InvertedValue MOTOR_DIRECTION = InvertedValue.CounterClockwise_Positive;

  public static final boolean OPOSE_MOTOR = true;

  public static final double POSITION_TARGET_EPSILON = 1;

  // SOFT LIMITS
  public static final double UPPER_EXTENSION_LIMIT = 32.5;

  // CURRENT LIMITS
  public static final double UPPER_VOLT_LIMIT = 10;

  public static final double LOWER_VOLT_LIMIT = -7;

  public static final double SUPPLY_CURRENT_LIMIT = 30;

  public static final int ZEROING_CURRENT_LIMIT = 20;

  // ZEROING CONSTANTS
  public static final double ZEROING_VOLTS = -1;

  public static final double ZEROING_OFFSET = 0; // offset in inches

  public static final double ZEROING_VOLTAGE_THRESHOLD = 4;

  // MIN HEIGHT TO MOVE PIVOT WITHOUT HITTING INTAKE
  public static final double MIN_SAFE_HEIGHT_FOR_PIVOT = 15;

  public static record ElevatorPhysicalConstants(
      double elevatorMassKg,
      double drumRadiusMeters,
      double minHeightMeters,
      double maxHeightMeters,
      boolean simulateGravity) {}

  public static final ElevatorPhysicalConstants PHYSICAL_CONSTANTS =
      switch (Constants.getRobotType()) {
        case SIM -> new ElevatorPhysicalConstants(4.0120245, 0.048874 / 2.0, 0, 1.5, true);
        case COMP, PROG, ALPHA -> new ElevatorPhysicalConstants(0, 0, 0, 0, false);
      };

  public static final Transform3d ELEVATOR_BASE_3D_OFFSET =
      switch (Constants.getRobotType()) {
        default -> new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
      };
}
