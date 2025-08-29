package frc.robot.subsystems.superstructure.pivot;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.canWatchdog.CANWatchdogConstants.CAN;

public class PivotConstants {
  public static final PivotConfig PIVOT_CONFIG =
      switch (Constants.getRobotType()) {
        case COMP -> new PivotConfig(CAN.at(8, "Pivot"), CAN.at(28, "Pivot Encoder"), -0.278, 1);
        case SIM -> new PivotConfig(
            CAN.at(8, "Pivot"), CAN.at(28, "Pivot Encoder"), 0, 12 * 0.3750);
        default -> new PivotConfig(0, 0, 0, 1);
      };

  public static final PIDGains GAINS =
      switch (Constants.getRobotType()) {
        case COMP -> new PIDGains(40, 0, 0, 0, 3.6144, 0.1807, 0.53);
        case ALPHA -> new PIDGains(1.5, 0, 0.01, 0.03, 0.09, 0, 0.51);
        case SIM -> new PIDGains(40, 0, 0, 0, 3.6144, 0.1807, 0.53);
        default -> new PIDGains(0, 0, 0, 0, 0, 0, 0);
      };

  public static final MotionMagicConfig MOTION_MAGIC_CONFIG =
      switch (Constants.getRobotType()) {
        case COMP -> new MotionMagicConfig(7.5, 10); // 3, 10
        case SIM -> new MotionMagicConfig(7.5, 10); // 3, 10
        default -> new MotionMagicConfig(0, 0);
      };

  public record PivotConfig(int motorID, int canCoderID, double canCoderOffset, double reduction) {}

  public record PIDGains(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  public record MotionMagicConfig(double acceleration, double cruiseVelocity) {}

  public static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Arm_Cosine;

  public static final InvertedValue MOTOR_DIRECTION = InvertedValue.CounterClockwise_Positive;

  public static final SensorDirectionValue CANCODER_DIRECTION =
      SensorDirectionValue.Clockwise_Positive;

  public static final double POSITION_TARGET_EPSILON = 0.01;
  public static final double PIVOT_LENGTH = 25; // inches

  // SOFT LIMITS
  public static final double UPPER_EXTENSION_LIMIT = 0.465;

  // CURRENT LIMITS
  public static final double UPPER_VOLT_LIMIT = 4;
  public static final double LOWER_VOLT_LIMIT = -6;
  public static final double SUPPLY_CURRENT_LIMIT = 30;

  // ZEROING CONSTANTS
  public static final double ZEROING_VOLTS = 1;
  public static final double ZEROING_OFFSET = 0; // offset in degrees
  public static final double ZEROING_VOLTAGE_THRESHOLD = 5;
  public static final double ZEROING_HIGH_THRESHOLD =
      -70.0; // the position where if the pivot is over, the pivot
  // will go up before zeroing

  // PIVOT POSITION CONSTANTS
  public static final Transform3d ELEVATOR_TO_PIVOT_TRANSFORM =
      switch (Constants.getRobotType()) {
        default -> new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-3.5), Units.inchesToMeters(0d), Units.inchesToMeters(33.875)),
            new Rotation3d(0, 0, 0));
      };

  // PHYSICAL CONSTANTS
  public static record PivotPhysicalConstants(
      double momentOfInertia,
      double lengthMeters,
      double minAngleRads,
      double maxAngleRads,
      boolean simulateGravity) {}

  public static final PivotPhysicalConstants PHYSICAL_CONSTANTS =
      switch (Constants.getRobotType()) {
        case SIM -> new PivotPhysicalConstants(0.02, 0.706747, -1000.0, 1000, true);
        case COMP, ALPHA, PROG -> new PivotPhysicalConstants(0.1, 0, 0, 0, false);
      };

  public static final Transform3d PIVOT_TO_OUTTAKE_TRANSFORM =
      switch (Constants.getRobotType()) {
        default -> new Transform3d(
            new Translation3d(
                Units.inchesToMeters(22.445),
                Units.inchesToMeters(0.0),
                Units.inchesToMeters(1.742)),
            new Rotation3d(0, 0, 0));
      };
}
