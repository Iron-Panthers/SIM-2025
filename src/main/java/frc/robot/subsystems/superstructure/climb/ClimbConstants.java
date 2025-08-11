package frc.robot.subsystems.superstructure.climb;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.robot.Constants;
import java.util.Optional;

public class ClimbConstants {
  public static final ClimbConfig CLIMB_CONFIG =
      switch (Constants.getRobotType()) {
        case COMP -> new ClimbConfig(37, 2.5, Optional.of(45), Optional.of(0.201));
        case PROG -> new ClimbConfig(0, 1, Optional.empty(), Optional.empty());
        case ALPHA -> new ClimbConfig(0, 0, Optional.empty(), Optional.empty());
        case SIM -> new ClimbConfig(37, 2.5, Optional.of(45), Optional.of(0.201));
      };

  public static final PIDGains GAINS =
      switch (Constants.getRobotType()) {
        case COMP -> new PIDGains(600, 0, 0, 0, 66.5, 5.714, 0);
        case PROG -> new PIDGains(0, 0, 0, 0, 0, 0, 0);
        case ALPHA -> new PIDGains(0, 0, 0, 0, 0, 0, 0);
        case SIM -> new PIDGains(600, 0, 0, 0, 66.5, 5.714, 0);
      };

  public static final MotionMagicConfig MOTION_MAGIC_CONFIG =
      switch (Constants.getRobotType()) {
        case COMP -> new MotionMagicConfig(2, 1, 0);
        case PROG -> new MotionMagicConfig(0, 0, 0);
        case ALPHA -> new MotionMagicConfig(0, 0, 0);
        case SIM -> new MotionMagicConfig(2, 1, 0);
      };

  public record ClimbConfig(
      int motorID,
      double reduction,
      Optional<Integer> canCoderID,
      Optional<Double> canCoderOffset) {}

  public record PIDGains(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  public static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Arm_Cosine;

  public static final boolean INVERT_MOTOR = false;
  public static final Optional<SensorDirectionValue> CANCODER_DIRECTION =
      Optional.of(SensorDirectionValue.CounterClockwise_Positive);

  public static final double POSITION_TARGET_EPSILON = 0.03;

  public static final Optional<Double> SENSOR_DISCONTINUITY_POINT = Optional.of(0.7);

  public record MotionMagicConfig(double acceleration, double cruiseVelocity, double jerk) {}

  // SOFT LIMITS
  public static final Optional<Double> UPPER_EXTENSION_LIMIT =
      Optional.of(121d); // top limit is 121 rotations
  public static final Optional<Double> LOWER_EXTENSION_LIMIT = Optional.empty();

  // CURRENT LIMITS
  public static final double UPPER_VOLT_LIMIT = 12;
  public static final double UPPER_VOLT_LIMIT_CLIMBING = 3;
  public static final double LOWER_VOLT_LIMIT = -12;
  public static final double SUPPLY_CURRENT_LIMIT = 30;

  // ZEROING CONSTANTS
  public static final double ZEROING_VOLTS = -1;
  public static final double ZEROING_OFFSET = 0; // offset in inches
  public static final double ZEROING_VOLTAGE_THRESHOLD = 4;

  // INDUCTION SENSOR
  public static final int INDUCTION_PORT_NUMBER = 6;

  // PHYSICAL CONSTANTS
  public static record ClimbPhysicalConstants(
      double momentOfInertia,
      double lengthMeters,
      double minAngleRads,
      double maxAngleRads,
      boolean simulateGravity) {}

  public static final ClimbPhysicalConstants PHYSICAL_CONSTANTS =
      switch (Constants.getRobotType()) {
        case SIM -> new ClimbPhysicalConstants(0.1, 0.5, -1000.0, 1000, false);
        case COMP, ALPHA, PROG -> new ClimbPhysicalConstants(0.1, 0, 0, 0, false);
      };
}
