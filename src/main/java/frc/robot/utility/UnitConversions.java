package frc.robot.utility;

public class UnitConversions {
  public static final double FEET_TO_METERS = 0.3048;
  public static final double METERS_TO_FEET = 1.0 / FEET_TO_METERS;
  public static final double INCHES_TO_METERS = 0.0254;
  public static final double METERS_TO_INCHES = 1.0 / INCHES_TO_METERS;

  public static double feetToMeters(double feet) {
    return feet * FEET_TO_METERS;
  }

  public static double metersToFeet(double meters) {
    return meters * METERS_TO_FEET;
  }

  public static double inchesToMeters(double inches) {
    return inches * INCHES_TO_METERS;
  }

  public static double metersToInches(double meters) {
    return meters * METERS_TO_INCHES;
  }

  public static double degreesToRadians(double degrees) {
    return Math.toRadians(degrees);
  }

  public static double radiansToDegrees(double radians) {
    return Math.toDegrees(radians);
  }
}
