package frc.robot.lib.generic_subsystems.superstructure;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class GenericSuperstructureConfiguration {

  /**
   * Non-zero ID of mechanism motor
   *
   * <ul>
   *   <li><b>Default Value:</b> 0
   * </ul>
   */
  public int id = 0;

  /**
   * Non-zero ID of mechanism motor
   *
   * <ul>
   *   <li><b>Default Value:</b> 0
   * </ul>
   *
   * @return itself
   */
  public GenericSuperstructureConfiguration withID(int id) {
    this.id = id;
    return this;
  }

  /**
   * Direction of the motor when it is moving in the positive direction, as seen from the front
   *
   * <ul>
   *   <li><b>Default Value:</b> Clockwise_Positive
   * </ul>
   */
  public InvertedValue motorDirection = InvertedValue.Clockwise_Positive;

  /**
   * Direction of the motor when it is moving in the positive direction, as seen from the front
   *
   * <ul>
   *   <li><b>Default Value:</b> Clockwise_Positive
   * </ul>
   *
   * @return itself
   */
  public GenericSuperstructureConfiguration withMotorDirection(InvertedValue motorDirection) {
    this.motorDirection = motorDirection;
    return this;
  }

  /**
   * The absolute maximum amount of supply current allowed.
   *
   * <p>Supply current limits commonly range from 20-80 A depending on the breaker used.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> 0.0
   *   <li><b>Maximum Value:</b> 800.0
   *   <li><b>Default Value:</b> 70
   *   <li><b>Units:</b> A
   * </ul>
   */
  public double supplyCurrentLimit = 70;

  /**
   * The absolute maximum amount of supply current allowed.
   *
   * <p>Supply current limits commonly range from 20-80 A depending on the breaker used.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> 0.0
   *   <li><b>Maximum Value:</b> 800.0
   *   <li><b>Default Value:</b> 70
   *   <li><b>Units:</b> A
   * </ul>
   *
   * @return itself
   */
  public GenericSuperstructureConfiguration withSupplyCurrentLimit(double supplyCurrentLimit) {
    this.supplyCurrentLimit = supplyCurrentLimit;
    return this;
  }

  /**
   * The non-zero ID for a CANCoder. The Talon will update its position and velocity whenever
   * CANcoder publishes its information on CAN bus, and the Talon internal rotor will not be used.
   */
  public int canCoderID = 0;

  /**
   * The non-zero ID for a CANCoder. The Talon will update its position and velocity whenever
   * CANcoder publishes its information on CAN bus, and the Talon internal rotor will not be used.
   *
   * @return itself
   */
  public GenericSuperstructureConfiguration withCANCoderID(int canCoderID) {
    this.canCoderID = canCoderID;
    return this;
  }

  /**
   * This offset is added to the reported CANCoder position if CANCoder ID is not 0.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -1
   *   <li><b>Maximum Value:</b> 1
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b> rotations
   * </ul>
   */
  double canCoderOffset = 0;

  /**
   * This offset is added to the reported CANCoder position if CANCoder ID is not 0.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -1
   *   <li><b>Maximum Value:</b> 1
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b> rotations
   * </ul>
   *
   * @return itself
   */
  public GenericSuperstructureConfiguration withCANCoderOffset(double canCoderOffset) {
    this.canCoderOffset = canCoderOffset;
    return this;
  }

  /**
   * Direction of the CANCoder to determine positive rotation, as seen facing the LED side of the
   * CANcoder.
   *
   * <ul>
   *   <li><b>Default Value:</b> CounterClockwise_Positive
   * </ul>
   */
  public SensorDirectionValue canCoderDirection = SensorDirectionValue.CounterClockwise_Positive;

  /**
   * Direction of the CANCoder to determine positive rotation, as seen facing the LED side of the
   * CANcoder.
   *
   * <ul>
   *   <li><b>Default Value:</b> CounterClockwise_Positive
   * </ul>
   *
   * @return itself
   */
  public GenericSuperstructureConfiguration withCANCoderDirection(
      SensorDirectionValue canCoderDirection) {
    this.canCoderDirection = canCoderDirection;
    return this;
  }

  /**
   * The ratio of sensor rotations to the mechanism's output, where a ratio greater than 1 is a
   * reduction.
   *
   * <p>This is equivalent to the mechanism's gear ratio if the sensor is located on the input of a
   * gearbox. If sensor is on the output of a gearbox, then this is typically set to 1.
   *
   * <p>Don't use this config to perform unit conversion.
   *
   * <p>*
   *
   * <ul>
   *   <li><b>Default Value:</b> 70
   * </ul>
   */
  public double reduction = 1.0;

  /**
   * The ratio of sensor rotations to the mechanism's output, where a ratio greater than 1 is a
   * reduction.
   *
   * <p>This is equivalent to the mechanism's gear ratio if the sensor is located on the input of a
   * gearbox. If sensor is on the output of a gearbox, then this is typically set to 1.
   *
   * <p>Don't use this config to perform unit conversion.
   *
   * <p>*
   *
   * <ul>
   *   <li><b>Default Value:</b> 70
   * </ul>
   *
   * @return itself
   */
  public GenericSuperstructureConfiguration withReduction(double reduction) {
    this.reduction = reduction;
    return this;
  }

  /**
   * Positive position soft limit (please implement if applicable)
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -3.4e+38
   *   <li><b>Maximum Value:</b> 3.4e+38
   *   <li><b>Default Value:</b> 3.4e+38
   *   <li><b>Units:</b> rotations
   * </ul>
   */
  public double upperExtensionLimit = 0;

  public boolean upperExtensionLimitEnabled = false;

  /**
   * Positive position soft limit (please implement if applicable)
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -3.4e+38
   *   <li><b>Maximum Value:</b> 3.4e+38
   *   <li><b>Default Value:</b> 3.4e+38
   *   <li><b>Units:</b> rotations
   * </ul>
   *
   * @return itself
   */
  public GenericSuperstructureConfiguration withUpperExtensionLimit(double upperExtensionLimit) {
    this.upperExtensionLimit = upperExtensionLimit;
    this.upperExtensionLimitEnabled = true;
    return this;
  }

  /**
   * Negative extension soft limit (please implement if applicable)
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -3.4e+38
   *   <li><b>Maximum Value:</b> 3.4e+38
   *   <li><b>Default Value:</b> -3.4e+38
   *   <li><b>Units:</b> rotations
   * </ul>
   */
  public double lowerExtensionLimit = 0;

  public boolean lowerExtensionLimitEnabled = false;

  /**
   * Negative extension soft limit (please implement if applicable)
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -3.4e+38
   *   <li><b>Maximum Value:</b> 3.4e+38
   *   <li><b>Default Value:</b> -3.4e+38
   *   <li><b>Units:</b> rotations
   * </ul>
   *
   * @return itself
   */
  public GenericSuperstructureConfiguration withLowerExtensionLimit(double lowerExtensionLimit) {
    this.lowerExtensionLimit = lowerExtensionLimit;
    this.lowerExtensionLimitEnabled = true;
    return this;
  }

  /**
   * The upper voltage limit for the motor.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -16
   *   <li><b>Maximum Value:</b> 16
   *   <li><b>Default Value:</b> 16
   *   <li><b>Units:</b> V
   * </ul>
   */
  public double upperVoltLimit = 16;

  /**
   * The upper voltage limit for the motor.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -16
   *   <li><b>Maximum Value:</b> 16
   *   <li><b>Default Value:</b> 16
   *   <li><b>Units:</b> V
   * </ul>
   *
   * @return itself
   */
  public GenericSuperstructureConfiguration withUpperVoltageLimit(double upperVoltLimit) {
    this.upperVoltLimit = upperVoltLimit;
    return this;
  }

  /**
   * The lower voltage limit for the motor.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -16
   *   <li><b>Maximum Value:</b> 16
   *   <li><b>Default Value:</b> -16
   *   <li><b>Units:</b> V
   * </ul>
   */
  public double lowerVoltLimit = 16;

  /**
   * The lower voltage limit for the motor.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -16
   *   <li><b>Maximum Value:</b> 16
   *   <li><b>Default Value:</b> -16
   *   <li><b>Units:</b> V
   * </ul>
   *
   * @return itself
   */
  public GenericSuperstructureConfiguration withLowerVoltageLimit(double lowerVoltLimit) {
    this.lowerVoltLimit = lowerVoltLimit;
    return this;
  }

  /**
   * Voltage applied to the motor during zeroing.
   *
   * <ul>
   *   <li><b>Default Value:</b> 0
   * </ul>
   */
  public double zeroingVolts = 0;

  /**
   * Voltage applied to the motor during zeroing.
   *
   * <ul>
   *   <li><b>Default Value:</b> 0
   * </ul>
   *
   * @return itself
   */
  public GenericSuperstructureConfiguration withZeroingVolts(double zeroingVolts) {
    this.zeroingVolts = zeroingVolts;
    return this;
  }

  /**
   * Offset applied to the extension after zeroing.
   *
   * <ul>
   *   <li><b>Default Value:</b> 0
   * </ul>
   */
  public double zeroingOffset = 0;

  /**
   * Offset applied to the extension after zeroing.
   *
   * <ul>
   *   <li><b>Default Value:</b> 0
   * </ul>
   *
   * @return itself
   */
  public GenericSuperstructureConfiguration withZeroingOffset(double zeroingOffset) {
    this.zeroingOffset = zeroingOffset;
    return this;
  }

  /**
   * Voltage threshold to determine if the mechanism has reached the zeroing position.
   *
   * <ul>
   *   <li><b>Default Value:</b> 0
   * </ul>
   */
  public double zeroingVoltageThreshold = 0;

  /**
   * Voltage threshold to determine if the mechanism has reached the zeroing position.
   *
   * <ul>
   *   <li><b>Default Value:</b> 0
   * </ul>
   *
   * @return itself
   */
  public GenericSuperstructureConfiguration withZeroingVoltageThreshold(
      double zeroingVoltageThreshold) {
    this.zeroingVoltageThreshold = zeroingVoltageThreshold;
    return this;
  }
}
