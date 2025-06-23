package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import java.util.Optional;

public class GenericSuperstructureIOTalonFX implements GenericSuperstructureIO {
  protected final TalonFX talon;
  protected Optional<TalonFX> talon2;

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final StatusSignal<Angle> positionRotations;
  private final StatusSignal<AngularVelocity> velocityRPS;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> temp;

  private final StatusSignal<Angle> positionRotations2;
  private final StatusSignal<AngularVelocity> velocityRPS2;
  private final StatusSignal<Voltage> appliedVolts2;
  private final StatusSignal<Current> supplyCurrent2;
  private final StatusSignal<Temperature> temp2;

  // zeroing stuff
  private final double zeroingVolts;
  private final double zeroingOffset;
  private final double zeroingVoltageThreshold;

  protected final VoltageOut voltageOutput = new VoltageOut(0).withUpdateFreqHz(0);
  private final NeutralOut neutralOutput = new NeutralOut();
  private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0).withUpdateFreqHz(0);

  /**
   * Constructs a new GenericSuperstructureIOTalonFX.
   *
   * @param id The ID of the TalonFX motor
   * @param inverted Whether the motor is inverted.
   * @param supplyCurrentLimit The supply current limit for the motor.
   * @param reduction The reduction ratio between the sensor and mechanism.
   * @param upperVoltLimit The upper voltage limit for the motor.
   * @param lowerVoltLimit The lower voltage limit for the motor.
   * @param zeroingVolts The voltage to apply during zeroing.
   * @param zeroingOffset The offset to set after zeroing.
   * @param zeroingVoltageThreshold The voltage threshold to determine if the mechanism has reached
   *     the zeroing position.
   */
  public GenericSuperstructureIOTalonFX(
      int id,
      boolean inverted,
      double supplyCurrentLimit,
      double reduction,
      double upperVoltLimit,
      double lowerVoltLimit,
      double zeroingVolts,
      double zeroingOffset,
      double zeroingVoltageThreshold) {

    // set the zeroing values such tha when the robot zeros it will apply the
    // zeroing volts and
    // when it reaches a resistance from part of the mechanism, it sets the position
    // to the zeroing
    // Offset
    this.zeroingVolts = zeroingVolts;
    this.zeroingOffset = zeroingOffset;
    this.zeroingVoltageThreshold = zeroingVoltageThreshold;

    // VOLTAGE, LIMITS AND RATIO CONFIG
    config.MotorOutput.Inverted =
        inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.Voltage.withPeakForwardVoltage(upperVoltLimit);
    config.Voltage.withPeakReverseVoltage(lowerVoltLimit);
    config.Feedback.withSensorToMechanismRatio(reduction);

    talon = new TalonFX(id);

    talon.getConfigurator().apply(config);
    setOffset();
    talon.setNeutralMode(NeutralModeValue.Brake);

    // STATUS SIGNALS
    velocityRPS = talon.getVelocity();
    appliedVolts = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    temp = talon.getDeviceTemp();
    positionRotations = talon.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, positionRotations, velocityRPS, appliedVolts, supplyCurrent, temp);

    if (talon2.isPresent()) {
      velocityRPS2 = talon2.get().getVelocity();
      appliedVolts2 = talon2.get().getMotorVoltage();
      supplyCurrent2 = talon2.get().getSupplyCurrent();
      temp2 = talon2.get().getDeviceTemp();
      positionRotations2 = talon2.get().getPosition();

      BaseStatusSignal.setUpdateFrequencyForAll(
          50, positionRotations2, velocityRPS2, appliedVolts2, supplyCurrent2, temp2);
    } else {
      velocityRPS2 = null;
      appliedVolts2 = null;
      supplyCurrent2 = null;
      temp2 = null;
      positionRotations2 = null;
    }
  }

  @Override
  public void updateInputs(GenericSuperstructureIOInputs inputs) {
    inputs.connected1 =
        BaseStatusSignal.refreshAll(
                positionRotations, velocityRPS, appliedVolts, supplyCurrent, temp)
            .isOK();
    inputs.positionRotations = positionRotations.getValueAsDouble();
    inputs.velocityRotPerSec = velocityRPS.getValueAsDouble();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.tempCelsius = temp.getValueAsDouble();
  }

  @Override
  public void updateSecondaryInputs(GenericSuperstructureIOInputsMotor2 inputs) {
    if (talon2.isPresent()) {
      inputs.connected2 =
          BaseStatusSignal.refreshAll(
                  positionRotations2, velocityRPS2, appliedVolts2, supplyCurrent2, temp2)
              .isOK();
      inputs.positionRotations2 = positionRotations2.getValueAsDouble();
      inputs.velocityRotPerSec2 = velocityRPS2.getValueAsDouble();
      inputs.appliedVolts2 = appliedVolts2.getValueAsDouble();
      inputs.supplyCurrentAmps2 = supplyCurrent2.getValueAsDouble();
      inputs.tempCelsius2 = temp2.getValueAsDouble();
    }
  }

  @Override
  public void runPosition(double rotations) {
    talon.setControl(positionControl.withPosition(rotations));
  }

  @Override
  public void runCharacterization() {
    talon.setControl(voltageOutput.withOutput(zeroingVolts));
  }

  @Override
  public void stop() {
    talon.setControl(neutralOutput);
  }

  @Override
  public void setOffset() {
    talon.getConfigurator().setPosition(zeroingOffset);
    if (talon2.isPresent()) talon2.get().getConfigurator().setPosition(zeroingOffset);
  }

  @Override
  public double getZeroingVoltageThreshold() {
    return zeroingVoltageThreshold;
  }

  @Override
  public void setSlot0(
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kA,
      double kG,
      double motionMagicAcceleration,
      double motionMagicCruiseVelocity,
      double motionMagicJerk,
      GravityTypeValue gravityTypeValue) {
    Slot0Configs gainsConfig = new Slot0Configs();
    gainsConfig.kP = kP;
    gainsConfig.kI = kI;
    gainsConfig.kD = kD;
    gainsConfig.kS = kS;
    gainsConfig.kV = kV;
    gainsConfig.kA = kA;
    gainsConfig.kG = kG;
    gainsConfig.GravityType = gravityTypeValue;

    MotionMagicConfigs motionMagicConfig = new MotionMagicConfigs();
    motionMagicConfig.MotionMagicAcceleration = motionMagicAcceleration;
    motionMagicConfig.MotionMagicCruiseVelocity = motionMagicCruiseVelocity;
    motionMagicConfig.MotionMagicJerk = motionMagicJerk;

    talon.getConfigurator().apply(gainsConfig);
    talon.getConfigurator().apply(motionMagicConfig);
  }

  @Override
  public void setCanCoderConfigs(
      int canCoderID, double canCoderOffset, SensorDirectionValue direction) {
    CANcoder canCoder = new CANcoder(canCoderID);
    canCoder
        .getConfigurator()
        .apply(
            new CANcoderConfiguration()
                .withMagnetSensor(
                    new MagnetSensorConfigs()
                        .withAbsoluteSensorDiscontinuityPoint(0.5)
                        .withSensorDirection(direction)
                        .withMagnetOffset(canCoderOffset)));

    config.Feedback.withRemoteCANcoder(canCoder);
    talon.getConfigurator().apply(config);
  }

  @Override
  public void setSecondMotorConfigs(int id2, boolean opposeFirst) {
    talon2.get().getConfigurator().apply(config);
    talon2.get().setNeutralMode(NeutralModeValue.Brake);
    talon2.get().setControl(new Follower(talon.getDeviceID(), opposeFirst));
    talon.getConfigurator().apply(config);
  }

  @Override
  public void setUpperExtensionLimit(double upperLimit) {
    config.SoftwareLimitSwitch.withReverseSoftLimitEnable(true);
    config.SoftwareLimitSwitch.withReverseSoftLimitThreshold(upperLimit);
    talon.getConfigurator().apply(config);
  }

  @Override
  public void setLowerExtensionLimit(double lowerLimit) {
    config.SoftwareLimitSwitch.withReverseSoftLimitEnable(true);
    config.SoftwareLimitSwitch.withReverseSoftLimitThreshold(lowerLimit);
    talon.getConfigurator().apply(config);
  }
}
