package frc.robot.subsystems.superstructure.elevator;

import static frc.robot.subsystems.superstructure.elevator.ElevatorConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.lib.generic_subsystems.superstructure.*;

public class ElevatorIOTalonFX extends GenericSuperstructureIOTalonFX implements ElevatorIO {
  private final StatusSignal<Angle> positionRotations2;
  private final StatusSignal<AngularVelocity> velocityRPS2;
  private final StatusSignal<Voltage> appliedVolts2;
  private final StatusSignal<Current> supplyCurrent2;
  private final StatusSignal<Temperature> temp2;

  protected TalonFX talon2;

  public ElevatorIOTalonFX() {

    super(
        new GenericSuperstructureConfiguration()
            .withID(ELEVATOR_CONFIG.motorID())
            .withMotorDirection(MOTOR_DIRECTION)
            .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
            .withReduction(ELEVATOR_CONFIG.reduction())
            .withUpperVoltageLimit(UPPER_VOLT_LIMIT)
            .withLowerVoltageLimit(LOWER_VOLT_LIMIT)
            .withZeroingVolts(ZEROING_VOLTS)
            .withZeroingOffset(ZEROING_OFFSET)
            .withZeroingVoltageThreshold(ZEROING_VOLTAGE_THRESHOLD)
            .withUpperExtensionLimit(UPPER_EXTENSION_LIMIT));

    talon2 = new TalonFX(ELEVATOR_CONFIG.motorID2());

    talon2.getConfigurator().apply(config);
    talon2.setNeutralMode(NeutralModeValue.Brake);
    talon2.setControl(new Follower(talon.getDeviceID(), OPOSE_MOTOR));

    velocityRPS2 = talon2.getVelocity();
    appliedVolts2 = talon2.getMotorVoltage();
    supplyCurrent2 = talon2.getSupplyCurrent();
    temp2 = talon2.getDeviceTemp();
    positionRotations2 = talon2.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, positionRotations2, velocityRPS2, appliedVolts2, supplyCurrent2, temp2);

    setSlot0(
        GAINS.kP(),
        GAINS.kI(),
        GAINS.kD(),
        GAINS.kS(),
        GAINS.kV(),
        GAINS.kA(),
        GAINS.kG(),
        MOTION_MAGIC_CONFIG.acceleration(),
        MOTION_MAGIC_CONFIG.cruiseVelocity(),
        MOTION_MAGIC_CONFIG.jerk(),
        GRAVITY_TYPE);
  }

  @Override
  public void updateSecondaryInputs(GenericSuperstructureIOInputsMotor2 inputs) {
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
