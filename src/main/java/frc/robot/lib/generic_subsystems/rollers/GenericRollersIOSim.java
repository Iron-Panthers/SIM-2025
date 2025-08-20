package frc.robot.lib.generic_subsystems.rollers;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public abstract class GenericRollersIOSim implements GenericRollersIO {
  protected final TalonFX talon;

  private final VoltageOut voltageOutput = new VoltageOut(0).withUpdateFreqHz(0);

  private final NeutralOut neutralOutput = new NeutralOut();

  private final double mechanismReduction;

  public GenericRollersIOSim(
      int id, int currentLimitAmps, boolean inverted, boolean brake, double reduction) {
    talon = new TalonFX(id);

    mechanismReduction = reduction;

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted =
        inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    talon.getConfigurator().apply(config);

    talon.optimizeBusUtilization();
  }

  @Override
  public abstract void updateInputs(GenericRollersIOInputs inputs);

  @Override
  public void runVolts(double volts) {
    talon.setControl(voltageOutput.withOutput(volts));
  }

  @Override
  public void stop() {
    talon.setControl(neutralOutput);
  }
}
