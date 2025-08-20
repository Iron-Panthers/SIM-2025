package frc.robot.subsystems.superstructure.pivot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.lib.generic_subsystems.superstructure.*;

public class PivotIOSim extends GenericSuperstructureIOSim implements PivotIO {

  private final SingleJointedArmSim pivotSim;
  private final double reduction;

  public PivotIOSim() {
    super(PivotConstants.PIVOT_CONFIG.motorID());

    this.reduction = PivotConstants.PIVOT_CONFIG.reduction();

    pivotSim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(1),
            reduction,
            PivotConstants.PHYSICAL_CONSTANTS.momentOfInertia(),
            PivotConstants.PHYSICAL_CONSTANTS.lengthMeters(),
            PivotConstants.PHYSICAL_CONSTANTS.minAngleRads(),
            PivotConstants.PHYSICAL_CONSTANTS.maxAngleRads(),
            PivotConstants.PHYSICAL_CONSTANTS.simulateGravity(),
            0);
    setOffset();
    setSlot0(
        PivotConstants.GAINS.kP(),
        PivotConstants.GAINS.kI(),
        PivotConstants.GAINS.kD(),
        PivotConstants.GAINS.kS(),
        PivotConstants.GAINS.kV(),
        PivotConstants.GAINS.kA(),
        PivotConstants.GAINS.kG(),
        PivotConstants.MOTION_MAGIC_CONFIG.acceleration(),
        PivotConstants.MOTION_MAGIC_CONFIG.cruiseVelocity(),
        0,
        PivotConstants.GRAVITY_TYPE);
  }

  @Override
  public void updateInputs(GenericSuperstructureIOInputs inputs) {
    // Update TalonFX state
    talon.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

    double appliedVoltage = talon.getSimState().getMotorVoltage();

    // Simulate physics
    pivotSim.setInputVoltage(appliedVoltage);
    pivotSim.update(0.02);

    // Convert position and velocity from meters to rotations for the TalonFX sensor
    double rotations = pivotSim.getAngleRads() / (2 * Math.PI * reduction);
    double velocityRPS = pivotSim.getVelocityRadPerSec() / (2 * Math.PI * reduction);

    talon.getSimState().setRawRotorPosition(rotations);
    talon.getSimState().setRotorVelocity(velocityRPS);

    inputs.connected = true;
    inputs.positionRotations = rotations;
    inputs.velocityRotPerSec = velocityRPS;
    inputs.appliedVolts = appliedVoltage;
    inputs.supplyCurrentAmps = 1.0; // Not simulated
    inputs.tempCelsius = 25.0; // Not simulated
  }

  @Override
  public void setOffset() {
    pivotSim.setState(0, 0);
  }

  @Override
  public void runPosition(double position) {
    super.runPosition(position / 360d); // convert degrees to rotations
  }
}
