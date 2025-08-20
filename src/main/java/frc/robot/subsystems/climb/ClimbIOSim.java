package frc.robot.subsystems.climb;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureIOSim;

public class ClimbIOSim extends GenericSuperstructureIOSim implements ClimbIO {

  private final SingleJointedArmSim climbSim;
  private final double reduction;

  public ClimbIOSim() {
    // int id,
    // boolean inverted,
    // double supplyCurrentLimit,
    // Optional<Integer> canCoderID,
    // Optional<Double> canCoderOffset,
    // Optional<com.ctre.phoenix6.signals.SensorDirectionValue> direction,
    // Optional<Double> sensorDiscontinuityPoint,
    // double reduction,
    // Optional<Double> upperLimit,
    // Optional<Double> lowerLimit,
    // double upperVoltLimit,
    // double lowerVoltLimit,
    // double zeroingVolts,
    // double zeroingOffset,
    // double zeroingVoltageThreshold) {
    super(ClimbConstants.CLIMB_CONFIG.motorID());

    this.reduction = ClimbConstants.CLIMB_CONFIG.reduction();

    climbSim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(1),
            reduction,
            ClimbConstants.PHYSICAL_CONSTANTS.momentOfInertia(),
            ClimbConstants.PHYSICAL_CONSTANTS.lengthMeters(),
            ClimbConstants.PHYSICAL_CONSTANTS.minAngleRads(),
            ClimbConstants.PHYSICAL_CONSTANTS.maxAngleRads(),
            ClimbConstants.PHYSICAL_CONSTANTS.simulateGravity(),
            0);
    setOffset();
    setSlot0(
        ClimbConstants.GAINS.kP(),
        ClimbConstants.GAINS.kI(),
        ClimbConstants.GAINS.kD(),
        ClimbConstants.GAINS.kS(),
        ClimbConstants.GAINS.kV(),
        ClimbConstants.GAINS.kA(),
        ClimbConstants.GAINS.kG(),
        ClimbConstants.MOTION_MAGIC_CONFIG.acceleration(),
        ClimbConstants.MOTION_MAGIC_CONFIG.cruiseVelocity(),
        0,
        ClimbConstants.GRAVITY_TYPE);
  }

  @Override
  public void updateInputs(GenericSuperstructureIOInputs inputs) {
    // Update TalonFX state
    talon.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

    double appliedVoltage = talon.getSimState().getMotorVoltage();

    // Simulate physics
    climbSim.setInputVoltage(appliedVoltage);
    climbSim.update(0.02);

    // Convert position and velocity from meters to rotations for the TalonFX sensor
    double rotations = climbSim.getAngleRads() / (2 * Math.PI * reduction);
    double velocityRPS = climbSim.getVelocityRadPerSec() / (2 * Math.PI * reduction);

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
    climbSim.setState(0, 0);
  }
}
