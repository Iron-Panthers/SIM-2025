package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.lib.generic_subsystems.superstructure.*;

public class ElevatorIOSim extends GenericSuperstructureIOSim implements ElevatorIO {

  private final ElevatorSim elevatorSim;

  private final double reduction;

  public ElevatorIOSim() {
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
    super(ElevatorConstants.ELEVATOR_CONFIG.motorID());
    this.reduction = ElevatorConstants.ELEVATOR_CONFIG.reduction();

    elevatorSim =
        new ElevatorSim(
            DCMotor.getKrakenX60Foc(2),
            reduction,
            ElevatorConstants.PHYSICAL_CONSTANTS.elevatorMassKg(),
            ElevatorConstants.PHYSICAL_CONSTANTS.drumRadiusMeters(),
            ElevatorConstants.PHYSICAL_CONSTANTS.minHeightMeters(),
            ElevatorConstants.PHYSICAL_CONSTANTS.maxHeightMeters(),
            ElevatorConstants.PHYSICAL_CONSTANTS.simulateGravity(),
            0);
    setOffset();
    setSlot0(
        ElevatorConstants.GAINS.kP(),
        ElevatorConstants.GAINS.kI(),
        ElevatorConstants.GAINS.kD(),
        ElevatorConstants.GAINS.kS(),
        ElevatorConstants.GAINS.kV(),
        ElevatorConstants.GAINS.kA(),
        ElevatorConstants.GAINS.kG(),
        ElevatorConstants.MOTION_MAGIC_CONFIG.acceleration(),
        ElevatorConstants.MOTION_MAGIC_CONFIG.cruiseVelocity(),
        ElevatorConstants.MOTION_MAGIC_CONFIG.jerk(),
        ElevatorConstants.GRAVITY_TYPE);
  }

  @Override
  public void updateInputs(GenericSuperstructureIOInputs inputs) {
    // Update TalonFX state
    talon.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

    double appliedVoltage = talon.getSimState().getMotorVoltage();

    // Simulate physics
    elevatorSim.setInputVoltage(appliedVoltage);
    elevatorSim.update(0.02);

    // Convert position and velocity from meters to rotations for the
    // TalonFX sensor
    // Correct unit conversion: meters to rotations
    double rotations =
        elevatorSim.getPositionMeters()
            / (2 * Math.PI * ElevatorConstants.PHYSICAL_CONSTANTS.drumRadiusMeters())
            * reduction;

    // Correct unit conversion: meters/s to rotations/s
    double velocityRPS =
        elevatorSim.getVelocityMetersPerSecond()
            / (2 * Math.PI * ElevatorConstants.PHYSICAL_CONSTANTS.drumRadiusMeters())
            * reduction;

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
    elevatorSim.setState(0, 0);
  }
}
