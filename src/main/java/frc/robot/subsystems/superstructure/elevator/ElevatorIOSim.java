package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.subsystems.superstructure.GenericSuperstructureIOSim;

public class ElevatorIOSim extends GenericSuperstructureIOSim implements ElevatorIO {
  private static final double DEFAULT_GEAR_RATIO = 10.0;

  private final ElevatorSim elevatorSim;
  private final double reduction;

  public ElevatorIOSim() {
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
    super(ElevatorConstants.ELEVATOR_CONFIG.motorID());

    boolean inverted = ElevatorConstants.INVERT_MOTOR;
    this.reduction = ElevatorConstants.ELEVATOR_CONFIG.reduction();

    // Example parameters
    double massKg = 5.0; // mass of the elevator carriage
    double drumRadiusMeters = 0.02; // radius of the drum
    double gearing = 10.0; // gear reduction
    double minHeightMeters = -1.0;
    double maxHeightMeters = 1.5;
    boolean simulateGravity = true;

    elevatorSim =
        new ElevatorSim(
            DCMotor.getKrakenX60Foc(1),
            reduction,
            massKg,
            drumRadiusMeters,
            minHeightMeters,
            maxHeightMeters,
            simulateGravity,
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

    // Convert position and velocity from meters to rotations for the TalonFX sensor
    double drumRadiusMeters = 0.02; // Must match your sim config
    double rotations =
        elevatorSim.getPositionMeters() / (2 * Math.PI * drumRadiusMeters) * reduction;
    double velocityRPS =
        elevatorSim.getVelocityMetersPerSecond() / (2 * Math.PI * drumRadiusMeters) * reduction;

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
