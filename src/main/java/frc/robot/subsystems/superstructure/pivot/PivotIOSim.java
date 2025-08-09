package frc.robot.subsystems.superstructure.pivot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.superstructure.GenericSuperstructureIOSim;
import com.ctre.phoenix6.signals.GravityTypeValue;

public class PivotIOSim extends GenericSuperstructureIOSim implements PivotIO {
    private static final double DEFAULT_GEAR_RATIO = 10.0;

    private final SingleJointedArmSim pivotSim;
    private final double reduction;

    public PivotIOSim() {
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
        super(PivotConstants.PIVOT_CONFIG.motorID());

        boolean inverted = PivotConstants.INVERT_MOTOR;
        this.reduction = PivotConstants.PIVOT_CONFIG.reduction();

        // FIXME: Example parameters
        double momentOfInertia = PivotConstants.PIVOT_LENGTH * PivotConstants.PIVOT_LENGTH;
        double lengthMeters = frc.robot.utility.UnitConversions.inchesToMeters(PivotConstants.PIVOT_LENGTH);
        double minAngleRads = -Math.PI / 2; // example: replace with meaningful constant if available
        double maxAngleRads = Math.PI / 2;
        boolean simulateGravity = PivotConstants.GRAVITY_TYPE == GravityTypeValue.Arm_Cosine;

        pivotSim = new SingleJointedArmSim(
                DCMotor.getKrakenX60Foc(1),
                reduction,
                momentOfInertia,
                lengthMeters,
                minAngleRads,
                maxAngleRads,
                simulateGravity,
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
        double rotations = pivotSim.getAngleRads() / (2 * Math.PI * PivotConstants.PIVOT_CONFIG.reduction());
        double velocityRPS = pivotSim.getVelocityRadPerSec() / (2 * Math.PI * PivotConstants.PIVOT_CONFIG.reduction());

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
}
