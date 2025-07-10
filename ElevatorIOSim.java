package frc.robot.subsystems.superstructure.elevator;

import java.util.Optional;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.system.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
    private static final double DEFAULT_GEAR_RATIO = 10.0;

    protected final TalonFX talon;
    private final ElevatorSim elevatorSim;
    private final double reduction;

    protected final VoltageOut voltageOutput = new VoltageOut(0).withUpdateFreqHz(0);
    private final NeutralOut neutralOutput = new NeutralOut();
    private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0).withUpdateFreqHz(0);

    public ElevatorIOSim(
            int id,
            boolean inverted,
            double supplyCurrentLimit,
            Optional<Integer> canCoderID,
            Optional<Double> canCoderOffset,
            Optional<com.ctre.phoenix6.signals.SensorDirectionValue> direction,
            Optional<Double> sensorDiscontinuityPoint,
            double reduction,
            Optional<Double> upperLimit,
            Optional<Double> lowerLimit,
            double upperVoltLimit,
            double lowerVoltLimit,
            double zeroingVolts,
            double zeroingOffset,
            double zeroingVoltageThreshold) {
        talon = new TalonFX(id);
        talon.setInverted(inverted);
        talon.setNeutralMode(NeutralModeValue.Brake);

        this.reduction = reduction;

        // Example parameters
        double massKg = 5.0; // mass of the elevator carriage
        double drumRadiusMeters = 0.02; // radius of the drum
        double gearing = 10.0; // gear reduction
        double minHeightMeters = 0.0;
        double maxHeightMeters = 1.5;
        boolean simulateGravity = true;

        motorSim = new ElevatorSim(
                DCMotor.getKrakenX60Foc(1),
                reduction,
                massKg,
                drumRadiusMeters,
                minHeightMeters,
                maxHeightMeters,
                simulateGravity);
        setOffset();
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        // Simulate physics
        double appliedVoltage = talon.getSimState().getMotorVoltageMeasure().getValueAsDouble();
        motorSim.setInputVoltage(appliedVoltage);
        motorSim.update(0.02);

        // Simulate position/velocity
        double rotations = motorSim.getAngularPositionRotations();
        double velocityRPS = motorSim.getAngularVelocityRPM() / 60.0;

        inputs.connected = true;
        inputs.positionRotations = rotations;
        inputs.velocityRotPerSec = velocityRPS;
        inputs.appliedVolts = appliedVoltage;
        inputs.supplyCurrentAmps = 0.0; // Not simulated
        inputs.tempCelsius = 25.0; // Not simulated
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
        motorSim.setState(0, 0);
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

        // In sim, just store configs or ignore
    }
}
