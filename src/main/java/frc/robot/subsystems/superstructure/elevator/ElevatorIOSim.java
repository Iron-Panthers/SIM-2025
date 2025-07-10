package frc.robot.subsystems.superstructure.elevator;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.system.simulation.DCMotorSim;
import edu.wpi.first.math.util.Units;

public class ElevatorIOSim implements ElevatorIO {
    private static final double GEAR_RATIO = 10.0;
    private final TalonFX talon = new TalonFX(1); // Device ID 1 adjust as needed
    private final TalonFXSimState talonSim = talon.getSimState();
    private final DCMotorSim motorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    DCMotor.getKrakenX60Foc(1), 0.001, GEAR_RATIO),
            DCMotor.getKrakenX60Foc(1));

    public ElevatorIOSim() {
        talonSim.Orientation = ChassisReference.CounterClockwise_Positive;
    }

    @Override
    public void runCharacterization() {
        // Not implemented for sim
    }

    @Override
    public void runPosition(double position) {
        talon.setPosition(position);
    }

    @Override
    public void setOffset() {
        talon.setPosition(0);
    }

    @Override
    public void setSlot0(double kP, double kI, double kD, double kS, double kV, double kA, double kG,
            double motionMagicAcceleration, double motionMagicCruiseVelocity, double jerk,
            GravityTypeValue gravityTypeValue) {
        // Not implemented for sim
    }

    @Override
    public void stop() {
        talon.set(0);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        // Simulate physics
        talonSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double motorVoltage = talonSim.getMotorVoltageMeasure().getValueAsDouble();
        motorSim.setInputVoltage(motorVoltage);
        motorSim.update(0.02); // 20 ms loop

        // Apply simulated position/velocity to TalonFX
        talonSim.setRawRotorPosition(motorSim.getAngularPosition().times(GEAR_RATIO));
        talonSim.setRotorVelocity(motorSim.getAngularVelocity().times(GEAR_RATIO));

        // Update inputs
        inputs.positionRotations = talon.getPosition().getValueAsDouble();
        inputs.velocityRPS = talon.getVelocity().getValueAsDouble();
        inputs.supplyCurrentAmps = talon.getSupplyCurrent().getValueAsDouble();
        inputs.appliedVolts = talon.getMotorVoltage().getValueAsDouble();
    }
}
