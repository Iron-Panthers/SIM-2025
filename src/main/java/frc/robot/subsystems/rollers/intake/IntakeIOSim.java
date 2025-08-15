package frc.robot.subsystems.rollers.intake;

import static frc.robot.subsystems.rollers.intake.IntakeConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.rollers.GenericRollersIOSim;
import frc.robot.subsystems.rollers.GenericRollersIOTalonFX;

public class IntakeIOSim extends GenericRollersIOSim implements IntakeIO {

    private final FlywheelSim intakeSim;

    public IntakeIOSim() {
        super(ID, CURRENT_LIMIT_AMPS, INVERTED, BRAKE, REDUCTION);
        intakeSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), MOI, REDUCTION),
                DCMotor.getKrakenX60Foc(1));

    }

    @Override
    public void updateInputs(GenericRollersIOInputs inputs) {
        // Update TalonFX state
        talon.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

        double appliedVoltage = talon.getSimState().getMotorVoltage();

        // Simulate physics
        intakeSim.setInputVoltage(appliedVoltage);
        intakeSim.update(0.02);

        double rotations = intakeSim.get()
                / (2 * Math.PI * ElevatorConstants.PHYSICAL_CONSTANTS.drumRadiusMeters())
                * reduction;

        // Correct unit conversion: meters/s to rotations/s
        double velocityRPS = elevatorSim.getAngularVelocityRadPerSec()
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
}
