package frc.robot.subsystems.rollers;

import edu.wpi.first.wpilibj.DigitalInput;

public class RollerSensorsIOSim implements RollerSensorsIO {

    @Override
    public void updateInputs(RollerSensorsIOInputs inputs) {
        inputs.intakeDetected = false;
    }
}
