package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import frc.robot.subsystems.swerve.DriveConstants.ModuleConfig;
import frc.robot.utility.PhoenixUtil;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

public class ModuleIOTalonFXSim extends ModuleIOTalonFX {
  private final SwerveModuleSimulation simulation;

  public ModuleIOTalonFXSim(ModuleConfig constants, SwerveModuleSimulation simulation) {
    super(PhoenixUtil.regulateModuleConstantForSimulation(constants));

    this.simulation = simulation;
    simulation.useDriveMotorController(new PhoenixUtil.TalonFXMotorControllerSim(driveTalon));

    simulation.useSteerMotorController(
        new PhoenixUtil.TalonFXMotorControllerWithRemoteCancoderSim(steerTalon, encoder));
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    super.updateInputs(inputs);
  }
}
