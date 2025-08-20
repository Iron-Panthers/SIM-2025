package frc.robot.subsystems.rollers.intake;

import static frc.robot.subsystems.rollers.intake.IntakeConstants.*;

import frc.robot.lib.generic_subsystems.rollers.*;

public class IntakeIOTalonFX extends GenericRollersIOTalonFX implements IntakeIO {

  public IntakeIOTalonFX() {
    super(ID, CURRENT_LIMIT_AMPS, INVERTED, BRAKE, REDUCTION);
  }
}
