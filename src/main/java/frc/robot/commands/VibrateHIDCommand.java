package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class VibrateHIDCommand extends Command {
  private GenericHID hid;
  private double duration;
  private double startTime;
  private double strength;

  /**
   * makes a command to vibrate a controller
   *
   * @param controller the controller to call the rumble methods on
   * @param duration the time (seconds) to vibrate for
   * @param strength the strength [0, 1] double of the vibration
   */
  public VibrateHIDCommand(GenericHID hid, double duration, double strength) {
    this.hid = hid;
    this.duration = duration;
    this.strength = strength;
    startTime = 0;
  }

  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    hid.setRumble(RumbleType.kRightRumble, strength);
    hid.setRumble(RumbleType.kLeftRumble, strength);
  }

  @Override
  public void end(boolean interrupted) {
    hid.setRumble(RumbleType.kRightRumble, 0);
    hid.setRumble(RumbleType.kLeftRumble, 0);
  }

  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() >= startTime + duration;
  }
}
