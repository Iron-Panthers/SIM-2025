package frc.robot.subsystems.canWatchdog;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.rgb.RGB;
import frc.robot.subsystems.rgb.RGB.RGBMessages;
import org.littletonrobotics.junction.Logger;

public class CANWatchdog {
  private final Thread canWatchdogThread;
  private CANWatchdogIO io;
  private RGB rgb;

  /** Creates a new CANWatchdog. */
  public CANWatchdog(CANWatchdogIO io, RGB rgb) {
    canWatchdogThread = new Thread(this::periodic, "CAN Watchdog Thread");
    canWatchdogThread.setDaemon(true);
    canWatchdogThread.start();
    this.io = io;
    this.rgb = rgb;
  }

  public void periodic() {
    while (!Thread.currentThread().isInterrupted()) {
      sleep(CANWatchdogConstants.SCAN_DELAY_MS);
      int[] missingDevices = io.checkForMissingIds();
      if (missingDevices.length == 0) {
        CommandScheduler.getInstance()
            .schedule(rgb.endMessageCommand(RGBMessages.MISSING_CAN_DEVICE));
      } else {
        CommandScheduler.getInstance()
            .schedule(rgb.startMessageCommand(RGBMessages.MISSING_CAN_DEVICE));
      }
      Logger.recordOutput("CANWatchdog/MissingDevices", missingDevices.length);
    }
  }

  public void matchStarting() {
    canWatchdogThread.interrupt();
    CommandScheduler.getInstance().schedule(rgb.endMessageCommand(RGBMessages.MISSING_CAN_DEVICE));
  }

  /**
   * Sleep that handles interrupts and uses an int. Don't do this please?
   *
   * @param millis
   */
  public void sleep(final int millis) {
    try {
      Thread.sleep(millis);
    } catch (InterruptedException e) {
      // restore the interrupted status
      Thread.currentThread().interrupt();
    }
  }
}
