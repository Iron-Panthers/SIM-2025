package frc.robot.subsystems.superstructure.elevator;

import static frc.robot.utility.UnitConversions.inchesToMeters;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.superstructure.GenericSuperstructure;
import frc.robot.utility.LoggableMechanism3d;
import org.littletonrobotics.junction.Logger;

public class Elevator extends GenericSuperstructure<Elevator.ElevatorTarget>
    implements LoggableMechanism3d {

  public enum ElevatorTarget implements GenericSuperstructure.PositionTarget {
    BOTTOM(0.6),
    L1(11),
    L2(20),
    L3(32.4),
    SETUP_L4(31.6),
    SCORE_L4(30),
    TOP(31),
    INTAKE(0),
    CLIMB(13),
    DESCORE_HIGH(19.5),
    DESCORE_LOW(9.2),
    INTAKE_SIDE(13),
    SCORE_SIDE(13),
    SAFE_MIDWAY(11.5);

    private double position = 0;

    private static final double EPSILON = ElevatorConstants.POSITION_TARGET_EPSILON;

    private ElevatorTarget(double position) {
      this.position = position;
    }

    public double getPosition() {
      return position;
    }

    public double getEpsilon() {
      return EPSILON;
    }
  }

  private final LinearFilter supplyCurrentFilter;

  private double filteredSupplyCurrentAmps = 0;

  private boolean zeroing = false;

  public Elevator(ElevatorIO io) {
    super("Elevator", io);
    setPositionTarget(ElevatorTarget.INTAKE);
    setControlMode(ControlMode.STOP);

    // setup the linear filter
    supplyCurrentFilter = LinearFilter.movingAverage(30);
  }

  @Override
  public void periodic() {

    super.periodic();

    // for zeroing
    // calculate our new filtered supply current for the elevator
    filteredSupplyCurrentAmps = supplyCurrentFilter.calculate(getSupplyCurrentAmps());
    if (zeroing) {
      superstructureIO.runCharacterization();
    }
    Logger.recordOutput(
        "Superstructure/" + name + "/Filtered supply current amps", getFilteredSupplyCurrentAmps());
  }

  public double getFilteredSupplyCurrentAmps() {
    return filteredSupplyCurrentAmps;
  }

  public boolean aboveSafeHeightForPivot() {
    return this.getPosition() > ElevatorConstants.MIN_SAFE_HEIGHT_FOR_PIVOT;
  }

  public void setZeroing(boolean zeroing) {
    this.zeroing = zeroing;
  }

  public boolean isZeroing() {
    return zeroing;
  }

  @Override
  public Pose3d getDisplayPose3d(Pose3d parentPose3d) {
    return parentPose3d.plus(
        new Transform3d(
            new Translation3d(inchesToMeters(0), inchesToMeters(0), inchesToMeters(getPosition())),
            new Rotation3d(0, 0, 0))); // Placeholder,
    // replace
    // with
    // actual
    // pose
    // logic
  }
}
