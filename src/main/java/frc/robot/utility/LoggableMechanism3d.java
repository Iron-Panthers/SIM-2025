package frc.robot.utility;

import edu.wpi.first.math.geometry.Pose3d;

/**
 * The {@code LoggableMechanism3d} interface represents a mechanism that can be logged and
 * visualized in a 3D environment. Implementing classes must provide a method of storing the parent
 * object and getting its display position
 */
public interface LoggableMechanism3d {

  /**
   * Retrieves the position of the parent mechanism in the 3D scene.
   *
   * @return a {@link Pose3d} object representing the position of the parent mechanism.
   */
  public Pose3d getParentPosition();

  /**
   * Sets the parent mechanism for this object.
   *
   * @param parent the parent mechanism, represented as a {@link LoggableMechanism3d}.
   */
  public void setParent(LoggableMechanism3d parent);

  /*
   * Calculates and returns the position of this mechanism in the 3D scene.
   *
   * @return a {@link Pose3d} object representing the position of this mechanism in the 3D scene.
   */
  public Pose3d getDisplayPose3d();
}
